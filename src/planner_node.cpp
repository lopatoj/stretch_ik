//
// Created by lopatoj on 12/10/25.
//
#include <iostream>
#include <string>
#include <map>

#include "stretch_ik/stretch_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/Planner.h"
#include "ompl/geometric/PathSimplifier.h"
#include "vamp/collision/environment.hh"
#include "vamp/robots/stretch.hh"
#include "vamp/planning/validate.hh"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using Robot = vamp::robots::Stretch;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer{};

    // Copy OMPL data into aligned buffer
    const auto *as = state->as<ob::CompoundStateSpace::StateType>();
    const auto *rs = as->components[0]->as<ompl::base::ReedsSheppStateSpace::StateType>();
    aligned_buffer[0] = static_cast<float>(rs->getX());
    aligned_buffer[1] = static_cast<float>(rs->getY());
    aligned_buffer[2] = static_cast<float>(rs->getYaw());
    const auto *rv = as->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
    aligned_buffer[3] = static_cast<float>(rv->values[0]);
    aligned_buffer[4] = static_cast<float>(rv->values[1]);
    aligned_buffer[5] = static_cast<float>(rv->values[2]);
    aligned_buffer[6] = static_cast<float>(rv->values[3]);
    aligned_buffer[7] = static_cast<float>(rv->values[4]);
    const auto *s1 = as->components[2]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[8] = static_cast<float>(s1->value);
    const auto *s2 = as->components[3]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[9] = static_cast<float>(s2->value);
    const auto *s3 = as->components[4]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[10] = static_cast<float>(s3->value);
    const auto *s4 = as->components[5]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[11] = static_cast<float>(s4->value);
    const auto *s5 = as->components[6]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[12] = static_cast<float>(s5->value);

    // Create configuration from aligned buffer data
    return {aligned_buffer.data()};
}

// inline static auto vamp_to_ompl(const Configuration& c, ob::State* state)
// {
//     const auto* as = state->as<ob::CompoundStateSpace::StateType>();
//     auto* rs = as->components[0]->as<ompl::base::ReedsSheppStateSpace::StateType>();
//     rs->setX(c[{0, 0}]);
//     rs->setY(c[{1, 0}]);
//     rs->setYaw(c[{2, 0}]);
//     const auto* rv = as->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
//     rv->values[0] = static_cast<double>(c[{3, 0}]);
//     rv->values[1] = static_cast<double>(c[{4, 0}]);
//     rv->values[2] = static_cast<double>(c[{5, 0}]);
//     rv->values[3] = static_cast<double>(c[{6, 0}]);
//     rv->values[4] = static_cast<double>(c[{7, 0}]);
//     auto* s1 = as->components[2]->as<ompl::base::SO2StateSpace::StateType>();
//     s1->value = static_cast<double>(c[{8, 0}]);
//     auto* s2 = as->components[3]->as<ompl::base::SO2StateSpace::StateType>();
//     s2->value = static_cast<double>(c[{9, 0}]);
//     auto* s3 = as->components[4]->as<ompl::base::SO2StateSpace::StateType>();
//     s3->value = static_cast<double>(c[{10, 0}]);
//     auto* s4 = as->components[5]->as<ompl::base::SO2StateSpace::StateType>();
//     s4->value = static_cast<double>(c[{11, 0}]);
//     auto* s5 = as->components[6]->as<ompl::base::SO2StateSpace::StateType>();
//     s5->value = static_cast<double>(c[{12, 0}]);
// }

// State validator using VAMP
struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
        : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
        : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        auto configuration = ompl_to_vamp(state);
        return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
    }

    const EnvironmentVector &env_v;
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
        : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
        : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        // Convert OMPL states to VAMP vectors and check motion between states
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    const EnvironmentVector &env_v;
};

class IKPlannerNode : public rclcpp::Node
{
public:
    IKPlannerNode() : Node("ik_planner_node")
    {
        const std::string ik_type_str = this->declare_parameter<std::string>("ik_type", "KDL_NR_JL");
        base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
        ee_link_ = this->declare_parameter<std::string>("ee_link", "link_grasp_center");
        world_frame_ = this->declare_parameter<std::string>("world_frame", "odom");
        ik_urdf_ = this->declare_parameter<std::string>("ik_urdf", "");

        grasp_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/grasp_pose",
            10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            { this->graspPoseCallback(msg); });

        cloud_map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_map",
            10,
            [this](const sensor_msgs::msg::PointCloud2 &msg)
            { this->cloudMapCallback(msg); });

        joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            [this](const sensor_msgs::msg::JointState &msg)
            { joint_states_ = msg; });

        follow_joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this,
            "/stretch_controller/follow_joint_trajectory"
        );


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        try
        {
            ik_type_ = string_to_ik_type_.at(ik_type_str);
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_WARN(get_logger(), "%s is not a valid ik_type", ik_type_str.c_str());
            ik_type_ = StretchIK::IKType::KDL_NR_JL;
        }

        // IK initialization
        ik_.setBaseLink(base_link_);
        ik_.setEndEffectorLink(ee_link_);
        ik_.initModel(ik_urdf_);
        ik_.initSolver(ik_type_);

        // State space bounds
        static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
        static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};

        auto zero_v = Configuration(zeros);
        auto one_v = Configuration(ones);

        Robot::scale_configuration(zero_v);
        Robot::scale_configuration(one_v);

        space_ = std::make_shared<ob::CompoundStateSpace>();

        auto rs = std::make_shared<ob::ReedsSheppStateSpace>();

        ob::RealVectorBounds rsBounds(2);
        for (auto i = 0U; i < 2; ++i)
        {
            float low = zero_v[{0, i}];
            float high = one_v[{0, i}];
            rsBounds.setLow(i, low);
            rsBounds.setHigh(i, high);
        }
        rs->setBounds(rsBounds);
        space_->addSubspace(rs, 1.0);

        auto rv = std::make_shared<ob::RealVectorStateSpace>(5);

        ob::RealVectorBounds rvBounds(5);
        for (auto i = 0U; i < 5; ++i)
        {
            float low = zero_v[{0, i + 3}];
            float high = one_v[{0, i + 3}];
            rvBounds.setLow(i, low);
            rvBounds.setHigh(i, high);
        }
        rv->setBounds(rvBounds);
        space_->addSubspace(rv, 1.0);

        for (auto i = 0U; i < 5; ++i)
        {
            auto so2 = std::make_shared<ob::SO2StateSpace>();
            space_->addSubspace(so2, 1.0);
        }

        si_ = std::make_shared<ob::SpaceInformation>(space_);
        pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
        planner_ = std::make_shared<og::RRTConnect>(si_);
    }

private:
    /**
     * @brief Pose callback to initiate planning to specified pose.
     */
    void graspPoseCallback(const geometry_msgs::msg::PoseStamped &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recieved grasp pose, performing IK and planning.");

        const auto pos = msg.pose.position;
        const auto quat = msg.pose.orientation;
        const KDL::Frame pose(KDL::Rotation::Quaternion(quat.x, quat.y, quat.z, quat.w),
                              KDL::Vector(pos.x, pos.y, pos.z));

        if (ik_.solve(pose) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "IK failed");
            return;
        }

        geometry_msgs::msg::TransformStamped t;

        try
        {
            t = tf_buffer_->lookupTransform(
                base_link_, world_frame_,
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                this->get_logger(), "Could not transform %s to %s: %s",
                base_link_.c_str(), world_frame_.c_str(), ex.what());
            return ;
        }

        tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        const std::array<float, dimension> start{
            static_cast<float>(t.transform.translation.x),
            static_cast<float>(t.transform.translation.y),
            static_cast<float>(y),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_lift"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_arm_l0"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_arm_l1"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_arm_l2"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_arm_l3"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_wrist_yaw"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_wrist_pitch"]]),
            static_cast<float>(joint_states_.position[joint_name_to_idx_["joint_wrist_roll"]]),
            0.0,
            0.0
        };

        auto cfg = ik_.getJointPositions();
        const std::array<float, dimension> goal{
            static_cast<float>(cfg[0]),
            static_cast<float>(cfg[1]),
            static_cast<float>(cfg[2]),
            static_cast<float>(cfg[3]),
            static_cast<float>(cfg[4]) / 4.0f,
            static_cast<float>(cfg[4]) / 4.0f,
            static_cast<float>(cfg[4]) / 4.0f,
            static_cast<float>(cfg[4]) / 4.0f,
            static_cast<float>(cfg[5]),
            static_cast<float>(cfg[6]),
            static_cast<float>(cfg[7]),
            0.0,
            0.0
        };

        ob::ScopedState<> start_ompl(space_), goal_ompl(space_);
        for (auto i = 0U; i < dimension; ++i)
        {
            start_ompl[i] = start[i];
            goal_ompl[i] = goal[i];
        }
        
        pdef_->setStartAndGoalStates(start_ompl, goal_ompl);
        planner_->setProblemDefinition(pdef_);
        planner_->setup();

        auto start_time = std::chrono::steady_clock::now();
        ob::PlannerStatus solved = planner_->ob::Planner::solve(10); // TODO: set time from parameter or other means
        auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

        // Only accept exact solutions
        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            RCLCPP_INFO(this->get_logger(), "Found solution in %f ms", nanoseconds / 1e6);

            // Simplify the path using OMPL's path simplification
            const ob::PathPtr &path = pdef_->getSolutionPath();
            auto &pathGeometric = dynamic_cast<og::PathGeometric &>(*path);

            auto pathSimplifier = std::make_shared<og::PathSimplifier>(si_, pdef_->getGoal());

            pathSimplifier->simplify(pathGeometric, 10);
            pathGeometric.interpolate(10);

            control_msgs::action::FollowJointTrajectory_Goal trajectoryGoal;
            trajectoryGoal.trajectory.header.stamp = this->get_clock()->now();
            trajectoryGoal.trajectory.joint_names = joint_control_names_;
            
            const int dt = 1.0;

            for(auto i = 0U; i < pathGeometric.getStateCount(); ++i) {
                const ob::State *state = pathGeometric.getState(i);
                trajectory_msgs::msg::JointTrajectoryPoint jtp;

                const ob::CompoundState *cState = state->as<ob::CompoundState>();
                const ob::RealVectorStateSpace::StateType *rvState = cState->as<ob::RealVectorStateSpace::StateType>(1);
                const ob::SO2StateSpace::StateType *yawState = cState->as<ob::SO2StateSpace::StateType>(2);
                const ob::SO2StateSpace::StateType *pitchState = cState->as<ob::SO2StateSpace::StateType>(3);
                const ob::SO2StateSpace::StateType *rollState = cState->as<ob::SO2StateSpace::StateType>(4);

                jtp.positions[0] = rvState->values[0];
                jtp.positions[1] = rvState->values[1] + rvState->values[2] + rvState->values[3] + rvState->values[4];
                jtp.positions[2] = yawState->value;
                jtp.positions[3] = pitchState->value;
                jtp.positions[4] = rollState->value;

                jtp.time_from_start = rclcpp::Duration(i * dt, 0);

                trajectoryGoal.trajectory.points.push_back(jtp);
            }

            follow_joint_trajectory_client_->async_send_goal(trajectoryGoal);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No solution found");
        }
    }

    /**
     * @brief Loads in environment points from latest pointcloud into a VAMP state validator.
     */
    void cloudMapCallback(const sensor_msgs::msg::PointCloud2 &msg)
    {
        EnvironmentInput environment;
        std::vector<vamp::collision::Point> points;

        sensor_msgs::PointCloud2ConstIterator<float> iter(msg, "x");
        for (; iter != iter.end(); ++iter)
        {
            points.push_back({iter[0], iter[1], iter[2]});
        }
        environment.pointclouds.emplace_back(points, 0.006, 0.105, 0.1); // TODO: replace with parameters
        environment.sort();

        auto envVector = EnvironmentVector(environment);

        si_->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si_, envVector));
        si_->setMotionValidator(std::make_shared<VAMPMotionValidator>(si_, envVector));
        si_->setup();
    }

    /**
     * @brief Callback to set joint_states_ field and set joint_name to idx mapping.
     */
    void jointStatesCallback(const sensor_msgs::msg::JointState &msg) {
        joint_states_ = msg;

        for(auto i = 0U; i < joint_states_.name.size(); ++i) {
            const std::string& name = joint_states_.name[i];
            if(joint_name_to_idx_.contains(name)) {
                joint_name_to_idx_[name] = i;
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client_;

    sensor_msgs::msg::JointState joint_states_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    StretchIK ik_;

    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<ob::CompoundStateSpace> space_;
    std::shared_ptr<ob::ProblemDefinition> pdef_;
    std::shared_ptr<ob::Planner> planner_;

    StretchIK::IKType ik_type_{StretchIK::KDL_NR_JL};
    std::string base_link_;
    std::string ee_link_;
    std::string ik_urdf_; // URDF file information loaded from parameter for IK specific stretch version
    std::string world_frame_;

    std::map<std::string, StretchIK::IKType> string_to_ik_type_{
        {"KDL_NR", StretchIK::IKType::KDL_NR},
        {"KDL_NR_JL", StretchIK::IKType::KDL_NR_JL},
        {"KDL_LMA", StretchIK::IKType::KDL_LMA},
        {"TRAC_IK", StretchIK::IKType::TRAC_IK},
    };

    std::map<std::string, int> joint_name_to_idx_{
        {"joint_lift", -1},
        {"joint_arm_l0", -1},
        {"joint_arm_l1", -1},
        {"joint_arm_l2", -1},
        {"joint_arm_l3", -1},
        {"joint_wrist_yaw", -1},
        {"joint_wrist_pitch", -1},
        {"joint_wrist_roll", -1},
    };

    std::vector<std::string> joint_control_names_ = {
        "joint_lift",
        "wrist_extension",
        "joint_wrist_yaw",
        "joint_wrist_pitch",
        "joint_wrist_roll",
    };
};

int main(const int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
