#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "stretch_ik/stretch_ik.hpp"

std::map<std::string, StretchIK::IKType> stringToIkType = {
    {"KDL_NR", StretchIK::IKType::KDL_NR},
    {"KDL_NR_JL", StretchIK::IKType::KDL_NR_JL},
    {"KDL_LMA", StretchIK::IKType::KDL_LMA},
    {"TRAC_IK", StretchIK::IKType::TRAC_IK}
};

class StretchIKNode : public rclcpp::Node
{
public:
    StretchIKNode() : Node("stretch_ik_node")
    {
        declare_parameter("ik_type", "KDL_NR_JL");
        declare_parameter("chain_root", "base_link");
        declare_parameter("chain_tip", "link_grasp_center");

        robotDescriptionSubscription_ = create_subscription<std_msgs::msg::String>(
            "robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1))
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            [this](const std_msgs::msg::String& msg) { this->robotDescriptionCallback(msg); });

        graspPoseSubscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose",
            10,
            [this](const geometry_msgs::msg::PoseStamped& msg) { this->graspPoseCallback(msg); });

        jointStatePublisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS());

        const std::string ik_type_str = get_parameter("ik_type").as_string();

        try
        {
            ikType_ = stringToIkType.at(ik_type_str);
        }
        catch (const std::out_of_range& e)
        {
            RCLCPP_WARN(get_logger(), "%s is not a valid ik_type", ik_type_str.c_str());
            ikType_ = StretchIK::IKType::KDL_NR_JL;
        }

        ik_.setBaseLink(get_parameter("chain_root").as_string());
        ik_.setEndEffectorLink(get_parameter("chain_tip").as_string());

        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters)
        {
            return this->setParametersCallback(parameters);
        });
    }

private:
    void robotDescriptionCallback(const std_msgs::msg::String& msg)
    {
        const std::string urdf = msg.data;

        ik_.initModel(urdf);

        jointStates_.header.stamp = this->get_clock()->now();
        jointStates_.name = ik_.getJointNames();
        jointStates_.position = ik_.getJointPositions();
        jointStatePublisher_->publish(jointStates_);

        ik_.initSolver(ikType_);
    }

    void graspPoseCallback(const geometry_msgs::msg::PoseStamped& msg)
    {
        const auto pos = msg.pose.position;
        const auto quat = msg.pose.orientation;
        const KDL::Frame pose(KDL::Rotation::Quaternion(quat.x, quat.y, quat.z, quat.w),
                              KDL::Vector(pos.x, pos.y, pos.z));

        if (ik_.solve(pose) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "IK failed");
            return;
        }

        jointStates_.header.stamp = this->get_clock()->now();
        jointStates_.name = ik_.getJointNames();
        jointStates_.position = ik_.getJointPositions();
        jointStatePublisher_->publish(jointStates_);
    }

    rcl_interfaces::msg::SetParametersResult setParametersCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        for (const auto& parameter : parameters)
        {
            if (parameter.get_name().compare("ik_type"))
            {
                setIkType(parameter.as_string());
            }
        }

        return rcl_interfaces::msg::SetParametersResult();
    }

    void setIkType(std::string ikTypeStr)
    {
        try
        {
            ikType_ = stringToIkType.at(ikTypeStr);
        }
        catch (const std::out_of_range& e)
        {
            RCLCPP_WARN(this->get_logger(), "%s is not a valid ik_type", ikTypeStr.c_str());
            ikType_ = StretchIK::IKType::KDL_NR_JL;
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr graspPoseSubscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;

    sensor_msgs::msg::JointState jointStates_;
    StretchIK ik_;
    StretchIK::IKType ikType_{StretchIK::KDL_NR_JL};
};

int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StretchIKNode>());
    rclcpp::shutdown();
    return 0;
}
