//
// Created by lopatoj on 11/17/25.
//
#include <iostream>
#include <fstream>
#include <sstream>

#include "stretch_ik/stretch_ik.hpp"
#include "boost/program_options.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "vamp/collision/environment.hh"
#include "vamp/robots/stretch.hh"
#include "vamp/planning/validate.hh"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rcpputils/filesystem_helper.hpp>

// #include "rerun/recording_stream.hpp"
// #include "rerun/archetypes/coordinate_frame.hpp"
// #include "rerun/archetypes/points3d.hpp"
// #include "rerun/archetypes/transform3d.hpp"
#include "ompl/geometric/PathSimplifier.h"

using Robot = vamp::robots::Stretch;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

inline static auto ompl_to_vamp(const ob::State* state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
        aligned_buffer{};

    // Copy OMPL data into aligned buffer
    const auto* as = state->as<ob::CompoundStateSpace::StateType>();
    const auto* rs = as->components[0]->as<ompl::base::ReedsSheppStateSpace::StateType>();
    aligned_buffer[0] = static_cast<float>(rs->getX());
    aligned_buffer[1] = static_cast<float>(rs->getY());
    aligned_buffer[2] = static_cast<float>(rs->getYaw());
    const auto* rv = as->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
    aligned_buffer[3] = static_cast<float>(rv->values[0]);
    aligned_buffer[4] = static_cast<float>(rv->values[1]);
    aligned_buffer[5] = static_cast<float>(rv->values[2]);
    aligned_buffer[6] = static_cast<float>(rv->values[3]);
    aligned_buffer[7] = static_cast<float>(rv->values[4]);
    const auto* s1 = as->components[2]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[8] = static_cast<float>(s1->value);
    const auto* s2 = as->components[3]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[9] = static_cast<float>(s2->value);
    const auto* s3 = as->components[4]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[10] = static_cast<float>(s3->value);
    const auto* s4 = as->components[5]->as<ompl::base::SO2StateSpace::StateType>();
    aligned_buffer[11] = static_cast<float>(s4->value);
    const auto* s5 = as->components[6]->as<ompl::base::SO2StateSpace::StateType>();
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
    VAMPStateValidator(ob::SpaceInformation* si, const EnvironmentVector& env_v)
        : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr& si, const EnvironmentVector& env_v)
        : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State* state) const -> bool override
    {
        auto configuration = ompl_to_vamp(state);
        return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
    }

    const EnvironmentVector& env_v;
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    VAMPMotionValidator(ob::SpaceInformation* si, const EnvironmentVector& env_v)
        : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr& si, const EnvironmentVector& env_v)
        : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State* s1, const ob::State* s2) const -> bool override
    {
        // Convert OMPL states to VAMP vectors and check motion between states
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto checkMotion(const ob::State*, const ob::State*, std::pair<ob::State*, double>&) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    const EnvironmentVector& env_v;
};


StretchIK initIK(const std::string& urdfPath)
{
    std::ifstream urdfFileStream(urdfPath);
    std::stringstream buffer;
    buffer << urdfFileStream.rdbuf();
    std::string urdfFile = buffer.str();

    auto stretchIk = StretchIK();
    stretchIk.setBaseLink("base_link");
    stretchIk.setEndEffectorLink("link_grasp_center");
    stretchIk.initModel(urdfFile);
    stretchIk.initSolver(StretchIK::TRAC_IK);
    return stretchIk;
}

EnvironmentVector initEnvironment(const std::string& pcdPath)
{
    EnvironmentInput environment;
    pcl::PointCloud<pcl::PointXYZ> cloud = pcl::PointCloud<pcl::PointXYZ>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, cloud) != -1)
    {
        std::vector<vamp::collision::Point> points;
        for (auto point : cloud.points)
        {
            points.push_back({point.x, point.y, point.z});
        }
        environment.pointclouds.emplace_back(points, 0.006, 0.105, 0.1);
    }
    environment.sort();

    // Log a batch of points.
    return EnvironmentVector(environment);
}

int main(const int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "print version string")
        ("urdf_path,u", po::value<std::string>()->required())
        ("pcd_path,p", po::value<std::string>()->required());

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);


    auto stretchIk = initIK(vm["urdf_path"].as<std::string>());
    auto env_v = initEnvironment(vm["pcd_path"].as<std::string>());

    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);

    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);

    auto space = std::make_shared<ob::CompoundStateSpace>();

    auto rs = std::make_shared<ob::ReedsSheppStateSpace>();

    ob::RealVectorBounds rsBounds(2);
    for (auto i = 0U; i < 2; ++i)
    {
        float low = zero_v[{0, i}];
        float high = one_v[{0, i}];
        std::cout << "low: " << low << std::endl;
        std::cout << "high: " << high << std::endl;
        rsBounds.setLow(i, low);
        rsBounds.setHigh(i, high);
    }
    rs->setBounds(rsBounds);
    space->addSubspace(rs, 1.0);

    auto rv = std::make_shared<ob::RealVectorStateSpace>(5);

    ob::RealVectorBounds rvBounds(5);
    for (auto i = 0U; i < 5; ++i)
    {
        float low = zero_v[{0, i+3}];
        float high = one_v[{0, i+3}];
        std::cout << "low: " << low << std::endl;
        std::cout << "high: " << high << std::endl;
        rvBounds.setLow(i, low);
        rvBounds.setHigh(i, high);
    }
    rv->setBounds(rvBounds);
    space->addSubspace(rv, 1.0);

    for (auto i = 0U; i < 5; ++i)
    {
        auto so2 = std::make_shared<ob::SO2StateSpace>();
        space->addSubspace(so2, 1.0);
    }

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
    si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
    si->setup();

    KDL::Frame grasp_pose(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.5, 0.5));

    stretchIk.solve(grasp_pose);
    auto cfg = stretchIk.getJointPositions();

    const std::array<float, dimension> start = {-1.0, -3.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const std::array<float, dimension> goal = {
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

    ob::ScopedState<> start_ompl(space), goal_ompl(space);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_ompl[i] = start[i];
        goal_ompl[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);

    auto planner = std::make_shared<og::RRTConnect>(si);

    planner->setProblemDefinition(pdef);
    planner->setup();

    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(10);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms" << std::endl;

        // Simplify the path using OMPL's path simplification
        const ob::PathPtr& path = pdef->getSolutionPath();
        auto& path_geometric = dynamic_cast<og::PathGeometric&>(*path);

        auto path_simplifier = std::make_shared<og::PathSimplifier>(si, pdef->getGoal());

        std::cout << "Solution:" << std::endl;
        path_simplifier->simplify(path_geometric, 10);
        path_geometric.interpolate(100);
        path_geometric.printAsMatrix(std::cout);

        std::ofstream file("/home/lopatoj/ros2_ws/path.txt");
        path_geometric.printAsMatrix(file);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
