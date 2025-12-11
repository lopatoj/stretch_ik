//
// Created by lopatoj on 11/17/25.
//

#ifndef ROS2_WS_STRETCHIK_H
#define ROS2_WS_STRETCHIK_H

#include <fstream>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "urdf/model.h"

class StretchIK
{
public:
    enum IKType
    {
        KDL_NR, // basic KDL IK Solver
        KDL_NR_JL, // KDL Solver that accounts for joint limits
        KDL_LMA,
        TRAC_IK
    };

    void setBaseLink(const std::string& chainRoot)
    {
        chainRoot_ = chainRoot;
    }

    void setEndEffectorLink(const std::string& chainTip)
    {
        chainTip_ = chainTip;
    }

    std::vector<std::string> getJointNames()
    {
        return jointNames_;
    }

    [[nodiscard]] std::vector<double> getJointPositions() const
    {
        const Eigen::VectorXd& data = curr_.data;
        return {data.data(), data.data() + data.size()};
    }

    void initModel(const std::string& urdf);

    void initSolver(IKType ikType);

    int solve(const KDL::Frame& pose);

private:
    std::string chainRoot_;
    std::string chainTip_;

    urdf::Model model_;
    KDL::Tree tree_;
    KDL::Chain chain_;

    std::unique_ptr<KDL::ChainFkSolverPos> fkpSolver_;
    std::unique_ptr<KDL::ChainIkSolverVel> ikvSolver_;
    std::unique_ptr<KDL::ChainIkSolverPos> solver_;

    std::vector<std::string> jointNames_;
    KDL::JntArray curr_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;
};


#endif //ROS2_WS_STRETCHIK_H