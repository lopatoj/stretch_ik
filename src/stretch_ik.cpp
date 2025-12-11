//
// Created by lopatoj on 11/17/25.
//

#include "stretch_ik/stretch_ik.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv_givens.hpp"
#include "stretch_ik/chainiksolverpos_tl_adapter.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr.hpp"


void StretchIK::initModel(const std::string& urdf)
{
    model_.initString(urdf);
    kdl_parser::treeFromUrdfModel(model_, tree_);
    tree_.getChain(chainRoot_, chainTip_, chain_);

    curr_ = KDL::JntArray(chain_.getNrOfJoints());
    q_min_ = KDL::JntArray(chain_.getNrOfJoints());
    q_max_ = KDL::JntArray(chain_.getNrOfJoints());

    jointNames_.clear();

    unsigned int i = 0;
    for (const auto& segment : chain_.segments)
    {
        if (KDL::Joint joint = segment.getJoint(); joint.getType() != KDL::Joint::Fixed && i < chain_.getNrOfJoints())
        {
            jointNames_.push_back(joint.getName());
            if (const auto limits = model_.getJoint(joint.getName())->limits)
            {
                q_min_(i) = limits->lower;
                q_max_(i) = limits->upper;
            }
            i++;
        }
    }
}

void StretchIK::initSolver(const IKType ikType)
{
    fkpSolver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ikvSolver_ = std::make_unique<KDL::ChainIkSolverVel_pinv_givens>(chain_);

    switch (ikType)
    {
    case KDL_NR:
        solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(chain_, *fkpSolver_, *ikvSolver_);
        break;
    case KDL_NR_JL:
        solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain_, q_min_, q_max_, *fkpSolver_, *ikvSolver_);
        break;
    case KDL_LMA:
        solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
        break;
    case TRAC_IK:
        solver_ = std::make_unique<KDL::ChainIkSolverPos_TL_Adapter>(chain_, q_min_, q_max_);
        break;
    }
}

int StretchIK::solve(const KDL::Frame& pose)
{
    KDL::JntArray q_out(chain_.getNrOfJoints());

    if (const int err = solver_->CartToJnt(curr_, pose, q_out); err < 0)
    {
        return err;
    }

    curr_ = q_out;
    return 0;
}
