//
// Created by lopatoj on 11/17/25.
//

#include "stretch_ik/chainiksolverpos_tl_adapter.hpp"
#include "kdl/chainiksolvervel_pinv_givens.hpp"


KDL::ChainIkSolverPos_TL_Adapter::ChainIkSolverPos_TL_Adapter(const Chain& chain, const JntArray& q_min,
                                                              const JntArray& q_max, double maxtime,
                                                              double eps, bool random_restart,
                                                              bool try_jl_wrap) :
    solver_(ChainIkSolverPos_TL(chain, q_min, q_max, maxtime, eps, random_restart, try_jl_wrap))
{
}

int KDL::ChainIkSolverPos_TL_Adapter::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in,
                                                KDL::JntArray& q_out)
{
    return solver_.CartToJnt(q_init, p_in, q_out);
}
