//
// Created by lopatoj on 11/17/25.
//

#ifndef ROS2_WS_CHAINIKSOLVERPOS_TL_ADAPTER_H
#define ROS2_WS_CHAINIKSOLVERPOS_TL_ADAPTER_H

#include "trac_ik/kdl_tl.hpp"

namespace KDL
{
    class ChainIkSolverPos_TL_Adapter : public ChainIkSolverPos
    {
    public:
        ChainIkSolverPos_TL_Adapter(const Chain& chain, const JntArray& q_min, const JntArray& q_max,
                                    double maxtime = 0.005, double eps = 1e-3, bool random_restart = false,
                                    bool try_jl_wrap = false);

        int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out) override;

        ~ChainIkSolverPos_TL_Adapter() override {};

        void updateInternalDataStructures() override {};

    private:
        ChainIkSolverPos_TL solver_;
    };
}

#endif //ROS2_WS_CHAINIKSOLVERPOS_TL_ADAPTER_H
