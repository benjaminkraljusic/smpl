/// \author Benjamin Kraljusic

#ifndef SMPL_MANIP_LATTICE_DIST_H
#define SMPL_MANIP_LATTICE_DIST_H

#include <smpl/graph/manip_lattice.h>

namespace smpl {

class ManipLatticeDist : public ManipLattice {

public : 
    ~ManipLatticeDist();

    void GetSuccs(
            int state_id,
            std::vector<int>* succs,
            std::vector<int>* costs);

    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions);

private :
    std::mutex m_lock;
    // std::vector<RobotState> q_e; // States to expand bur spines towards

    size_t m_num_spines; // Number of bur spines 

    double d_c; // latest minimum workspace distance

   // void extendSpine(const RobotState &q, const RobotState &q_e, RobotState &q_new);
};

}

#endif