/// \author Benjamin Kraljusic

#ifndef SMPL_MANIP_LATTICE_DIST_H
#define SMPL_MANIP_LATTICE_DIST_H

#include <smpl/graph/manip_lattice.h>

namespace smpl {

class ManipLatticeDist : public ManipLattice {

public : 

    void GetSuccs(
            int state_id,
            std::vector<int>* succs,
            std::vector<int>* costs);

private :
    std::mutex m_lock;
};

}

#endif