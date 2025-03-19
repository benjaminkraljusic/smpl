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
    std::vector<ManipLatticeState*>* m_states;

    std::shared_ptr<Eigen::VectorXd> m_goal_vec{ nullptr };

    size_t m_num_spines; // Number of bur spines 
    
    size_t num_DOFs;
    
    double d_c = 0; // Latest workspace collision distance

    void extendSpine(ManipLatticeState* parent_entry, Eigen::VectorXd q_e, std::shared_ptr<Eigen::VectorXd> q_new, std::vector<int>* succs, std::vector<int>* costs, bool is_q_e_goal);
    void computeEnclosingRadii(std::shared_ptr<Eigen::MatrixXd> skeleton, std::shared_ptr<Eigen::VectorXd> R);
    bool addSuccWithCollisionCheck(std::shared_ptr<Eigen::VectorXd> q, Eigen::VectorXd q_e, std::shared_ptr<Eigen::VectorXd> q_new); 

    std::vector<double> m_spheres_radii;

    size_t m_num_iter_spine = 5;

    double m_prim_len = 8.0/180*M_PI; // TODO: Load this externally from .mprim file
};

}

#endif