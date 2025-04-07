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
    size_t m_num_DOFs;
    std::vector<double> m_collision_spheres_radii; // Radii of the leaf spheres for each link. Needed for safe enclosing radii calculation

    size_t m_num_iter_spine = 1; // spines expanded in axis directions, i.e. the bubble is sampled in the first expansion -> only one spine iteration is needed
   
    double m_delta;
    
    double m_d_c = 0; // Latest workspace collision distance
    
    void extendSpine(const std::shared_ptr<const Eigen::VectorXd> q, const Eigen::VectorXd q_e, std::shared_ptr<Eigen::VectorXd> q_new);
    void computeEnclosingRadii(std::shared_ptr<const Eigen::MatrixXd> skeleton, std::shared_ptr<Eigen::VectorXd> R);
    bool generateSuccWithCollisionCheck(std::shared_ptr<const Eigen::VectorXd> q, const Eigen::VectorXd q_e, std::shared_ptr<Eigen::VectorXd> q_new); 

    // bool m_add_intermediate_states = false;
    // bool m_add_last_step = false;
};

}

#endif