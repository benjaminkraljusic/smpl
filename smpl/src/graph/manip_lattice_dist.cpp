#include <smpl/graph/manip_lattice_dist.h>

// standard includes
#include <iomanip>
#include <sstream>

// system includes
#include <sbpl/planners/planner.h>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/spatial.h>
#include "../profiling.h"

namespace smpl {

// BENO 02/25
ManipLatticeDist::~ManipLatticeDist() {
}

bool ManipLatticeDist::init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions) {
    
    // actions not used, kept for compatibility
    ManipLattice::init(robot, checker, resolutions, actions);
    
    num_DOFs = RobotPlanningSpace::robot()->jointVariableCount(); 
    m_num_spines = 2*num_DOFs;
    m_spheres_radii = collisionChecker()->getCollisionSpheresRadii();
    m_states = getStates();    
}

void ManipLatticeDist::GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) 
{

    if(m_goal_vec == nullptr) {
        auto goal_tmp = goal().angles;
        m_goal_vec = std::make_shared<Eigen::VectorXd>(goal_tmp.size());
        *m_goal_vec = Eigen::Map<Eigen::VectorXd>(goal_tmp.data(), goal_tmp.size());
    }

    // goal state should be absorbing
    if (state_id == getGoalStateID())
        return;

    ManipLatticeState* parent_entry = (*m_states)[state_id];

    // Eigen vector representing state to be expanded
    std::shared_ptr<Eigen::VectorXd> q = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Map(parent_entry->state.data(), parent_entry->state.size()));

    // Collision distance
    this->d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);

    int goal_succ_count = 0;

    // Prepare states towards which bur spines are extended
    std::vector<Eigen::VectorXd> q_es;

    for(size_t i = 0; i < parent_entry->state.size(); i++) {
        Eigen::VectorXd stateTmp = *q; 
        stateTmp[i] = 2*M_PI; // state "infinitely far" along the i-th axis in the C-space
        q_es.push_back(stateTmp);
        stateTmp[i] = -2*M_PI;
        q_es.push_back(stateTmp);
    }
        
    // Generate bur in m_states[state_id]
    std::shared_ptr<Eigen::VectorXd> q_new = std::make_shared<Eigen::VectorXd>(parent_entry->state.size());

    RobotCoord succ_coord(num_DOFs, 0);

    for(size_t i = 0; i < m_num_spines; i++) {
        
        // If a minimum distance is too small -> use collision checking approach 
        if(d_c < 0.01) {
            if(!addSuccWithCollisionCheck(q, q_es.at(i), q_new))
                continue;
        } else 
            // Get q_new by extending the spine towards q_e = q_es[i]
            extendSpine(parent_entry, q_es.at(i), q_new, succs, costs, false);
    }

    // Try expanding towards the goal state every time - snap it if you are close
    extendSpine(parent_entry, *m_goal_vec, q_new, succs, costs, true);
}

void ManipLatticeDist::extendSpine(
        ManipLatticeState* parent_entry, 
        Eigen::VectorXd q_e, 
        std::shared_ptr<Eigen::VectorXd> q_new, 
        std::vector<int>* succs,
        std::vector<int>* costs,
        bool is_q_e_goal) 
{
    double rho(0), rho_k(0); 				        // The path length in W-space for (complete) robot
	double step(0);
    size_t counter(0);
       
    // Eigen vector representing the state to be expanded
    std::shared_ptr<Eigen::VectorXd> q = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Map(parent_entry->state.data(), parent_entry->state.size()));
    int goal_succ_count = 0;
    RobotCoord succ_coord(num_DOFs, 0);
 
    Eigen::VectorXd q_temp = *q;

	Eigen::VectorXd delta_q;
    std::shared_ptr<Eigen::VectorXd> R = std::make_shared<Eigen::VectorXd>(num_DOFs);
    
    RobotState q_newRS(q->size());
    RobotState q_tempRS(q->size());
    Eigen::Map<Eigen::VectorXd>(q_tempRS.data(), q_tempRS.size()) = q_temp;

    std::shared_ptr<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> skeleton_pair = std::make_shared<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>(std::make_pair(Eigen::MatrixXd(3, num_DOFs + 1), Eigen::MatrixXd(3, num_DOFs))); 
    collisionChecker()->computeSkeleton(0, q_tempRS, skeleton_pair);
    std::shared_ptr<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> skeleton_pair_new = std::make_shared<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>(std::make_pair(Eigen::MatrixXd(3, num_DOFs + 1), Eigen::MatrixXd(3, num_DOFs)));
    collisionChecker()->computeSkeleton(0, q_tempRS, skeleton_pair_new);

    while (true) {   
        computeEnclosingRadii(std::make_shared<Eigen::MatrixXd>(skeleton_pair_new->first), R);

		delta_q = (q_e - q_temp).cwiseAbs();

		step = (d_c - rho) / R->dot(delta_q);	// 'd_c - rho' is the remaining path length in W-space

		if (step > 1) {
			*q_new = q_e;
            break;
        }
		else
			*q_new = q_temp + step * (q_e - q_temp);     
		
        if (++counter == m_num_iter_spine)
			break;

        Eigen::Map<Eigen::VectorXd>(q_newRS.data(), q_newRS.size()) = *q_new;

	    collisionChecker()->computeSkeleton(0, q_newRS, skeleton_pair_new);
	
		for (size_t k = 0; k < skeleton_pair->second.cols(); k++) {
			rho_k = (skeleton_pair->second.col(k) - skeleton_pair_new->second.col(k)).norm();
			rho = std::max(rho, rho_k);
		}

		q_temp = *q_new;

	} // New state obtained

    // If a spine is too short - apply regular collision checking approach
    if((*q - *q_new).norm() < m_prim_len) {
        // If successor is in collision don't add it
        if(!addSuccWithCollisionCheck(q, q_e, q_new))
            return;
    }

    // compute destination coords
    Eigen::Map<Eigen::VectorXd>(q_newRS.data(), q_newRS.size()) = *q_new;
    stateToCoord(q_newRS, succ_coord);

    // check if hash entry already exists, if not then create one
    int succ_state_id = getOrCreateState(succ_coord, q_newRS);
    ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

    // check if this state meets the goal criteria
    auto is_goal_succ = isGoal(q_newRS);

    if (is_goal_succ) 
        ++goal_succ_count; // update goal state

    // put successor on successor list with the proper cost
    if (is_goal_succ) {
        succs->push_back(getGoalStateID());
    } else {
        if(is_q_e_goal) // Don't add states obtained by snapping unless the state is goal
            return;
        else 
            succs->push_back(succ_state_id);
    }
    costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));

}

// Compute enclosing radii and store it in R. Currently works only for planar robots.
void ManipLatticeDist::computeEnclosingRadii(std::shared_ptr<Eigen::MatrixXd> skeleton, std::shared_ptr<Eigen::VectorXd> R) {

	for (size_t i = 0; i < num_DOFs; i++) { 			// Starting point on skeleton
        std::vector<double> endpoint_row;

		for (size_t j = i+1; j <= num_DOFs; j++)	// Final point on skeleton
			endpoint_row.push_back(((*skeleton).col(j) - (*skeleton).col(i)).norm() + m_spheres_radii[j-1]); /*+ m_spheres_radii[i]*/

        (*R)[i] = *std::max_element(endpoint_row.begin(), endpoint_row.end());
	}

}

bool ManipLatticeDist::addSuccWithCollisionCheck(
    std::shared_ptr<Eigen::VectorXd> q,
    Eigen::VectorXd q_e,  
    std::shared_ptr<Eigen::VectorXd> q_new) 
{
    for(int i = 0; i < q->size(); i++)
        (*q_new)[i] = (*q)[i] + (q_e - *q)[i]/(q_e - *q).norm()*m_prim_len;

    // Corresponding RobotStates for ManipLattice methods
    RobotState q_RS(q->size());
    Eigen::Map<Eigen::VectorXd>(q_RS.data(), q_RS.size()) = *q;
    RobotState q_newRS(q->size());
    Eigen::Map<Eigen::VectorXd>(q_newRS.data(), q_newRS.size()) = *q_new;
    
    // If the new state is not in collision -> ok
    if(collisionChecker()->isStateToStateValid(q_RS, q_newRS))
        return true;
        
    // New state in collision
    return false;
}

}