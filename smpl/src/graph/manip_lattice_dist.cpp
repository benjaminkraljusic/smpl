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

#define DIST 1
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
    m_num_spines = 2*num_DOFs + 1;
    m_spheres_radii = collisionChecker()->getCollisionSpheresRadii();
    m_states = getStates();
    // std::cout << "RADIJUSI SFERA: " << m_spheres_radii << std::endl;
}

#ifndef DIST
void ManipLatticeDist::GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) 
{
   
    // goal state should be absorbing
    if (state_id == getGoalStateID()) {
        return;
    }

    auto m_states = getStates();
    ManipLatticeState* parent_entry = (*m_states)[state_id];

    //Getting collision distance
    double d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);
    
    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!getActions()->apply(parent_entry->state, actions)) {
        SMPL_WARN("Failed to get actions");
        return;
    }

    // SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(getGoalStateID());
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));
    }

}

#else
void ManipLatticeDist::GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) 
{
   
    // goal state should be absorbing
    if (state_id == getGoalStateID()) {
        return;
    }

    ManipLatticeState* parent_entry = (*m_states)[state_id];

    this->d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);

    int goal_succ_count = 0;

    // Generate states towards which bur spines are extended
    std::vector<RobotState> q_es;

    for(size_t i = 0; i < parent_entry->state.size(); i++) {
        RobotState stateTmp = parent_entry->state;
        stateTmp.at(i) = 2*M_PI; // state "infinitely far" along the i-th axis in the C-space
        q_es.push_back(stateTmp);
        stateTmp.at(i) = -2*M_PI;
        q_es.push_back(stateTmp);
    }

    // Try expanding towards the goal state every time - snap it if you are close
    q_es.push_back(goal().angles);
        
    // Generate bur in m_states[state_id]
    RobotState q_new(parent_entry->state.size());
    RobotCoord succ_coord(num_DOFs, 0);

    for(size_t i = 0; i < m_num_spines; i++) {
        
        // If a minimum distance is too small -> use collision checking approach 
        if(d_c < 0.01) {
           // std::cout << "DESILO SE " << std::endl;
            if(!addSuccWithCollisionCheck(parent_entry->state, q_es.at(i), &q_new))
                continue;
        } else {
            // Get q_new by extending the spine towards q_e = q_es[i]
            q_new = extendSpine(parent_entry->state, q_es.at(i));
            
            //std::cout << "DUZINA SPAJNA: " << (Eigen::VectorXd::Map(parent_entry->state.data(), parent_entry->state.size()) - Eigen::VectorXd::Map(q_new.data(), q_new.size())).norm() << std::endl;

            // If a spine is too short - apply regular collision checking approach
            if((Eigen::VectorXd::Map(parent_entry->state.data(), parent_entry->state.size()) - Eigen::VectorXd::Map(q_new.data(), q_new.size())).norm() < m_prim_len) {
               // std::cout << "KRATAK SPAJN DO BOLA" << std::endl;
                // If successor is in collision don't add it
                if(!addSuccWithCollisionCheck(parent_entry->state, q_es.at(i), &q_new))
                    continue;
                
            // std::cout << "COLLISION CHECK STATE: " << q_new << std::endl;
            }
        }

        //std::cout << "STANJE: " << q_new << std::endl;
        // compute destination coords
        stateToCoord(q_new, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, q_new);
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(q_new);
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(getGoalStateID());
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));
    }
}

#endif
RobotState ManipLatticeDist::extendSpine(RobotState q, RobotState q_e) {
    // Provjeri rho_profile zbog broja linkova
    // std::vector<double> rho_profile(num_DOFs + 1);	// TODO: The path length in W-space for each robot's link. -2 for base link and tool. Will be fixed
	double rho(0), rho_k(0); 				        // The path length in W-space for (complete) robot
	double step(0);
    size_t counter(0);
        
	// Eigen versions
    Eigen::VectorXd q_Vec = Eigen::VectorXd::Map(q.data(), q.size());
    Eigen::VectorXd q_tempVec = q_Vec;
	Eigen::VectorXd q_eVec = Eigen::VectorXd::Map(q_e.data(), q_e.size());
    Eigen::VectorXd q_newVec;

	// Eigen::MatrixXd skeleton;
	// Eigen::MatrixXd skeleton_new;
	Eigen::VectorXd delta_q;
    Eigen::VectorXd R;
    
    RobotState q_new(q.size());
    RobotState q_temp { q };

    //std::cout << "q.size(): " << q.size() << std::endl;
    auto skeleton_pair = collisionChecker()->computeSkeleton(0, q_temp);
    auto skeleton_pair_new = skeleton_pair;

    while (true) {   
    //    std::cout << "q_temp: " << q_temp << std::endl;
        
      //  std::cout << "q_tempVec: " << std::endl << q_tempVec << std::endl;
     //   std::cout << "q_eVec: " << std::endl << q_eVec << std::endl; 
      //std::cout << "SKELETON: " << std::endl << skeleton << std::endl;

        R = computeEnclosingRadii(skeleton_pair_new.first);

        // std::cout << "Enclosing radii: " << std::endl << R << std::endl;

		delta_q = (q_eVec - q_tempVec).cwiseAbs();

        // std::cout << "delta_q: " << std::endl << delta_q << std::endl;

		step = (d_c - rho) / R.dot(delta_q);	// 'd_c - rho' is the remaining path length in W-space
        // std::cout << "PREOSTALA DISTANCA: " << d_c - rho << std::endl;
        // std::cout << "step: " << step << std::endl;

		if (step > 1) {
			Eigen::Map<Eigen::VectorXd>(q_new.data(), q_new.size()) = q_eVec;
            break;
        }
		else
			q_newVec = q_tempVec + step * (q_eVec - q_tempVec);     
        
        // q_newVec = wrapState(q_newVec);

        // std::cout << "q_newVec: " << std::endl << q_newVec << std::endl;

        Eigen::Map<Eigen::VectorXd>(q_new.data(), q_new.size()) = q_newVec;
		
        if (++counter == m_num_iter_spine)
			break;

	    skeleton_pair_new = collisionChecker()->computeSkeleton(0, q_new);

       // std::cout << "SKELETON NEW: " << std::endl << skeleton_new << std::endl;
	
		for (size_t k = 0; k < skeleton_pair.second.cols(); k++) {
			rho_k = (skeleton_pair.second.col(k) - skeleton_pair_new.second.col(k)).norm();
			rho = std::max(rho, rho_k);
		}

        // std::cout << "rho: " << rho << std::endl; 
		q_tempVec = q_newVec;
        q_temp = q_new;
	}

    return q_new;
}

Eigen::VectorXd ManipLatticeDist::computeEnclosingRadii(Eigen::MatrixXd skeleton) {
    Eigen::VectorXd radii(num_DOFs);
	for (size_t i = 0; i < num_DOFs; i++) { 			// Starting point on skeleton
        std::vector<double> endpoint_row;

		for (size_t j = i+1; j <= num_DOFs; j++)	// Final point on skeleton
			endpoint_row.push_back((skeleton.col(j) - skeleton.col(i)).norm() + m_spheres_radii[j-1]); /*+ m_spheres_radii[i]*/

        // std::cout << "ENDPOINT ROW " << i << std::endl << endpoint_row << std::endl;       
        radii(i) = *std::max_element(endpoint_row.begin(), endpoint_row.end());
	}

    // std::cout << "KAD TEK IZRACUNAM RADII: " << std::endl << radii << std::endl;
    return radii;
}

Eigen::VectorXd ManipLatticeDist::wrapState(Eigen::VectorXd state) {
    
    for (int i = 0; i < state.size(); ++i) {
        state(i) = fmod(state(i), M_PI);
        if (state(i) < -M_PI) {
            state(i) += M_PI;
        } else if (state(i) >= M_PI) {
            state(i) -= M_PI;
        }
    }
    
    return state;
}

bool ManipLatticeDist::addSuccWithCollisionCheck(
    RobotState q,
    RobotState q_e,  
    RobotState* q_new) 
{
    Eigen::VectorXd q_Vec = Eigen::VectorXd::Map(q.data(), q.size());
    Eigen::VectorXd q_eVec = Eigen::VectorXd::Map(q_e.data(), q_e.size());
    Eigen::VectorXd q_newVec(q_Vec.size());

    for(int i = 0; i < q_Vec.size(); i++)
        q_newVec[i] = q_Vec[i] + (q_eVec - q_Vec)[i]/(q_eVec - q_Vec).norm()*m_prim_len;


    Eigen::Map<Eigen::VectorXd>(q_new->data(), q_new->size()) = q_newVec;

    // If the new state is not in collision -> ok
    if(collisionChecker()->isStateToStateValid(q, *q_new))
        return true;
        
    // New state in collision
    return false;
}

}