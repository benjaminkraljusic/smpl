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
    ManipLatticeState* parent_entry = m_states[state_id];

    //Getting collision distance
    // double d_c = collisionChecker()->collisionDistance(0, parent_entry->state);
    double d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);
    std::cout << "Hello from manip_latice_dist.cpp; d_c = " << d_c << std::endl;
    
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

// BENO 02/25
ManipLatticeDist::~ManipLatticeDist() {
}

bool ManipLatticeDist::init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions) {

    ManipLattice::init(robot, checker, resolutions, actions);

    m_num_spines = 2*RobotPlanningSpace::robot()->jointVariableCount();
}

// void ManipLatticeDist::GetSuccs(
//         int state_id,
//         std::vector<int>* succs,
//         std::vector<int>* costs) 
// {
   
//     // goal state should be absorbing
//     if (state_id == getGoalStateID()) {
//         return;
//     }

//     auto m_states = getStates();
//     ManipLatticeState* parent_entry = m_states[state_id];

//     // Getting minimum distance to an obstacle in the workspace
//     d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);
    
//     int goal_succ_count = 0;

//     // Generate states towards which bur spines are extended
//     std::vector<RobotState> q_es(m_num_spines);

//     for(size_t i = 0; i < parent_entry->state.size(); i++) {
//         RobotState stateTmp = parent_entry->state;
//         stateTmp.at(i) = 2*M_PI; // state "infinitely far" along the i-th axis in the C-space
//         q_es.push_back(stateTmp);
//         stateTmp.at(i) = -2*M_PI;
//         q_es.push_back(stateTmp);
//     }

//     // TODO: every q_e in q_es should be rotated in the space so that bur doesn't coincide with bubble

//     // Generate bur in m_states[state_id]
//     RobotState q_new;
//     RobotCoord succ_coord(robot()->jointVariableCount(), 0);

//     for(size_t i = 0; i < m_num_spines; i++) {

//         // Get q_new by extending the spine towards q_e = q_es[i]
//         extendSpine(parent_entry->state, q_es.at(i), &q_new);

//         // compute destination coords
//         stateToCoord(q_new, succ_coord);

//         // check if hash entry already exists, if not then create one
//         int succ_state_id = getOrCreateState(succ_coord, q_new);
//         ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

//         // check if this state meets the goal criteria
//         auto is_goal_succ = isGoal(q_new);
//         if (is_goal_succ) {
//             // update goal state
//             ++goal_succ_count;
//         }

//         // put successor on successor list with the proper cost
//         if (is_goal_succ) {
//             succs->push_back(m_goal_state_id);
//         } else {
//             succs->push_back(succ_state_id);
//         }
//         costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));
//     }

// }

// void ManipLatticeDist::extendSpine(const RobotState &q, const RobotState &q_e, RobotState *q_new) {
//     // Provjeri rho_profile zbog broja linkova
//     std::vector<double> rho_profile(collisionChecker()->m_rcm->linkCount() - 1);	// The path length in W-space for each robot's link. -1 for base link
// 	double rho(0), rho_k(0), rho_k_prev(0); 				        // The path length in W-space for (complete) robot
// 	double step(0);
//     size_t counter(0);
// 	std::shared_ptr<RobotState> q_temp(q);
// 	std::shared_ptr<RobotState> q_new(nullptr);
// 	std::shared_ptr<Eigen::MatrixXd> skeleton { collisionChecker()->m_rcs->computeSkeleton(q) }; // !!! STAO SI OVDJE !!! Implementacija computeSkeleton
// 	std::shared_ptr<Eigen::MatrixXd> skeleton_new { nullptr };
// 	std::shared_ptr<Eigen::MatrixXd> R { nullptr };
// 	Eigen::VectorXd delta_q {};
// 	// base::State::Status status { base::State::Status::Advanced };
// 	// bool self_collision { false };
	

// }

}