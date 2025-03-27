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
#include <smpl/graph/manip_lattice_action_space.h>

#define D_CRIT 0.01
#define EPS 0.0000001

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
    m_states = getStates(); // pointer to the vector of states
    delta = getDeltas()[0]; // All joints should have the same discretization
    PARENTS.clear();
    KIDS.clear();
    outputDbgFile.close();
    outputDbgFile.open("/home/beno/TezaETF/code/dok_ne_skontam_sto/manip_dist.txt");
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

    int goal_succ_count = 0;

    // Eigen vector representing state to be expanded
    std::shared_ptr<Eigen::VectorXd> q = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Map(parent_entry->state.data(), parent_entry->state.size()));
   
    // Collision distance
    d_c = collisionChecker()->distanceToCollision(0, parent_entry->state);
    
    // Prepare states towards which bur spines are extended
    std::vector<Eigen::VectorXd> q_es;

    for(size_t i = 0; i < parent_entry->state.size(); i++) {
        Eigen::VectorXd stateTmp = *q; 
        stateTmp[i] = M_PI; // state "infinitely far" along the i-th axis in the C-space
        q_es.push_back(stateTmp);
        stateTmp[i] = -M_PI;
        q_es.push_back(stateTmp);
    }
    

    // Generate bur in m_states[state_id]
    std::shared_ptr<Eigen::VectorXd> q_new = std::make_shared<Eigen::VectorXd>(parent_entry->state.size());

    RobotCoord succ_coord(num_DOFs, 0);

    std::vector<RobotState> successors_RS; // sucessor RobotStates
    RobotState q_tmpRS(q->size());
std::vector<RobotState> kidsTmp;
    // Generating bur
    for(size_t i = 0; i < m_num_spines; i++) {
        if(d_c < D_CRIT) { // If the minimum distance is too small -> collison check approach
            if(!generateSuccWithCollisionCheck(q, q_es.at(i), q_new)) // If the new state is in collision -> continue
                continue;
            
            Eigen::Map<Eigen::VectorXd>(q_tmpRS.data(), q_tmpRS.size()) = *q_new;
            successors_RS.push_back(q_tmpRS);
        } else {
            // Get q_new by extending the spine towards q_e = q_es[i]
            extendSpine(q, q_es.at(i), q_new);

            // If a spine is too short (shorter than a motion primitive length) - apply collision checking approach
            if((*q - *q_new).norm() < m_prim_len) {
                // If successor is in collision don't add it
                if(!generateSuccWithCollisionCheck(q, q_es.at(i), q_new)) {
                    continue;
                }
            }
            // // Save the spine extension result
            // Eigen::Map<Eigen::VectorXd>(q_tmpRS.data(), q_tmpRS.size()) = *q_new;
            // successors_RS.push_back(q_tmpRS); 

            // Add fixed increments of collision free extensions of the spine that have equal lenght as motion primitives
            int num_ext_steps = std::floor(((*q - *q_new).norm() + EPS) / m_prim_len); // Added EPS because, for some reason, floor(1.0) was sometimes 0

            for(int k = 1; k <= num_ext_steps; k++) {
                // for(int j = 0; j < q->size(); j++) // Add next int number of m_prim_lens
                //     (*q_new)[j] = (*q)[j] + (q_es.at(i) - *q)[j]/(q_es.at(i) - *q).norm()*k*m_prim_len;
                *q_new = *q + (q_es.at(i) - *q)/(q_es.at(i) - *q).norm()*double(k)*m_prim_len;

                Eigen::Map<Eigen::VectorXd>(q_tmpRS.data(), q_tmpRS.size()) = *q_new;
                successors_RS.push_back(q_tmpRS);
            }
            
        }
        
    } // Bur generated

    // Greedy snap
    if(collisionChecker()->isStateToStateValid(parent_entry->state, goal().angles))
        successors_RS.push_back(goal().angles);

    // // Try expanding towards the goal state every time // POSSIBLY CAN BE USED AS SNAP FOR GBurs
    // extendSpine(q, *m_goal_vec, q_new);
    // Eigen::Map<Eigen::VectorXd>(q_tmpRS.data(), q_tmpRS.size()) = *q_new;
    // successors_RS.push_back(q_tmpRS);

    for(int i = 0; i < successors_RS.size(); i++) {
        RobotState S = successors_RS.at(i);

        // // joint limits (Not needed if the q_e states are chosen to follow limits of C-space)
        // if(std::any_of(S.begin(), S.end(), [](double x) {return x > M_PI || x < -M_PI;}))
        //     continue;
    
        stateToCoord(S, succ_coord);
        
        int succ_state_id = getOrCreateState(succ_coord, S);
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(S);
        // if (is_goal_succ)
        //     ++goal_succ_count; // update goal state

        // if(!is_goal_succ && i == successors_RS.size() - 1) // Don't add the state obtained by snap if not a goal POSSIBLY CAN BE USED AS SNAP FOR GBurs
        //     break;

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            ++goal_succ_count;
            succs->push_back(getGoalStateID());
        }
        else 
            succs->push_back(succ_state_id);
kidsTmp.push_back(S);    
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));
    }
if(goal().angles.size() == 2) {
PARENTS.push_back(parent_entry->state);    
KIDS.push_back(kidsTmp);
}
}

void ManipLatticeDist::extendSpine(
        std::shared_ptr<const Eigen::VectorXd> q, 
        const Eigen::VectorXd q_e, 
        std::shared_ptr<Eigen::VectorXd> q_new) 
{
    double rho(0), rho_k(0); 				        // The path length in W-space for (complete) robot
	double step(0);
    size_t counter(0);

    int goal_succ_count = 0;
    RobotCoord succ_coord(num_DOFs, 0);

    RobotState q_newRS(q->size());

    Eigen::VectorXd q_temp = *q;
    Eigen::VectorXd delta_q;
    std::shared_ptr<Eigen::VectorXd> R = std::make_shared<Eigen::VectorXd>(num_DOFs); // Enclosing radii
    
    RobotState q_tempRS(q->size());
    Eigen::Map<Eigen::VectorXd>(q_tempRS.data(), q_tempRS.size()) = q_temp;

    std::shared_ptr<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> skeleton_pair = std::make_shared<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>(std::make_pair(Eigen::MatrixXd(3, num_DOFs + 1), Eigen::MatrixXd(3, num_DOFs))); 
    collisionChecker()->computeSkeleton(0, q_tempRS, skeleton_pair);
    std::shared_ptr<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> skeleton_pair_new = std::make_shared<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>(std::make_pair(Eigen::MatrixXd(3, num_DOFs + 1), Eigen::MatrixXd(3, num_DOFs)));
    collisionChecker()->computeSkeleton(0, q_tempRS, skeleton_pair_new);

    // Regular spine extending algorithm
    while (true) {   
        computeEnclosingRadii(std::make_shared<Eigen::MatrixXd>(skeleton_pair_new->first), R);

        delta_q = (q_e - q_temp).cwiseAbs();

        step = (d_c - rho) / R->dot(delta_q);	// 'd_c - rho' is the remaining path length in W-space

        if (step > 1) { // This should only be the case when q_e is the goal
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
}

// Compute enclosing radii and store it in R. Currently works only for planar robots.
void ManipLatticeDist::computeEnclosingRadii(std::shared_ptr<const Eigen::MatrixXd> skeleton, std::shared_ptr<Eigen::VectorXd> R) {

	for (size_t i = 0; i < num_DOFs; i++) { 			// Starting point on skeleton
        // std::vector<double> endpoint_row;
        double max_element = 0;
		for (size_t j = i+1; j <= num_DOFs; j++)	{// Final point on skeleton
			// endpoint_row.push_back(((*skeleton).col(j) - (*skeleton).col(i)).norm() + m_spheres_radii[j-1]); /*+ m_spheres_radii[i]*/
            auto r = ((*skeleton).col(j) - (*skeleton).col(i)).norm() + m_spheres_radii[j-1]; 
            if(r > max_element)
                max_element = r;
        }
        (*R)[i] = max_element;
        // (*R)[i] = *std::max_element(endpoint_row.begin(), endpoint_row.end());
	}

}

bool ManipLatticeDist::generateSuccWithCollisionCheck(
    std::shared_ptr<const Eigen::VectorXd> q,
    const Eigen::VectorXd q_e,  
    std::shared_ptr<Eigen::VectorXd> q_new) 
{
    *q_new = *q + (q_e - *q)/(q_e - *q).norm()*m_prim_len;

    if (((*q_new).array() > M_PI).any() || ((*q_new).array() < -M_PI).any()) // joint limits check
        return false;

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