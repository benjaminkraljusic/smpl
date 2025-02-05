// standard includes
#include <ros/console_backend.h>
#include <string>

// system includes
#include <ros/ros.h>

#include "planner.h"

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle const nh;
    ros::NodeHandle const ph("~");

    bool verbose;
    if (!ph.getParam("verbose", verbose)) {
        ROS_ERROR("Failed to read 'verbose' from the param server");
        return 1;
    }

    bool visualize;
    if (!ph.getParam("visualize", visualize)) {
        ROS_ERROR("Failed to read 'visualize' from the param server");
        return 1;
    }

    bool reverse;
    if (!ph.getParam("reverse", reverse)) {
        ROS_ERROR("Failed to read 'reverse' from the param server");
        return 1;
    }

    bool check;
    if (!ph.getParam("check", check)) {
        ROS_ERROR("Failed to read 'check' from the param server");
        return 1;
    }

    // Get planning problems' info
    std::string problems_dir;
    if (!ph.getParam("planning_problems_directory", problems_dir)) {
        ROS_ERROR("Failed to read 'planning_problems_directory' from the param server");
        return 1;
    }

    int problem_index;
    if (!ph.getParam("planning_problem_index", problem_index)) {
        ROS_ERROR("Failed to read 'planning_problem_index' from the param server");
        return 1;
    }

    Planner planner(nh, ph, verbose, visualize);

    if (!planner.initForProblemsDir(problems_dir, reverse)) {
        ROS_ERROR(
          "Failed to initialize planner for the problems in the specified directory: %s.",
          problems_dir.c_str()
        );
        return 1;
    }

    ROS_INFO("Planning for problem no. %d...", problem_index);
    if (!planner.planForProblemIdx(problem_index, check)) {
        ROS_INFO("FAILED");
    } else {
        ROS_INFO("SUCCEEDED");
    }

    return 0;
}
