#include "HybridAstarMotionPlanner.hpp"

// basic constructor
astar::HybridAstarMotionPlanner::HybridAstarMotionPlanner(int argc, char **argv) : robot() {

    // the current pointer
    current = this;

    // read all parameters
    readParameters(argc, argv);

}

// PRIVATE METHODS

// publish the current path
void astar::HybridAstarMotionPlanner::publishPath() {}

// publish the current status
void astar::HybridAstarMotionPlanner::publishStatus() {}

// read all parameters
void astar::HybridAstarMotionPlanner::readParameters(int argc, char **argv) {

    // TODO

}

// PUBLIC METHODS

// the main handler
void astar::HybridAstarMotionPlanner::globalPoseMessageHandler(carmen_localize_ackerman_globalpos_message *msg) {

}

// for simulation purpose
void astar::HybridAstarMotionPlanner::truePoseMessageHandler(carmen_simulator_ackerman_truepos_message *msg) {}

// receive the odometry
void astar::HybridAstarMotionPlanner::odometryMessageHandler(carmen_base_ackerman_odometry_message *msg) {}

// set the new goal
void astar::HybridAstarMotionPlanner::setGoalMessageHandler(carmen_navigator_ackerman_set_goal_message *msg) {}

//
void astar::HybridAstarMotionPlanner::goalListMessageHandler(carmen_behavior_selector_goal_list_message *msg) {}

//
void astar::HybridAstarMotionPlanner::stateMessageHandler(carmen_behavior_selector_state_message *msg) {}

//
void astar::HybridAstarMotionPlanner::compactCostMapMessageHandler(carmen_map_server_compact_cost_map_message *msg) {}

//
void astar::HybridAstarMotionPlanner::compactLaneMapMessageHandler(carmen_map_server_compact_lane_map_message *msg) {}

// register all handlers
void astar::HybridAstarMotionPlanner::registerAllHandlers() {

    // subscribe the odometry message
    //
    carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::odometryMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    // subscribe the behavior selector current state message
    //
    carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::stateMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    // subscribe the compact cost map message handler
    carmen_map_server_subscribe_compact_cost_map(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::compactCostMapMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    // the compact lane map handler
    carmen_map_server_subscribe_compact_lane_map(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::compactLaneMapMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    // the goal list handler
    carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::goalListMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    // the set goal handler
    carmen_subscribe_message((char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME, (char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
            NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
            (carmen_handler_t) astar::HybridAstarMotionPlanner::setGoalMessageHandler,
            CARMEN_SUBSCRIBE_LATEST);

    // simulator?
    if (!simulatorMode) {

        // subscribe the main handler
        carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::globalPoseMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    } else {

        // subcribe the simulator handler
        carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) astar::HybridAstarMotionPlanner::truePoseMessageHandler, CARMEN_SUBSCRIBE_LATEST);

    }


}
