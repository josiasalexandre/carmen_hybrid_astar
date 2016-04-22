#include "HybridAstarMotionPlanner.hpp"

#include "../../localize_ackerman/localize_ackerman_messages.h"
#include "../../global/global.h"

// basic constructor
astar::HybridAstarMotionPlanner::HybridAstarMotionPlanner(int argc, char **argv) : validPath(false) {

    // read all parameters
    readParameters(argc, argv);

}

// PRIVATE METHODS

// publish the current path
void astar::HybridAstarMotionPlanner::publishPath(std::list<Pose2D>& path) {

    // build a new message
    /* TODO
    * what is the output message formaŧ?
    * We need to find what is the appropriate kind of message
    */

}

// publish the current status
void astar::HybridAstarMotionPlanner::publishStatus() {

    // TODO
}

// read all parameters
void astar::HybridAstarMotionPlanner::readParameters(int argc, char **argv) {

    // TODO

}

// PUBLIC METHODS

// registering the currrent pointer
bool astar::HybridAstarMotionPlanner::init() {

    if (nullptr == current) {

        current = this;

        return true;
    }

    return false;

}

// the main handler
void astar::HybridAstarMotionPlanner::globalPoseMessageHandler(carmen_localize_ackerman_globalpos_message *msg) {

    // the main subcribed method

    // convert to internal pose representation
    astar::Pose2D start(msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, msg->phi, msg->v);

    // find a path to the goal
    std::list<Pose2D> path = pathFinder.findPath(start, goal, map);

    // is it a valid path?
    if (0 < path.size()) {

        // smooth and interpolate the current path by our conjugate gradient method
        std::list<Pose2D> smoothPath = pathSmoother.smooth(path, map);

        // publish the resulting smooth path
        publishPath(smoothPath);

    }

    //
    return;

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

    if (init()) {

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

}
