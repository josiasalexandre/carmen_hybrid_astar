#include <iostream>

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>

#include "PathFinding/HybridAstarMotionPlanner.hpp"

// ugly global pointer
astar::HybridAstarMotionPlanner *g_hybrid_astar;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
publish_hybrid_astar_motion_commands()
{
    std::vector<astar::State2D> path = g_hybrid_astar->get_path();
    unsigned int p_size = path.size();

    carmen_ackerman_motion_command_p commands = new carmen_ackerman_motion_command_t[p_size];

    if (commands)
    {
        for (unsigned int i = 0;  i < p_size; i++)
        {
            commands[i].v = path[i].v;
            commands[i].phi = path[i].wheel_angle;
            commands[i].time = path[i].time;
        }

        if (g_hybrid_astar->use_obstacle_avoider)
            carmen_robot_ackerman_publish_motion_command(commands, p_size);
        else
            carmen_base_ackerman_publish_motion_command(commands, p_size);

        delete(commands);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    if (g_hybrid_astar->activated)
    {
        astar::State2D start(msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, msg->phi, msg->v);

        if (g_hybrid_astar->replan(start))
            publish_hybrid_astar_motion_commands();
    }
}

static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
    if (g_hybrid_astar->activated)
    {
        // the current message is incomplete
        astar::State2D start =
                g_hybrid_astar->estimate_initial_pose(msg->truepose.x, msg->truepose.y, msg->truepose.theta, carmen_get_time() - msg->timestamp);

        if (g_hybrid_astar->replan(start))
            publish_hybrid_astar_motion_commands();
    }
}

static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
    g_hybrid_astar->set_goal_pose(msg->x, msg->y, carmen_normalize_theta(msg->theta), 0);
}

static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
    if (msg && 0 < msg->size && msg->goal_list)
        g_hybrid_astar->set_goal_pose(msg->goal_list->x, msg->goal_list->y, msg->goal_list->theta, msg->goal_list->v);
}

static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
    g_hybrid_astar->set_odometry(msg->v, msg->phi);
}

static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
    if (msg && (CARMEN_BEHAVIOR_SELECTOR_A_STAR == msg->algorithm) && (BEHAVIOR_SELECTOR_PARKING == msg->state))
        g_hybrid_astar->activated = true;
    else
        g_hybrid_astar->activated = false;
}

static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *msg)
{
    g_hybrid_astar->update_map(msg);
}

static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
    return;
}

static void
navigator_astar_go_message_handler()
{
    g_hybrid_astar->activated = true;
}

static void
navigator_astar_stop_message_handler()
{
    g_hybrid_astar->activated = false;
}

int
signal_handler(int sig)
{
    std::cout << std::endl << "Signal " << sig << "received, exiting program ..." << std::endl;

    // remove the global pointer
    delete(g_hybrid_astar);

    exit(1);

}

///////////////////////////////////////////////////////////////////////////////////////////////
void
register_handlers_specific()
{
    carmen_subscribe_message(
            (char *)CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
            (char *)CARMEN_DEFAULT_MESSAGE_FMT,
            NULL, sizeof(carmen_navigator_ackerman_go_message),
            (carmen_handler_t)navigator_astar_go_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message(
            (char *)CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
            (char *)CARMEN_DEFAULT_MESSAGE_FMT,
            NULL, sizeof(carmen_navigator_ackerman_stop_message),
            (carmen_handler_t)navigator_astar_stop_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    carmen_map_server_subscribe_compact_cost_map(
            NULL,
            (carmen_handler_t) map_server_compact_cost_map_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    carmen_map_server_subscribe_compact_lane_map(
            NULL, (carmen_handler_t) map_server_compact_lane_map_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message(
            (char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
            (char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
            NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
            (carmen_handler_t)navigator_ackerman_set_goal_message_handler,
            CARMEN_SUBSCRIBE_LATEST);
}

void
rddf_message_handler(/*carmen_rddf_road_profile_message *message*/)
{
    // TODO
}

void
register_handlers()
{
    signal(SIGINT, signal_handler);

    if (g_hybrid_astar->simulation_mode)
        carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);
    else
        carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

    // carmen_rddf_subscribe_road_profile_message(&goal_list_message, (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);

    register_handlers_specific();
}

int
main(int argc, char **argv)
{


    // build the HybridAstarMotionPlanner
    g_hybrid_astar = new astar::HybridAstarMotionPlanner(argc, argv);

    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    register_handlers();

    carmen_ipc_dispatch();

    // delete the global pointer
    delete(g_hybrid_astar);

    return 0;
}
