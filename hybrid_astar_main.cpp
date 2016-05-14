#include <iostream>
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
    if (!GlobalState::following_path)
        return;

    carmen_ackerman_motion_command_t* commands =
            (carmen_ackerman_motion_command_t*) (malloc(path.size() * sizeof(carmen_ackerman_motion_command_t)));
    int i = 0;
    for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
    {
        commands[i].v = it->v;
        commands[i].phi = it->phi;
        commands[i].time = it->time;

        i++;
    }

    int num_commands = path.size();
    if (GlobalState::use_obstacle_avoider)
        carmen_robot_ackerman_publish_motion_command(commands, num_commands);
    else
        carmen_base_ackerman_publish_motion_command(commands, num_commands);

    free(commands);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    if (g_hybrid_astar->find_path(msg))
        publish_hybrid_astar_motion_commands();
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
    if (g_hybrid_astar->find_path(msg))
        publish_hybrid_astar_motion_commands();
}


static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
    if (msg)
        g_hybrid_astar->set_goal_pose(msg->x, msg->y, msg->theta);
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
    if (msg)
        g_hybrid_astar->set_odometry(msg->v, msg->phi);
}


static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
    if (msg && 0 < msg->size && msg->goal_list)
        g_hybrid_astar->set_goal_list(msg->goal_list->x, msg->goal_list->y, msg->goal_list->theta);
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
    if (msg && (CARMEN_BEHAVIOR_SELECTOR_A_STAR == msg->algorithm) && (BEHAVIOR_SELECTOR_PARKING == msg->state))
        g_hybrid_astar->set_parking_mode();
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *msg)
{
    if (msg)
    {
        g_hybrid_astar->update_map(msg);
    }
}


static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
    return;
}


static void
navigator_ackerman_go_message_handler()
{
    g_hybrid_astar->go();
}


static void
navigator_ackerman_stop_message_handler()
{
    g_hybrid_astar->stop();
}


static int
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
            (carmen_handler_t)navigator_ackerman_go_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message(
            (char *)CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
            (char *)CARMEN_DEFAULT_MESSAGE_FMT,
            NULL, sizeof(carmen_navigator_ackerman_stop_message),
            (carmen_handler_t)navigator_ackerman_stop_message_handler,
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
//  printf("RDDF NUM POSES: %d \n", message->number_of_poses);
//
//  for (int i = 0; i < message->number_of_poses; i++)
//  {
//      printf("RDDF %d: x  = %lf, y = %lf , theta = %lf\n", i, message->poses[i].x, message->poses[i].y, message->poses[i].theta);
//      //getchar();
//  }
}


void
register_handlers()
{
    signal(SIGINT, signal_handler);

    if (!g_hybrid_astar->simulation_mode)
        carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
    else
        carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_road_profile_message(&goal_list_message, (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);

    register_handlers_specific();
}


void
read_parameters_specific(int argc, char **argv)
{
    g_hybrid_astar->get_parameters();
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
