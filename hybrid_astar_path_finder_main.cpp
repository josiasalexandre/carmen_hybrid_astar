#include <iostream>

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/rddf_interface.h>

#include "Entities/State2D.hpp"
#include "Interface/hybrid_astar_interface.h"

#include "PathFinding/HybridAstarPathFinder.hpp"

// ugly global pointers
astar::HybridAstarPathFinder *g_hybrid_astar;

void save_to_file(astar::StateArrayPtr states) {

    std::vector<astar::State2D> &msg(states->states);

    if (0 < msg.size()) {

        // open the external file
        FILE* F = fopen("path.m", "w");
        if (!F) {
            std::cout << "could not open 'path_file.m' for writing!\n";
            return;
        }

        for (unsigned int i = 0; i < msg.size(); ++i) {

            // fprintf(F, "%lf %lf\n", msg[i].position.x, msg[i].position.y);
            fprintf(F, "%lf ", msg[i].phi);
        }

        fclose(F);
    }

    std::cout << "\nDone!\n";
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
publish_hybrid_astar_path()
{
    // get the the resulting path
    astar::StateArrayPtr path = g_hybrid_astar->get_path();

    std::vector<astar::State2D> &states(path->states);

    unsigned int s_size = states.size();

    // save the current command list
    save_to_file(path);

    IPC_RETURN_TYPE err;
    static int first_time = 1;
    if (first_time)
    {
        err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME,
                IPC_VARIABLE_LENGTH,
                CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
        carmen_test_ipc_exit(err, "Could not define message",
                CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);
        first_time = 0;
    }

    carmen_robot_ackerman_motion_command_message ackerman_msg;
    ackerman_msg.num_motion_commands = s_size;
    ackerman_msg.motion_command = nullptr;

    ackerman_msg.motion_command = new carmen_ackerman_motion_command_t[s_size];

    // direct access
    carmen_ackerman_motion_command_p motion_commands = ackerman_msg.motion_command;

    // copy the commands
    for (unsigned int i = 0; i < s_size; ++i) {

        motion_commands[i].v = states[i].v;
        motion_commands[i].phi = states[i].phi;
        motion_commands[i].time = states[i].t;
    }

    ackerman_msg.timestamp = carmen_get_time();
    ackerman_msg.host = carmen_get_host();

    err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, &ackerman_msg);
    carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);

    // remove the temp data
    delete [] ackerman_msg.motion_command;

    // remove the current resulting path
    delete path;

}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    // get the start pose
    g_hybrid_astar->set_initial_state(
            msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, carmen_get_time() - msg->timestamp);

    // se replan() method returns true if there's a path to the goal
    if (g_hybrid_astar->activated && g_hybrid_astar->replan())
        publish_hybrid_astar_path();
}

static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
    // the current message is incomplete
    g_hybrid_astar->set_initial_state(
            msg->truepose.x, msg->truepose.y, msg->truepose.theta, carmen_get_time() - msg->timestamp);

    // se replan() method returns true if there's a path to the goal
    if (g_hybrid_astar->activated && g_hybrid_astar->replan())
        publish_hybrid_astar_path();

}

static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
    g_hybrid_astar->set_goal_state(msg->x, msg->y, carmen_normalize_theta(msg->theta), 0);
}

static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
    g_hybrid_astar->set_goal_list(msg);
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
    static bool first = false;

    if (first) {

        g_hybrid_astar->update_map(msg);

        first = false;
    }
}

static void
grid_mapping_map_handler(carmen_grid_mapping_message *online_map_message)
{
    g_hybrid_astar->update_map(online_map_message);
}

static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
    (void) message;
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
    // TODO publish a stop message

    // disable the current path finder
    g_hybrid_astar->activated = false;

}

static void
signal_handler(int sig)
{
    std::cout << "\nSignal " << sig << "received, exiting program ...\n";

    // remove the global pointer
    delete(g_hybrid_astar);

    exit(1);

}

void
rddf_message_handler(carmen_rddf_road_profile_message *message)
{
    g_hybrid_astar->update_rddf(message);
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

    /*
    carmen_map_server_subscribe_compact_cost_map(
            NULL,
            (carmen_handler_t) map_server_compact_cost_map_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    /*
    carmen_map_server_subscribe_compact_lane_map(
            NULL, (carmen_handler_t) map_server_compact_lane_map_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

    */

    carmen_subscribe_message(
            (char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
            (char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
            NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
            (carmen_handler_t)navigator_ackerman_set_goal_message_handler,
            CARMEN_SUBSCRIBE_LATEST);

}

void
register_handlers()
{
    signal(SIGINT, signal_handler);


    carmen_grid_mapping_subscribe_message(NULL, (carmen_handler_t) grid_mapping_map_handler, CARMEN_SUBSCRIBE_LATEST);

    if (g_hybrid_astar->simulation_mode)
        carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);
    else
        carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

    /*
    carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

    */
    carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);

    register_handlers_specific();
}

int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    // build the main planner object
    g_hybrid_astar = new astar::HybridAstarPathFinder(argc, argv);

    // define the hybrid astar message
    carmen_hybrid_astar_define_path_message();

    // register all the current handlers
    register_handlers();

    // the main IPC loop
    carmen_ipc_dispatch();

    // delete the global pointer
    delete(g_hybrid_astar);

    return 0;
}
