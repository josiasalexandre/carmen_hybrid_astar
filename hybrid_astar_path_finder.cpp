#include <iostream>

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>

#include "PathFinding/HybridAstarPathFinder.hpp"
#include "Interface/HybridAstarInterface.hpp"

// ugly global pointer
astar::HybridAstarPathFinder *g_hybrid_astar;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
publish_hybrid_astar_path(astar::StateArrayPtr path)
{

    static bool first_time = true;

    if (nullptr != path)
    {
        std::vector<astar::State2D> &states(path->states);

        unsigned int p_size = states.size();

        // build a new message
        carmen_hybrid_astar_path_message_t msg;

        // get the direct access
        carmen_ackerman_path_point_p path = msg.path;

        // build the path message
        msg.path = new carmen_ackerman_path_point_t[p_size];
        msg.path_length = p_size;

        // some helpers
        carmen_ackerman_path_point_t &current;
        astar::State2D &state;

        if (msg && 0 < p_size)
        {
            for (unsigned int i = 0;  i < p_size; i++)
            {
                current = path[i];
                state = states[i];

                // copy all the values
                current.x = state.position.x;
                current.y = state.position.y;
                current.theta = state.orientation;
                current.v = state.v;
                current.phi = state.phi;
                current.time = state.t;
            }

            // publish the current data
            carmen_hybrid_astar_publish_path_message(&msg);
        }

        // remove the temp data
        delete(msg.path);
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
        // get the start pose
        g_hybrid_astar->set_initial_state(
                msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, carmen_get_time() - msg->timestamp);

        // get the resulting path
        astar::StateArrayPtr path = g_hybrid_astar->replan();

        if (nullptr != path)
            publish_hybrid_astar_path(path);
    }
}

static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
    if (g_hybrid_astar->activated)
    {
        // the current message is incomplete
        g_hybrid_astar->set_initial_state(
                msg->truepose.x, msg->truepose.y, msg->truepose.theta, carmen_get_time() - msg->timestamp);

        // get the resulting path
        astar::StateArrayPtr path = g_hybrid_astar->replan();

        if (nullptr != path)
            publish_hybrid_astar_path(path);
    }
}

static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
    g_hybrid_astar->set_goal_state(msg->x, msg->y, carmen_normalize_theta(msg->theta), 0);
}

static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
    if (msg && 0 < msg->size && msg->goal_list)
    {
        // goal states
        astar::State2D goal;

        // the goal list
        astar::StateArrayPtr internal_goal_list = new astar::StateArray();
        std::vector<astar::State2D> &goals(internal_goal_list->states);

        unsigned int msg_size = msg->size;
        carmen_ackerman_traj_point_t *goal_list = msg->goal_list;

        unsigned int index = 0;

        // the current robot state
        astar::State2D robot = g_hybrid_astar->get_robot_state();

        bool first_goal_found = false;

        // save the current goal list
        for (unsigned int i = 0; i < msg_size; i++)
        {
            goal.position.x = goal_list[i].x;
            goal.position.y = goal_list[i].y;
            goal.orientation = goal_list[i].theta;
            goal.v = goal_list[i].v;
            goal.phi = goal_list[i].phi;

            if (0 > goal.v)
                goal.gear = astar::ForwardGear;
            else
                goal.gear = astar::BackwardGear;

            // save the current goal to the internal list
            goals.push_back(goal);

            // registering the current goal index
            if (8.0 < goal.Distance(robot) && !first_goal_found)
            {
                index = i;
                first_goal_found = true;
            }
        }

        // set the goal state
        g_hybrid_astar->set_goal_state(goals[index]);

        // set the goal state list
        g_hybrid_astar->set_goal_list(internal_goal_list);
    }
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
    g_hybrid_astar = new astar::HybridAstarPathFinder(argc, argv);

    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    // register all the current handlers
    register_handlers();

    // define the hybrid astar message
    carmen_hybrid_astar_define_path_message();

    // the main IPC loop
    carmen_ipc_dispatch();

    // delete the global pointer
    delete(g_hybrid_astar);

    return 0;
}
