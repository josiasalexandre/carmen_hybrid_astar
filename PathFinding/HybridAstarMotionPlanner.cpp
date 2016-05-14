#include "HybridAstarMotionPlanner.hpp"

// basic constructor
astar::HybridAstarMotionPlanner::HybridAstarMotionPlanner(int argc, char **argv) : valid_path(false) {

    // read all parameters
    get_parameters(argc, argv);

}

// PRIVATE METHODS
void
astar::HybridAstarMotionPlanner::get_parameters(int argc, char **argv)
{
    // vehicle parameters
    carmen_param_t vehicle_params_list[21] = {
        {(char *)"robot",   (char *)"max_steering_angle",                           CARMEN_PARAM_DOUBLE, &vehicle_model.maxphi,                                         1, NULL},
        {(char *)"robot",   (char *)"desired_steering_command_rate",                CARMEN_PARAM_DOUBLE, &vehicle_model.steering_command_rate,                          1, NULL},
        {(char *)"robot",   (char *)"understeer_coeficient",                        CARMEN_PARAM_DOUBLE, &vehicle_model.understeer,                                     1, NULL},
        {(char *)"robot",   (char *)"robot_maximum_capable_curvature",              CARMEN_PARAM_DOUBLE, &vehicle_model.max_turn_radius,                                1, NULL},
        {(char *)"robot",   (char *)"default_speed",                                CARMEN_PARAM_DOUBLE, &vehicle_model.default_speed,                                  1, NULL},
        {(char *)"robot",   (char *)"maximum_speed_forward",                        CARMEN_PARAM_DOUBLE, &vehicle_model.max_forward_speed,                              1, NULL},
        {(char *)"robot",   (char *)"maximum_speed_reverse",                        CARMEN_PARAM_DOUBLE, &vehicle_model.max_backward_speed,                             1, NULL},
        {(char *)"robot",   (char *)"length",                                       CARMEN_PARAM_DOUBLE, &vehicle_model.length,                                         1, NULL},
        {(char *)"robot",   (char *)"width",                                        CARMEN_PARAM_DOUBLE, &vehicle_model.width,                                          1, NULL},
        {(char *)"robot",   (char *)"distance_between_front_and_rear_axles",        CARMEN_PARAM_DOUBLE, &vehicle_model.axledist,                                       1, NULL},
        {(char *)"robot",   (char *)"distance_between_rear_wheels",                 CARMEN_PARAM_DOUBLE, &vehicle_model.rear_wheels_dist,                               1, NULL},
        {(char *)"robot",   (char *)"distance_between_rear_car_and_rear_wheels",    CARMEN_PARAM_DOUBLE, &vehicle_model.rear_car_wheels_dist,                           1, NULL},
        {(char *)"robot",   (char *)"distance_between_front_car_and_front_wheels",  CARMEN_PARAM_DOUBLE, &vehicle_model.front_car_wheels_dist,                          1, NULL},
        {(char *)"robot",   (char *)"maximum_acceleration_forward",                 CARMEN_PARAM_DOUBLE, &vehicle_model.max_forward_acceleration,                       1, NULL},
        {(char *)"robot",   (char *)"maximum_deceleration_forward",                 CARMEN_PARAM_DOUBLE, &vehicle_model.max_forward_deceleration,                       1, NULL},
        {(char *)"robot",   (char *)"maximum_acceleration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.max_backward_acceleration,                      1, NULL},
        {(char *)"robot",   (char *)"maximum_deceleration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.max_backward_deceleration,                      1, NULL},
        {(char *)"robot",   (char *)"desired_acceleration",                         CARMEN_PARAM_DOUBLE, &vehicle_model.desired_forward_acceleration,                   1, NULL},
        {(char *)"robot",   (char *)"desired_decelaration_forward",                 CARMEN_PARAM_DOUBLE, &vehicle_model.desired_forward_deceleration,                   1, NULL},
        {(char *)"robot",   (char *)"desired_accelaration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.desired_backward_acceleration,                  1, NULL},
        {(char *)"robot",   (char *)"desired_decelaration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.desired_backward_deceleration,                  1, NULL}
    }

    carmen_param_allow_unfound_variables(1);
    carmen_param_install_params(argc, argv, vehicle_params_list, 21);

}
// PUBLIC METHODS

// find a smooth find to the goal and publish
bool
astar::HybridAstarMotionPlanner::find_path()
{
}

// set the the new goal
void
astar::HybridAstarMotionPlanner::set_goal_pose(double x, double y, double theta)
{
}

// set the goal list
void
astar::HybridAstarMotionPlanner::set_goal_list(double x, double y, double theta)
{
}

// update the map
void
astar::HybridAstarMotionPlanner::update_map(carmen_map_server_compact_cost_map_message *msg)
{
}

// update the odometry value
void
astar::HybridAstarMotionPlanner::set_odometry(double, double)
{
}

// update the motion planner mode
void
astar::HybridAstarMotionPlanner::set_parking_mode()
{
}

//
void
astar::HybridAstarMotionPlanner::go()
{
}

void
astar::HybridAstarMotionPlanner::stop()
{
}

// get all the necessary parameters
void
astar::HybridAstarMotionPlanner::get_parameters()
{
}

// publish the current path
void
astar::HybridAstarMotionPlanner::publish_path(std::list<astar::Pose2D>& path)
{
}

// publish the current status
void
astar::HybridAstarMotionPlanner::publish_status()
{
}

