#include "HybridAstarMotionPlanner.hpp"

using namespace astar;

// basic constructor
HybridAstarMotionPlanner::HybridAstarMotionPlanner(int argc, char **argv) :
    vehicle_model(), path_finder(vehicle_model), valid_path(false), simulation_mode(true), activated(true), initialized_grid_map(false) {

    // read all parameters
    get_parameters(argc, argv);

}

// PRIVATE METHODS
void
HybridAstarMotionPlanner::get_parameters(int argc, char **argv)
{
	carmen_param_t planner_params_list[] = {
	// get the motion planner parameters
		{(char *)"astar",   (char *)"simulation_mode",                           	CARMEN_PARAM_ONOFF, &simulation_mode,                    		                    1, NULL},
	};

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
    };

    carmen_param_allow_unfound_variables(1);
    carmen_param_install_params(argc, argv, vehicle_params_list, 21);

}

// PUBLIC METHODS
// find a smooth find to the goal
bool
HybridAstarMotionPlanner::replan(State2D &start)
{
    path.clear();

    if (grid.isSafePlace(goal))
    {
        // the Hybrid A* search, first stage
        StateListPtr raw_path = path_finder.FindPath(grid, start, goal);

        if (raw_path)
        {
            // the path smoothing through conjugate gradient optimization and the final non-parametric interpolation, second stage
            StateListPtr smoothed_path = path_smoother.Smooth(grid, raw_path);

            // get the appropriate orientations and build the path's command list
            StateListPtr final_path = path_follower.ConsolidateStateList(smoothed_path, vehicle_model);

            // copy the final path to the output path
            std::copy(final_path->states.begin(), final_path->states.end(), path);

            // remove the intermediate paths
            delete raw_path;
            delete smoothed_path;
            delete final_path;
        }

    }

    return (0 < path.size());
}

// set the the new goal
void
HybridAstarMotionPlanner::set_goal_pose(double x, double y, double theta, double vel = 0.0)
{
    goal.position.x = x;
    goal.position.y = y;
    goal.orientation = theta;
    goal.v = vel;

}

// get the carmen compact cost map and rebuild our internal grid map representation
void
HybridAstarMotionPlanner::update_map(carmen_map_server_compact_cost_map_message *msg)
{
    if (!initialized_grid_map ||
            msg->config.x_size != grid.width ||
            msg->config.y_size != grid.height ||
            msg->config.size != grid.size)
    {
        Vector2D<double> map_origin(msg->config.x_origin, msg->config.y_origin);

        initialized_grid_map = grid.InitializeGridMap(
                msg->config.x_size, msg->config.y_size, msg->config.resolution, map_origin, 0);
    }

    int compact_map_size = msg->size;

    int *x_coord = msg->coord_x;
    int *y_coord = msg->coord_y;
    int x, y;
    double *val = msg->value;
    double current_value, *old_value;

    GridMapCellPtr c = grid.grid_map;
    GridMapCellPtr tmp;

    for (int i = 0; i < compact_map_size; i++)
    {
        current_value = val[i];
        x = x_coord[i];
        y = y_coord[i];

        tmp = c + GRID_MAP_INDEX(x, y);

        if (0.5 > current_value && 0 != tmp->occupancy)
            grid.ClearCell(tmp);
        else if (0.5 <= current_value && && 1 != tmp->occupancy)
            grid.OccupyCell(tmp);
    }

    grid.UpdateGridMap();
}

// update the odometry value
void
HybridAstarMotionPlanner::set_odometry(double v, double phi)
{

    // save the speed
    odometry_speed = v;

    // save the wheel angle
    odometry_wheel_angle = phi;

}

// estimate the initial robot state
State2D HybridAstarMotionPlanner::estimate_initial_pose(double x, double y, double theta, double delta_t) {

    // predict the initial robot state
    return vehicle_model.NextState(State2D(x, y, theta, odometry_wheel_angle, odometry_speed, delta_t));


}

std::vector<State2D>
HybridAstarMotionPlanner::get_path()
{
    return path;
}

