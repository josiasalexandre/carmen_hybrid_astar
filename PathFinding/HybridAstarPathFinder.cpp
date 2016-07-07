#include "HybridAstarPathFinder.hpp"

using namespace astar;

// basic constructor
HybridAstarPathFinder::HybridAstarPathFinder(int argc, char **argv) :
    vehicle_model(), grid(), initialized_grid_map(false), path_finder(vehicle_model), odometry_speed(0.0),
    odometry_steering_angle(0.0), robot(), goal(), valid_goal(false), goal_list(nullptr), path(),
    activated(false), use_obstacle_avoider(true), simulation_mode(false)
{
    // read all parameters
    get_parameters(argc, argv);
}

// PRIVATE METHODS
void
HybridAstarPathFinder::get_parameters(int argc, char **argv)
{
	carmen_param_t planner_params_list[] = {
	// get the motion planner parameters
		{(char *)"astar",   (char *)"simulation_mode",                           	CARMEN_PARAM_ONOFF, &simulation_mode,                    		                    1, NULL},
	};

    // vehicle parameters
    carmen_param_t vehicle_params_list[21] = {
        {(char *)"robot",   (char *)"max_steering_angle",                           CARMEN_PARAM_DOUBLE, &vehicle_model.max_wheel_deflection,                           1, NULL},
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
StateArrayPtr
HybridAstarPathFinder::replan() {

    if (valid_goal) {

        // find the path to the goal
        StateArrayPtr path = path_finder.FindPath(grid, robot, goal);

        if (3 < path->states.size()) {

            // the actual path smoothing process
            path_smoother.Smooth(grid, path);

            // return the smothed path
            return path;

        }

        // delete the raw path
        delete(path);
    }

    return nullptr;
}

// set the the new goal
void
HybridAstarPathFinder::set_goal_state(const State2D &goal_state) {

    if (grid.isSafePlace(goal_state)) {

        goal = goal_state;
        valid_goal = true;
    }
}

// set the the new goal
void
HybridAstarPathFinder::set_goal_state(double x, double y, double theta, double vel = 0.0) {

    goal.position.x = x;
    goal.position.y = y;
    goal.orientation = theta;
    goal.v = vel;

}

// set the goal list
void HybridAstarPathFinder::set_goal_list(astar::StateArrayPtr goals) {

    if (0 < goals->states.size())
    {
        if (nullptr != goal_list)
        {
            delete(goal_list);
        }

        goal_list = goals;
    }
}

// get the carmen compact cost map and rebuild our internal grid map representation
void
HybridAstarPathFinder::update_map(carmen_map_server_compact_cost_map_message *msg) {
    if (grid.isEmpty() ||
            msg->config.x_size != grid.width ||
            msg->config.y_size != grid.height ||
            msg->config.size != grid.size)
    {
        Vector2D<double> map_origin(msg->config.x_origin, msg->config.y_origin);

        initialized_grid_map = grid.InitializeGridMap(
                msg->config.x_size, msg->config.y_size, msg->config.resolution, map_origin, 0.0);
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

    grid.Update();
}

// update the odometry value
void
HybridAstarPathFinder::set_odometry(double v, double phi) {

    // save the speed
    odometry_speed = v;

    // save the wheel angle
    odometry_steering_angle = phi;

}

// estimate the initial robot state
void HybridAstarPathFinder::set_initial_state(double x, double y, double theta, double v, double phi, double dt) {

    // predict the initial robot pose
    robot = vehicle_model.NextPose(Pose2D(x, y, theta), v, phi, dt);
    robot.v = v;
    robot.phi = phi;
    robot.t = dt;
    robot.gear = (0 > v) ? BackwardGear : ForwardGear;

}

// estimate the initial robot state
void HybridAstarPathFinder::set_initial_state(double x, double y, double theta, double dt) {

    set_initial_state(x, y, theta, odometry_speed, odometry_steering_angle, dt);

}

// get the robot state
astar::State2D HybridAstarPathFinder::get_robot_state() {

    return robot;

}

astar::StateArrayPtr HybridAstarPathFinder::get_path() {

    return path;

}

