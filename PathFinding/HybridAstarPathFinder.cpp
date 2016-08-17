#include <iostream>

#include "HybridAstarPathFinder.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace astar;

// basic constructor
HybridAstarPathFinder::HybridAstarPathFinder(int argc, char **argv) :
    vehicle_model(), grid(), initialized_grid_map(false), gm_mutex(), path_finder(vehicle_model, grid),
    stanley_method(grid, vehicle_model), path_smoother(grid, vehicle_model), path(), odometry_speed(0.0),
    odometry_steering_angle(0.0), robot(), goal(), valid_goal(false), goal_list(),
    use_obstacle_avoider(true), activated(false), simulation_mode(false)
{
    // read all parameters
    get_parameters(argc, argv);

    // set the safety factor
    vehicle_model.safety_factor = 1.0;

    // set the half width
    vehicle_model.width_2 = vehicle_model.width * 0.5;

    // unlock the mutex
    gm_mutex.unlock();
}

// PRIVATE METHODS
void
HybridAstarPathFinder::get_parameters(int argc, char **argv)
{
    // hard setup
    simulation_mode = true;

    carmen_param_t planner_params_list[] = {
        //get the motion planner parameters
        {(char *)"astar",   (char *)"simulation_mode",                           	CARMEN_PARAM_ONOFF, &this->simulation_mode,                    		                    1, NULL},
    };

    // vehicle parameters
    carmen_param_t vehicle_params_list[] = {
        {(char *)"robot",   (char *)"max_steering_angle",                           CARMEN_PARAM_DOUBLE, &vehicle_model.max_wheel_deflection,                           1, NULL},
        {(char *)"robot",   (char *)"desired_steering_command_rate",                CARMEN_PARAM_DOUBLE, &vehicle_model.steering_command_rate,                          1, NULL},
        {(char *)"robot",   (char *)"understeer_coeficient",                        CARMEN_PARAM_DOUBLE, &vehicle_model.understeer,                                     1, NULL},
        {(char *)"robot",   (char *)"maximum_capable_curvature",              		CARMEN_PARAM_DOUBLE, &vehicle_model.max_curvature, 	                                1, NULL},
        {(char *)"robot",   (char *)"max_velocity",                                	CARMEN_PARAM_DOUBLE, &vehicle_model.max_velocity,                                   1, NULL},
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
        {(char *)"robot",   (char *)"maximum_acceleration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.desired_backward_acceleration,                  1, NULL},
        {(char *)"robot",   (char *)"desired_decelaration_reverse",                 CARMEN_PARAM_DOUBLE, &vehicle_model.desired_backward_deceleration,                  1, NULL}
    };


    carmen_param_install_params(argc, argv, vehicle_params_list, sizeof(vehicle_params_list) / sizeof(vehicle_params_list[0]));

    carmen_param_allow_unfound_variables(1);
    carmen_param_install_params(argc, argv, planner_params_list, sizeof(planner_params_list) / sizeof(planner_params_list[0]));

    // do some pre-computations and update some indirect parameters
    vehicle_model.Configure();

}

// PUBLIC METHODS
// find a smooth find to the goal
bool
HybridAstarPathFinder::replan() {

    // the returning flag
    bool ret = false;

    if (valid_goal) {

        if (path.states.empty()) {

            // find the path to the goal
            StateArrayPtr raw_path = path_finder.FindPath(grid, robot, goal);

            if (0 < raw_path->states.size()) {

                // smoooth the current path
                StateArrayPtr smooth_path = path_smoother.Smooth(grid, vehicle_model, raw_path);

                // the final command list
                StateArrayPtr commands = stanley_method.RebuildCommandList(robot, smooth_path);

                //path.states = smooth_path->states;
                path.states = commands->states;

                delete smooth_path;
                delete commands;

                // set the returning flag
                ret = true;

            }

            // delete the raw path
            delete raw_path;

        } else {

            // the final command list
            StateArrayPtr commands = stanley_method.GetCommandList(robot);

            // copy to the internal path
            path.states = commands->states;

            delete commands;

            // set the returning flag
            ret = true;

        }

    }

    return ret;

}

// get the resulting path
StateArrayPtr
HybridAstarPathFinder::get_path() {

    // build a new state array
    StateArrayPtr current_path = new StateArray();

    // copy the path
    current_path->states = path.states;

    return current_path;

}

// set the the new goal
void
HybridAstarPathFinder::set_goal_state(const State2D &goal_state) {

    // copy the goal
    goal = goal_state;

    if (initialized_grid_map) {

        // verify the safety
        activated = valid_goal = grid.isSafePlace(vehicle_model.GetVehicleBodyCircles(goal_state), vehicle_model.safety_factor);

        return;
    }

    valid_goal = false;
}

// get the external goal list, convert to our internal representation and set the new goal
bool
HybridAstarPathFinder::same_goal_list(carmen_behavior_selector_goal_list_message *msg) {

    // direct access
    std::vector<astar::State2D> &igl(goal_list.states);

    Vector2D<double> position;

    for (unsigned int i = 0; i < msg->size; ++i) {

        Gear g = (0 > msg->goal_list[i].v) ? BackwardGear : ForwardGear;
        position.x = msg->goal_list[i].x;
        position.y = msg->goal_list[i].y;

        if (position != igl[i].position || 0.001 > std::fabs(msg->goal_list[i].phi - igl[0].phi) || igl[i].v != msg->goal_list[i].v || g != igl[i].gear) {

            return false;

        }


    }

    return true;

}

// set the the new goal
void
HybridAstarPathFinder::set_goal_state(double x, double y, double theta, double vel) {

    goal.position.x = x;
    goal.position.y = y;
    goal.orientation = theta;
    goal.v = vel;

    if (initialized_grid_map) {

        // verify the safety
        valid_goal = grid.isSafePlace(vehicle_model.GetVehicleBodyCircles(goal), vehicle_model.safety_factor);

        return;
    }

    valid_goal = false;
}

// set the goal list
void
HybridAstarPathFinder::set_goal_list(carmen_behavior_selector_goal_list_message *msg) {

    if (msg->size != goal_list.states.size() || !same_goal_list(msg)) {

        // direct access
        std::vector<astar::State2D> &igl(goal_list.states);

        // clear the internal goal list
        igl.clear();

        // now, convert the external list to our internal representation

        // goal states
        astar::State2D next_goal;

        unsigned int msg_size = msg->size;
        carmen_ackerman_traj_point_t *msg_gl = msg->goal_list;

        // registering the next goal index
        unsigned int index = 0;

        bool first_goal_found = false;

        // save the current goal list
        for (unsigned int i = 0; i < msg_size; i++)
        {
            next_goal.position.x = msg_gl[i].x;
            next_goal.position.y = msg_gl[i].y;
            next_goal.orientation = msg_gl[i].theta;
            next_goal.v = msg_gl[i].v;
            next_goal.phi = msg_gl[i].phi;

            next_goal.gear = (0 > goal.v) ? astar::BackwardGear : astar::ForwardGear;

            // save the current goal to the internal list
            igl.push_back(next_goal);

            // registering the current goal index
            if (8.0 < next_goal.Distance(robot) && !first_goal_found)
            {
                index = i;
                first_goal_found = true;
            }
        }

        // set the goal state
        set_goal_state(igl[index]);

    }
}

// voronoi thread update
void
HybridAstarPathFinder::voronoi_update(carmen_map_server_compact_cost_map_message *msg) {

    double x_origin = msg->config.x_origin;
    double y_origin = msg->config.y_origin;
    double resolution = msg->config.resolution;
    double inverse_resolution = 1.0/resolution;

    grid.InitializeGridMap(msg->config.y_size, msg->config.x_size, resolution, Vector2D<double>(x_origin, y_origin));

    int *x_coord = msg->coord_x;
    int *y_coord = msg->coord_y;
    double *val = msg->value;
    unsigned int size = msg->size;


    for (unsigned int i = 0; i < size; i++) {

        if (0.5 < val[i]) {
            grid.OccupyCell(y_coord[i], x_coord[i]);
        } else {
            grid.ClearCell(y_coord[i], x_coord[i]);
        }

    }

    // update the entire grid map
    grid.UpdateGridMap();

    // unlock the given mutex
    gm_mutex.unlock();

}

// voronoi thread update
void
HybridAstarPathFinder::voronoi_update_2(carmen_grid_mapping_message *msg) {

    double x_origin = msg->config.x_origin;
    double y_origin = msg->config.y_origin;
    double resolution = msg->config.resolution;
    double inverse_resolution = 1.0/resolution;
    double *val = msg->complete_map;

    unsigned int width = msg->config.x_size;
    unsigned int height = msg->config.y_size;
    unsigned int size = msg->size;

    int r, c;
    grid.InitializeGridMap(msg->config.y_size, msg->config.x_size, resolution, Vector2D<double>(x_origin, y_origin));

    for (unsigned int i = 0; i < size; i++) {

        r = i % width;
        c = i / width;

        // let's see the row and col
        if (val[i] > 0.4) {
            grid.OccupyCell(r, c);
        } else {
            grid.ClearCell(r, c);
        }
    }

    // update the current grid map
    grid.UpdateGridMap();

    initialized_grid_map = true;

    // unlock the given mutex
    gm_mutex.unlock();

}

// get the carmen compact cost map and rebuild our internal grid map representation
void
HybridAstarPathFinder::update_map(carmen_map_server_compact_cost_map_message *msg) {

    if (nullptr != msg) {

        // try to lock the mutex
        if (gm_mutex.try_lock()) {

            // spaw a new thread to update the voronoi map
            std::thread gvd_update(&HybridAstarPathFinder::voronoi_update, this, msg);

            // detach from the current thread
            gvd_update.detach();
        }
    }
}

// get the general map and save it
void
HybridAstarPathFinder::update_map(carmen_grid_mapping_message *msg) {


    if (nullptr != msg) {

        // try to lock the mutex
        if (gm_mutex.try_lock()) {

            // spaw a new thread to update the voronoi map
            std::thread gvd_update(&HybridAstarPathFinder::voronoi_update_2, this, msg);

            // detach from the current thread
            gvd_update.detach();
        }
    }
}

// update the odometry value
void
HybridAstarPathFinder::set_odometry(double v, double phi) {

    // set the current speed
    odometry_speed = (std::fabs(v) > 0.01) ? v : 0.0;

    // set the current wheel angle
    odometry_steering_angle =(std::fabs(phi) > 0.001) ? phi : 0.0;

}

// estimate the initial robot state
void
HybridAstarPathFinder::set_initial_state(double x, double y, double theta, double v, double phi, double dt) {

    // predict the initial robot pose
    if (std::fabs(v) > 0.01) {
        robot = vehicle_model.NextPose(Pose2D(x, y, theta), v, phi, dt);
        robot.v = v;
    } else {
        robot.position.x = x;
        robot.position.y = y;
        robot.orientation = theta;
        robot.v = 0.0;
    }

    robot.phi = phi;
    robot.t = dt;
    robot.gear = (0 > v) ? BackwardGear : ForwardGear;

}

// estimate the initial robot state
void
HybridAstarPathFinder::set_initial_state(double x, double y, double theta, double dt) {

    set_initial_state(x, y, theta, odometry_speed, odometry_steering_angle, dt);

}

// get the robot state
State2D
HybridAstarPathFinder::get_robot_state() {

    return robot;

}
