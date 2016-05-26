#ifndef HYBRID_ASTAR_MOTION_PLANNER_HPP
#define HYBRID_ASTAR_MOTION_PLANNER_HPP

#include <carmen/carmen.h>
#include <carmen/map_server_messages.h>

#include "GridMap/InternalGridMap.hpp"
#include "HybridAstar/HybridAstar.hpp"
#include "Smoother/CGSmoother.hpp"
#include "VehicleModel/VehicleModel.hpp"
#include "../PathFollower/StanleyController.hpp"

namespace astar {

class HybridAstarMotionPlanner {

    private:

        // PRIVATE ATTRIBUTES

        // the robot configuration
        astar::VehicleModel vehicle_model;

        // the internal map
        astar::InternalGridMap grid;

        // flag to indicate the grid map status
        bool initialized_grid_map;

        // the hybrid astar search algorithm
        astar::HybridAstar path_finder;

        // the path smoother
        astar::CGSmoother path_smoother;

        // custom Stanley method
        astar::StanleyController path_follower;

        // the current odometry speed
        double odometry_speed;

        // the current odometry wheel_angle
        double odometry_wheel_angle;

        // the current goal
        astar::State2D goal;

        // the path to be followed
        std::vector<astar::State2D> path;

        // flag to validate the path
        bool valid_path;

        // flag to activate the motion planner
        bool activated;

        // flag to set obstacle avoider usage
        bool use_obstacle_avoider;

        // flag to register the simulation mode
        bool simulation_mode;

        // PRIVATE METHODS

        // get all the necessary parameters
        void get_parameters(int argc, char **argv);

    public:

        // basic constructor
        HybridAstarMotionPlanner(int argc, char **argv);

        // update the odometry value
        void set_odometry(double v, double phi);

        // estimate the robot inital pose
        astar::State2D estimate_initial_pose(double x, double y, double theta, double timestamp);

        // find a smooth find to the goal and publish
        bool replan(astar::State2D &start);

        // set the the new goal
        void set_goal_pose(double x, double y, double theta, double vel = 0);

        // update the map
        void update_map(carmen_map_server_compact_cost_map_message *msg);


        // convert the current path to the desired output format (carmen_ackerman_motion_command_t)
        std::vector<astar::State2D> get_path();

};

}

#endif
