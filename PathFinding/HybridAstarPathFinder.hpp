#ifndef HYBRID_ASTAR_PATH_FINDER_HPP
#define HYBRID_ASTAR_PATH_FINDER_HPP

#include <thread>
#include <mutex>

#include <carmen/carmen.h>
#include <carmen/map_server_messages.h>

#include "../Entities/State2D.hpp"
#include "../GridMap/InternalGridMap.hpp"
#include "../VehicleModel/VehicleModel.hpp"

#include "HybridAstar/HybridAstar.hpp"
#include "Smoother/CGSmoother.hpp"

namespace astar {

class HybridAstarPathFinder {

    private:

        // PRIVATE ATTRIBUTES

        // the robot configuration
        astar::VehicleModel vehicle_model;

        // flag to indicate the grid map status
        bool initialized_grid_map;

        // the current grid map mutex
        std::mutex gm_mutex;

        // the hybrid astar search algorithm
        astar::HybridAstar path_finder;

        // the path smoother
        astar::CGSmoother path_smoother;

        // the current path
        astar::StateArrayPtr path;

        // the current odometry speed
        double odometry_speed;

        // the current odometry wheel_angle
        double odometry_steering_angle;

        // the current robot state
        astar::State2D robot;

        // the current terminal state
        astar::State2D goal;

        // is it a valid goal?
        bool valid_goal;

        // the goal list
        astar::StateArrayPtr goal_list;

        // flag to set obstacle avoider usage
        bool use_obstacle_avoider;

        // PRIVATE METHODS

        // get all the necessary parameters
        void get_parameters(int argc, char **argv);

        // voronoi thread update
        void voronoi_update(carmen_map_server_compact_cost_map_message *msg);

        // voronoi thread update
        void voronoi_update_2(carmen_grid_mapping_message *msg);

    public:
        // the internal map represetation
        astar::InternalGridMap grid;

        // basic constructor
        HybridAstarPathFinder(int argc, char **argv);

        // update the odometry value
        void set_odometry(double v, double phi);

        // estimate the robot inital pose
        void set_initial_state(double x, double y, double theta, double v, double phi, double dt);

        // estimate the robot initial state
        void set_initial_state(double x, double y, double theta, double dt);

        // get the robot state
        astar::State2D get_robot_state();

        // find a smooth find to the goal and publish
        astar::StateArrayPtr replan();

        // set the new goal
        void set_goal_state(const State2D&);

        // set the new goal
        void set_goal_state(double x, double y, double theta, double vel = 0);

        // set the goal list
        void set_goal_list(astar::StateArrayPtr);

        // update the map
        void update_map(carmen_map_server_compact_cost_map_message *msg);

        // update the map
        void update_map(carmen_grid_mapping_message *msg);

        // PUBLIC ATTRIBUTES
        // flag to activate the motion planner
        bool activated;

        // flag to register the simulation mode
        bool simulation_mode;

};

}

#endif
