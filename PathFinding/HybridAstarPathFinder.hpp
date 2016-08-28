#ifndef HYBRID_ASTAR_PATH_FINDER_HPP
#define HYBRID_ASTAR_PATH_FINDER_HPP

#include <thread>
#include <mutex>

#include <carmen/carmen.h>
#include <carmen/map_server_messages.h>
#include <carmen/rddf_interface.h>

#include "../Entities/State2D.hpp"
#include "../GridMap/InternalGridMap.hpp"
#include "../VehicleModel/VehicleModel.hpp"

#include "HybridAstar/HybridAstar.hpp"
#include "Smoother/CGSmoother.hpp"

#include "../PathFollower/StanleyController.hpp"

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
        astar::StateArray path;

        // the current path
        astar::StateArray command_path;

        // the stanley method
        astar::StanleyController stanley_method;

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
        astar::StateArray goal_list;

        // flag to set obstacle avoider usage
        bool use_obstacle_avoider;

        // the rddf timestamp
        double rddf_timestamp;

        // the RDDF vector
        std::vector<astar::Vector2D<double>> rddf;

        // PRIVATE METHODS

        // get all the necessary parameters
        void get_parameters(int argc, char **argv);

        // voronoi thread update
        void voronoi_update(carmen_map_server_compact_cost_map_message *msg);

        // voronoi thread update
        void voronoi_update_2(carmen_grid_mapping_message *msg);

        // verify if the robot is closer enough to the path
        bool RobotIsLost();

        // verify if a given path is valid
        bool isValidPath(astar::StateArrayRef path);

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

        // find a smooth path to the goal
        bool replan();

        // get the resulting path
        astar::StateArrayPtr get_path();

        // set the new goal
        void set_goal_state(const State2D&);

        // verify if the external goal list is the same old one
        bool same_goal_list(carmen_behavior_selector_goal_list_message *msg);

        // set the new goal
        void set_goal_state(double x, double y, double theta, double vel = 0);

        // set the goal list
        void set_goal_list(carmen_behavior_selector_goal_list_message *msg);

        // update the map
        void update_map(carmen_map_server_compact_cost_map_message *msg);

        // update the map
        void update_map(carmen_grid_mapping_message *msg);

        // update the rddf
        void update_rddf(carmen_rddf_road_profile_message *msg);

        // PUBLIC ATTRIBUTES
        // flag to activate the motion planner
        bool activated;

        // flag to register the simulation mode
        bool simulation_mode;

};

}

#endif
