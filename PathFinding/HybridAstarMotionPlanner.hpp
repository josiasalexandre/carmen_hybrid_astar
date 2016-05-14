#ifndef HYBRID_ASTAR_MOTION_PLANNER_HPP
#define HYBRID_ASTAR_MOTION_PLANNER_HPP

#include "GridMap/InternalGridMap.hpp"
#include "HybridAstar/HybridAstar.hpp"
#include "CGSmoother.hpp"
#include "VehicleModel/VehicleModel.hpp"

namespace astar {

class HybridAstarMotionPlanner {

    private:

        // PRIVATE ATTRIBUTES

        // the robot configuration
        astar::VehicleModel vehicle_model;

        // the internal map
        astar::InternalGridMap grid;

        // the hybrid astar search algorith
        astar::HybridAstar path_finder;

        // the path smoother
        astar::CGSmoother path_smoother;

        // the current goal
        astar::Pose2D goal;

        // the path to be published
        std::list<astar::Pose2D> path;

        // flag to validates the path
        bool valid_path;

        // a static member pointer
        static HybridAstarMotionPlanner* current;

        // PRIVATE METHODS

        // get all the necessary parameters
        void get_parameters(int argc, char **argv);

    public:

        // basic constructor
        HybridAstarMotionPlanner(int argc, char **argv);

        // find a smooth find to the goal and publish
        bool find_path();

        // set the the new goal
        void set_goal_pose(double x, double y, double theta);

        // set the goal list
        void set_goal_list(double x, double y, double theta);

        // update the map
        void update_map(carmen_map_server_compact_cost_map_message *msg);

        // update the odometry value
        void set_odometry(double, double);

        // update the motion planner mode
        void set_parking_mode();

        //
        void go();

        //
        void stop();

        // publish the current path
        void publish_path(std::list<astar::Pose2D>& path);

        // publish the current status
        void publish_status();
};

}

#endif
