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
        astar::VehicleModel vehicleModel;

        // the internal map
        astar::InternalGridMap grid;

        // the hybrid astar search algorith
        astar::HybridAstar pathFinder;

        // the path smoother
        astar::CGSmoother pathSmoother;

        // the current goal
        astar::Pose2D goal;

        // the path to be published
        std::list<astar::Pose2D> path;

        // flag to validates the path
        bool validPath;

        // PRIVATE METHODS

        // publish the current path
        void publishPath(std::list<astar::Pose2D>& path);

        // publish the current status
        void publishStatus();

        // get all the necessary parameters
        // read all parameters
        void readParameters(int argc, char **argv);

        // a static member pointer
        static HybridAstarMotionPlanner* current;

    public:

        // basic constructor
        HybridAstarMotionPlanner(int argc, char** argv);

        // registering the currrent static pointer
        void init();

        // the main handler
        static void globalPoseMessageHandler(carmen_localize_ackerman_globalpos_message *msg);

        // for simulation purpose
        static void truePoseMessageHandler(carmen_simulator_ackerman_truepos_message *msg);

        // receive the odometry
        static void odometryMessageHandler(carmen_base_ackerman_odometry_message *msg);

        // set the new goal handler
        static void setGoalMessageHandler(carmen_navigator_ackerman_set_goal_message *msg);

        // goal list message handler
        static void goalListMessageHandler(carmen_behavior_selector_goal_list_message *msg);

        // behavior selector state message handler
        static void stateMessageHandler(carmen_behavior_selector_state_message *msg);

        // the compact cost map handler
        static void compactCostMapMessageHandler(carmen_map_server_compact_cost_map_message *msg);

        // the compact lane map message handler
        static void compactLaneMapMessageHandler(carmen_map_server_compact_lane_map_message *msg);

        // register all handlers
        void registerAllHandlers();

};

}

#endif
