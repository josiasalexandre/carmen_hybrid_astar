#ifndef HYBRID_ASTAR_ALGORITHM_HPP
#define HYBRID_ASTAR_ALGORITHM_HPP

#include <vector>
#include <list>
#include <mutex>

#include "../../PriorityQueue/PriorityQueue.hpp"
#include "HybridAstarNode.hpp"
#include "../../Entities/Pose2D.hpp"
#include "../GridMap/InternalGridMap.hpp"
#include "Heuristic.hpp"
#include "../ReedsShepp/ReedsSheppModel.hpp"
#include "../VehicleModel/VehicleModel.hpp"

namespace astar {

class HybridAstar {

private:

    // PRIVATE ATTRIBUTES

    // the reverse factor penalty
    double reverse_factor;

    // the gear switch cost penalty
    double gear_switch_cost;

    // the ReedsSheppModel
    ReedsSheppModel rs;

    // the Vehicle model
    VehicleModel vehicle;

    // grid map pointer
    astar::InternalGridMapPtr grid;

    // the heuristic
    astar::Heuristic heuristic;

    // the opened nodes set
    astar::PriorityQueue<HybridAstarNodePtr> open;

    // the expaned nodes set
    std::vector<HybridAstarNodePtr> discovered;

    // the invalid nodes set
    std::vector<HybridAstarNodePtr> invalid;

    // a simple mutex to the open PriorityQueue
    std::mutex open_mutex;

    // PRIVATE METHODS

    // clear all the sets
    void RemoveAllNodes();

    // reconstruct the path from the goal to the start pose
    astar::PoseListPtr ReBuildPath(HybridAstarNodePtr);

    // get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
    HybridAstarNodePtr GetReedsSheppChild(const astar::Pose2D&, const astar::Pose2D&);

    // get the children nodes by expanding all gears and steeering
    HybridAstarNodeArrayPtr GetChidlren(const astar::Pose2D&, const astar::Pose2D&, double);

    // get the path cost
    double PathCost(const astar::Pose2D& start, const astar::Pose2D& goal, double length, bool reverse_gear);

public:

    // PUBLIC ATTRIBUTES

    // PUBLIC METHODS

    // basic constructor
    HybridAstar();

    // find a path to the goal
    PoseListPtr FindPath(const astar::Pose2D&, const astar::Pose2D&, astar::InternalGridMap&);

};

}


#endif
