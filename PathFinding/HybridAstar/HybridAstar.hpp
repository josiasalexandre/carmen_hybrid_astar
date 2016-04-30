#ifndef HYBRID_ASTAR_ALGORITHM_HPP
#define HYBRID_ASTAR_ALGORITHM_HPP

#include <vector>
#include <list>
#include <queue>

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

        // the ReedsSheppModel
        ReedsSheppModel rs;

        // the Vehicle model
        VehicleModel vehicle;

        // grid map pointer
        astar::InternalGridMapPtr grid;

        // the heuristic
        astar::Heuristic heuristic;

        // the opened nodes set
        std::priority_queue<HybridAstarNodePtr, std::vector<HybridAstarNodePtr>, HyridAstarNodePtrComparator> open;

        // the expaned nodes set
        std::vector<HybridAstarNodePtr> discovered;

        // the invalid nodes set
        std::vector<HybridAstarNodePtr> invalid;

        // PRIVATE METHODS

        // clear all the sets
        void removeAllNodes();

        // reconstruct the path from the goal to the start pose
        astar::PoseListPtr rebuildPath(HybridAstarNodePtr);

        // get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
        HybridAstarNodePtr getReedsSheppChild(const Pose2D&, const Pose2D&);

        // get the children nodes by expanding all gears and steeering
        HybridAstarNodeArrayPtr getChildren(const Pose2D&, const Pose2D&, double);


    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // basic constructor
        HybridAstar();

        // find a path to the goal
        PoseListPtr findPath(const astar::Pose2D&, const astar::Pose2D&, astar::InternalGridMap&);

};

}


#endif
