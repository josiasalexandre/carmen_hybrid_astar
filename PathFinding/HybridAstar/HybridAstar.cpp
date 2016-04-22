#include "HybridAstar.hpp"

#include <queue>

using namespace astar;

HybridAstar::HybridAstar() {}

// PRIVATE METHODS
// rebuild an entire path given a node
std::list<Pose2D> rebuildPath(HybridAstarNodePtr n) {

    // create the list
    std::list<Pose2D> path;

    // auxiliar node ptr
    HybridAstarNodePtr tmp = nullptr;

    // counter
    unsigned int subPathCounter = 0;

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // save the current pose to the list
            path.push_front();

        }

    }


    return path;


}

// PUBLIC METHODS

// receives the map, start and goal poses and find a path, if possible
std::list<Pose2D> HybridAstar::findPath(const Pose2D& start, const Pose2D& goal, InternalGridMap &map) {


    // update the heuristic to the new goal goal
    heuristic.updateGoal(map, goal);

    // build a new priority queue
    std::priority_queue<HybridAstarNodePtr, std::vector<HybridAstarNodePtr>, HyridAstarNodePtrComparator> open;

    // helpers
    double heuristicValue = heuristic.getHeuristicValue(map, start, goal);

    // find the current cell
    CellPtr c = map.poseToCell(start);

    // create a new Node
    HybridAstarNodePtr n = new HybridAstarNode(start, new ReedsSheppAction(astar::RSStraight, ForwardGear, 0), c, 0, heuristicValue, nullptr);

    // push the start node to the queue
    open.push(n);

    // the actual A* algorithm
    while(!open.empty()) {

        // get the hight priority node
        n = open.pop();

        // is it the desired goal?
        if (goal == n.pose) {

            // return the path
            return rebuildPath(n);

        }

    }



}
