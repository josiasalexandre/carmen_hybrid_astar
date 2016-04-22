#include "HybridAstarNode.hpp"

// the basic constructor
astar::HybridAstarNode::HybridAstarNode(
                                        const astar::Pose2D &p,
                                        astar::ReedsSheppActionSetPtr rsActionSet,
                                        const astar::Cell &mapCell,
                                        double cost,
                                        double heuristicCost,
                                        astar::HybridAstarNode* n
                                       ) : pose(p), actionSet(rsActionSet), cell(mapCell), g(cost), f(heuristicCost), parent(n) {}

//
// < operator overloading, for priority queue compare purpose
bool astar::HybridAstarNode::operator<(const astar::HybridAstarNode& other) const {

    //
    return (f < other.f);

}

