#include "HybridAstarNode.hpp"
#include "../../GridMap/GridMapCell.hpp"

using namespace astar;

// basic constructor with a given action
HybridAstarNode::HybridAstarNode(
        const Pose2D &p,
        ReedsSheppActionPtr rsAction,
        const GridMapCellPtr c,
        double cost,
        double heuristicCost,
        HybridAstarNode *n
    ) : pose(p), action(rsAction), action_set(0), g(cost), f(heuristicCost), parent(n), cell(c), handle(nullptr)
{}

// the basic constructor with a given action set
HybridAstarNode::HybridAstarNode(
    const Pose2D &p,
    ReedsSheppActionSetPtr rsActionSet,
    GridMapCellPtr c,
    double cost,
    double heuristicCost,
    HybridAstarNodePtr n
    ) : pose(p), action(nullptr), action_set(rsActionSet), g(cost), f(heuristicCost), parent(n), cell(c), handle(nullptr)
{}

// basic destructor
HybridAstarNode::~HybridAstarNode() {

    // update the cell status

    // the cell should not point to any HybridAstarNode
	if (nullptr != cell)
		cell->node = nullptr;

    // delete the action
    if (nullptr != action) {

        delete(action);

    }

    // delete the action set
    if (nullptr != action_set) {

        delete(action_set);

    }

}


// PUBLIC METHODS

// < operator overloading, for element comparison inside the priority queue
bool HybridAstarNode::operator<(const astar::HybridAstarNode& n) const {

    return f < n.f;

}

// assignment operator
void HybridAstarNode::operator=(const astar::HybridAstarNode& n) {

    // copy the input node values

    // the current pose
    pose = n.pose;

    // the steering action
    if (nullptr != action) {

        delete action;

    }

    // update the steering action
    action = n.action;

    // the action set
    if (nullptr != action_set) {

        delete(action_set);

    }

    // the steering action set
    action_set = n.action_set;

    // the current cell
    cell = n.cell;

    // the current node cost
    g = n.g;;

    // the current node cost + estimated heuristic cost
    f = n.f;

    return;

}
