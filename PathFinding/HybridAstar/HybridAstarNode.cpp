#include "HybridAstarNode.hpp"
#include <boost/concept_check.hpp>

using namespace astar;

// basic constructor with a given action
HybridAstarNode::HybridAstarNode(
                                    const Pose2D &p,
                                    ReedsSheppActionPtr rsAction,
                                    const MapCellPtr c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNode *n
                                ) : pose(p), action(rsAction), actionSet(0), cell(c), g(cost), f(heuristicCost), parent(n)
{

    // the current node should be known by the pointed MapCell

}

// the basic constructor with a given action set
HybridAstarNode::HybridAstarNode(
                                    const Pose2D &p,
                                    ReedsSheppActionSetPtr rsActionSet,
                                    MapCellPtr c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNodePtr n
                                ) : pose(p), action(nullptr), actionSet(rsActionSet), cell(c), g(cost), f(heuristicCost), parent(n) {}

// basic destructor
HybridAstarNode::~HybridAstarNode() {

    // update the cell status

    // the cell should not point to any HybridAstarNode
    if (nullptr != cell) {

        cell->node = nullptr;

        // the cell is available now
        cell->available = true;

        // update the cell status
        cell->status = UnknownNode;

    }

    // delete the action
    if (nullptr != action) {

        delete(action);

    }

    // delete the action set
    if (nullptr != actionSet) {

        delete(actionSet);

    }

}


// PUBLIC METHODS

// < operator overloading, for priority queue compare purpose
bool HybridAstarNode::operator<(const astar::HybridAstarNode& n) const {

    return f < n.f;

}

// assignement operator
void HybridAstarNode::operator=(const astar::HybridAstarNode& n) {

    // copy the input node values

    // the current pose
    pose  = n.pose;

    // the steering action
    if (nullptr != action) {

        delete action;

    }

    // update the steering action
    action = n.action;

    // the action set
    if (nullptr != actionSet) {

        delete(actionSet);

    }

    // the steering action set
    actionSet = n.actionSet;

    // the current cell
    cell = n.cell;

    // the current node cost
    g = n.g;;

    // the current node cost + estimated heuristic cost
    f = n.f;

    return;

}
