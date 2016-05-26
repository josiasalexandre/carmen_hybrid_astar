#include "HybridAstarNode.hpp"

using namespace astar;

// basic constructor with a given action
HybridAstarNode::HybridAstarNode(
                                    const State2D &p,
                                    ReedsSheppActionPtr rsAction,
                                    const MapCellPtr c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNode *n
                                ) : state(p), action(rsAction), action_set(0), g(cost), f(heuristicCost), parent(n), cell(c), handle(nullptr)
{

    // the current node should be known by the pointed MapCell
    // set the node
    cell->node = this;

}

// the basic constructor with a given action set
HybridAstarNode::HybridAstarNode(
                                    const State2D &p,
                                    ReedsSheppActionSetPtr rsActionSet,
                                    MapCellPtr c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNodePtr n
                                ) : state(p), action(nullptr), action_set(rsActionSet), g(cost), f(heuristicCost), parent(n), cell(c), handle(nullptr)
{

    // the current node should be known by the pointed MapCell
    // set the node
    cell->node = this;

}

// basic destructor
HybridAstarNode::~HybridAstarNode() {

    // update the cell status

    // the cell should not point to any HybridAstarNode
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

    // the current state
    state = n.state;

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
