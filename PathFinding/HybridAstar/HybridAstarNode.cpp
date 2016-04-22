#include "HybridAstarNode.hpp"
#include <boost/concept_check.hpp>

using namespace astar;

// basic constructor with a given action
HybridAstarNode::HybridAstarNode(
                                    const Pose2D &p,
                                    ReedsSheppActionPtr rsAction,
                                    const Cell* c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNode *n
                                ) : pose(p), action(rsAction), actionSet(nullptr), cell(c), g(cost), f(heuristicCost), parent(n) {
}

// the basic constructor with a given action set
HybridAstarNode::HybridAstarNode(
                                    const Pose2D &p,
                                    ReedsSheppActionSetPtr rsActionSet,
                                    Cell &c,
                                    double cost,
                                    double heuristicCost,
                                    HybridAstarNode* n
                                ) : pose(p), action(nullptr), actionSet(rsActionSet), cell(c), g(cost), f(heuristicCost), parent(n) {}

// basic destructor
HybridAstarNode::~HybridAstarNode() {

    // update the cell status

    // the cell should not point to any HybridAstarNode
    cell.node = nullptr;

    // the cell is available now
    cell.available = true;

    // delete the action
    if (nullptr != action) {

        delete(action);

    }

    // delete the action set
    if (nullptr != actionSet) {

        delete(actionSet);

    }

}
