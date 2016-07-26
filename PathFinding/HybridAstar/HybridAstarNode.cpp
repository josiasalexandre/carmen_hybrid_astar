#include <exception>

#include "HybridAstarNode.hpp"
#include "../../GridMap/GridMapCell.hpp"

using namespace astar;

// basic constructor with a given action
HybridAstarNode::HybridAstarNode(
        const Pose2D &_pose,
        ReedsSheppActionPtr rsAction,
        const GridMapCellPtr c,
        double cost,
        double heuristicCost,
        HybridAstarNode *p
    ) : pose(_pose), action(rsAction), action_set(0), g(cost), f(heuristicCost), parent(p), cell(c), handle(nullptr)
{
	// update the cell values
	if (nullptr != cell) {

		// set the current node
		cell->node = this;

		// set the cell status to opened
		cell->status = astar::OpenedNode;

	}
}

// the basic constructor with a given action set
HybridAstarNode::HybridAstarNode(
    const Pose2D &_pose,
    ReedsSheppActionSetPtr rsActionSet,
    GridMapCellPtr c,
    double cost,
    double heuristicCost,
    HybridAstarNodePtr p
    ) : pose(_pose), action(nullptr), action_set(rsActionSet), g(cost), f(heuristicCost), parent(p), cell(c), handle(nullptr)
{
	// update the cell values
	if (nullptr != cell) {

		// set the current node
		cell->node = this;

		// set the cell status to opened
		cell->status = astar::OpenedNode;

	}
}

// basic destructor
HybridAstarNode::~HybridAstarNode() {

    // update the cell status

    // the cell should not point to any HybridAstarNode
	if (nullptr != cell) {

		// update the cell's node pointer
		cell->node = nullptr;

		// set to unknown node status
		cell->status = astar::UnknownNode;

	}

    // delete the action
    if (nullptr != action) {

        delete action;

    } else if (nullptr != action_set) {

    	// delete the action set
        delete action_set;

    }

    parent = nullptr;

}


// PUBLIC METHODS
// update the node values
void HybridAstarNode::UpdateValues(const astar::HybridAstarNode &n) {

	// copy the input node values

	// the parent node
	parent = n.parent;

	// the current pose
	pose = n.pose;

	if (nullptr != n.action) {

		if (nullptr == action) {

			action = new ReedsSheppAction(*(n.action));

		} else {

			// the steering action
			*action = *(n.action);
		}

	} else if (nullptr != n.action_set) {

		if (nullptr == action_set) {

			action_set = new ReedsSheppActionSet(*(n.action_set));

		} else {

			// the steering action
			*action_set = *(n.action_set);

		}

	} else {

		throw std::exception();

	}

	// the current node cost
	g = n.g;;

	// the current node cost + estimated heuristic cost
	f = n.f;


}

// < operator overloading, for element comparison inside the priority queue
bool HybridAstarNode::operator<(const astar::HybridAstarNode& n) const {

    return f < n.f;

}

// assignment operator
void HybridAstarNode::operator=(const astar::HybridAstarNode& n) {

    // copy the input node values
	UpdateValues(n);

    // the current cell
	// CAUTION
    cell = n.cell;

}
