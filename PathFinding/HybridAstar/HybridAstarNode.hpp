#ifndef HYBRID_ASTAR_NODE_HPP
#define HYBRID_ASTAR_NODE_HPP

#include <exception>

#include "../../Entities/Pose2D.hpp"
#include "../GridMap/GridMapCell.hpp"
#include "../../PriorityQueue/PriorityQueueNode.hpp"
#include "../ReedsShepp/ReedsSheppActionSet.hpp"

namespace astar {

class HybridAstarNode {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // the current pose
        astar::Pose2D pose;

        // the steering action
        astar::ReedsSheppActionPtr action;

        // the steering action set
        astar::ReedsSheppActionSetPtr action_set;

        // the current node cost
        double g;

        // the current node cost + estimated heuristic cost
        double f;

        // the parent node
        HybridAstarNode *parent;

        // the current cell
        astar::GridMapCellPtr cell;

        // the priority queue handle
        astar::PriorityQueueNodePtr<HybridAstarNode*> handle;

        // basic constructor with a given action
        HybridAstarNode(
                const astar::Pose2D&, ReedsSheppActionPtr,
                astar::GridMapCellPtr = nullptr, double cost_ = 0.0, double h_cost = 0.0, astar::HybridAstarNode *n= nullptr);

        // basic constructor
        HybridAstarNode(const astar::Pose2D&, ReedsSheppActionSetPtr,
                astar::GridMapCellPtr = nullptr, double cost_ = 0.0, double h_cost = 0.0, astar::HybridAstarNode *n = nullptr);

        // basic destructor
        ~HybridAstarNode();

        // PUBLIC METHODS
        // < operator overloading, for priority queue compare purpose
        bool operator<(const astar::HybridAstarNode& n) const;

        // Assignment operator
        void operator=(const astar::HybridAstarNode& n);

};


typedef HybridAstarNode* HybridAstarNodePtr;

class HyridAstarNodePtrComparator {

    public:

        // the compare method
        bool operator()(HybridAstarNodePtr a, HybridAstarNodePtr b) {

            if (nullptr != a && nullptr != b) {

                return a->f < b->f;

            }

            // exception handling
            throw std::invalid_argument("Can't compare nullptr of type HybridAstarNodePtr!");

        }

};

// a simple container
class HybridAstarNodeArray {

    public:

        // PUBLIC ATTRIBUTES
        // the pointers must be handled outside this class
        std::vector<HybridAstarNodePtr> nodes;

};

// just another pointer syntactic sugar
typedef HybridAstarNodeArray* HybridAstarNodeArrayPtr;

}

#endif
