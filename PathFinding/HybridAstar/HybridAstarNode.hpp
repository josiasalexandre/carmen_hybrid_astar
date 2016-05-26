#ifndef HYBRID_ASTAR_NODE_HPP
#define HYBRID_ASTAR_NODE_HPP

#include "../ReedsShepp/ReedsSheppActionSet.hpp"

#include "../../PriorityQueue/PriorityQueueNode.hpp"

#include <exception>
#include "../../Entities/State2D.hpp"

namespace astar {

class HybridAstarNode {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // the current state
        astar::State2D state;

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
        astar::MapCellPtr cell;

        // the priority queue handle
        astar::PriorityQueueNodePtr<HybridAstarNode*> handle;

        // basic constructor with a given action
        HybridAstarNode(const astar::State2D&, astar::ReedsSheppActionPtr, astar::MapCellPtr, double, double, astar::HybridAstarNode*);

        // basic constructor
        HybridAstarNode(const astar::State2D&, ReedsSheppActionSetPtr, astar::MapCellPtr, double, double, astar::HybridAstarNode*);

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
