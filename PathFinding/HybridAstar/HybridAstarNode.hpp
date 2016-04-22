#ifndef HYBRID_ASTAR_NODE_HPP
#define HYBRID_ASTAR_NODE_HPP

#include "../../Entities/Pose2D.hpp"

#include "../ReedsShepp/ReedsSheppActionSet.hpp"

#include <exception>

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
        astar::ReedsSheppActionSetPtr actionSet;

        // the current cell
        astar::Cell &cell;

        // the current node cost
        double g;

        // the current node cost + estimated heuristic cost
        double f;

        // the parent node
        HybridAstarNode *parent;

        // basic constructor with a given action
        HybridAstarNode(const astar::Pose2D&, astar::ReedsSheppActionPtr, astar::Cell*, double, double, astar::HybridAstarNode*);

        // basic constructor
        HybridAstarNode(const astar::Pose2D&, astar::ReedsSheppActionSetPtr, astar::Cell*, double, double, astar::HybridAstarNode*);

        // basic destructor
        ~HybridAstarNode();

        // PUBLIC METHODS
        // < operator overloading, for priority queue compare purpose
        bool operator<(const astar::HybridAstarNode& n) const;

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
            throw std::invalid_argument("Cant compare nullptr of type HybridAstarNodePtr!");

        }

};

}

#endif
