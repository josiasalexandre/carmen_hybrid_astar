#ifndef HYBRID_ASTAR_NODE_HPP
#define HYBRID_ASTAR_NODE_HPP

#include "../Entities/Pose2D.hpp"

#include "ReedsShepp/ReedsSheppActionSet.hpp"

namespace astar {

class HybridAstarNode {

    private:

        // private members

        // private methods

    public:

        // public members

        // the current pose
        astar::Pose2D pose;

        // the steering action set
        astar::ReedsSheppActionSetPtr actionSet;

        // the current cell
        astar::CellPtr cell;

        // the current node cost
        double g;

        // the current node cost + estimated heuristic cost
        double f;

        // the parent node
        HybridAstarNode *parent;

        // basic constructor
        HybridAstarNode(const astar::Pose2D&, astar::ReedsSheppActionSetPtr, const astar::Cell&, double, double, const astar::HybridAstarNode*);

        // public methods
        // < operator overloading, for priority queue compare purpose
        bool operator<(const astar::HybridAstarNode& n) const;

};

typedef HybridAstarNode* HybridAstarNodePtr;

}

#endif
