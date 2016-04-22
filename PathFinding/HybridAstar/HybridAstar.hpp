#ifndef HYBRID_ASTAR_ALGORITHM_HPP
#define HYBRID_ASTAR_ALGORITHM_HPP

#include <list>

#include "HybridAstarNode.hpp"
#include "../../Entities/Pose2D.hpp"
#include "../Map/InternalGridMap.hpp"
#include "Heuristic.hpp"

namespace astar {

class HybridAstar {

    private:

        // private members

        // the heuristic
        astar::Heuristic heuristic;

        // private methods

    public:

        // public members

        // public methods

        // basic constructor
        HybridAstar();

        // find a path to the goal
        std::list<astar::Pose2D> findPath(const astar::Pose2D&, const astar::Pose2D&, astar::InternalGridMap&);

};

}


#endif
