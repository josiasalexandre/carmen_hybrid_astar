#ifndef COMBINED_HEURISTIC_HPP
#define COMBINED_HEURISTIC_HPP

#include "../../Entities/Pose2D.hpp"
#include "../InternalGridMap.hpp"

namespace astar {

class Heuristic {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // update the heuristic around a new goal
        void updateGoal(const astar::InternalGridMap& map, const astar::Pose2D&);

        // get a heuristic value
        double getHeuristicValue(astar::InternalGridMap&, const astar::Pose2D&, const astar::Pose2D&);

};

}

#endif
