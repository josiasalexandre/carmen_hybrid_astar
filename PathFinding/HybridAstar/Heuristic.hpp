#ifndef COMBINED_HEURISTIC_HPP
#define COMBINED_HEURISTIC_HPP

#include "../State2D.hpp"
#include "../GridMap/InternalGridMap.hpp"

namespace astar {

class Heuristic {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // update the heuristic around a new goal
        void UpdateGoal(const astar::InternalGridMap& map, const astar::State2D&);

        // get a heuristic value
        double GetHeuristicValue(astar::InternalGridMap&, const astar::State2D&, const astar::State2D&);

};

}

#endif
