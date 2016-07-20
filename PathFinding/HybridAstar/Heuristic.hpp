#ifndef COMBINED_HEURISTIC_HPP
#define COMBINED_HEURISTIC_HPP

#include "../../Entities/Pose2D.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "NonholonomicHeuristicInfo.hpp"

namespace astar {


class Heuristic {

    private:

        // PRIVATE ATTRIBUTES

		// the current precomputed nonholonomic heuristic
		astar::NonholonomicHeuristicInfo info;

		// non holonomic heuristic, adapted from Chen Chao
		// It's hard to obtain the Djikstra cost from all nodes/cells to each other node/cell in real time
		// the map is constantly changed, so we must adapt the current holonomic heuristic
		std::vector<astar::Circle> circle_path;

		// the next goal, updates the circle path heuristic
		astar::Pose2D goal;

        // PRIVATE METHODS

		// obstacle relaxed heuristic
		double GetObstacleRelaxedHeuristicValue(astar::Pose2D, const astar::Pose2D&);

		// nonholonomic relaxed heuristic
		double GetNonholonomicRelaxedHeuristicValue(const astar::InternalGridMapRef,const astar::Pose2D&, const astar::Pose2D&);

		// get the circle children


    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

		// basic constructor
		Heuristic();

        // update the heuristic around a new goal
        void UpdateCirclePathHeuristic(const astar::InternalGridMap& map, const astar::Pose2D&, const astar::Pose2D&);

        // get a heuristic value
        double GetHeuristicValue(const astar::InternalGridMapRef, const astar::Pose2D&, const astar::Pose2D&);

};

}

#endif
