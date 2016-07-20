#ifndef COMBINED_HEURISTIC_HPP
#define COMBINED_HEURISTIC_HPP

#include "../../Entities/Pose2D.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "NonholonomicHeuristicInfo.hpp"

namespace astar {


class Heuristic {

    private:

        // PRIVATE ATTRIBUTES
		astar::NonholonomicHeuristicInfo info;

        // PRIVATE METHODS

		// obstacle relaxed heuristic
		double GetObstacleRelaxedHeuristicValue(astar::Pose2D, const astar::Pose2D&);

		// nonholonomic relaxed heuristic
		double GetNonholonomicRelaxedHeuristicValue(const astar::InternalGridMapRef,const astar::Pose2D&, const astar::Pose2D&);

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

		// basic constructor
		Heuristic();

        // update the heuristic around a new goal
        void UpdateGoal(const astar::InternalGridMap& map, const astar::Pose2D&);

        // get a heuristic value
        double GetHeuristicValue(const astar::InternalGridMapRef, const astar::Pose2D&, const astar::Pose2D&);

};

}

#endif
