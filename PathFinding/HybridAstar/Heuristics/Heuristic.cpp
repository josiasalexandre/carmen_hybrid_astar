#include "Heuristic.hpp"
#include "../../../Helpers/wrap2pi.hpp"
#include "../../../PriorityQueue/PriorityQueue.hpp"

#include <set>
#include <cmath>

using namespace astar;

// basic constructor
astar::Heuristic::Heuristic(InternalGridMapRef map) : info(), holonomic(map) {

	// load the external heuristic file
	NonholonomicHeuristicInfo::Load(info, "heuristic_info.txt");

}

// define the nonholonomic without obstacles heuristic
double Heuristic::GetObstacleRelaxedHeuristicValue(Pose2D start_, const Pose2D &goal_) {

	// rotate the start position around the goal
	start_.position.Subtract(goal_.position);
	start_.position.RotateZ(-goal_.orientation);

	// get the angle displacement
	start_.orientation = mrpt::math::angDistance<double>(start_.orientation, goal_.orientation);

	// get the indexes in the heuristic table
	int row = (int) std::floor((start_.position.y + info.position_offset) / info.resolution + 0.5);
	int col = (int) std::floor((start_.position.x + info.position_offset) / info.resolution + 0.5);
	int o = (int) std::floor(start_.orientation / info.orientation_offset + 0.5);

	if (o == info.orientations) {
		o = 0;
	}

	if (0 > row || info.num_cells <= row || 0 > col || info.num_cells <= col) {

		// return the trivial euclidian norm
		return start_.position.Distance(goal.position);

	}

	return info.heuristic[row][col][o];

}

// the holonomic with obstacles heuristic
double Heuristic::GetNonholonomicRelaxedHeuristicValue(const Pose2D &start, const Pose2D &goal) {

	// just a simple euclidian distance now
	// it's not ready
	return holonomic.GetHeuristicValue(start);

}

// find a new circel path connecting the start and goal poses
void astar::Heuristic::UpdateHeuristic(astar::InternalGridMap &grid, const Pose2D &start_, const Pose2D &goal_) {

	holonomic.UpdateHeuristic(grid, start_, goal_);

}

// get a heuristic value
double astar::Heuristic::GetHeuristicValue(const Pose2D &start_, const Pose2D &goal_) {

	return std::max(GetObstacleRelaxedHeuristicValue(start_, goal_), start_.position.Distance(goal_.position));

	// return the the combined heuristic
	return std::max(GetObstacleRelaxedHeuristicValue(start_, goal_), GetNonholonomicRelaxedHeuristicValue(start_, goal_));

}
