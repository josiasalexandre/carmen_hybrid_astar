#include "Heuristic.hpp"
#include "../../Helpers/wrap2pi.hpp"

#include <cmath>

using namespace astar;

// basic constructor
astar::Heuristic::Heuristic() : info() {

	// load the external heuristic file
	NonholonomicHeuristicInfo::Load(info, "heuristic_info.txt");

}

// define the nonholonomic without obstacles heuristic
double Heuristic::GetObstacleRelaxedHeuristicValue(Pose2D start, const Pose2D &goal) {

	// rotate the start position around the goal
	start.position.RotateZ(goal.position, -goal.orientation);

	// get the angle displacement
	start.orientation = mrpt::math::angDistance<double>(start.orientation, goal.orientation);

	// get the indexes in the heuristic table
	int row = (int) std::floor((start.position.y + info.position_offset) / info.resolution + 0.5);
	int col = (int) std::floor((start.position.x + info.position_offset) / info.resolution + 0.5);
	int o = (int) std::floor(start.orientation / info.orientation_offset + 0.5);

	if (o == info.orientations) {
		o = 0;
	}

	if (0 > row || info.num_cells <= row || 0 > col || info.num_cells <= col) {

		// return the trivial euclidian norm
		return start.position.Distance(goal.position);

	}

	return info.heuristic[row][col][o];

}

// the holonomic with obstacles heuristic
double Heuristic::GetNonholonomicRelaxedHeuristicValue(const InternalGridMapRef grid, Pose2D start, const Pose2D &goal) {

	return 0;

}


void astar::Heuristic::UpdateGoal(const astar::InternalGridMap &grid, const astar::Pose2D &goal) {

	(void) grid;
	(void) goal;

}

// get a heuristic value
double astar::Heuristic::GetHeuristicValue(const InternalGridMapRef grid, const Pose2D &start, const Pose2D &goal) {

	// return the the combined heuristic
	return std::max(GetObstacleRelaxedHeuristicValue(start, goal), GetNonholonomicRelaxedHeuristicValue(grid, start, goal));

}
