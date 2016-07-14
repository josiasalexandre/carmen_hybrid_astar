#include "Heuristic.hpp"

using namespace astar;

void Heuristic::UpdateGoal(const astar::InternalGridMap &grid, const astar::Pose2D &goal) {

	(void) grid;
	(void) goal;

}

// get a heuristic value
double Heuristic::GetHeuristicValue(astar::InternalGridMap &grid, const astar::Pose2D &start, const astar::Pose2D &goal) {

	(void) grid;
	(void) start;
	(void) goal;

	return 0;

}
