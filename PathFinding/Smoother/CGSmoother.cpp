#include "CGSmoother.hpp"

astar::StateArrayPtr astar::CGSmoother::Smooth(astar::InternalGridMap &grid, astar::StateArrayPtr raw_path) {

	// build the output
	StateArrayPtr smoothed_path;

	// copy the state vector
	smoothed_path->states = raw_path->states;

	(void) grid;

	return smoothed_path;

}
