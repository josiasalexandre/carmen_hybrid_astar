#include "CGSmoother.hpp"

// the main function to be minimized
double astar::CGSmoother::CostFunction(astar::StateArrayPtr input) {

	return 0;

}

astar::StateArrayPtr astar::CGSmoother::Smooth(astar::InternalGridMap &grid, astar::VehicleModel& vehicle, astar::StateArrayPtr raw_path) {

	// build the output
	StateArrayPtr smoothed_path = new StateArray();

	// the input array
	std::vector<State2D> &raw_states(raw_path->states);

	// the output array
	std::vector<State2D> &smoothed_states(smoothed_path->states);

	// copy the state vector
	unsigned int p_size = raw_states.size();

	smoothed_states.resize(p_size);

	std::cout << "\n";
	for(unsigned int i = 0; i < p_size; ++i) {

		std::cout << "i: " << i+1 << "(" << raw_states[i].position.x << "," << raw_states[i].position.y << ", " <<raw_states[i].orientation << ")\n";
		smoothed_states.push_back(raw_states[i]);
	}

	// copy the vectors
	//smoothed_path->states = raw_states;

	(void) grid;

	return smoothed_path;

}
