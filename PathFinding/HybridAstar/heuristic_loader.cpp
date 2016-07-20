#include <iostream>
#include "NonholonomicHeuristicInfo.hpp"
int main() {

	// the non holonomic heuristic info loads the file
	astar::NonholonomicHeuristicInfo info;

	astar::NonholonomicHeuristicInfo::Load(info, "heuristic_info.txt");

	// show the first heuristic value
	std::cout << "First: " << info.heuristic[0][0][0] << "\n";
	std::cout << "Second: " << info.heuristic[info.num_cells - 1][info.num_cells - 1][info.orientations - 1] << "\n";

	std::cout << "\n?\n";

}
