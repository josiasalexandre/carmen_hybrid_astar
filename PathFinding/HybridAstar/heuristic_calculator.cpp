#include <iostream>
#include "HeuristicCalculator.hpp"

int main ()  {

	astar::HeuristicCalculator::Calculate(150, 0.2, 80, 5.0);

	std::cout << "\nDone!\n";

	return 0;

}

