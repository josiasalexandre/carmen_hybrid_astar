#include <iostream>
#include "HeuristicCalculator.hpp"

int main ()  {

	astar::HeuristicCalculator::Calculate(60, 0.2, 80, 5.08);

	std::cout << "\nDone!\n";

	return 0;

}

