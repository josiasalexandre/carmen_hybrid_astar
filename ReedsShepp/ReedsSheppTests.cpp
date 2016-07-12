#include <iostream>

#include "../Entities/Pose2D.hpp"
#include "ReedsSheppModel.hpp"


int main () {

    std::cout << "RS MODEL TESTS" << std::endl;

    astar::ReedsSheppModel rs;

    astar::ReedsSheppActionSetPtr set = nullptr;

    double inverse_unit = 1.0/(2.93/std::tan(0.43633));

    double x1 = 10;
    double y1 = 10;
    double t1 = 0;

    double x2 = 10;
    double y2 = 100;
    double t2 = M_PI_2;

    astar::Pose2D start(x1, y1, t1);
    std::cout << "\nStart: " << x1 << ", " << y1 << ", " << t1 << ".\n";

    astar::Pose2D goal(x2, y2, t2);
    std::cout << "Goal: " << x2 << ", " << y2 << ", " << t2 << ".\n";

    set = rs.Solve(start, goal, inverse_unit);

    if (nullptr != set) {

        std::cout << std::endl << "OK, set pronto" << std::endl;
        unsigned int s_size = set->actions.size();

        for (unsigned int i = 0; i < s_size; i++) {

            std::cout << "Gear: " << set->actions[i].gear << std::endl;
            std::cout << "Steer: " << set->actions[i].steer << std::endl;
            std::cout << "Length: " << set->actions[i].length/inverse_unit << std::endl;

        }

        std::cout << std::endl << "Total length: " << set->length/inverse_unit << std::endl;

    }

    delete(set);

	return 0;

}
