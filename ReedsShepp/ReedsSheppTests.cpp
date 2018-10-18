#include <iostream>

#include <exception>
#include "../Entities/Pose2D.hpp"
#include "ReedsSheppModel.hpp"


int main ()
{
    std::cout << "RS MODEL TESTS" << std::endl;

    astar::ReedsSheppModel rs;

    double pi = M_PI;
    double pi2 = 2*pi;
    double pi_2 = pi*0.5;

    astar::ReedsSheppActionSetPtr set = nullptr;

    double inverse_unit = 5.08;

    double x1 = 10;
    double y1 = 10;
    double t1 = pi_2;

    double x2 = 10;
    double y2 = 100;
    double t2 = pi_2;

    astar::Pose2D start(x1, y1, t1);
    std::cout << "\nStart: " << x1 << ", " << y1 << ", " << t1 << ".\n";

    astar::Pose2D goal(x2, y2, t2);
    std::cout << "Goal: " << x2 << ", " << y2 << ", " << t2 << ".\n";

    set = rs.Solve(start, goal, inverse_unit);

    if (nullptr != set)
    {
        std::cout << std::endl << "OK, set pronto" << std::endl;
        unsigned int s_size = set->actions.size();

        for (unsigned int i = 0; i < s_size; i++)
        {
            std::cout << "Gear: ";
            if (astar::BackwardGear == set->actions[i].gear)
            {
                std::cout << "Backward\n";
            }
            else
            {
                std::cout << "Forward\n";
            }
            
            std::cout << "Steer: " << set->actions[i].steer << "\n";
            std::cout << "Length: " << set->actions[i].length*inverse_unit << "\n";
        }

        std::cout << std::endl << "Total length: " << set->length*inverse_unit << std::endl;

    }
    else
    {
        throw "Invalid Action Set\n";
    }

    delete(set);

	return 0;
}
