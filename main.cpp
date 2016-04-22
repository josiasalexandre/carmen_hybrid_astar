#include <iostream>
#include <cmath>
#include <string>

#include "Entities/Vector2D.hpp"
#include "Entities/Pose2D.hpp"
#include "PathFinding/ReedsShepp/ReedsSheppModel.hpp"

void showVector(astar::Vector2D &v, std::string name) {

    std::cout << std::endl << "Vetor " << name << ": (" << v.x << ", " << v.y << ")" << std::endl;

    return;
}

void showPose2D(astar::Pose2D &p) {

    std::cout << std::endl << "Pose";

    showVector(p.position, "");
    std::cout << "    Pose.orientation: " << p.orientation << std::endl;

}

int main() {

    // test basic constructor
    astar::Vector2D a(0, 0);
    // test copy constructor
    astar::Vector2D b(a);
    // test add single element
    b.add(1);
    // test add multiple elements
    b.add(-1, -1);
    // test add vector
    b.add(a);
    // test subtract
    b.subtract(1);
    // test subtract multiple elements
    b.subtract(-1, -1);
    // test subtract vector
    b.subtract(a);
    // test assignment operator
    astar::Vector2D c = b;
    // test translation
    b.translate(10);
    // test translation, multiple element
    b.translate(-10, -10);
    showVector(b, "b");
    // test translate vector
    b.translate(a);
    // test norm
    double n = b.norm();
    // test distance
    double d = b.distance(a + 1);
    std::cout << std::endl << "D: " << d << std::endl;
    // test scale
    b.scale(2);
    showVector(b, "b scale 2");
    // test scale, multiple elements
    b.scale(1, 1);
    showVector(b, "b scale 1 1");
    // test dot products
    double dot = b.dot(c);
    showVector(b, "b after dot");
    // distance between two vectors
    // test rotate
    astar::Vector2D rot(10, 10);

    astar::Vector2D rot1 = rot;

    rot.rotateZ(M_PI);
    rot.rotateZ(M_PI_2);
    rot.rotateZ(M_PI_2);
    rot.rotateZ(astar::Vector2D(9, 9), M_PI);
    rot.rotateZ(astar::Vector2D(9, 9), M_PI_2);
    rot.rotateZ(astar::Vector2D(9, 9), M_PI_2);
    if (rot == rot1) {

        std::cout << std::endl << "Ok, rotation" << std::endl;

    } else if (rot != rot1) {

        std::cout << std::endl << "Wrong rotation" << std::endl;

    }

    // test overloading operators
    showVector(rot, "ROT");
    astar::Vector2D sumV = rot + rot;
    showVector(rot, "ROT");
    showVector(sumV, "sum");
    astar::Vector2D sum2 = rot + rot*2;
    astar::Vector2D sub = rot - b;
    astar::Vector2D sub2 = rot - 2;
    showVector(rot, "ROT after rot -2");
    astar::Vector2D mult = (rot + 2)*2;
    astar::Vector2D mult2 = rot*rot;
    showVector(mult2, "mult2");
    astar::Vector2D div = rot/1.5;
    showVector(rot, "ROT after all");
    showVector(b, "b after all");

    std::cout << "POSES" << std::endl;

    astar::Pose2D p;
    showPose2D(p);
    astar::Pose2D p1(p.position.x + 3, p.position.y + 4, p.orientation + M_PI_4);
    showPose2D(p1);
    std::cout << std::endl << "Distante between poses: " << p.distance(p1) << std::endl;
    std::cout << std::endl << "Distante2 between poses: " << p.distance2(p1) << std::endl;

    std::cout << "RS MODEL" << std::endl;

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

    astar::Pose2D goal(x2, y2, t2);

    set = rs.solve(start, goal, inverse_unit);

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

    return 0;

}
