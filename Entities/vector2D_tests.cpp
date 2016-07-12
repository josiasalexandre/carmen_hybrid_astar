#include <iostream>
#include <cmath>
#include <string>

#include "Vector2D.hpp"
#include "Pose2D.hpp"
#include "../ReedsShepp/ReedsSheppModel.hpp"

void showVector(astar::Vector2D<double> &v, std::string name) {

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
	std::cout << "\nTesting the Vector2D basic constructor ...\n";
    astar::Vector2D<double> a(0, 0);
	std::cout << "Done!";

	// test copy constructor
	std::cout << "\nTesting the Vector2D copy constructor ...\n";
    astar::Vector2D<double> b(a);
	std::cout << "Done!";

	// test add single element
	std::cout << "\nAdd a simple scalar b.Add(1)\n";
    b.Add(1);
    showVector(b, "b");
    std::cout << "Done!\n";

    // test add multiple elements
    std::cout << "\nTest add multiple (-1, -1), should be (0,0)\n";
    b.Add(-1, -1);
    showVector(b, "b");
    std::cout << "Done!\n";

    // test add vector
    std::cout << "\nTest Add vectors Add(a), should be (0,0)\n";
    b.Add(a);
    showVector(b, "b");
    std::cout << "Done!\n";

    // test subtract
    std::cout << "\nTest subtract vectors Subtract(1), should be (-1,-1)\n";
    b.Subtract(1);
    showVector(b, "b");
    std::cout << "Done!\n";

    // test subtract multiple elements
    b.Subtract(-1, -1);

    // test subtract vector
    b.Subtract(a);

    // test assignment operator
    astar::Vector2D<double> c = b;

    // test translation
    b.Translate(10);

    // test translation, multiple element
    b.Translate(-10, -10);
    showVector(b, "b");

    // test translate vector
    b.Translate(a);

    // test norm
    double n = b.Norm();

    // test distance
    double d = b.Distance(a + 1);
    std::cout << std::endl << "D: " << d << std::endl;

    // test scale
    b.Scale(2);
    showVector(b, "b scale 2");

    // test scale, multiple elements
    b.Scale(1, 1);
    showVector(b, "b scale 1 1");

    // test dot products
    double dot = b.Dot(c);
    showVector(b, "b after dot");

    // distance between two vectors
    // test rotate
    astar::Vector2D<double> rot(10, 10);

    astar::Vector2D<double> rot1 = rot;

    rot.RotateZ(M_PI);
    rot.RotateZ(M_PI_2);
    rot.RotateZ(M_PI_2);
    rot.RotateZ(astar::Vector2D<double>(9, 9), M_PI);
    rot.RotateZ(astar::Vector2D<double>(9, 9), M_PI_2);
    rot.RotateZ(astar::Vector2D<double>(9, 9), M_PI_2);

    // test rotation and ==, != operators overloading
    if (rot == rot1) {

        std::cout << std::endl << "Ok, rotation" << std::endl;

    } else if (rot != rot1) {

        std::cout << std::endl << "Wrong rotation" << std::endl;

    }

    // test overloading operators
    showVector(rot, "ROT");

    astar::Vector2D<double> sumV = rot + rot;
    showVector(rot, "ROT");

    showVector(sumV, "sum");

    astar::Vector2D<double> sum2 = rot + rot*2;
    astar::Vector2D<double> sub = rot - b;
    astar::Vector2D<double> sub2 = rot - 2;

    showVector(rot, "ROT after rot -2");

    astar::Vector2D<double> mult = (rot + 2)*2;
    astar::Vector2D<double> mult2 = rot*rot;

    showVector(mult2, "mult2");

    astar::Vector2D<double> div = rot/1.5;

    showVector(rot, "ROT after all");
    showVector(b, "b after all");

    std::cout << "POSES" << std::endl;

    astar::Pose2D p;
    showPose2D(p);

    astar::Pose2D p1(p.position.x + 3, p.position.y + 4, p.orientation + M_PI_4);

    showPose2D(p1);

    std::cout << std::endl << "Distante between poses: " << p.position.Distance(p1.position) << std::endl;
    std::cout << std::endl << "Distante2 between poses: " << p.position.Distance2(p1.position) << std::endl;

    return 0;

}
