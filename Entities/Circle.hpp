#ifndef CAR_BODY_CIRCLES_HPP
#define CAR_BODY_CIRCLES_HPP

#include <vector>

#include "Vector2D.hpp"

namespace astar {

class Circle {

    public:

        // the circle position
        astar::Vector2D<double> position;

        // the circle radius
        double r;

        // simplest constructor
        Circle();

        // basic constructor
        Circle(const astar::Vector2D<double>&, double);

        // most explicit constructor
        Circle(double x, double y, double radius);

        // copy constructor
        Circle(const Circle&);

};

// define the general circle references and pointers
typedef Circle* CirclePtr;
typedef Circle& CircleRef;

class CircleArray {

    public:

        // the circle list
        std::vector<Circle> circles;

};

typedef CircleArray* CircleArrayPtr;

}

#endif
