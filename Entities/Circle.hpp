#ifndef CAR_BODY_CIRCLES_HPP
#define CAR_BODY_CIRCLES_HPP

#include "Vector2D.hpp"

namespace astar {

class Circle {

	public:

		// the circle position
		astar::Vector2D<double> position;

		// the circle radius
		double r;

		// basic constructor
		Circle(const astar::Vector2D<double>&, double);

		// copy constructor
		Circle(const Circle&);

};


}

#endif
