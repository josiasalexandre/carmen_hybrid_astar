#include "Circle.hpp"

using namespace astar;

// basic constructor
Circle::Circle(const astar::Vector2D<double> &p, double _r) : position(p), r(_r) {}

// copy constructor
Circle::Circle(const Circle &c) : position(c.position), r(c.r) {}
