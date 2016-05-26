#include "State2D.hpp"

#include <cmath>

using namespace astar;

// simple constructor
State2D::State2D() : position(0.0, 0.0), orientation(0.0), wheel_angle(0.0), time(0.0), v(0.0), gear(ForwardGear){}

// basic pose constructor with Vector2D
State2D::State2D(const Vector2D<double> &pos, double o, double w_angle = 0.0, double vel = 0.0, double dt = 0.0, Gear g = ForwardGear) :
        position(pos), orientation(o), wheel_angle(w_angle), v(vel), time(dt), gear(g) {}


// explicit constructor
State2D::State2D(double x_, double y_, double o, double w_angle = 0.0, double vel = 0.0, double dt = 0.0, Gear g = ForwardGear) :
		position(x_, y_), orientation(o), wheel_angle(w_angle), v(vel), time(0), gear(ForwardGear) {}

// copy constructor
State2D::State2D(const State2D &s) :
        position(s.position), orientation(s.orientation), wheel_angle(s.wheel_angle), v(s.v), time(s.time), gear(s.gear) {}

// distance between two poses
double State2D::Distance(const State2D &p)
{
    return position.Distance(p.position);
}

// distance squared between two poses
// Euclidean distance
double State2D::Distance2(const State2D &p)
{
    return position.Distance2(p.position);
}

// get the minimum difference between two angles
double State2D::GetOrientationDiff(const State2D &p)
{
    return mrpt::math::angDistance<double>(orientation, p.orientation);
}

// get difference between two orientations (yaw), overloading
double State2D::GetOrientationDiff(double t)
{
    return mrpt::math::angDistance<double>(orientation, t);
}

// the assignment operator
void State2D::operator=(const State2D &p)
{
    position = p.position;
    orientation = p.orientation;
    wheel_angle = p.wheel_angle;
    v = p.v;
    gear = p.gear;
    timestamp = p.timestamp;
}

// == operator overloading
bool State2D::operator ==(const State2D &p)
{
    return std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001;
}

// != operator overloading
bool State2D::operator !=(const State2D &p)
{
    return !(std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001);
}

