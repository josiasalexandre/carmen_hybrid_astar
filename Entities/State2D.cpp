#include "State2D.hpp"

#include <cmath>

using namespace astar;

// simple constructor
State2D::State2D() :
        Pose2D::Pose2D(), gear(), v(0.0), phi(0.0), t(0.0), last_cusp_dist(0.0), coming_to_stop(false) {}

// simple constructor, the input is a pose
State2D::State2D(
    const Pose2D &p, Gear g,
    double vel, double wheel_angle,
    double time, double lcd, bool stop
        ) : Pose2D::Pose2D(p), gear(g), v(vel), phi(wheel_angle), t(time), last_cusp_dist(lcd), coming_to_stop(stop) {}

// copy constructor
State2D::State2D(const astar::State2D &s):
    Pose2D::Pose2D(s), gear(s.gear), v(s.v), phi(s.phi), t(s.t), last_cusp_dist(s.last_cusp_dist), coming_to_stop(s.coming_to_stop) {}


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
void State2D::operator=(const State2D &s)
{
    position = s.position;
    orientation = s.orientation;
    v = s.v;
    phi = s.phi;
    t = s.t;
    gear = s.gear;
    coming_to_stop = s.coming_to_stop;
    last_cusp_dist = s.last_cusp_dist;
}


// alternative assignement operator
void State2D::operator=(const Pose2D &p) {

    position = p.position;
    orientation = p.orientation;

}

// == operator overloading
bool State2D::operator==(const State2D &s)
{
    return (0.0001 > std::fabs(s.position.Distance2(position)) && 0.001 > std::fabs(s.orientation - orientation));
}

// != operator overloading
bool State2D::operator!=(const State2D &s)
{
    return (0.0001 < std::fabs(s.position.Distance2(position)) || 0.001 < std::fabs(s.orientation - orientation));
}

