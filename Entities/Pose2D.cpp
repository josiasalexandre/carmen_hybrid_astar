#include "Pose2D.hpp"

#include <cmath>

using namespace astar;

// simple constructor
Pose2D::Pose2D() : position(0.0, 0.0), orientation(0.0), wheel_angle(0.0) {}

// basic constructor without wheel_angle
Pose2D::Pose2D(const Vector2D &p, double o) : position(p), orientation(o), wheel_angle(0.0) {}

// basic constructor without wheel_angle
Pose2D::Pose2D(const Vector2D &p, double o, double wheel_angle_) : position(p), orientation(o), wheel_angle(wheel_angle_) {}

// basic constructor without wheel_angle
Pose2D::Pose2D(double x_, double y_, double orientation_) : position(x_, y_), orientation(orientation_), wheel_angle(0.0) {}

// basic constructor with wheel_angle
Pose2D::Pose2D(double x_, double y_, double orientation_, double wheel_angle_) : position(x_, y_), orientation(orientation_), wheel_angle(wheel_angle_) {}

// copy constructor
Pose2D::Pose2D(const Pose2D &p) : position(p.position), orientation(p.orientation), wheel_angle(p.wheel_angle), v(p.v), gear(p.gear) {}

// distance between two poses
double
Pose2D::Distance(const Pose2D &p)
{
    return position.Distance(p.position);
}

// distance squared between two poses
// Euclidian distance
double
Pose2D::Distance2(const Pose2D &p)
{
    return position.Distance2(p.position);
}

// get the minimum difference between two angles
double
Pose2D::GetOrientationDiff(const Pose2D &p)
{
    return mrpt::math::angDistance<double>(orientation, p.orientation);
}

// get difference between two orientations (yaw), overloading
double
Pose2D::GetOrientationDiff(double t)
{
    return mrpt::math::angDistance<double>(orientation, t);
}

// the assignment operator
void
Pose2D::operator=(const Pose2D &p)
{
    position = p.position;
    orientation = p.orientation;
    wheel_angle = p.wheel_angle;
    v = p.v;
    gear = p.gear;
}

// == operator overloading
bool
Pose2D::operator ==(const Pose2D &p)
{
    return std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001;
}

// != operator overloading
bool
Pose2D::operator !=(const Pose2D &p)
{
    return !(std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001);
}

