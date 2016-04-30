#include "Pose2D.hpp"

#include <cmath>

using namespace astar;

// simple constructor
Pose2D::Pose2D() : position(0.0, 0.0), orientation(0.0), wheelAngle(0.0) {}

// basic constructor without wheelAngle
Pose2D::Pose2D(const Vector2D &p, double o) : position(p), orientation(o), wheelAngle(0.0) {}

// basic constructor without wheelAngle
Pose2D::Pose2D(const Vector2D &p, double o, double wheelAngle_) : position(p), orientation(o), wheelAngle(wheelAngle_) {}

// basic constructor without wheelAngle
Pose2D::Pose2D(double x_, double y_, double orientation_) : position(x_, y_), orientation(orientation_), wheelAngle(0.0) {}

// basic constructor with wheelAngle
Pose2D::Pose2D(double x_, double y_, double orientation_, double wheelAngle_) : position(x_, y_), orientation(orientation_), wheelAngle(wheelAngle_) {}

// copy constructor
Pose2D::Pose2D(const Pose2D &p) : position(p.position), orientation(p.orientation), wheelAngle(p.wheelAngle), v(p.v), gear(p.gear) {}

// distance between two poses
double Pose2D::distance(const Pose2D &p) {

    // Euclidian distance
    return position.distance(p.position);

}

// distance squared between two poses
double Pose2D::distance2(const Pose2D &p) {

    // Euclidian distance
    return position.distance2(p.position);

}

// get difference between two orientations (yaw)
double Pose2D::get_orientation_diff(const Pose2D &p) {

    // get the minimum difference between two angles
    return mrpt::math::angDistance<double>(orientation, p.orientation);

}

// get difference between two orientations (yaw), overloading
double Pose2D::get_orientation_diff(double t) {

    // get the minimum difference between two angles
    return mrpt::math::angDistance<double>(orientation, t);
}

// the assignment operator
void Pose2D::operator=(const Pose2D &p) {

    // copy all values
    // the position, x and y coordinates
    position = p.position;

    // the heading
    orientation = p.orientation;

    // registering the wheel angle
    wheelAngle = p.wheelAngle;

    // the speed at the current pose
    v = p.v;

    // the gear
    gear = p.gear;

    return;

}

// == operator overloading
bool Pose2D::operator ==(const Pose2D &p) {

    return std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001;

}

// != operator overloading
bool Pose2D::operator !=(const Pose2D &p) {

    return !(std::fabs(position.x - p.position.x) < 0.001 && std::fabs(position.y - p.position.y) < 0.001 && std::fabs(orientation - p.orientation) < 0.001);

}

