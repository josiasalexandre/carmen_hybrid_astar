#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include "../Helpers/wrap2pi.hpp"
#include "Vector2D.hpp"
#include "../PathFinding/ReedsShepp/ReedsSheppAction.hpp"

namespace astar {

class Pose2D {

    public:

        // PUBLIC ATTRIBUTES

        // the position, x and y coordinates
        astar::Vector2D position;

        // the heading
        double orientation;

        // registering the wheel angle
        double wheelAngle;

        // the speed at the current pose
        double v;

        // the gear
        astar::Gear gear;

        // simple constructor
        Pose2D();

        // base pose constructor without wheelAngle
        Pose2D(const astar::Vector2D&, double);

        // base pose constructor with wheelAngle
        Pose2D(const astar::Vector2D&, double, double);

        // explicit constructor
        Pose2D(double, double, double);

        // basic constructor with wheelAngle
        Pose2D(double, double, double, double);

        // copy constructor
        Pose2D(const Pose2D&);

        // distance between two poses
        double distance(const Pose2D&);

        // distance squared between two poses
        double distance2(const Pose2D&);

        // get difference between two thetas (yaw)
        double get_orientation_diff(const Pose2D&);

        // get difference between two thetas (yaw), overloading
        double get_orientation_diff(double);

        // == operator overloading
        bool operator==(const Pose2D&);

        // != operator overloading
        bool operator!=(const Pose2D&);

};

}

#endif
