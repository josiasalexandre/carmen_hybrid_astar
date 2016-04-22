#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include "../Helpers/wrap2pi.hpp"
#include "Vector2D.hpp"

namespace astar {

class Pose2D {

    public:

        // PUBLIC ATTRIBUTES

        // the position, x and y coordinates
        astar::Vector2D position;

        // the heading
        double orientation;

        // simple constructor
        Pose2D();

        // base pose constructor
        Pose2D(const astar::Vector2D&, double);

        // explicit constructor
        Pose2D(double, double, double);

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
