#include <limits>
#include <cmath>
#include "VehicleModel.hpp"

using namespace astar;

// get the next pose with Pose, Steer, Gear, length and turn radius
astar::Pose2D VehicleModel::NextPose(
    const astar::Pose2D &current_pose, Steer steer, Gear gear, double length, double t_radius = min_turn_radius) const
{
    // auxiliary variables
    double x, y, angle;

    // no turning radius?
    if (astar::RSStraight != steer) {

        angle = length/t_radius;

        double angle2 = angle/2;

        double sin_angle_2 = std::sin(angle2);

        double L = 2*sin_angle_2*t_radius;

        // get the x coordinate
        x = L*std::cos(angle2);

        // get the y coordinate
        y = L*sin_angle_2;

        if (astar::RSTurnRight == steer) {

            // change direction
            y = -y;
            angle = -angle;

        }

    } else {

        // just a straight movement, no turning radius
        x = length;
        y = 0.0;
        angle = 0.0;

    }

    if (astar::BackwardGear == gear) {

        // change direction
        x = -x;
        angle = -angle;

    }

    // build a position vector at the relative position
    Vector2D<double> position(x, y);

    // rotate around z axis
    position.RotateZ(current_pose.orientation);

    // rotate the point to the appropriated result
    return Pose2D(current_pose.position + position, mrpt::math::wrapToPi<double>(current_pose.orientation + angle));

}

// get the next pose
Pose2D VehicleModel::NextPose(const astar::Pose2D &current, double vel, double phi, double time) const {

    // temp variables
    double x, y, angle;

    // get the absolute length
    double length = vel*time;

    if (0.0 != phi) {

        // get the turn radius
        double turn_radius = distance_between_front_and_rear_axles /
                        std::tan(phi/(1.0 + vel*vel*understeer));

        angle = length/turn_radius;

        double angle2 = angle/2;

        double sin_angle_2 = std::sin(angle2);

        double L = 2*sin_angle_2*turn_radius;

        // get the x coordinate
        x = L*std::cos(angle2);

        // get the y coordinate
        y = L*sin_angle_2;

    } else {

        x = length;
        y = 0.0;
        angle = 0.0;

    }

    // build a position vector at the relative position
    Vector2D<double> position(x, y);

    // rotate around z axis
    position.RotateZ(current.orientation);

    // rotate the point to the appropriated result
    return Pose2D(current.position + position, mrpt::math::wrapToPi<double>(current.orientation + angle));

}

// get the next state
astar::State2D VehicleModel::NextState(const astar::State2D &current) {

    // creates a new state based on the current input
    State2D s(current);

    // get the next position and orientation
    s = NextPose(s, s.v, s.phi, s.t);

    return s;
}

// get the front axle pose with respect to the rear axle pose
State2D VehicleModel::GetFrontAxleState(const astar::State2D &s) const {

    double x = s.position.x + std::cos(s.orientation) * distance_between_front_and_rear_axles;
    double y = s.position.y + std::sin(s.orientation) * distance_between_front_and_rear_axles;

    return State2D(x, y, s.orientation, s.phi, s.v, s.gear, s.t, s.coming_to_stop, s.last_cusp_dist);

}

// get the front axle pose with respect to the rear axle pose
State2D VehicleModel::GetFakeFrontAxleState(const astar::State2D &s) const {

    double x = s.position.x - std::cos(s.orientation) * distance_between_front_and_rear_axles;
    double y = s.position.y - std::sin(s.orientation) * distance_between_front_and_rear_axles;

    return State2D(x, y, s.orientation, s.phi, s.v, s.gear, s.t, s.coming_to_stop, s.last_cusp_dist);

}

// get the desired wheel angle that connects two states
double VehicleModel::GetDesiredWheelAngle(const Pose2D &a, const Pose2D &b) const {

    // move the second position to the first position reference
    Vector2D<double> goal(b.position - a.position);

    // rotate the goal point around the z axis
    goal.RotateZ(-a.orientation);

    // get the turn radius
    double turnRadius = std::min((goal.x*goal.x + goal.y*goal.y)/(2.0*goal.y), min_turn_radius);

    // get the desired wheel angle
    return std::atan(distance_between_front_and_rear_axles/turnRadius);

}

// get the desired speed
double VehicleModel::GetCurvatureConstraint(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    // get the appropriated vectors
    Vector2D<double> current_v(current.position - prev.position);
    Vector2D<double> next_v(next.position - current.position);

    // get the angle between the two vectors
    double angle = std::fabs(mrpt::math::wrapToPi<double>((std::atan2(next_v.y, next_v.x) - std::atan2(current_v.y, current_v.x))));

    // get the turn radius
    double radius = current_v.Norm() / angle;

    // get the curvature constraint
    return std::sqrt(radius * max_lateral_acceleration);

}

// get the desired forward speed
double VehicleModel::GetForwardSpeed(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    return std::min(max_forward_speed, GetCurvatureConstraint(prev, current, next));

}

// get the desired backward speed
double VehicleModel::GetBackwardSpeed(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    return std::min(max_backward_speed, GetCurvatureConstraint(prev, current, next));

}

// get the acceleration constraint
double VehicleModel::GetAccelerationConstraint(double initial_speed, double displacement, astar::Gear g) const {

    if (astar::ForwardGear == g) {

        return std::sqrt(initial_speed * initial_speed + 2 * max_forward_acceleration * displacement);

    } else {

        return std::sqrt(initial_speed * initial_speed + 2 * max_backward_acceleration * displacement);

    }

}

// get the acceleration constraint
double VehicleModel::GetDecelerationConstraint(double final_speed, double displacement, astar::Gear g) const {

    if (astar::ForwardGear == g) {

        return std::sqrt(final_speed * final_speed + 2 * max_forward_deceleration * displacement);

    } else {

        return std::sqrt(final_speed * final_speed + 2 * max_backward_deceleration * displacement);

    }

}

// get the desired orientation
double VehicleModel::GetForwardOrientation(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const {

    astar::Vector2D<double> displacement(0.25 * (next.position - prev.position) + 0.75 * (next.position - current.position));

    return std::atan2(displacement.y, displacement.x);

}

// get the desired orientation
double VehicleModel::GetBackwardOrientation(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const {

    return mrpt::math::wrapToPi<double>(GetForwardOrientation(prev, current, next) + M_PI);

}
