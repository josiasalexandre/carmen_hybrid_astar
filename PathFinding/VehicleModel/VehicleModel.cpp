#include <limits>
#include <cmath>
#include "VehicleModel.hpp"

using namespace astar;

// get the next pose
State2D VehicleModel::NextState(
                                            const State2D &p,
                                            Steer s,
                                            Gear g,
                                            double length,
                                            double turn_radius
                                           )
{

    // auxiliary variables
    double x, y, angle;

    // no turning radius?
    if (RSStraight != s) {

        // get the angle between the start and endpoint
        angle = length/turn_radius;

        double angle2 = angle/2;

        double sin_angle_2 = (double) std::sin(angle2);

        double L = 2*sin_angle_2*turn_radius;

        // get the x coordinate
        x = L*((double) std::cos(angle2));

        // get the y coordinate
        y = L*sin_angle_2;

        if (RSTurnRight == s) {

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

    if (BackwardGear == g) {

        // change direction
        x = -x;
        angle = -angle;

    }

    // build a position vector at the relative position
    Vector2D<double> v(x, y);

    // rotate around z axis
    v.RotateZ(p.orientation);

    // rotate the point to the appropriated result
    return State2D(p.position + v, mrpt::math::wrapToPi<double>(p.orientation + angle));

}

// get the next pose
State2D VehicleModel::NextState(
                                            const State2D &p,
                                            Steer s,
                                            Gear g,
                                            double speed,
											double dt,
                                            double turn_radius
                                           )
{
	// get the length
	double length = speed*dt;

	return NextState(p, s, g, length, turn_radius);

}

// get the next pose, using the state steering
State2D VehicleModel::NextState(const astar::State2D& current_state) {

    // auxiliary variables
    double x, y, angle;

    // get the displacement
    double length = current_state.v*current_state.time;

    // no turning radius?
    if (0 != current_state.wheel_angle) {

        // get the turn radius
        double turn_radius = distance_between_front_and_rear_axles /
                std::tan(current_state.wheel_angle/(1.0 + current_state.v*current_state.v*understeer));

        angle = length/turn_radius;

        double angle2 = angle/2;

        double sin_angle_2 = (double) std::sin(angle2);

        double L = 2*sin_angle_2*turn_radius;

        // get the x coordinate
        x = L*((double) std::cos(angle2));

        // get the y coordinate
        y = L*sin_angle_2;

        if (0 > current_state.wheel_angle) {

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

    if (0 > current_state.v) {

        // change direction
        x = -x;
        angle = -angle;

    }

    // build a position vector at the relative position
    Vector2D<double> pose(x, y);

    // rotate around z axis
    pose.RotateZ(current_state.orientation);

    // rotate the point to the appropriated result
    return State2D(current_state.position + pose, mrpt::math::wrapToPi<double>(current_state.orientation + angle));

}

// get the desired wheel angle that connects two states
double VehicleModel::GetWheelAngle(const State2D &a, const State2D &b) {

    // move the second position to the first position reference
    Vector2D<double> goal(b.position - a.position);

    // rotate the goal point around the z axis
    goal.RotateZ(-a.orientation);

    // get the turn radius
    double turnRadius = (goal.x*goal.x + goal.y*goal.y)/(2.0*goal.y);

    // get the desired wheel angle
    return std::atan(distance_between_front_and_rear_axles/turnRadius);

}

// get the desired speed
double VehicleModel::GetDesiredSpeed(const State2D &prev, const State2D &current, const State2D &next) {

    // get the appropriated vectors
    Vector2D<double> current_v(current.position - prev.position);
    Vector2D<double> next_v(next.position - current.position);

    // set the max speed
    double max_speed = (astar::ForwardGear == current.gear) ? max_forward_speed : max_backward_speed;

    // get the angle between the two vectors
    double angle = std::fabs(mrpt::math::wrapToPi<double>((std::atan2(next_v.y, next_v.x) - std::atan2(current_v.y, current_v.x))));

    // get the turn radius
    double radius = current_v.Norm() / angle;

    // get the curvature constraint
    double curvature_constraint = std::sqrt(radius * max_lateral_acceleration);

    // return the minimum of curvature_constraint and max_speed
    return std::min(curvature_constraint, max_speed);


}

// get the desired orientation
double VehicleModel::GetDesiredOrientation(const astar::State2D &prev, const astar::State2D &current, const astar::State2D &next) {

    astar::Vector2D<double> displacement = 0.25 * (next.position - prev.position) + 0.75 * (next.position - current.position);
    double pathOrientation = std::atan2(displacement.y, displacement.x);

    if (current.gear == BackwardGear)
        pathOrientation = mrpt::math::wrapToPi<double>(pathOrientation + M_PI);

    return pathOrientation;

}
