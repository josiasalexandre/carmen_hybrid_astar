#include <limits>
#include <cmath>
#include "VehicleModel.hpp"

using namespace astar;

// configure the current vehicle model
void VehicleModel::Configure() {

    // set the max lateral acceleration
    max_lateral_acceleration = 1.2;

    // set the vehicle low speed - 5 m/h -- See Urmson and Thrun
    low_speed = 2.2352;

    // get the min turn radius for 2.2352 m/s
    if (0 < max_wheel_deflection && 0 < understeer)
        min_turn_radius = axledist / std::tan(max_wheel_deflection/(1.0 + low_speed*low_speed*understeer));
    else if (0 < max_curvature)
        min_turn_radius = 1.0/max_curvature;
    else
        min_turn_radius = 5;

    if (0 == max_curvature) {

        max_curvature = 0.22;

    }

    // set the phi command limits
    max_phi_acceleration = 6.0;
    max_phi_velocity = 0.3;

    // set the time to change gears
    change_gear_time = 1.0;

    // the circle radius
    circle_radius = 1.5;

}

// get the next pose with Pose, Steer, Gear, length and custom turn radius
astar::Pose2D VehicleModel::NextPose(
    const astar::Pose2D &current_pose, Steer steer, Gear gear, double length) const {

    return NextPose(current_pose, steer, gear, length, min_turn_radius);

}

// get the next pose with Pose, Steer, Gear, length and turn radius
astar::Pose2D VehicleModel::NextPose(
    const astar::Pose2D &current_pose, Steer steer, Gear gear, double length, double t_radius) const
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

    // move the point to the appropriated result
    return Pose2D(position + current_pose.position, mrpt::math::wrapToPi<double>(current_pose.orientation + angle));

}

// get the next pose
Pose2D VehicleModel::NextPose(const astar::Pose2D &current, double vel, double phi, double time) const {

    // temp variables
    double x, y, angle;

    // get the absolute length
    double length = vel*time;

    if (0.0 != phi) {

        // get the turn radius
        double turn_radius = axledist / std::tan(phi/(1.0 + vel*vel*understeer));

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
    return Pose2D(position + current.position, mrpt::math::wrapToPi<double>(current.orientation + angle));

}

// get the next state
astar::State2D VehicleModel::NextState(const astar::State2D &current) const {

    // creates a new state based on the current input
    State2D s(current);

    // get the next position and orientation
    s = NextPose(s, s.v, s.phi, s.t);

    return s;
}

// get the front axle pose with respect to the rear axle pose
State2D VehicleModel::GetFrontAxleState(const astar::State2D &s) const {

    double x = s.position.x + std::cos(s.orientation) * axledist;
    double y = s.position.y + std::sin(s.orientation) * axledist;

    return State2D(Pose2D(x, y, s.orientation), s.gear, s.v, s.phi, s.t, s.last_cusp_dist, s.coming_to_stop);

}

// get the front axle pose with respect to the rear axle pose
State2D VehicleModel::GetFakeFrontAxleState(const astar::State2D &s) const {

    double x = s.position.x - std::cos(s.orientation) * axledist;
    double y = s.position.y - std::sin(s.orientation) * axledist;

    return State2D(Pose2D(x, y, s.orientation), s.gear, s.v, s.phi, s.t, s.last_cusp_dist, s.coming_to_stop);

}

// get the car center position
Pose2D VehicleModel::GetCenterPosition(const astar::Pose2D &pose) const {

    Pose2D front;

    front.position.x = pose.position.x + std::cos(pose.orientation) * axledist * 0.5;
    front.position.y = pose.position.y + std::sin(pose.orientation) * axledist * 0.5;

    return front;

}

// get the list of circles that represents the safe area
std::vector<Circle> VehicleModel::GetVehicleBodyCircles(const astar::Vector2D<double> &p, double orientation) {

    // the output array
    std::vector<Circle> body;

    // TODO get the values in a dynamic manner
    // the circle radius = 1.25 m / resolution
    double x_position[4] = {-0.36, 0.760, 1.880, 3.00};

    for (unsigned int i = 0; i < 4; ++i) {

        // set the current position
        astar::Vector2D<double> position(x_position[i], 0);

        // rotate the current position, follow the car heading
        position.RotateZ(orientation);

        // move the current position to the car frame
        position.Add(p);

        // append the the circle to the body vector
        body.push_back(astar::Circle(position, circle_radius));

    }

    return body;
}

// get the list of circles that represents the safe area
std::vector<Circle> VehicleModel::GetVehicleBodyCircles(const astar::Pose2D &p) {

    // the output array
    std::vector<Circle> body;

    // TODO move to the class and get the values in a dynamic manner
    double x_position[4] = {-0.36, 0.760, 1.880, 3.00};

    for (unsigned int i = 0; i < 4; ++i) {

        // set the current position
        astar::Vector2D<double> position(x_position[i], 0);

        // rotate the current position, follow the car heading
        position.RotateZ(p.orientation);

        // move the current position to the car frame
        position.Add(p.position);

        // append the the circle to the body vector
        body.push_back(astar::Circle(position, circle_radius));

    }

    return body;
}

// get the desired wheel angle that connects two states
double VehicleModel::GetDesiredWheelAngle(const Pose2D &a, const Pose2D &b) const {

    // move the second position to the first position reference
    Vector2D<double> goal(b.position.x - a.position.x, b.position.y - a.position.y);

    // rotate the goal point around the z axis
    goal.RotateZ(-a.orientation);

    // get the turn radius
    double turnRadius = std::min((goal.x*goal.x + goal.y*goal.y)/(2.0*goal.y), min_turn_radius);

    // get the desired wheel angle
    return std::atan(axledist/turnRadius);

}

// get the desired speed
double VehicleModel::GetCurvatureConstraint(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    // get the appropriated displacement vectors
    Vector2D<double> dxi(current.position.x - prev.position.x, current.position.y - prev.position.y);
    Vector2D<double> dxip1(next.position.x - current.position.x, next.position.y - current.position.y);

    // get the angle between the two vectors
    double angle = std::fabs(mrpt::math::wrapToPi<double>(std::atan2(dxip1.y, dxip1.x) - std::atan2(dxi.y, dxi.x)));

    // get the turn radius
    double radius = dxi.Norm() / angle;

    // get the curvature constraint
    return std::sqrt(radius * max_lateral_acceleration);

}

// get the desired forward speed
double VehicleModel::GetForwardSpeed(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    return std::min(low_speed, GetCurvatureConstraint(prev, current, next));

}

// get the desired backward speed
double VehicleModel::GetBackwardSpeed(const Pose2D &prev, const Pose2D &current, const Pose2D &next) const {

    return std::min(low_speed, GetCurvatureConstraint(prev, current, next));

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

    astar::Vector2D<double> next_minus_prev(next.position.x - prev.position.x, next.position.y - prev.position.y);
    astar::Vector2D<double> next_minus_current(next.position.x - current.position.x, next.position.y - current.position.y);

    astar::Vector2D<double> displacement(next_minus_prev * 0.25 + next_minus_current * 0.75);

    return std::atan2(displacement.y, displacement.x);

}

// get the desired orientation
double VehicleModel::GetBackwardOrientation(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const {

    return mrpt::math::wrapToPi<double>(GetForwardOrientation(prev, current, next) + M_PI);

}
