/*
// Author: Josias Alexandre Oliveira

// Based on the Matt Bradley's Masters Degree thesis and work
// Copyright (c) 2012 Matt Bradley
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef KINEMATIC_VEHICLE_MODEL_HPP
#define KINEMATIC_VEHICLE_MODEL_HPP

#include "../Entities/State2D.hpp"
#include "../ReedsShepp/ReedsSheppAction.hpp"
#include "../PathFinding/HybridAstar/HybridAstarNode.hpp"
#include "../Entities/Circle.hpp"

namespace astar {

// defining the steering values

class VehicleModel
{
    private:

        // PRIVATE ATTRIBUTES
        double circle_radius;

        // PRIVATE METHODS

    public:

        // the default low speed for simulation purpose
        double low_speed;

        // PUBLIC ATTRIBUTES
        // the minimum turn radius
        double min_turn_radius;

        // the maximum wheel deflection
        double max_wheel_deflection;

        // the desired steering command rate
        double steering_command_rate;

        // the sensitivity to the understeer dynamic
        double understeer;

        // default turn radius, it
        double max_curvature;

        // the max velocity
        double max_velocity;

        // max allowed forward speed
        double max_forward_speed;

        // max allowed backward speed
        double max_backward_speed;

        // the car length dimension
        double length;

        // the car width dimension
        double width, width_2;

        // the car safety facor
        double safety_factor;

        // the axle distance
        double axledist;

        // distance between rear wheels
        double rear_wheels_dist;

        // the distance between rear car and rear wheels
        double rear_car_wheels_dist;

        // the distance between front car and front wheels
        double front_car_wheels_dist;

        // the maximum forward acceleration
        double max_forward_acceleration;

        // the maximum forward deceleration
        double max_forward_deceleration;

        // the maximum backward acceleration
        double max_backward_acceleration;

        // the maximum backward deceleration
        double max_backward_deceleration;

        // the desired forward acceleration
        double desired_forward_acceleration;

        // the desired forward deceleration
        double desired_forward_deceleration;

        // the desired backward acceleration
        double desired_backward_acceleration;

        // the desired backward deceleration
        double desired_backward_deceleration;

        // the desired deceleration
        double desired_deceleration;

        // the max lateral acceleration
        double max_lateral_acceleration;

        // get the maximum steering command acceleration
        double max_phi_acceleration;

        // the maximum steering command speed
        double max_phi_velocity;

        // the time to change the gears
        double change_gear_time;

        // PUBLIC METHODS

        // Configure the current vehicle model
        void Configure();

        // get the next pose with Pose, Steer, Gear, length and custom turn radius
        astar::Pose2D NextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double length) const;

        // get the next pose with Pose, Steer, Gear, length and custom turn radius
        astar::Pose2D NextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double length, double radius) const;

        // get the next pose
        astar::Pose2D NextPose(const astar::Pose2D&, double vel, double phi, double time) const;

        // get the next state
        astar::State2D NextState(const astar::State2D&) const;

        // get the front axle state with respect to the rear axle pose
        astar::State2D GetFrontAxleState(const astar::State2D&) const;

        // get the fake front axle state with respect to the rear axle pose
        astar::State2D GetFakeFrontAxleState(const astar::State2D&) const;

        // get the car center position
        astar::Pose2D GetCenterPosition(const astar::Pose2D&) const;

        // get the list of circles that represents the safe area
        std::vector<astar::Circle> GetVehicleBodyCircles(const astar::Vector2D<double>&, double);

        // get the list of circles that represents the safe area
        // overloaded version
        std::vector<astar::Circle> GetVehicleBodyCircles(const astar::Pose2D&);

        // get the desired wheel angle that connects two states
        double GetDesiredWheelAngle(const astar::Pose2D&, const astar::Pose2D&) const;

        // get the desired speed
        double GetCurvatureConstraint(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const;

        // get the desired speed
        double GetForwardSpeed(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const;

        // get the desired speed
        double GetBackwardSpeed(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const;

        // get the max acceleration constraint
        double GetAccelerationConstraint(double initial_speed, double displacement, astar::Gear g) const;

        // get the max acceleration constraint
        double GetDecelerationConstraint(double final_speed, double displacement, astar::Gear g) const;

        // get the desired forward orientation
        double GetForwardOrientation(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const;

        // get the desired orientation
        double GetBackwardOrientation(const astar::Pose2D &prev, const astar::Pose2D &current, const astar::Pose2D &next) const;

        // get the
};

typedef VehicleModel& VehicleModelRef;
typedef VehicleModel* VehicleModelPtr;

}

#endif
