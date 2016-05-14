#ifndef KINEMATIC_VEHICLE_MODEL_HPP
#define KINEMATIC_VEHICLE_MODEL_HPP

#include "../../Entities/Pose2D.hpp"
#include "../ReedsShepp/ReedsSheppAction.hpp"
#include "../HybridAstar/HybridAstarNode.hpp"

namespace astar {

// defining the steering values

class VehicleModel {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // max wheel angle
        // the phi
        double maxphi;

        // the desired steering command rate
        double steering_command_rate;

        // the sensivity to the understeer dynamic
        double understeer;

        // default turn radius, it
        double max_turn_radius;

        // default speed to simulation purpose
        double default_speed;

        // max allowed forward speed
        double max_forward_speed;

        // max allowed backward speed
        double max_backward_speed;

        // the car length dimension
        double length;

        // the car width dimension
        double width;

        // the axle distance
        double axledist;

        // distance between rear wheels
        double rear_wheels_dist;

        // the distance between rear car and rear wheels
        double rear_car_wheels_dist;

        // the distace between front car and front wheels
        double front_car_wheels_dist;

        // the maximum foward accelerarion
        double max_forward_acceleration;

        // the maximum forward deceleration
        double max_forward_deceleration;

        // the maximum backward accelerarion
        double max_backward_acceleration;

        // the maxium backward deceleration
        double max_backward_deceleration;

        // the desired foward accelerarion
        double desired_forward_acceleration;

        // the desired forward deceleration
        double desired_forward_deceleration;

        // the desired backward acceleration
        double desired_backward_acceleration;

        // the desired backward deceleration
        double desired_backward_deceleration;

        // PUBLIC METHODS
        // get the next pose, using the custom vehicle parameters
        astar::Pose2D NextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double, double, double);

};

}

#endif
