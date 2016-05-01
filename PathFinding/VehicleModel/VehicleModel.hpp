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

        // the axle distance
        double axledist;

        // default speed
        double default_speed;

        // default turn radius
        double default_turn_radius;

        // the car length dimension
        double length;

        // the car width dimension
        double width;

        // the sensivity to the understeer dynamic
        double understeer;

        // distance between rear wheels
        double rear_wheels_dist;

        // the distance between front and rear axles
        double front_rear_axles_dist;

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

        // the desired steering command rate
        double steering_command_rate;

        // the desired foward accelerarion
        double desired_forward_acceleration;

        // the desired forward deceleration
        double desired_forward_deceleration;

        // the desired backward deceleration
        double desired_backward_deceleration;

        // PUBLIC METHODS

        // get the next pose, using the custom vehicle parameters
        astar::Pose2D NextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double, double, double);

};

}

#endif
