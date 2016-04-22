#ifndef KINEMATIC_VEHICLE_MODEL_HPP
#define KINEMATIC_VEHICLE_MODEL_HPP

#include "../../Entities/Pose2D.hpp"
#include "../ReedsShepp/ReedsSheppAction.hpp"

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
        double maxPhi;

        // the axle distance
        double axleDist;

        // default speed
        double defaultSpeed;

        // default turn radius
        double defaultTurnRadius;

        // the car length dimension
        double length;

        // the car width dimension
        double width;

        // the sensivity to the understeer dynamic
        double understeer;

        // distance between rear wheels
        double rearWheelsDist;

        // the distance between front and rear axles
        double frontRearAxlesDist;

        // the distance between rear car and rear wheels
        double rearCarWheelsDist;

        // the distace between front car and front wheels
        double frontCarWheelsDist;

        // the maximum foward accelerarion
        double maxForwardAcceleration;

        // the maximum forward deceleration
        double maxForwardDeceleration;

        // the maximum backward accelerarion
        double maxBackwardAcceleration;

        // the maxium backward deceleration
        double maxBackwardDeceleration;

        // the desired steering command rate
        double steeringCommandRate;

        // the desired foward accelerarion
        double desiredForwardAcceleration;

        // the desired forward deceleration
        double desiredForwardDeceleration;

        // the desired backward deceleration
        double desiredBackwardDeceleration;

        // PUBLIC METHODS

        // get the next pose, using the default vehicle parameters
        astar::Pose2D nextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double, float *);

        // get the next pose, using the custom vehicle parameters
        astar::Pose2D nextPose(const astar::Pose2D&, astar::Steer, astar::Gear, double, double, double, float *);

};

}

#endif
