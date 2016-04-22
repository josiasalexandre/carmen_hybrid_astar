#include "VehicleModel.hpp"

// get the next pose
astar::Pose2D astar::VehicleModel::nextPose(
                                            const astar::Pose2D &p,
                                            astar::Steer s,
                                            astar::Gear g,
                                            double dt,
                                            float &len
                                           )
{

    // use the default parameters
    return nextPose(p, s, g, defaultSpeed, dt, defaultTurnRadius, len);

}
// get the next pose, using the custom vehicle parameters
astar::Pose2D astar::VehicleModel::nextPose(
                                            const astar::Pose2D &p,
                                            astar::Steer s,
                                            astar::Gear g,
                                            double speed,
                                            double dt,
                                            double turnRadius, float &len
                                           )
{

    // update the length
    len = speed*dt;

    // auxiliar variables
    double x, y, phi;

    // no turning radius?
    if (astar::RSStraight == s) {

        // just a straight movement, no turning radius
        x = len;
        y = 0.0;
        phi = 0.0;

    } else {

        // get the wheel angle
        phi = len/turnRadius;

        if (maxPhi < phi) {

            phi = maxPhi;

        }

        double phi2 = phi/2;

        double sinPhi = (double) std::sin(phi2);

        double L = 2*sinPhi*turnRadius;

        // get the x coordinate
        x = L*((double) std::cos(phi2));

        // get the y coordinate
        y = L*sinPhi;

        if (astar::RSTurnRight == s) {

            // change direction
            y = -y;
            phi = -phi;

        }

    }


    if (astar::BackwardGear == g) {

        // change direction
        x = -x;
        phi = -phi;

    }

    // build a position vector at the relative position
    astar::Vector2D v(x, y);

    // rotate around z axis
    v.rotateZ(p.orientation);

    // rotate the point to the appropriated result
    return astar::Pose2D(p.position + v, mrpt::math::wrapToPi<double>(p.orientation + phi));

}

