#include "VehicleModel.hpp"

// get the next pose
astar::Pose2D astar::VehicleModel::NextPose(
                                            const astar::Pose2D &p,
                                            astar::Steer s,
                                            astar::Gear g,
                                            double length,
                                            double turn_radius
                                           )
{

    // auxiliar variables
    double x, y, phi;

    // no turning radius?
    if (astar::RSStraight == s) {

        // just a straight movement, no turning radius
        x = length;
        y = 0.0;
        phi = 0.0;

    } else {

        // get the wheel angle
        phi = length/turn_radius;

        if (maxphi < phi) {

            phi = maxphi;

        }

        double phi2 = phi/2;

        double sinphi = (double) std::sin(phi2);

        double L = 2*sinphi*turn_radius;

        // get the x coordinate
        x = L*((double) std::cos(phi2));

        // get the y coordinate
        y = L*sinphi;

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
