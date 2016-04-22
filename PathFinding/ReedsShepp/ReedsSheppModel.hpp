#ifndef REEDS_SHEPP_MODEL_HPP
#define REEDS_SHEPP_MODEL_HPP

#include "../../Entities/Pose2D.hpp"

#include "ReedsSheppActionSet.hpp"

namespace astar {

// the reeds sheep path path words

// Lf == left forward
// Sf == straight forward
// Rf == right forward

// Lb == left backward
// Sb == straight backward
// Rb == right backward

enum PathWords {

    // Reeds-Shepp 8.1: CSC, same turn
    LfSfLf,
    LbSbLb,
    RfSfRf,
    RbSbRb,

    // Reeds-Shepp 8.2: CSC, different turn
    LfSfRf,
    LbSbRb,
    RfSfLf,
    RbSbLb,

    // Reeds-Shepp 8.3: C|C|C
    LfRbLf,
    LbRfLb,
    RfLbRf,
    RbLfRb,

    // Reeds-Shepp 8.4: C|CC
    LfRbLb,
    LbRfLf,
    RfLbRb,
    RbLfRf,

    // Reeds-Shepp 8.4: CC|C
    LfRfLb,
    LbRbLf,
    RfLfRb,
    RbLbRf,

    // Reeds-Shepp 8.7: CCu|CuC
    LfRufLubRb,
    LbRubLufRf,
    RfLufRubLb,
    RbLubRufLf,

    // Reeds-Shepp 8.8: C|CuCu|C
    LfRubLubRf,
    LbRufLufRb,
    RfLubRubLf,
    RbLufRufLb,

    // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
    LfRbpi2SbLb,
    LbRfpi2SfLf,
    RfLbpi2SbRb,
    RbLfpi2SfRf,

    // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
    LfRbpi2SbRb,
    LbRfpi2SfRf,
    RfLbpi2SbLb,
    RbLfpi2SfLf,

    // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
    LfSfRfpi2Lb,
    LbSbRbpi2Lf,
    RfSfLfpi2Rb,
    RbSbLbpi2Rf,

    // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
    LfSfLfpi2Rb,
    LbSbLbpi2Rf,
    RfSfRfpi2Lb,
    RbSbRbpi2Lf,

    // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
    LfRbpi2SbLbpi2Rf,
    LbRfpi2SfLfpi2Rb,
    RfLbpi2SbRbpi2Lf,
    RbLfpi2SfRfpi2Lb

};

class ReedsSheppModel {

    private:

        // PRIVATE ATTRIBUTES
        const static unsigned int NumPathWords = 48;

        // PRIVATE METHODS

        // invalid angle test
        bool isInvalidAngle(double);

        // calculate the optimal path length
        double getPathLength(PathWords w, double, double, double, double, double, double&, double&, double&);

        // build the Reeds-Shepp path
        ReedsSheppActionSetPtr buildPath(PathWords w, double, double, double);

        // left forward, straight forward and left forward movement - get the path length
        double getLfSfLf(double, double, double, double, double, double&, double&, double&);

        // build the actual LfSfLf path based on given t, u and v
        ReedsSheppActionSetPtr getLfSfLfpath(double, double, double);

        // left forward, straight forward and right forward movement - get the path length
        double getLfSfRf(double, double, double, double, double, double&, double&, double&);

        // build the actual LfSfRf path based on given t, u and v
        ReedsSheppActionSetPtr getLfSfRfpath(double, double, double);

        // left forward, right backward and left forward movement - get the path length
        double getLfRbLf(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRbLf path based on given t, u and v
        ReedsSheppActionSetPtr getLfRbLfpath(double, double, double);

        // left forward, right backward and left backward movement - get the path length
        double getLfRbLb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRbLb path based on given t, u and v
        ReedsSheppActionSetPtr getLfRbLbpath(double, double, double);

        // left forward, right forward and left backward movement - get the path length
        double getLfRfLb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRfLb path based on given t, u and v
        ReedsSheppActionSetPtr getLfRfLbpath(double, double, double);

        // left forward, right forward, left backward and right forward movement - get the path length
        double getLfRufLubRb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRufLubRb path based on given t, u and v
        ReedsSheppActionSetPtr getLfRufLubRbpath(double, double, double);

        // left forward, right backward, left backward and right forward movement - get the path length
        double getLfRubLubRf(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRubLubRf path based on given t, u and v
        ReedsSheppActionSetPtr getLfRubLubRfpath(double, double, double);

        // left forward, right backward PI over 2, straight backward and left backward movement - get the path length
        double getLfRbpi2SbLb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRbpi2SbLb path based on given t, u and v
        ReedsSheppActionSetPtr getLfRbpi2SbLbpath(double, double, double);

        // left forward, right backward PI over 2, straight backward and right backward movement - get the path length
        double getLfRbpi2SbRb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRbpi2SbRb path based on given t, u and v
        ReedsSheppActionSetPtr getLfRbpi2SbRbpath(double, double, double);

        // left forward, straight forward, right forward PI over 2 and right backward movement - get the path length
        double getLfSfRfpi2Lb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfSfRfpi2Lb path based on given t, u and v
        ReedsSheppActionSetPtr getLfSfRfpi2Lbpath(double, double, double);

        // left forward, straight forward, left forward PI over 2 and right backward movement - get the path length
        double getLfSfLfpi2Rb(double, double, double, double, double, double&, double&, double&);

        // build the actual LfSfLfpi2Rb path based on given t, u and v
        ReedsSheppActionSetPtr getLfSfLfpi2Rbpath(double, double, double);

        // left forward, right backward PI over 2, straight backward, left backward PI over 2 and right forward movement - get the path length
        double getLfRbpi2SbLbpi2Rf(double, double, double, double, double, double&, double&, double&);

        // build the actual LfRbpi2SbLbpi2Rf path based on given t, u and v
        ReedsSheppActionSetPtr getLfRbpi2SbLbpi2Rfpath(double, double, double);

    public:

        // basic constructor
        ReedsSheppModel();

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // solve the current start to goal pathfinding
        astar::ReedsSheppActionSetPtr solve(const astar::Pose2D&, const astar::Pose2D&, double);

        // return a list of poses from a given action set
        std::vector<Pose2D> discretize(const Pose2D&, ReedsSheppActionSetPtr, double, double);

};

}

#endif
