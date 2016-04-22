#include "ReedsSheppModel.hpp"

#include <limits>

// local namespace
using namespace astar;

// basic constructor
ReedsSheppModel::ReedsSheppModel() {}


// PUBLIC METHODS

// solve the current start to goal pathfinding
ReedsSheppActionSetPtr ReedsSheppModel::solve(const Pose2D &start, const Pose2D &goal, double inverse_unit) {

    // Translate the goal so that the start position is at the origin
    Vector2D position((goal.position.x - start.position.x)*inverse_unit, (goal.position.y - start.position.y)*inverse_unit);

    // Rotate the goal so that the start orientation is 0
    position.rotateZ(-start.orientation);

    // get the angle difference
    double orientation = mrpt::math::wrapToPi<double>(goal.orientation - start.orientation);

    // the sin of the orientation
    double sinPhi = std::sin(orientation);

    // the cos of the orientation
    double cosPhi = std::cos(orientation);

    // assuming the infinity as the min length
    double bestPathLength = std::numeric_limits<double>::infinity();

    // maybe the next best path length, let's see
    double potentialLength;

    // define the length helpers
    double t = 0.0, u = 0.0, v = 0.0, t_, u_, v_;

    // what is the best path word?
    PathWords bestWord = static_cast<PathWords>(0), word;

    // iterating over the PathWords
    for (unsigned int w = 0; w < NumPathWords; w++) {

        // get the RS word
        word = static_cast<PathWords>(w);

        // maybe the new best path length, who knows?
        potentialLength = getPathLength(word, position.x, position.y, orientation, sinPhi, cosPhi, t_, u_, v_);

        if (potentialLength < bestPathLength) {

            bestPathLength = potentialLength;

            bestWord = word;

            t = t_;

            u = u_;

            v = v_;

        }

    }

    if (std::numeric_limits<double>::quiet_NaN() == bestPathLength) {

        return new ReedsSheppActionSet(std::numeric_limits<double>::infinity());

    }

    return buildPath(bestWord, t, u, v);

}

// return a list of poses from a given action set
std::vector<Pose2D> ReedsSheppModel::discretize(const Pose2D &start, ReedsSheppActionSetPtr actionSet, double radcurv, double inverse_resolution) {

    // get the prev pose
    Pose2D prev(start);

    // create the pose list
    std::vector<Pose2D> poses;

    // append the first pose
    poses.push_back(prev);

    // get the action size list size
    unsigned int a_size = actionSet->actions.size();

    if (0 < a_size) {

        // get the iterator
        for (std::vector<ReedsSheppAction>::iterator it = actionSet->actions.begin();  it < actionSet->actions.end(); ++it) {

            // subdivide the entire arc length by the grid resolution
            unsigned int n = std::ceil(it->length * radcurv * inverse_resolution);

            // is it a line path?
            if (RSStraight != it->steer) {

                // it's a curve
                // get the piece angle
                double pieceAngle = it->length/n;

                // get the current phi
                double phi = pieceAngle / 2;

                // avoiding sin repetitions
                double sinPhi = std::sin(phi);

                double L = 2*radcurv*sinPhi;

                // get the x displacement
                double dx = L*std::cos(phi);

                // get the y displacement
                double dy = L*sinPhi;

                // assuming TurnLeft, is it a TurnRight?
                if (RSTurnRight == it->steer) {

                    // invert the y displacement
                    dy = -dy;

                    // invert the pieceAngle
                    pieceAngle = -pieceAngle;

                }

                // assuming a ForwardGear, is it a backward instead?
                if (BackwardGear == it->gear) {

                    // invert the x displacement
                    dx = -dx;

                    // invert the pieceAngle
                    // if the pieceAngle was inverted becase the RSTurnRight
                    /// let's invert it again
                    pieceAngle = -pieceAngle;

                }

                // the resulting Pose, after the movement
                astar::Vector2D pos;

                // iterate over the entire arc length
                for (unsigned int i = 0; i < n; i++) {

                    // update the position
                    pos.x = dx;
                    pos.y = dy;

                    // rotate the position to the correct position
                    pos.rotateZ(-prev.orientation);

                    // update the prev Pose2D position
                    prev.position.add(pos);

                    // update the orientation
                    prev.orientation += pieceAngle;

                    // update the gear
                    prev.gear = it->gear;

                    // append to the pose list
                    poses.push_back(prev);

                }

            } else {

                // it's a straight line, so piece of cake
                // get the piece arch length
                double pieceLength = it->length * radcurv/n;

                // the x displacement
                double dx = pieceLength*std::cos(prev.orientation);

                // the y displacement
                double dy = pieceLength*std::sin(prev.orientation);

                // verify the direction
                if (BackwardGear == it->gear) {

                    // invert the displacements
                    dx = -dx;
                    dy = -dy;

                }

                // iterate over the entire arc and append the new pose
                for (unsigned int i = 0; i < n; i++) {

                    // update the new position
                    prev.position.add(dx, dy);

                    // get the gear action
                    prev.gear = it->gear;

                    // append to the poses list
                    poses.push_back(prev);

                }

            }

        }

    }

    // return the pose list
    return poses;
}

// PRIVATE METHODS
// invalid angle test
bool ReedsSheppModel::isInvalidAngle(double angle) {

    return angle < 0 || angle > M_PI;

}

// calculate the optimal path length
double ReedsSheppModel::getPathLength(PathWords w, double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // the current path length
    double len;

    switch (w) {

        // Reeds-Shepp 8.1: CSC, same turn
        case LfSfLf: { len = getLfSfLf(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbSbLb: { len = getLfSfLf(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfSfRf: { len = getLfSfLf(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbSbRb: { len = getLfSfLf(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.2: CSC, different turn
        case LfSfRf: { len = getLfSfRf(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbSbRb: { len = getLfSfRf(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfSfLf: { len = getLfSfRf(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbSbLb: { len = getLfSfRf(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.3: C|C|C
        case LfRbLf: { len = getLfRbLf(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRfLb: { len = getLfRbLf(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLbRf: { len = getLfRbLf(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLfRb: { len = getLfRbLf(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.4: C|CC
        case LfRbLb: { len = getLfRbLb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRfLf: { len = getLfRbLb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLbRb: { len = getLfRbLb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLfRf: { len = getLfRbLb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.4: CC|C
        case LfRfLb: { len = getLfRfLb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRbLf: { len = getLfRfLb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLfRb: { len = getLfRfLb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLbRf: { len = getLfRfLb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.7: CCu|CuC
        case LfRufLubRb: { len = getLfRufLubRb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRubLufRf: { len = getLfRufLubRb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLufRubLb: { len = getLfRufLubRb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLubRufLf: { len = getLfRufLubRb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.8: C|CuCu|C
        case LfRubLubRf: { len = getLfRubLubRf(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRufLufRb: { len = getLfRubLubRf(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLubRubLf: { len = getLfRubLubRf(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLufRufLb: { len = getLfRubLubRf(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
        case LfRbpi2SbLb: { len = getLfRbpi2SbLb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRfpi2SfLf: { len = getLfRbpi2SbLb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLbpi2SbRb: { len = getLfRbpi2SbLb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLfpi2SfRf: { len = getLfRbpi2SbLb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
        case LfRbpi2SbRb: { len = getLfRbpi2SbRb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRfpi2SfRf: { len = getLfRbpi2SbLb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLbpi2SbLb: { len = getLfRbpi2SbLb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLfpi2SfLf: { len = getLfRbpi2SbLb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
        case LfSfRfpi2Lb: { len = getLfSfRfpi2Lb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbSbRbpi2Lf: { len = getLfSfRfpi2Lb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfSfLfpi2Rb: { len = getLfSfRfpi2Lb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbSbLbpi2Rf: { len = getLfSfRfpi2Lb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
        case LfSfLfpi2Rb: { len = getLfSfLfpi2Rb(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbSbLbpi2Rf: { len = getLfSfLfpi2Rb(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfSfRfpi2Lb: { len = getLfSfLfpi2Rb(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbSbRbpi2Lf: { len = getLfSfLfpi2Rb(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}

        // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
        case LfRbpi2SbLbpi2Rf: { len = getLfRbpi2SbLbpi2Rf(goal_x, goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        case LbRfpi2SfLfpi2Rb: { len = getLfRbpi2SbLbpi2Rf(-goal_x, goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RfLbpi2SbRbpi2Lf: { len = getLfRbpi2SbLbpi2Rf(goal_x, -goal_y, -goal_orientation, -sinPhi, cosPhi, t, u, v); break;}
        case RbLfpi2SfRfpi2Lb: { len = getLfRbpi2SbLbpi2Rf(-goal_x, -goal_y, goal_orientation, sinPhi, cosPhi, t, u, v); break;}
        default:
            // set to the default value
            len = std::numeric_limits<double>::infinity();
            break;

    }

    // return the desired len
    return len;

}

// build the Reeds-Shepp path
ReedsSheppActionSetPtr ReedsSheppModel::buildPath(PathWords w, double t, double u, double v) {

    // build the correct path, based on the PathWords and path lengths
    switch(w) {

        // Reeds-Shepp 8.1: CSC, same turn
        case LfSfLf: return getLfSfLfpath(t, u, v);
        case LbSbLb: return ReedsSheppActionSet::timeFlip(getLfSfLfpath(t, u, v));
        case RfSfRf: return ReedsSheppActionSet::reflect(getLfSfLfpath(t, u, v));
        case RbSbRb: return ReedsSheppActionSet::timeFlipAndReflect(getLfSfLfpath(t, u, v));

        // Reeds-Shepp 8.2: CSC, different turn
        case LfSfRf: return getLfSfRfpath(t, u, v);
        case LbSbRb: return ReedsSheppActionSet::timeFlip(getLfSfRfpath(t, u, v));
        case RfSfLf: return ReedsSheppActionSet::reflect(getLfSfRfpath(t, u, v));
        case RbSbLb: return ReedsSheppActionSet::timeFlipAndReflect(getLfSfRfpath(t, u, v));

        // Reeds-Shepp 8.3: C|C|C
        case LfRbLf: return getLfRbLfpath(t, u, v);
        case LbRfLb: return ReedsSheppActionSet::timeFlip(getLfRbLfpath(t, u, v));
        case RfLbRf: return ReedsSheppActionSet::reflect(getLfRbLfpath(t, u, v));
        case RbLfRb: return ReedsSheppActionSet::timeFlipAndReflect(getLfRbLfpath(t, u, v));

        // Reeds-Shepp 8.4: C|CC
        case LfRbLb: return getLfRbLbpath(t, u, v);
        case LbRfLf: return ReedsSheppActionSet::timeFlip(getLfRbLbpath(t, u, v));
        case RfLbRb: return ReedsSheppActionSet::reflect(getLfRbLbpath(t, u, v));
        case RbLfRf: return ReedsSheppActionSet::timeFlipAndReflect(getLfRbLbpath(t, u, v));

        // Reeds-Shepp 8.4: CC|C
        case LfRfLb: return getLfRfLbpath(t, u, v);
        case LbRbLf: return ReedsSheppActionSet::timeFlip(getLfRfLbpath(t, u, v));
        case RfLfRb: return ReedsSheppActionSet::reflect(getLfRfLbpath(t, u, v));
        case RbLbRf: return ReedsSheppActionSet::timeFlipAndReflect(getLfRfLbpath(t, u, v));

        // Reeds-Shepp 8.7: CCu|CuC
        case LfRufLubRb: return getLfRufLubRbpath(t, u, v);
        case LbRubLufRf: return ReedsSheppActionSet::timeFlip(getLfRufLubRbpath(t, u, v));
        case RfLufRubLb: return ReedsSheppActionSet::reflect(getLfRufLubRbpath(t, u, v));
        case RbLubRufLf: return ReedsSheppActionSet::timeFlipAndReflect(getLfRufLubRbpath(t, u, v));

        // Reeds-Shepp 8.8: C|CuCu|C
        case LfRubLubRf: return getLfRubLubRfpath(t, u, v);
        case LbRufLufRb: return ReedsSheppActionSet::timeFlip(getLfRubLubRfpath(t, u, v));
        case RfLubRubLf: return ReedsSheppActionSet::reflect(getLfRubLubRfpath(t, u, v));
        case RbLufRufLb: return ReedsSheppActionSet::timeFlipAndReflect(getLfRubLubRfpath(t, u, v));

        // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
        case LfRbpi2SbLb: return getLfRbpi2SbLbpath(t, u, v);
        case LbRfpi2SfLf: return ReedsSheppActionSet::timeFlip(getLfRbpi2SbLbpath(t, u, v));
        case RfLbpi2SbRb: return ReedsSheppActionSet::reflect(getLfRbpi2SbLbpath(t, u, v));
        case RbLfpi2SfRf: return ReedsSheppActionSet::timeFlipAndReflect(getLfRbpi2SbLbpath(t, u, v));

        // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
        case LfRbpi2SbRb: return getLfRbpi2SbRbpath(t, u, v);
        case LbRfpi2SfRf: return ReedsSheppActionSet::timeFlip(getLfRbpi2SbRbpath(t, u, v));
        case RfLbpi2SbLb: return ReedsSheppActionSet::reflect(getLfRbpi2SbRbpath(t, u, v));
        case RbLfpi2SfLf: return ReedsSheppActionSet::timeFlipAndReflect(getLfRbpi2SbRbpath(t, u, v));

        // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
        case LfSfRfpi2Lb: return getLfSfRfpi2Lbpath(t, u, v);
        case LbSbRbpi2Lf: return ReedsSheppActionSet::timeFlip(getLfSfRfpi2Lbpath(t, u, v));
        case RfSfLfpi2Rb: return ReedsSheppActionSet::reflect(getLfSfRfpi2Lbpath(t, u, v));
        case RbSbLbpi2Rf: return ReedsSheppActionSet::timeFlipAndReflect(getLfSfRfpi2Lbpath(t, u, v));

        // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
        case LfSfLfpi2Rb: return getLfSfLfpi2Rbpath(t, u, v);
        case LbSbLbpi2Rf: return ReedsSheppActionSet::timeFlip(getLfSfLfpi2Rbpath(t, u, v));
        case RfSfRfpi2Lb: return ReedsSheppActionSet::reflect(getLfSfLfpi2Rbpath(t, u, v));
        case RbSbRbpi2Lf: return ReedsSheppActionSet::timeFlipAndReflect(getLfSfLfpi2Rbpath(t, u, v));

        // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
        case LfRbpi2SbLbpi2Rf: return getLfRbpi2SbLbpi2Rfpath(t, u, v);
        case LbRfpi2SfLfpi2Rb: return ReedsSheppActionSet::timeFlip(getLfRbpi2SbLbpi2Rfpath(t, u, v));
        case RfLbpi2SbRbpi2Lf: return ReedsSheppActionSet::reflect(getLfRbpi2SbLbpi2Rfpath(t, u, v));
        case RbLfpi2SfRfpi2Lb: return ReedsSheppActionSet::timeFlipAndReflect(getLfRbpi2SbLbpi2Rfpath(t, u, v));
        default:
            return new ReedsSheppActionSet(std::numeric_limits<double>::infinity());

    }

}

// left forward, straight forward and left forward movement - get the path length
double ReedsSheppModel::getLfSfLf(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.1
    // C_C_C

    // get the x and y parameters
    double x = goal_x - sinPhi;
    double y = goal_y - 1 + cosPhi;

    // update the t value
    t = std::atan2(y, x);

    // update the u value
    u = std::sqrt(x*x + y*y);

    // update the v value
    v = mrpt::math::wrapToPi<double>(goal_orientation - t);

    if (isInvalidAngle(t) || isInvalidAngle(v)) {

        return std::numeric_limits<double>::infinity();

    }

    return t + u + v;

}

// build the actual LfSfLf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfSfLfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions = new ReedsSheppActionSet();

    // add the appropriate gears and steering

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSStraight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, ForwardGear, v);

    return actions;

}

// left forward, straight forward and right forward movement - get the path length
double ReedsSheppModel::getLfSfRf(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.2

    // get the x and y parameters
    double x = goal_x + sinPhi;
    double y = goal_y - 1 - cosPhi;

    double u1squared = x * x + y * y;
    double t1 = std::atan2(y, x);

    if (4 > u1squared) {

        return std::numeric_limits<double>::infinity();

    }

    // update the u value
    u = std::sqrt(u1squared - 4);

    // get the phi value
    double phi = std::atan2(2.0, (u));

    // update the t value
    t = mrpt::math::wrapToPi<double>(t1 + phi);

    // update the v value
    v = mrpt::math::wrapToPi<double>(t - goal_orientation);

    if (isInvalidAngle(t) || isInvalidAngle(v)) {

        return std::numeric_limits<double>::infinity();

    }

    return t + u + v;

}

// build the actual LfSfRf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfSfRfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSStraight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnRight, ForwardGear, v);

    return actions;

}

// left forward, right backward and left forward movement - get the path length
double ReedsSheppModel::getLfRbLf(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.3
    // Uses a modified formula adapted from the c_c_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sinPhi;
    double eta = goal_y - 1 + cosPhi;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);
    double alpha = std::acos(u1 / 4);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + alpha + phi);

    // update the u value
    u = mrpt::math::wrapTo2Pi<double>(M_PI - 2 * alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(goal_orientation - t - u);

    if (isInvalidAngle(t) || isInvalidAngle(u) || isInvalidAngle(v)) {

        return std::numeric_limits<double>::infinity();

    }

    return t + u + v;
}

// build the actual LfRbLf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRbLfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, ForwardGear, v);

    return actions;

}

// left forward, right backward and left backward movement - get the path length
double ReedsSheppModel::getLfRbLb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.4
    // Uses a modified formula adapted from the c_cc function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sinPhi;
    double eta = goal_y - 1 + cosPhi;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);
    double alpha = std::acos(u1 / 4);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + alpha + phi);

    // update the u value
    u = mrpt::math::wrapTo2Pi<double>(M_PI - 2 * alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t + u - goal_orientation);

    return t + u + v;

}

// build the actual LfRbLb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRbLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right forward and left backward movement - get the path length
double ReedsSheppModel::getLfRfLb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {
    // Reeds-Shepp 8.4
    // Uses a modified formula adapted from the cc_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sinPhi;
    double eta = goal_y - 1 + cosPhi;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::acos((8.0 - u1 * u1) / 8.0);

    double va = std::sin(u);

    double alpha = std::asin(2 * va / u1);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 - alpha + phi);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - u - goal_orientation);

    return t + u + v;

}

// build the actual LfRfLb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRfLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right forward, left backward and right forward movement - get the path length
double ReedsSheppModel::getLfRufLubRb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.7
    // Uses a modified formula adapted from the ccu_cuc function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sinPhi;
    double eta = goal_y - 1 - cosPhi;

    double u1 = std::sqrt(x*x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    if (u1 > 2)
    {
        double alpha = std::acos(u1 / 4 - 0.5);

        // update the t value
        t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi - alpha);

        // update the u value
        u = mrpt::math::wrapTo2Pi<double>(M_PI - alpha);

        // update the v value
        v = mrpt::math::wrapTo2Pi<double>(goal_orientation - t + 2 * (u));

    } else {

        double alpha = std::acos(u1 / 4 + 0.5);

        // update the t value
        t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

        // update the u value
        u = mrpt::math::wrapTo2Pi<double>(alpha);

        // update the v value
        v = mrpt::math::wrapTo2Pi<double>(goal_orientation - t + 2 * (u));

    }

    return t + u + u + v;

}

// build the actual LfRufLubRb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRufLubRbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, BackwardGear, u);

    // the fourth action
    actions->addAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, right backward, left backward and right forward movement - get the path length
double ReedsSheppModel::getLfRubLubRf(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.8
    // Uses a modified formula adapted from the c_cucu_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sinPhi;
    double eta = goal_y - 1 - cosPhi;

    double u1 = std::sqrt(x * x + eta * eta);
    if (6 < u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);
    double va1 = 1.25f - u1 * u1 / 16;
    if (va1 < 0 || va1 > 1.0) {

        return std::numeric_limits<double>::infinity();

    }

    // update the u value
    u = std::acos(va1);
    double va2 = std::sin(u);
    double alpha = std::asin(2 * va2 / u1);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - goal_orientation);

    return t + u + u + v;

}

// build the actual LfRubLubRf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRubLubRfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, BackwardGear, u);

    // the fourth action
    actions->addAction(RSTurnRight, ForwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward and left backward movement - get the path length
double ReedsSheppModel::getLfRbpi2SbLb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.9
    // Uses a modified formula adapted from the c_c2sca function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sinPhi;
    double eta = goal_y - 1 + cosPhi;

    double u1squared = x * x + eta * eta;
    if (u1squared < 4) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 2;
    if (0 > u) {

        return std::numeric_limits<double>::infinity();

    }

    double alpha = std::atan2(2, (u) + 2);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t + M_PI_2 - goal_orientation);

    return t + M_PI_2 + u + v;
}

// build the actual LfRbpi2SbLb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRbpi2SbLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->addAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->addAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward and right backward movement - get the path length
double ReedsSheppModel::getLfRbpi2SbRb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.10
    // Uses a modified formula adapted from the c_c2scb function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sinPhi;
    double eta = goal_y - 1 - cosPhi;

    double u1 = std::sqrt(x*x + eta*eta);
    if (2.0 > u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi);

    // update the u value
    u = u1 - 2;

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(goal_orientation - t - M_PI_2);

    return t + M_PI_2 + u + v;

}

// build the actual LfRbpi2SbRb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRbpi2SbRbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->addAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->addAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, straight forward, right forward PI over 2 and right backward movement - get the path length
double ReedsSheppModel::getLfSfRfpi2Lb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.9 (reversed)
    // Uses a modified formula adapted from the csc2_ca function
//     // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sinPhi;
    double eta = goal_y - 1 + cosPhi;

    double u1squared = x*x + eta*eta;
    if (4 > u1squared) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 2;
    if (0 > (u)) {

        return std::numeric_limits<double>::infinity();

    }

    double alpha = std::atan2((u) + 2, 2);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi - alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - M_PI_2 - goal_orientation);

    return t + u + M_PI_2 + v;

}

// build the actual LfSfRfpi2Lb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfSfRfpi2Lbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSStraight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnRight, ForwardGear, M_PI_2);

    // the fourth action
    actions->addAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, straight forward, left forward PI over 2 and right backward movement - get the path length
double ReedsSheppModel::getLfSfLfpi2Rb(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.10 (reversed)
    // Uses a modified formula adapted from the csc2_cb function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sinPhi;
    double eta = goal_y - 1 - cosPhi;

    double u1 = std::sqrt(x*x + eta * eta);
    if (2.0 > u1) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(phi);

    // update the u value
    u = u1 - 2;

    // update the t value
    v = mrpt::math::wrapTo2Pi<double>(-t - M_PI_2 + goal_orientation);

    return t + u + M_PI_2 + v;

}

// build the actual LfSfLfpi2Rb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfSfLfpi2Rbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSStraight, ForwardGear, u);

    // the third action
    actions->addAction(RSTurnLeft, ForwardGear, M_PI_2);

    // the fourth action
    actions->addAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward, left backward PI over 2 and right forward movement - get the path length
double ReedsSheppModel::getLfRbpi2SbLbpi2Rf(double goal_x, double goal_y, double goal_orientation, double sinPhi, double cosPhi, double &t, double &u, double &v) {

    // Reeds-Shepp 8.11
    // Uses a modified formula adapted from the c_c2sc2_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sinPhi;
    double eta = goal_y - 1 - cosPhi;

    double u1squared = x * x + eta * eta;
    if (16 > u1squared) {

        return std::numeric_limits<double>::infinity();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 4;
    if (0 > (u)) {

        return std::numeric_limits<double>::infinity();

    }

    double alpha = std::atan2(2, (u) + 4);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - goal_orientation);

    return t + u + v + M_PI;

}

// build the actual LfRbpi2SbLbpi2Rf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::getLfRbpi2SbLbpi2Rfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->addAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->addAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->addAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->addAction(RSTurnLeft, BackwardGear, M_PI_2);

    // the fifth action
    actions->addAction(RSTurnRight, ForwardGear, v);

    return actions;

}
