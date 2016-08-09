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

#include "ReedsSheppModel.hpp"

#include <iostream>
#include <limits>
#include <cmath>

using namespace astar;

// PUBLIC METHODS

// solve the current start to goal pathfinding
ReedsSheppActionSetPtr ReedsSheppModel::Solve(const Pose2D &start, const Pose2D &goal, double unit) {

    // Translate the goal so that the start position is at the origin
    Vector2D<double> position((goal.position.x - start.position.x)/unit, (goal.position.y - start.position.y)/unit);

    // Rotate the goal so that the start orientation is 0
    position.RotateZ(-start.orientation);

    // get the angle difference
    double orientation = mrpt::math::angDistance<double>(goal.orientation, start.orientation);

    // the sin of the orientation
    double sin_orientation = std::sin(orientation);

    // the cos of the orientation
    double cos_orientation = std::cos(orientation);

    // assuming the infinity as the min length
    double bestPathLength = std::numeric_limits<double>::max();

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
        potentialLength = GetPathLength(word, position.x, position.y, orientation, sin_orientation, cos_orientation, t_, u_, v_);

        if (potentialLength < bestPathLength) {

            bestPathLength = potentialLength;

            bestWord = word;

            t = t_;

            u = u_;

            v = v_;

        }

    }

    if (std::numeric_limits<double>::max() == bestPathLength) {

        return new ReedsSheppActionSet(std::numeric_limits<double>::max());

    }

    return BuildPath(bestWord, t, u, v);

}

// return a list of poses from a given action set
StateArrayPtr ReedsSheppModel::DiscretizeRS(
        const Pose2D &start, ReedsSheppActionPtr action, double radcurv, double inverse_max_length)
{

    // create the pose array
    StateArrayPtr path = new StateArray();

    // get the previous pose
    State2D prev(start);

    // a reference helper
    std::vector<State2D> &states(path->states);

    // append the first pose
    states.push_back(State2D(prev, action->gear));

    // subdivide the entire arc length by the grid resolution
    unsigned int n = ceil(action->length * radcurv * inverse_max_length);

    // is it a line path?
    if (RSStraight != action->steer) {

        // it's a curve
        // get the piece angle
        double pieceAngle = action->length/((double) n);

        // get the current phi
        double phi = pieceAngle / 2;

        // avoiding sin repetitions
        double sinPhi = std::sin(phi);

        double L = 2 * radcurv * sinPhi;

        // get the x displacement
        double dx = L * std::cos(phi);

        // get the y displacement
        double dy = L * sinPhi;

        // assuming TurnLeft, is it a TurnRight?
        if (RSTurnRight == action->steer) {

            // invert the y displacement
            dy = -dy;

            // invert the pieceAngle
            pieceAngle = -pieceAngle;

        }

        // we have assumed ForwardGear, is it a backward instead?
        if (BackwardGear == action->gear) {

            // invert the x displacement
            dx = -dx;

            // invert the pieceAngle
            // if the pieceAngle was inverted because the RSTurnRight
            // let's invert it again
            pieceAngle = -pieceAngle;

        }

        // the resulting position, after the movement
        astar::Vector2D<double> pos;

        // iterate over the entire arc length
        for (unsigned int i = 0; i < n; ++i) {

            // update the position
            pos.x = dx;
            pos.y = dy;

            // rotate the position to the correct position
            pos.RotateZ(prev.orientation);

            // update the prev State2D position
            prev.position.Add(pos);

            // update the orientation
            prev.orientation = mrpt::math::wrapToPi<double>(prev.orientation + pieceAngle);

            // update the gear
            //prev.gear = action->gear;

            // append to the pose list
            states.push_back(prev);

        }

    } else {

        // it's a straight line, so piece of cake
        // get the piece arch length
        double pieceLength = action->length * radcurv / n;

        // the x displacement
        double dx = pieceLength * std::cos(prev.orientation);

        // the y displacement
        double dy = pieceLength * std::sin(prev.orientation);

        // verify the direction
        if (BackwardGear == action->gear) {

            // invert the displacements
            dx = -dx;
            dy = -dy;

        }

        // iterate over the entire arc and append the new pose
        for (unsigned int i = 0; i < n; ++i) {

            // update the new positio
            // prev.position.Add(dx, dy);

            // get the gear action
            prev.gear = action->gear;

            // append to the poses list
            states.push_back(prev);

        }

    }

    // return the pose list
    return path;

}

// return a list of poses from a given action set
StateArrayPtr ReedsSheppModel::DiscretizeRS(
        const Pose2D &start, ReedsSheppActionSetPtr action_set, double radcurv, double inverse_max_length)
{

    // create the pose array
    StateArrayPtr path = new StateArray();

    // get the action size list size
    unsigned int a_size = action_set->actions.size();

    if (0 < a_size) {

        // get the previous pose
        State2D prev(start);

        // a reference helper
        std::vector<State2D> &states(path->states);

        // append the first pose
        states.push_back(State2D(prev, action_set->actions[1].gear));

        // get the end pointer
        std::vector<ReedsSheppAction>::iterator end = action_set->actions.end();

        // avoiding a lot of computations
        double coefficient = radcurv * inverse_max_length;

        // get the iterator
        for (std::vector<ReedsSheppAction>::iterator it = action_set->actions.begin();  it != end; ++it) {

            // subdivide the entire arc length by the grid resolution
            unsigned int n = ceil(it->length);

            // is it a line path?
            if (RSStraight != it->steer) {

                // it's a curve
                // get the piece angle
                double pieceAngle = it->length/((double) n);

                // get the current phi
                double phi = pieceAngle / 2;

                // avoiding sin repetitions
                double sinPhi = std::sin(phi);

                double L = 2 * radcurv * sinPhi;

                // get the x displacement considering a constant rate
                double dx = L * std::cos(phi);

                // get the y displacement considering a constante rate
                double dy = L * sinPhi;

                // assuming TurnLeft, is it a TurnRight?
                if (RSTurnRight == it->steer) {

                    // invert the y displacement
                    dy = -dy;

                    // invert the pieceAngle
                    pieceAngle = -pieceAngle;

                }

                // we have assumed ForwardGear, is it a backward instead?
                if (BackwardGear == it->gear) {

                    // invert the x displacement
                    dx = -dx;

                    // invert the pieceAngle
                    // if the pieceAngle was inverted because the RSTurnRight
                    // let's invert it again
                    pieceAngle = -pieceAngle;

                }

                // the resulting position, after the movement
                astar::Vector2D<double> pos;

                // iterate over the entire arc length
                for (unsigned int i = 0; i < n; ++i) {

                    // update the position
                    pos.x = dx;
                    pos.y = dy;

                    // rotate the position to the correct position
                    pos.RotateZ(prev.orientation);

                    // update the prev State2D position
                    prev.position.Add(pos);

                    // update the orientation
                    prev.orientation = mrpt::math::wrapToPi<double>(prev.orientation + pieceAngle);

                    // update the gear
                    prev.gear = it->gear;

                    // append to the pose list
                    states.push_back(prev);

                }

            } else {

                // it's a straight line, so piece of cake
                // get the piece arch length
                double pieceLength = it->length * radcurv / n;

                // the x displacement
                double dx = pieceLength * std::cos(prev.orientation);

                // the y displacement
                double dy = pieceLength * std::sin(prev.orientation);

                // verify the direction
                if (BackwardGear == it->gear) {

                    // invert the displacements
                    dx = -dx;
                    dy = -dy;

                }

                // iterate over the entire arc and append the new pose
                for (unsigned int i = 0; i < n; ++i) {

                    // update the new position
                    prev.position.Add(dx, dy);

                    // get the gear action
                    prev.gear = it->gear;

                    // append to the poses list
                    states.push_back(prev);

                }

            }

        }

    }

    // return the pose list
    return path;

}

// return a list of poses from a given action set
StateArrayPtr ReedsSheppModel::Discretize_LR(
        const Pose2D &start, ReedsSheppActionSetPtr action_set, double radcurv, double inverse_max_length)
{

    // create the pose array
    StateArrayPtr path = new StateArray();

    // get the action size list size
    unsigned int a_size = action_set->actions.size();

    if (0 < a_size) {

        // get the previous pose
        State2D prev(start);

        // a reference helper
        std::vector<State2D> &states(path->states);

        // append the first pose
        states.push_back(State2D(prev, action_set->actions[1].gear));

        // get the end pointer
        std::vector<ReedsSheppAction>::iterator end = action_set->actions.end();

        // get the iterator
        for (std::vector<ReedsSheppAction>::iterator it = action_set->actions.begin();  it != end; ++it) {

            // is it a line path?
            if (RSStraight != it->steer) {

                // it's a curve
                // get the piece angle
                double pieceAngle = it->length;

                // get the current phi
                double phi = pieceAngle / 2;

                // avoiding sin repetitions
                double sinPhi = std::sin(phi);

                double L = 2 * radcurv * sinPhi;

                // get the x displacement
                double dx = L * std::cos(phi);

                // get the y displacement
                double dy = L * sinPhi;

                // assuming TurnLeft, is it a TurnRight?
                if (RSTurnRight == it->steer) {

                    // invert the y displacement
                    dy = -dy;

                    // invert the pieceAngle
                    pieceAngle = -pieceAngle;

                }

                // we have assumed ForwardGear, is it a backward instead?
                if (BackwardGear == it->gear) {

                    // invert the x displacement
                    dx = -dx;

                    // invert the pieceAngle
                    // if the pieceAngle was inverted because the RSTurnRight
                    // let's invert it again
                    pieceAngle = -pieceAngle;

                }

                // the resulting position, after the movement
                astar::Vector2D<double> pos(dx, dy);

                // rotate the position to the correct position
                pos.RotateZ(prev.orientation);

                // update the prev State2D position
                prev.position.Add(pos);

                // update the gear
                prev.gear = it->gear;

                // append to the pose list
                states.push_back(prev);

            } else {

                // it's a straight line, so piece of cake
                // get the piece arch length
                double pieceLength = it->length * radcurv;

                // the x displacement
                double dx = pieceLength * std::cos(prev.orientation);

                // the y displacement
                double dy = pieceLength * std::sin(prev.orientation);

                // verify the direction
                if (BackwardGear == it->gear) {

                    // invert the displacements
                    dx = -dx;
                    dy = -dy;

                }

                // update the new position
                prev.position.Add(dx, dy);

                // get the gear action
                prev.gear = it->gear;

                // append to the poses list
                states.push_back(prev);

            }

        }

    }

    // return the pose list
    return path;

}

// PRIVATE METHODS
// invalid angle test
bool ReedsSheppModel::isInvalidAngle(double angle) {

    return angle < 0 || angle > M_PI;

}

// calculate the optimal path length
double ReedsSheppModel::GetPathLength(PathWords w, double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // the current path length
    double len;

    switch (w) {

        // Reeds-Shepp 8.1: CSC, same turn
        case LfSfLf: { len = GetLfSfLf(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbSbLb: { len = GetLfSfLf(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfSfRf: { len = GetLfSfLf(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbSbRb: { len = GetLfSfLf(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.2: CSC, different turn
        case LfSfRf: { len = GetLfSfRf(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbSbRb: { len = GetLfSfRf(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfSfLf: { len = GetLfSfRf(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbSbLb: { len = GetLfSfRf(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.3: C|C|C
        case LfRbLf: { len = GetLfRbLf(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRfLb: { len = GetLfRbLf(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLbRf: { len = GetLfRbLf(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLfRb: { len = GetLfRbLf(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.4: C|CC
        case LfRbLb: { len = GetLfRbLb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRfLf: { len = GetLfRbLb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLbRb: { len = GetLfRbLb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLfRf: { len = GetLfRbLb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.4: CC|C
        case LfRfLb: { len = GetLfRfLb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRbLf: { len = GetLfRfLb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLfRb: { len = GetLfRfLb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLbRf: { len = GetLfRfLb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.7: CCu|CuC
        case LfRufLubRb: { len = GetLfRufLubRb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRubLufRf: { len = GetLfRufLubRb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLufRubLb: { len = GetLfRufLubRb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLubRufLf: { len = GetLfRufLubRb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.8: C|CuCu|C
        case LfRubLubRf: { len = GetLfRubLubRf(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRufLufRb: { len = GetLfRubLubRf(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLubRubLf: { len = GetLfRubLubRf(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLufRufLb: { len = GetLfRubLubRf(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
        case LfRbpi2SbLb: { len = GetLfRbpi2SbLb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRfpi2SfLf: { len = GetLfRbpi2SbLb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLbpi2SbRb: { len = GetLfRbpi2SbLb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLfpi2SfRf: { len = GetLfRbpi2SbLb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
        case LfRbpi2SbRb: { len = GetLfRbpi2SbLb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRfpi2SfRf: { len = GetLfRbpi2SbLb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLbpi2SbLb: { len = GetLfRbpi2SbLb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLfpi2SfLf: { len = GetLfRbpi2SbLb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
        case LfSfRfpi2Lb: { len = GetLfSfRfpi2Lb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbSbRbpi2Lf: { len = GetLfSfRfpi2Lb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfSfLfpi2Rb: { len = GetLfSfRfpi2Lb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbSbLbpi2Rf: { len = GetLfSfRfpi2Lb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
        case LfSfLfpi2Rb: { len = GetLfSfLfpi2Rb(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbSbLbpi2Rf: { len = GetLfSfLfpi2Rb(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfSfRfpi2Lb: { len = GetLfSfLfpi2Rb(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbSbRbpi2Lf: { len = GetLfSfLfpi2Rb(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}

        // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
        case LfRbpi2SbLbpi2Rf: { len = GetLfRbpi2SbLbpi2Rf(goal_x, goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        case LbRfpi2SfLfpi2Rb: { len = GetLfRbpi2SbLbpi2Rf(-goal_x, goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RfLbpi2SbRbpi2Lf: { len = GetLfRbpi2SbLbpi2Rf(goal_x, -goal_y, -goal_orientation, -sin_o, cos_o, t, u, v); break;}
        case RbLfpi2SfRfpi2Lb: { len = GetLfRbpi2SbLbpi2Rf(-goal_x, -goal_y, goal_orientation, sin_o, cos_o, t, u, v); break;}
        default:
            // set to the default value
            len = std::numeric_limits<double>::infinity();
            break;

    }

    // return the desired len
    return len;

}

// build the Reeds-Shepp path
ReedsSheppActionSetPtr ReedsSheppModel::BuildPath(PathWords w, double t, double u, double v) {

    // build the correct path, based on the PathWords and path lengths
    switch(w) {

        // Reeds-Shepp 8.1: CSC, same turn
        case LfSfLf: return GetLfSfLfpath(t, u, v);
        case LbSbLb: return ReedsSheppActionSet::TimeFlip(GetLfSfLfpath(t, u, v));
        case RfSfRf: return ReedsSheppActionSet::Reflect(GetLfSfLfpath(t, u, v));
        case RbSbRb: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfSfLfpath(t, u, v));

        // Reeds-Shepp 8.2: CSC, different turn
        case LfSfRf: return GetLfSfRfpath(t, u, v);
        case LbSbRb: return ReedsSheppActionSet::TimeFlip(GetLfSfRfpath(t, u, v));
        case RfSfLf: return ReedsSheppActionSet::Reflect(GetLfSfRfpath(t, u, v));
        case RbSbLb: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfSfRfpath(t, u, v));

        // Reeds-Shepp 8.3: C|C|C
        case LfRbLf: return GetLfRbLfpath(t, u, v);
        case LbRfLb: return ReedsSheppActionSet::TimeFlip(GetLfRbLfpath(t, u, v));
        case RfLbRf: return ReedsSheppActionSet::Reflect(GetLfRbLfpath(t, u, v));
        case RbLfRb: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRbLfpath(t, u, v));

        // Reeds-Shepp 8.4: C|CC
        case LfRbLb: return GetLfRbLbpath(t, u, v);
        case LbRfLf: return ReedsSheppActionSet::TimeFlip(GetLfRbLbpath(t, u, v));
        case RfLbRb: return ReedsSheppActionSet::Reflect(GetLfRbLbpath(t, u, v));
        case RbLfRf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRbLbpath(t, u, v));

        // Reeds-Shepp 8.4: CC|C
        case LfRfLb: return GetLfRfLbpath(t, u, v);
        case LbRbLf: return ReedsSheppActionSet::TimeFlip(GetLfRfLbpath(t, u, v));
        case RfLfRb: return ReedsSheppActionSet::Reflect(GetLfRfLbpath(t, u, v));
        case RbLbRf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRfLbpath(t, u, v));

        // Reeds-Shepp 8.7: CCu|CuC
        case LfRufLubRb: return GetLfRufLubRbpath(t, u, v);
        case LbRubLufRf: return ReedsSheppActionSet::TimeFlip(GetLfRufLubRbpath(t, u, v));
        case RfLufRubLb: return ReedsSheppActionSet::Reflect(GetLfRufLubRbpath(t, u, v));
        case RbLubRufLf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRufLubRbpath(t, u, v));

        // Reeds-Shepp 8.8: C|CuCu|C
        case LfRubLubRf: return GetLfRubLubRfpath(t, u, v);
        case LbRufLufRb: return ReedsSheppActionSet::TimeFlip(GetLfRubLubRfpath(t, u, v));
        case RfLubRubLf: return ReedsSheppActionSet::Reflect(GetLfRubLubRfpath(t, u, v));
        case RbLufRufLb: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRubLubRfpath(t, u, v));

        // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
        case LfRbpi2SbLb: return GetLfRbpi2SbLbpath(t, u, v);
        case LbRfpi2SfLf: return ReedsSheppActionSet::TimeFlip(GetLfRbpi2SbLbpath(t, u, v));
        case RfLbpi2SbRb: return ReedsSheppActionSet::Reflect(GetLfRbpi2SbLbpath(t, u, v));
        case RbLfpi2SfRf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRbpi2SbLbpath(t, u, v));

        // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
        case LfRbpi2SbRb: return GetLfRbpi2SbLbpath(t, u, v);
        case LbRfpi2SfRf: return ReedsSheppActionSet::TimeFlip(GetLfRbpi2SbLbpath(t, u, v));
        case RfLbpi2SbLb: return ReedsSheppActionSet::Reflect(GetLfRbpi2SbLbpath(t, u, v));
        case RbLfpi2SfLf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRbpi2SbLbpath(t, u, v));

        // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
        case LfSfRfpi2Lb: return GetLfSfRfpi2Lbpath(t, u, v);
        case LbSbRbpi2Lf: return ReedsSheppActionSet::TimeFlip(GetLfSfRfpi2Lbpath(t, u, v));
        case RfSfLfpi2Rb: return ReedsSheppActionSet::Reflect(GetLfSfRfpi2Lbpath(t, u, v));
        case RbSbLbpi2Rf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfSfRfpi2Lbpath(t, u, v));

        // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
        case LfSfLfpi2Rb: return GetLfSfLfpi2Rbpath(t, u, v);
        case LbSbLbpi2Rf: return ReedsSheppActionSet::TimeFlip(GetLfSfLfpi2Rbpath(t, u, v));
        case RfSfRfpi2Lb: return ReedsSheppActionSet::Reflect(GetLfSfLfpi2Rbpath(t, u, v));
        case RbSbRbpi2Lf: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfSfLfpi2Rbpath(t, u, v));

        // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
        case LfRbpi2SbLbpi2Rf: return GetLfRbpi2SbLbpi2Rfpath(t, u, v);
        case LbRfpi2SfLfpi2Rb: return ReedsSheppActionSet::TimeFlip(GetLfRbpi2SbLbpi2Rfpath(t, u, v));
        case RfLbpi2SbRbpi2Lf: return ReedsSheppActionSet::Reflect(GetLfRbpi2SbLbpi2Rfpath(t, u, v));
        case RbLfpi2SfRfpi2Lb: return ReedsSheppActionSet::TimeFlipAndReflect(GetLfRbpi2SbLbpi2Rfpath(t, u, v));
        default:
            return new ReedsSheppActionSet(std::numeric_limits<double>::infinity());

    }

}

// left forward, straight forward and left forward movement - get the path length
double ReedsSheppModel::GetLfSfLf(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.1
    // C_C_C

    // get the x and y parameters
    double x = goal_x - sin_o;
    double y = goal_y - 1 + cos_o;

    // update the t value
    t = std::atan2(y, x);

    // update the u value
    u = std::sqrt(x*x + y*y);

    // update the v value
    v = mrpt::math::wrapToPi<double>(goal_orientation - t);

    if (isInvalidAngle(t) || isInvalidAngle(v)) {

        return std::numeric_limits<double>::max();

    }

    return t + u + v;

}

// build the actual LfSfLf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfSfLfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions = new ReedsSheppActionSet();

    // add the appropriate gears and steering

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSStraight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, ForwardGear, v);

    return actions;

}

// left forward, straight forward and right forward movement - get the path length
double ReedsSheppModel::GetLfSfRf(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.2

    // get the x and y parameters
    double x = goal_x + sin_o;
    double y = goal_y - 1 - cos_o;

    double u1squared = x * x + y * y;
    double t1 = std::atan2(y, x);

    if (4 > u1squared) {

        return std::numeric_limits<double>::max();

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

        return std::numeric_limits<double>::max();

    }

    return t + u + v;

}

// build the actual LfSfRf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfSfRfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSStraight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnRight, ForwardGear, v);

    return actions;

}

// left forward, right backward and left forward movement - get the path length
double ReedsSheppModel::GetLfRbLf(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.3
    // Uses a modified formula adapted from the c_c_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sin_o;
    double eta = goal_y - 1 + cos_o;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::max();

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

        return std::numeric_limits<double>::max();

    }

    return t + u + v;
}

// build the actual LfRbLf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRbLfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, ForwardGear, v);

    return actions;

}

// left forward, right backward and left backward movement - get the path length
double ReedsSheppModel::GetLfRbLb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.4
    // Uses a modified formula adapted from the c_cc function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sin_o;
    double eta = goal_y - 1 + cos_o;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRbLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right forward and left backward movement - get the path length
double ReedsSheppModel::GetLfRfLb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {
    // Reeds-Shepp 8.4
    // Uses a modified formula adapted from the cc_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sin_o;
    double eta = goal_y - 1 + cos_o;

    double u1 = std::sqrt(x * x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRfLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right forward, left backward and right forward movement - get the path length
double ReedsSheppModel::GetLfRufLubRb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.7
    // Uses a modified formula adapted from the ccu_cuc function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sin_o;
    double eta = goal_y - 1 - cos_o;

    double u1 = std::sqrt(x*x + eta * eta);
    if (4 < u1) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRufLubRbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, BackwardGear, u);

    // the fourth action
    actions->AddAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, right backward, left backward and right forward movement - get the path length
double ReedsSheppModel::GetLfRubLubRf(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.8
    // Uses a modified formula adapted from the c_cucu_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sin_o;
    double eta = goal_y - 1 - cos_o;

    double u1 = std::sqrt(x * x + eta * eta);
    if (6 < u1) {

        return std::numeric_limits<double>::max();

    }

    double phi = std::atan2(eta, x);
    double va1 = 1.25f - u1 * u1 / 16;
    if (va1 < 0 || va1 > 1.0) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRubLubRfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, BackwardGear, u);

    // the fourth action
    actions->AddAction(RSTurnRight, ForwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward and left backward movement - get the path length
double ReedsSheppModel::GetLfRbpi2SbLb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.9
    // Uses a modified formula adapted from the c_c2sca function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sin_o;
    double eta = goal_y - 1 + cos_o;

    double u1squared = x * x + eta * eta;
    if (u1squared < 4) {

        return std::numeric_limits<double>::max();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 2;
    if (0 > u) {

        return std::numeric_limits<double>::max();

    }

    double alpha = std::atan2(2, (u) + 2);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t + M_PI_2 - goal_orientation);

    return t + M_PI_2 + u + v;
}

// build the actual LfRbpi2SbLb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRbpi2SbLbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->AddAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->AddAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward and right backward movement - get the path length
double ReedsSheppModel::GetLfRbpi2SbRb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.10
    // Uses a modified formula adapted from the c_c2scb function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sin_o;
    double eta = goal_y - 1 - cos_o;

    double u1 = std::sqrt(x*x + eta*eta);
    if (2.0 > u1) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRbpi2SbRbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->AddAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->AddAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, straight forward, right forward PI over 2 and right backward movement - get the path length
double ReedsSheppModel::GetLfSfRfpi2Lb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.9 (reversed)
    // Uses a modified formula adapted from the csc2_ca function
//     // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x - sin_o;
    double eta = goal_y - 1 + cos_o;

    double u1squared = x*x + eta*eta;
    if (4 > u1squared) {

        return std::numeric_limits<double>::max();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 2;
    if (0 > (u)) {

        return std::numeric_limits<double>::max();

    }

    double alpha = std::atan2((u) + 2, 2);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi - alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - M_PI_2 - goal_orientation);

    return t + u + M_PI_2 + v;

}

// build the actual LfSfRfpi2Lb path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfSfRfpi2Lbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSStraight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnRight, ForwardGear, M_PI_2);

    // the fourth action
    actions->AddAction(RSTurnLeft, BackwardGear, v);

    return actions;

}

// left forward, straight forward, left forward PI over 2 and right backward movement - get the path length
double ReedsSheppModel::GetLfSfLfpi2Rb(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.10 (reversed)
    // Uses a modified formula adapted from the csc2_cb function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sin_o;
    double eta = goal_y - 1 - cos_o;

    double u1 = std::sqrt(x*x + eta * eta);
    if (2.0 > u1) {

        return std::numeric_limits<double>::max();

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
ReedsSheppActionSetPtr ReedsSheppModel::GetLfSfLfpi2Rbpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSStraight, ForwardGear, u);

    // the third action
    actions->AddAction(RSTurnLeft, ForwardGear, M_PI_2);

    // the fourth action
    actions->AddAction(RSTurnRight, BackwardGear, v);

    return actions;

}

// left forward, right backward PI over 2, straight backward, left backward PI over 2 and right forward movement - get the path length
double ReedsSheppModel::GetLfRbpi2SbLbpi2Rf(double goal_x, double goal_y, double goal_orientation, double sin_o, double cos_o, double &t, double &u, double &v) {

    // Reeds-Shepp 8.11
    // Uses a modified formula adapted from the c_c2sc2_c function
    // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c

    double x = goal_x + sin_o;
    double eta = goal_y - 1 - cos_o;

    double u1squared = x * x + eta * eta;
    if (16 > u1squared) {

        return std::numeric_limits<double>::max();

    }

    double phi = std::atan2(eta, x);

    // update the u value
    u = std::sqrt(u1squared - 4) - 4;
    if (0 > (u)) {

        return std::numeric_limits<double>::max();

    }

    double alpha = std::atan2(2, (u) + 4);

    // update the t value
    t = mrpt::math::wrapTo2Pi<double>(M_PI_2 + phi + alpha);

    // update the v value
    v = mrpt::math::wrapTo2Pi<double>(t - goal_orientation);

    return t + u + v + M_PI;

}

// build the actual LfRbpi2SbLbpi2Rf path based on given t, u and v
ReedsSheppActionSetPtr ReedsSheppModel::GetLfRbpi2SbLbpi2Rfpath(double t, double u, double v) {

    // build a new action set
    ReedsSheppActionSetPtr actions =  new ReedsSheppActionSet();

    // the first action
    actions->AddAction(RSTurnLeft, ForwardGear, t);

    // the second action
    actions->AddAction(RSTurnRight, BackwardGear, M_PI_2);

    // the third action
    actions->AddAction(RSStraight, BackwardGear, u);

    // the fourth action
    actions->AddAction(RSTurnLeft, BackwardGear, M_PI_2);

    // the fifth action
    actions->AddAction(RSTurnRight, ForwardGear, v);

    return actions;

}
