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

#include "ReedsSheppActionSet.hpp"

#include <limits>
#include <cmath>

using namespace astar;

// basic constructor
ReedsSheppActionSet::ReedsSheppActionSet() : length(0.0) {}

// copy constructor
ReedsSheppActionSet::ReedsSheppActionSet(const ReedsSheppActionSet& set) : actions(set.actions), length(set.length) {}

// the null set
ReedsSheppActionSet::ReedsSheppActionSet(double l) : actions(0), length(l) {}

// add a new action
void ReedsSheppActionSet::AddAction(Steer s, Gear g, double len) {

    // append the new action to the list
    actions.push_back(ReedsSheppAction(s, g, len));

    // update the total length
    length += std::fabs(len);

}

//
unsigned int ReedsSheppActionSet::Size() {

    return actions.size();
}

// the entire set cost
double ReedsSheppActionSet::CalculateCost(double unit, double reverseFactor, double gearSwitchCost) {

    // get the actions size
    unsigned int a_size = actions.size();

    if (0 < a_size) {

        if (1.0 == reverseFactor && 0.0 == gearSwitchCost) {

            return length * unit;

        }

        // the final cost
        double actionCost, cost = 0.0;

        // the first gear
        Gear prevGear = actions[0].gear;

        for (unsigned int i = 0; i < a_size; i++) {

            // get the current action cost
            actionCost = actions[i].length * unit;

            if (BackwardGear == actions[i].gear) {

                // multiply by the reverse cost
                actionCost *= reverseFactor;

            }

            if (prevGear != actions[i].gear) {

                // multiply by the gearSwitchCost
                actionCost += gearSwitchCost;

            }

            // update the prevGear to the current gear
            prevGear = actions[i].gear;

            // update the total costa
            cost += actionCost;

        }

        return cost;

    }

    // infinity cost
    return std::numeric_limits<double>::max();

}

// flip the actions in time
ReedsSheppActionSet* ReedsSheppActionSet::TimeFlip(ReedsSheppActionSet *set) {

    if (nullptr != set) {

        if (0 < set->actions.size()) {

            // flip
            for (std::vector<ReedsSheppAction>::iterator it = set->actions.begin(); it != set->actions.end(); ++it) {

                // update the gear
            	it->gear = (BackwardGear == it->gear) ? ForwardGear : BackwardGear;

            }

        }

    }

    return set;

}

// reflect the path
ReedsSheppActionSet* ReedsSheppActionSet::Reflect(ReedsSheppActionSet *set) {

    if (nullptr != set) {

        if (0 < set->actions.size()) {

            // flip
            for (std::vector<ReedsSheppAction>::iterator it = set->actions.begin(); it < set->actions.end(); ++it) {

                // update the gear
                if (RSTurnLeft == it->steer) {

                    // set to right steering
                    it->steer = RSTurnRight;

                } else if (RSTurnRight == it->steer) {

                    // set to left steering
                    it->steer = RSTurnLeft;

                }

            }

        }

    }

    return set;

}

// time flip and reflect in sequence
ReedsSheppActionSet* ReedsSheppActionSet::TimeFlipAndReflect(ReedsSheppActionSet *set) {

    return ReedsSheppActionSet::Reflect(ReedsSheppActionSet::TimeFlip(set));

}

// the assignement operator overloading
void ReedsSheppActionSet::operator=(const ReedsSheppActionSet &set) {

    // the list of actions
	actions = set.actions;

    // the path length
    length = set.length;

}
