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
void ReedsSheppActionSet::addAction(Steer s, Gear g, double len) {

    // append the new action to the list
    actions.push_back(ReedsSheppAction(s, g, len));

    // update the total length
    length += std::fabs(len);

}

unsigned int ReedsSheppActionSet::size() {

    return actions.size();
}


// the entire set cost
double ReedsSheppActionSet::CalculateCost(double unit, double reverseFactor, double gearSwitchCost) {

    // get the actions size
    unsigned int a_size = actions.size();

    if (0 < a_size) {

        if (1.0 == reverseFactor && 0.0 == gearSwitchCost) {

            return length*unit;

        }

        // the final cost
        double actionCost, cost = 0.0;

        // the first gear
        Gear prevGear = actions[0].gear;

        for (unsigned int i = 1; i < a_size; i++) {

            // get the current action cost
            actionCost = actions[i].length*unit;

            if (BackwardGear == actions[i].gear) {

                // multiply by the reverse cost
                actionCost *= reverseFactor;

            }
            if (prevGear != actions[i].gear) {

                // multiply by the gearSwitchCost
                actionCost *= gearSwitchCost;

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
ReedsSheppActionSet* ReedsSheppActionSet::timeFlip(ReedsSheppActionSet *set) {

    if (nullptr != set) {

        if (0 < set->actions.size()) {

            // flip
            for (std::vector<ReedsSheppAction>::iterator it = set->actions.begin(); it != set->actions.end(); ++it) {

                // update the gear
                if (BackwardGear == it->gear) {

                    // set to forward movement
                    it->gear = BackwardGear;

                } else {

                    // set to backward movement
                    it->gear = ForwardGear;

                }

            }

        }

    }

    return set;

}

// reflect the path
ReedsSheppActionSet* ReedsSheppActionSet::reflect(ReedsSheppActionSet *set) {

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

//
// time flip and reflect in sequence
ReedsSheppActionSet* ReedsSheppActionSet::timeFlipAndReflect(ReedsSheppActionSet *set) {

    return ReedsSheppActionSet::reflect(ReedsSheppActionSet::timeFlip(set));

}
