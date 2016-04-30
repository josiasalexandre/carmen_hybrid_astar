#ifndef REEDS_SHEPP_ACTION_HPP
#define REEDS_SHEPP_ACTION_HPP

#include "ReedsSheppDefinitions.hpp"

namespace astar {

class ReedsSheppAction {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // steering
        astar::Steer steer;

        // direction
        astar::Gear gear;

        // the path length
        double length;

        // PUBLIC METHODS

        // basic constructor
        ReedsSheppAction() {}

        // basic constructor
        ReedsSheppAction(const astar::Steer &s, const astar::Gear &g, const double &len) : steer(s), gear(g), length(len) {}

        // copy constructor
        ReedsSheppAction(const ReedsSheppAction &action) : steer(action.steer), gear(action.gear), length(action.length) {}

        // = operator overloading
        void operator=(const ReedsSheppAction &action) {

            // get the steering
            steer = action.steer;

            // get the gear action
            gear = action.gear;

            // get the path length
            length = action.length;

        }

};

// define a pointer handler
typedef ReedsSheppAction* ReedsSheppActionPtr;

}

#endif
