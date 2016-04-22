#ifndef REEDS_SHEPP_ACTION_SET_HPP
#define REEDS_SHEPP_ACTION_SET_HPP

#include <vector>

#include "ReedsSheppAction.hpp"

namespace astar {

class ReedsSheppActionSet {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:


        // basic constructor
        ReedsSheppActionSet();

        // copy constructor
        ReedsSheppActionSet(const ReedsSheppActionSet&);

        // the null set
        ReedsSheppActionSet(double);

        // PUBLIC ATTRIBUTES

        // the list of actions
        std::vector<ReedsSheppAction> actions;

        // the path length
        double length;

        // PUBLIC METHODS

        // add a new ReedsSheppAction
        void addAction(astar::Steer, astar::Gear, double);

        // the entire set cost
        double calculateCost(double, double, double);

        // PUBLIC STATIC CLASS METHODS

        // flip the actions in time
        static ReedsSheppActionSet* timeFlip(ReedsSheppActionSet*);

        // reflect the path
        static ReedsSheppActionSet* reflect(ReedsSheppActionSet*);

        // time flip and reflect in sequence
        static ReedsSheppActionSet* timeFlipAndReflect(ReedsSheppActionSet*);

};

// easy handling
typedef ReedsSheppActionSet* ReedsSheppActionSetPtr;

}

#endif
