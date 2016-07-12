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
        void AddAction(astar::Steer, astar::Gear, double);

        // get the actions vector size
        unsigned int size();

        // the entire set cost
        double CalculateCost(double, double, double);

        // PUBLIC STATIC CLASS METHODS

        // flip the actions in time
        static ReedsSheppActionSet* TimeFlip(ReedsSheppActionSet*);

        // reflect the path
        static ReedsSheppActionSet* Reflect(ReedsSheppActionSet*);

        // time flip and reflect in sequence
        static ReedsSheppActionSet* TimeFlipAndReflect(ReedsSheppActionSet*);

};

// easy handling
typedef ReedsSheppActionSet* ReedsSheppActionSetPtr;

}

#endif
