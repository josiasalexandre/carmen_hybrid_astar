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
            ReedsSheppAction() : steer(RSStraight), gear(ForwardGear), length(0.0) {}

            // basic constructor
            ReedsSheppAction(const astar::Steer &s, const astar::Gear &g, const double &len) : steer(s), gear(g), length(len) {}

            // copy constructor
            ReedsSheppAction(const ReedsSheppAction &action) : steer(action.steer), gear(action.gear), length(action.length)
            {}

            // = operator overloading
            void operator=(const ReedsSheppAction &action)
            {
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
