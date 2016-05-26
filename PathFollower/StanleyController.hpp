/*
// Author: Josias Alexandre Oliveira
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

#ifndef STANLEY_METHOD_CONTROLLER_HPP
#define STANLEY_METHOD_CONTROLLER_HPP

#include "../Entities/State2D.hpp"
#include "../PathFinding/VehicleModel/VehicleModel.hpp"

namespace astar {

    class StanleyController {

        private:

            // the current path
            astar::StateListPtr path;

             // set the appropriated low speeds around the stopping points
            void UpdateLowSpeedRegions(astar::StateListPtr, std::vector<std::list<astar::State2D>::iterator>&, astar::VehicleModel&);

            // update the path around stopping points
            void UpdateStoppingPoints(astar::StateListPtr, astar::VehicleModel&);

            // get the desired command given two states
            // the first and the last states remains the same
            astar::StateListPtr ConsolidateStateList(astar::StateListPtr, astar::VehicleModel&);

        public:

            // basic constructor
            StanleyController();

            // get a Command list to follow a given path
            std::vector<State2D> BuildCommandList(astar::StateListPtr, astar::VehicleModel&);

    };

}

#endif
