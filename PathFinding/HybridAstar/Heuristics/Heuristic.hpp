/*
// Author: josiasalexandre@gmail.com

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

#ifndef COMBINED_HEURISTIC_HPP
#define COMBINED_HEURISTIC_HPP

#include "../../../Entities/Pose2D.hpp"
#include "../../../GridMap/InternalGridMap.hpp"
#include "NonholonomicHeuristicInfo.hpp"
#include "HolonomicHeuristic.hpp"

namespace astar {

class Heuristic {

    private:

        // PRIVATE ATTRIBUTES
        //

        // the current precomputed nonholonomic heuristic info
        astar::NonholonomicHeuristicInfo info;

        // non holonomic heuristic, adapted from Chen Chao
        // It's hard to obtain the Djikstra cost from all nodes/cells to each other node/cell in real time
        // the map is constantly changed, so we must adapt the current holonomic heuristic
        astar::HolonomicHeuristic holonomic;

        // the next goal, updates the circle path heuristic
        astar::Pose2D goal;

        // PRIVATE METHODS

        // obstacle relaxed heuristic
        double GetObstacleRelaxedHeuristicValue(astar::Pose2D, const astar::Pose2D&);

        // nonholonomic relaxed heuristic
        double GetNonholonomicRelaxedHeuristicValue(const astar::Pose2D&);

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // basic constructor
        Heuristic(astar::InternalGridMapRef);

        // update the heuristic around a new goal
        void UpdateHeuristic(astar::InternalGridMap& map, const astar::Pose2D&, const astar::Pose2D&);

        // get a heuristic value
        double GetHeuristicValue(const astar::Pose2D&, const astar::Pose2D&);

};

}

#endif
