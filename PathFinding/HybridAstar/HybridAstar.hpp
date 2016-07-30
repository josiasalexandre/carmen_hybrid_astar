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

#ifndef HYBRID_ASTAR_ALGORITHM_HPP
#define HYBRID_ASTAR_ALGORITHM_HPP

#include <vector>
#include <list>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../PriorityQueue/PriorityQueue.hpp"
#include "../../Entities/Pose2D.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "../../ReedsShepp/ReedsSheppModel.hpp"
#include "../../VehicleModel/VehicleModel.hpp"
#include "HybridAstarNode.hpp"
#include "Heuristics/Heuristic.hpp"

namespace astar {

class HybridAstar {

private:

    // PRIVATE ATTRIBUTES

    // the reverse factor penalty
    double reverse_factor;

    // the gear switch cost penalty
    double gear_switch_cost;

    // the voronoi field factor
    double voronoi_field_factor;

    // the ReedsSheppModel
    ReedsSheppModel rs;

    // the Vehicle model
    astar::VehicleModel &vehicle;

    // grid map pointer
    astar::InternalGridMapRef grid;

    // the heuristic
    astar::Heuristic heuristic;

    // the opened nodes set
    astar::PriorityQueue<HybridAstarNodePtr> open;

    // the expanded nodes set
    std::vector<HybridAstarNodePtr> discovered;

    // the invalid nodes set
    std::vector<HybridAstarNodePtr> invalid;

    // the current grid map
    unsigned char *map;
    unsigned w, h;

    // PRIVATE METHODS

    // clear all the sets
    void RemoveAllNodes();

    // reconstruct the path from the goal to the start pose
    astar::StateArrayPtr RebuildPath(HybridAstarNodePtr, const State2D&);

    // get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
    HybridAstarNodePtr GetReedsSheppChild(const astar::Pose2D&, const astar::Pose2D&);

    // get the children nodes by expanding all gears and steering
    HybridAstarNodeArrayPtr GetChidlren(const astar::Pose2D&, const astar::Pose2D&, astar::Gear, double);

    // get the path cost
    double PathCost(
		astar::Gear start_gear,
		const astar::Pose2D& goal_pose,
		astar::Gear next_gear,
		double length);

public:

    // PUBLIC ATTRIBUTES

    // PUBLIC METHODS

    // basic constructor
    HybridAstar(astar::VehicleModel &vehicle_, astar::InternalGridMapRef);

    // basic destructor
    ~HybridAstar();

    // find a path to the goal
    astar::StateArrayPtr FindPath(astar::InternalGridMapRef, const astar::State2D&, const astar::State2D&);

};

}


#endif
