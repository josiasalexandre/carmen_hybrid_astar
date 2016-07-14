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

#include "HybridAstar.hpp"

using namespace astar;

HybridAstar::HybridAstar(VehicleModel &vehicle_, InternalGridMapRef map) : reverse_factor(4), gear_switch_cost(8), rs(),
	vehicle(vehicle_), grid(map), heuristic(), open(nullptr), discovered(), invalid() {}

// remove all nodes
void HybridAstar::RemoveAllNodes() {

    HybridAstarNodePtr tmp;

    // clear the open set
    open.DestroyHeap();

    // clear the closed set
    while(!discovered.empty()) {

        // get the last element
        tmp = discovered.back();

        // remove the node from the set
        discovered.pop_back();

        // delete the node
        delete(tmp);

    }

    // clear the invalid set
    while(!invalid.empty()) {

        // get the last element
        tmp = invalid.back();

        // remove the node from the set
        invalid.pop_back();

        // delete the node
        delete(tmp);

    }

    return;

}

// rebuild an entire path given a node
// reconstruct the path from the goal to the start state
StateArrayPtr HybridAstar::RebuildPath(HybridAstarNodePtr n, const State2D &goal)
{
    // create the list
    StateListPtr nodes = new StateList();

    // reference syntactic sugar
    std::list<State2D> &states(nodes->states);

    // a helper node pointer
    // counter
    unsigned int subpathSize = 0;

    // the current state
    State2D s;

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // update the current state
            s = n->pose;
            s.gear = n->action->gear;

            // save the current state to the list
            states.push_front(s);

        } else if (nullptr != n->action_set) {

            // get the subpath provided by the action set discretization
            StateArrayPtr subpath = ReedsSheppModel::Discretize(n->parent->pose, n->action_set, vehicle.min_turn_radius, grid.inverse_resolution);

            // get the path size
            subpathSize = subpath->states.size();

            // prepend the resulting subpath to the current path
            for (int i = subpathSize - 1; i >= 0; i--) {

                // save the current state to the list
                states.push_front(subpath->states[i]);

            }

        }

        // move to the parent node
        n = n->parent;

    }

    // the output state array
    StateArrayPtr path = new StateArray();

    // reference syntactic sugar
    std::vector<State2D> &out(path->states);

    // move every node's gear to the it's parent node
    // and append the current
    if (1 < states.size()) {

    	// the auxiliary iterators
        std::list<State2D>::iterator prev, next;

        // get the first iterator
        prev = next = states.begin();

        // update the next pointer
        ++next;

        // get the end pointer
        std::list<State2D>::iterator end = states.end();

        while (next != end) {

            // copy the next gear
            prev->gear = next->gear;

            // append to the output vector
            out.push_back(*prev);

            // advance to the next states
            prev = next;
            ++next;
        }

        // append the last state
        out.push_back(*prev);

    }
    // save the endpoint speed to the final state
    if (0 < states.size()) {

        out.back().v = goal.v;

    }

    return path;

}

// get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
HybridAstarNodePtr HybridAstar::GetReedsSheppChild(const Pose2D &start, const Pose2D &goal) {

    // get the resulting set of actions
    ReedsSheppActionSetPtr action_set = rs.Solve(start, goal, vehicle.min_turn_radius);

    if (0 < action_set->actions.size()) {

        // flag to validate the Reeds-Shepp curve
        bool safe = true;

        // get the states in the Reeds-Shepp curve
        StateArrayPtr path = rs.Discretize(start, action_set, vehicle.min_turn_radius, grid.inverse_resolution);

        // a reference helper, just in case
        std::vector<State2D>& states(path->states);

        // get the states end pointer
        std::vector<State2D>::iterator end = states.end();

        // iterate over the state list
        for (std::vector<State2D>::iterator it = states.begin(); it < end; ++it) {

            if (!grid.isValidPoint(it->position) || !grid.isSafePlace(vehicle.GetVehicleBodyCircles(*it), vehicle.safety_factor)) {

                // not a good state
                safe = false;

                break;

            }

        }

        if (safe) {

            // return the HybridAstarNode on the goal state
            return new HybridAstarNode(start, action_set);

        }

    }

    return nullptr;

}

// get the children nodes by expanding all gears and steering
HybridAstarNodeArrayPtr HybridAstar::GetChidlren(const Pose2D &start, const Pose2D &goal, Gear gear, double length) {

    // build the vector
    HybridAstarNodeArrayPtr children = new HybridAstarNodeArray();

    // a reference syntactic sugar
    std::vector<HybridAstarNodePtr> &nodes(children->nodes);

    // a state helper, the current child
    Pose2D child_pose;

    // iterate over the steering moves
    for (unsigned int j = 0; j < astar::NumSteering; j++) {

        // casting the steering
        Steer steer = static_cast<Steer>(j);

        // get the next state
        child_pose = vehicle.NextPose(start, steer, gear, length, vehicle.min_turn_radius);

        // verify the safety condition and the grid boundary
        if (grid.isValidPoint(child_pose.position) && grid.isSafePlace(vehicle.GetVehicleBodyCircles(child_pose), vehicle.safety_factor)) {

            // append to the children list
            nodes.push_back(new HybridAstarNode(child_pose, new ReedsSheppAction(steer, gear, length)));

        }

    }

    // expanding Reeds-Shepp curves now

    // Reeds-Shepp curve threshold value
    double threshold = heuristic.GetHeuristicValue(grid, start, goal);
    double inverseThreshold = 10.0/(threshold*threshold);

    // quadratic falloff
    if (10.0 > threshold || inverseThreshold < rand()) {

        // the a new HybridAstarNode based on ReedsSheppModel
        HybridAstarNodePtr rsNode = GetReedsSheppChild(start, goal);

        if (nullptr != rsNode) {

            // append to the children list
            nodes.push_back(rsNode);

        }
    }

    return children;

}

// PUBLIC METHODS
// receives the grid, start and goal states and find a path, if possible

StateArrayPtr HybridAstar::FindPath(InternalGridMap &grid_map, const State2D &start, const State2D &goal) {

    // get the grid map pointer
    // useful inside others methods, just to avoid passing the parameter constantly
    // now, all HybridAstar methods have access to the same grid pointer
    grid = grid_map;

    // syntactic sugar
    // ge the reference to the base class
    Pose2D goal_pose(goal.position, goal.orientation);
    Pose2D start_pose(start.position, start.orientation);

    // update the heuristic to the new goal
    heuristic.UpdateGoal(grid_map, goal_pose);

    // the start state heuristic value
    double heuristic_value = heuristic.GetHeuristicValue(grid_map, start_pose, goal_pose);

    // find the current cell
    GridMapCellPtr c = grid_map.PoseToCell(start_pose);

    // the available space around the vehicle
    // provides the total length between the current state and the node's children
    double length = 0.0;

    // the simple case of length
    // length = grid_map.resolution

    // the simple case of dt
    // dt = grid_map.resolution/vehicle.default_speed

    // create a new Node
    // the node updates the cell node pointer and the cell status
    // see the HybridAstarNode constructor
    HybridAstarNodePtr n = new HybridAstarNode(start_pose, new ReedsSheppAction(), c, length, heuristic_value, nullptr);

    // push the start node to the queue
    n->handle = open.Add(n, heuristic_value);

    // push the start node to the discovered set
    discovered.push_back(n);

    // reverse flag
    bool reverse_gear;

    // the cost from the start to the current position
    double tentative_g;

    // the total estimated cost
    // from the start to the goal
    double tentative_f;

    // the actual A* algorithm
    while(!open.isEmpty()) {

        // get the high priority node
        n = open.DeleteMin();

        // is it the desired goal?
        if (goal_pose == n->pose) {

            // rebuild the entire path
            StateArrayPtr resulting_path = RebuildPath(n, goal);

            // clear all opened and expanded nodes
            RemoveAllNodes();

            // return the path
            return resulting_path;

        }

        // add to the explored set
        n->cell->status = ExploredNode;

        // get the length based on the environment
        length = std::max(grid_map.resolution, 0.5 * (grid_map.GetObstacleDistance(n->pose.position) + grid_map.GetVoronoiDistance(n->pose.position)));

        // iterate over the gears
        for (unsigned int i = 0; i < NumGears; i++) {

            // casting the gears
            Gear gear = static_cast<Gear>(i);

            // is it reverse gear?
            reverse_gear = (ForwardGear == gear);

            // get the children nodes by expanding all gears and steering
            HybridAstarNodeArrayPtr children = GetChidlren(n->pose, goal_pose, gear, length);

            // reference, just a syntactic sugar
            std::vector<HybridAstarNodePtr> &nodes(children->nodes);

            //
            for (std::vector<HybridAstarNodePtr>::iterator it = nodes.begin(); it < nodes.end(); ++it) {

            	HybridAstarNodePtr child = *it;

                // find the appropriated location in the grid
                // reusing the same "c" pointer declared above
                c = grid_map.PoseToCell(child->pose);

                // we must avoid node->cell = nullptr inside the HybridAstarNode
                if (nullptr != c) {

                    if (ExploredNode != c->status) {

                        if (nullptr != child->action) {

                            // we a have a valid action, conventional expanding
                            tentative_g = n->g + PathCost(n->pose, child->pose, length, reverse_gear);

                        } else if (0 < child->action_set->size()) {

                            // we have a valid action set, it was a Reeds-Shepp analytic expanding
                            tentative_g = n->g + child->action_set->CalculateCost(vehicle.min_turn_radius, reverse_factor, gear_switch_cost);

                        } else {

                            // we don't have any valid action, it's a bad error
                            // add to the invalid nodes set
                            invalid.push_back(child);

                            // jump to the next iteration
                            continue;
                            // throw std::exception();

                        }

                        // get the estimated cost
                        tentative_f = tentative_g + heuristic.GetHeuristicValue(grid_map, child->pose, goal_pose);

                        // update the cost
                        child->g = tentative_g;

                        // update the total value -> g cost plus heuristic value
                        child->f = tentative_f;

                        // set the parent
                        child->parent = n;

                        // is it a not opened node?
                        if (UnknownNode == c->status) {

                            // the cell is empty and doesn't have any node

                            // set the cell pointer
                        	child->cell = c;

                            // update the cell pointer
                            c->node = child;

                            // update the cell status
                            c->status = OpenedNode;

                            // add to the open set
                            child->handle = open.Add(child, tentative_f);

                        } else if (OpenedNode == c->status && tentative_f < c->node->f) {

                            // the cell has a node but the current child is a better one

                            // update the node at the cell
                            // copy the handle to the current child, just to avoid losing that information
                        	child->handle = c->node->handle;

                            // the old node is updated but not the corresponding Key in the priority queue
                            *(c->node) = *child;

                            // update the cell status
                            c->status = OpenedNode;

                            // decrease the key at the priority queue
                            open.DecreaseKey(child->handle, tentative_f);

                        }

                    }

                }

                // add the node to the discovered set
                discovered.push_back(child);

            }

        }

    }

    // the A* could no find a valid path to the goal
    RemoveAllNodes();

    return new StateArray();

}

// get the path cost
double HybridAstar::PathCost(const astar::Pose2D& start, const astar::Pose2D& goal, double length, bool reverse_gear) {

	if (reverse_gear)
		return start.position.x + goal.orientation * length;
	else
		return std::numeric_limits<double>::max();

}
