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
#include "../VehicleModel/VehicleModel.hpp"

using namespace astar;

HybridAstar::HybridAstar(VehicleModel &vehicle_) : vehicle(vehicle_), grid(nullptr), reverse_factor(4), gear_switch_cost(8)
{
}

// PRIVATE METHODS

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
StateListPtr HybridAstar::RebuildPath(HybridAstarNodePtr n)
{
    // create the list
    StateListPtr path = new StateList();

    // a helper node pointer
    HybridAstarNodePtr tmp = nullptr;

    // counter
    unsigned int subpathSize = 0;

    // reference syntactic sugar
    std::list<State2D> &states(path->states);

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // save the current state to the list
            states.push_front(n->state);

        } else if (nullptr != n->action_set) {

            // get the subpath provided by the action set discretization
            StateArrayPtr subpath = ReedsSheppModel::Discretize(n->parent->state, n->action_set, vehicle.max_turn_radius, grid.inverse_resolution);

            // get the path size
            subpathSize = subpath->states.size();

            // prepend the resulting subpath to the current path
            for (unsigned int i = subpathSize - 1; i >= 0; i--) {

                // save the current state to the list
                states.push_front(subpath->states[i]);

            }

        }

        // move to the parent node
        n = n->parent;

    }

    // move every node's gear to it's parent node
    if (0 < states.size()) {

        // get the end
        std::list<State2D>::iterator last = states.end() - 1;

        // helper
        std::list<State2D>::iterator tmp;
        for (std::list<State2D>::iterator it = states.begin(); it < last; ++it) {

            // get the next node
            tmp = it + 1;

            // copy the next gear
            it->gear = tmp->gear;

        }

    }

    return path;


}

// get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
HybridAstarNodePtr HybridAstar::GetReedsSheppChild(const State2D &start, const State2D &goal) {

    // get the resulting set of actions
    ReedsSheppActionSetPtr action_set = rs.Solve(start, goal, vehicle.max_turn_radius);

    if (0 < action_set->actions.size()) {

        // flag to validate the Reeds-Shep curve
        bool safe = true;

        // get the states in the Reeds-Shepp curve
        StateArrayPtr path = rs.Discretize(start, action_set, vehicle.max_turn_radius, grid.inverse_resolution);

        // a reference helper, just in case
        std::vector<State2D>& states(path->states);

        // iterate over the state list
        for (std::vector<State2D>::iterator it = states.begin(); it < states.end(); ++it) {

            if (!grid.isValidPoint(it->position) || !grid.isSafePlace(*it)) {

                // not a good state
                safe = false;

                break;

            }

        }

        if (safe) {

            // return the HybridAstarNode on the goal state
            return new HybridAstarNode(goal, path, nullptr, 0, 0, nullptr);

        }

    }

    return nullptr;

}

// get the children nodes by expanding all gears and steering
HybridAstarNodeArrayPtr HybridAstar::GetChidlren(const State2D &start, const State2D &goal, Gear gear, double length) {

    // build the vector
    HybridAstarNodeArrayPtr children = new HybridAstarNodeArray();

    // a reference syntactic sugar
    std::vector<HybridAstarNodePtr> &nodes(children->nodes);

    // a state helper, the current child
    State2D child;

    // iterate over the steering moves
    for (unsigned int j = 0; j < astar::NumSteering; j++) {

        // casting the steering
        Steer s = static_cast<Steer>(j);

        // get the next state
        child = vehicle.NextState(start, s, gear, length, vehicle.max_turn_radius);

        // set the gear
        child.gear = gear;

        // verify the safety condition and the grid boundary
        if (grid.isValidPoint(child.position) && grid.isSafePlace(child)) {

            // append to the children list
            nodes.push_back(new HybridAstarNode(child, ReedsSheppAction(s, gear, length), nullptr, 0, 0, nullptr));

        }

    }

    // expanding Reeds-Shepp curves now

    // Reeds-Shepp curve threshold value
    double threshold = heuristic.GetHeuristicValue(grid, start, goal);
    double inverseThreshold = 10.0/(threshold*threshold);

    // quadratic falloff
    if (10 > threshold || inverseThreshold < rand()) {

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
StateListPtr HybridAstar::FindPath(InternalGridMap &grid_map, const State2D& start, const State2D& goal) {

    // get the grid map pointer
    // useful inside others methods, just to avoid passing the parameter constantly
    // now, all HybridAstar methods have access to the same grid pointer
    grid = grid_map;

    // update the heuristic to the new goal
    heuristic.UpdateGoal(grid_map, goal);

    // the start state heuristic value
    double heuristic_value = heuristic.GetHeuristicValue(grid_map, start, goal);

    // find the current cell
    MapCellPtr c = grid_map.StateToCell(start);

    // the available space around the vehicle
    // provides the total length between the current state and the node's children
    double length;

    // the simple case of length
    // length = grid_map.resolution

    // the simple case of dt
    // dt = grid_map.resolution/vehicle.default_speed

    // create a new Node
    // the node updates the cell node pointer and the cell status
    // see the HybridAstarNode constructor
    HybridAstarNodePtr n = new HybridAstarNode(start, new ReedsSheppAction(astar::RSStraight, ForwardGear, 0), c, 0, heuristic_value, nullptr);

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
        if (goal == n->state) {

            // rebuild the entire path
            StateListPtr resulting_path = RebuildPath(n);

            // clear all opened and expanded nodes
            RemoveAllNodes();

            // return the path
            return resulting_path;

        }

        // add to the explored set
        n->cell->status = ExploredNode;

        // get the length based on the environment
        length = std::max(grid_map.resolution, 0.5 * (grid_map.GetObstacleDistance(n->state.position) + grid_map.GetVoronoiDistance(n->state.position)));

        // iterate over the gears
        for (unsigned int i = 0; i < NumGears; i++) {

            // casting the gears
            Gear gear = static_cast<Gear>(i);

            // is it reverse gear?
            reverse_gear = (ForwardGear == gear);

            // get the children nodes by expanding all gears and steering
            HybridAstarNodeArrayPtr children = GetChidlren(n->state, goal, gear, length);

            // reference, just a syntactic sugar
            std::vector<HybridAstarNodePtr> &nodes(children->nodes);

            //
            for (std::vector<HybridAstarNodePtr>::iterator it = nodes.begin(); it < nodes.end(); ++it) {

            	HybridAstarNodePtr child = *it;
                // find the appropriated location in the grid

                // reusing the same "c" pointer declared above
                c = grid_map.StateToCell(child->state);

                // we must avoid cell = nullptr inside the HybridAstarNode
                if (nullptr != c) {

                    if (ExploredNode != c->status) {

                        if (nullptr != child->action) {

                            // we a have a valid action, conventional expanding
                            tentative_g = n->g + PathCost(n->state, child->state, length, reverse_gear);

                        } else if (0 < child->action_set->size()) {

                            // we have a valid action set, it was a Reeds-Shepp analytic expanding
                            tentative_g = n->g + child->action_set->CalculateCost(vehicle.max_turn_radius, reverse_factor, gear_switch_cost);

                        } else {

                            // we don't have any valid action, it's a bad error
                            // add to the invalid nodes set
                            invalid.push_back(child);

                            // jump to the next iteration
                            continue;
                            // throw std::exception();

                        }

                        // get the estimated cost
                        tentative_f = tentative_g + heuristic.GetHeuristicValue(grid_map, child->state, goal);

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
    return nullptr;

}
