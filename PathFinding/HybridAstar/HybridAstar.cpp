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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace astar;

HybridAstar::HybridAstar(
    VehicleModel &vehicle_,
    InternalGridMapRef map) :
    reverse_factor(10),
    gear_switch_cost(20),
    voronoi_field_factor(1.5),
    rs(),
    vehicle(vehicle_),
    grid(map),
    heuristic(map),
    open(nullptr),
    discovered(),
    invalid(),
    map(nullptr),
    width(), height()
{

    // define common window
    cv::namedWindow("Astar", cv::WINDOW_AUTOSIZE);

}

HybridAstar::~HybridAstar() {

    // remove all nodes
    RemoveAllNodes();

}

// remove all nodes
void HybridAstar::RemoveAllNodes() {

    HybridAstarNodePtr tmp;

    // clear the open set
    // since we put pointers inside the PriorityQueue, we should delete all by ourselfs
    // luckilly, they are copied in the discovered and invalid list
    open.ClearHeap();

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
StateArrayPtr HybridAstar::RebuildPath(HybridAstarNodePtr n, const State2D &start, const State2D &goal)
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

    double inverse_speed = 1.0/vehicle.low_speed;

    unsigned char *map2 = grid.GetGridMap();

    // draw the current map
    cv::Mat final_map(width, height, CV_8UC1, map2);

    double inverse_resolution = grid.GetInverseResolution();

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // update the current state
            // see operator=(Pose2D) overloading
            s = n->pose;
            s.gear = n->action->gear;

            // save the current state to the list
            states.push_front(s);

            // convert the current position to row and col
            astar::GridCellIndex index(grid.PoseToIndex(s.position));

            // convert to the opencv format
            cv::Point p1(index.col - 1, height - index.row - 1);
            cv::Point p2(index.col + 1, height - index.row + 1);

            // draw a single square
            cv::rectangle(final_map, p1, p2, cv::Scalar(0.0, 0.0, 0.0), -1);

            // show the imgae
            cv::imshow("Astar", final_map);
            cv::waitKey(30);

        } else {

            // get the subpath provided by the action set discretization
            StateArrayPtr subpath = ReedsSheppModel::DiscretizeRS(n->parent->pose, n->action_set, vehicle.min_turn_radius, 0.3);

            // get the path size
            subpathSize = subpath->states.size();

            // prepend the resulting subpath to the current path
            for (int i = subpathSize - 1; i > 0; i--) {

                // save the current state to the list
                states.push_front(subpath->states[i]);

                // convert the current position to row and col
                astar::GridCellIndex index(grid.PoseToIndex(subpath->states[i].position));

                // convert to the opencv format
                cv::Point p1(index.col - 1, height - index.row - 1);
                cv::Point p2(index.col + 1, height - index.row + 1);

                // draw a single square
                cv::rectangle(final_map, p1, p2, cv::Scalar(0.0, 0.0, 0.0), -1);

                // show the imgae
                cv::imshow("Astar", final_map);
                cv::waitKey(30);

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
    if (0 < out.size()) {

        out.front().phi = start.phi;
        out.front().v = start.v;

        out.back().v = goal.v;

    }

    delete [] map2;

    return path;

}

// get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
HybridAstarNodePtr HybridAstar::GetReedsSheppChild(const Pose2D &start, const Pose2D &goal) {

    // get the resulting set of actions
    ReedsSheppActionSetPtr action_set = rs.Solve(start, goal, vehicle.min_turn_radius);

    double inverse_resolution = grid.GetInverseResolution();

    if (0 < action_set->actions.size()) {

        // flag to validate the Reeds-Shepp curve
        bool safe = true;

        // get the states in the Reeds-Shepp curve
        StateArrayPtr path = rs.DiscretizeRS(start, action_set, vehicle.min_turn_radius, inverse_resolution);

        // a reference helper, just in case
        std::vector<State2D> &states(path->states);

        // get the states end pointer
        std::vector<State2D>::iterator end = states.end();

        // iterate over the state list
        for (std::vector<State2D>::iterator it = states.begin(); it != end; ++it) {

            if (!grid.isValidPoint(it->position) || !grid.isSafePlace(vehicle.GetVehicleBodyCircles(*it), vehicle.safety_factor)) {

                // not a good state
                safe = false;

                break;

            }

        }

        // clear the state array
        delete path;

        if (safe) {

            std::cout << "Valid RS!\n";

            // return the HybridAstarNode on the goal state
            return new HybridAstarNode(goal, action_set);

        } else {

            std::cout << "Invalid RS!\n";

        }

    }

    // delelete the action set
    delete action_set;

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

    // double turn_radius
    double tr = vehicle.min_turn_radius;

    // iterate over the steering moves
    for (unsigned int j = 0; j < astar::NumSteering; j++) {

        // casting the steering
        Steer steer = static_cast<Steer>(j);

        // get the next state
        child_pose = vehicle.NextPose(start, steer, gear, length, tr);

        // verify the safety condition and the grid boundary
        if (grid.isValidPoint(child_pose.position) && grid.isSafePlace(vehicle.GetVehicleBodyCircles(child_pose), vehicle.safety_factor)) {

            HybridAstarNodePtr tmp = new HybridAstarNode(child_pose, new ReedsSheppAction(steer, gear, length));

            if (nullptr == tmp) {

                std::cout << "Erro Memória!!\n";
            }

            // append to the children list
            nodes.push_back(tmp);

        }

    }

    // expanding Reeds-Shepp curves now

    // Reeds-Shepp curve threshold value
    // double threshold = heuristic.GetHeuristicValue(start, goal);
    double threshold = heuristic.GetHeuristicValue(start, goal);
    double inverseThreshold = 10.0/(threshold*threshold);

    // quadratic falloff
    if (10.0 > threshold || inverseThreshold > rand()) {

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
StateArrayPtr HybridAstar::FindPath(InternalGridMapRef grid_map, const State2D &start, const State2D &goal) {

    // get the grid map pointer
    // useful inside others methods, just to avoid passing the parameter constantly
    // now, all HybridAstar methods have access to the same grid pointer
    grid = grid_map;

    // get the current grid map
    map = grid.GetGridMap();
    width = grid.GetWidth();
    height = grid.GetHeight();

    // get the grid map resolution
    double resolution = grid.GetResolution();

    // draw the current map
    cv::Mat final_map(width, height, CV_8UC1, map);

    // show the image
    cv::imshow("Astar", final_map);

    cv::waitKey(30);

    // syntactic sugar
    // ge the reference to the base class
    Pose2D goal_pose(goal.position, goal.orientation);
    Pose2D start_pose(start.position, start.orientation);

    // update the heuristic to the new goal
    heuristic.UpdateHeuristic(grid_map, start_pose, goal_pose);

    // the start state heuristic value
    double heuristic_value = heuristic.GetHeuristicValue(start_pose, goal_pose);

    // find the current cell
    GridMapCellPtr c = grid_map.PoseToCell(start_pose);

    // the available space around the vehicle
    // provides the total length between the current state and the node's children
    double length = start.v * start.t;

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

    // the cost from the start to the current position
    double tentative_g;

    // the total estimated cost
    // from the start to the goal
    double tentative_f;

    // the actual A* algorithm
    while(!open.isEmpty()) {

        n = open.DeleteMin();

        // is it the desired goal?
        if (goal_pose == n->pose) {

            // rebuild the entire path
            StateArrayPtr resulting_path = RebuildPath(n, start, goal);

            // clear all opened and expanded nodes
            RemoveAllNodes();

            // return the path
            return resulting_path;

        }

        // add to the explored set
        n->cell->status = ExploredNode;

        // get the length based on the environment
        double obst = grid_map.GetObstacleDistance(n->pose.position);
        double voro_dist = grid_map.GetVoronoiDistance(n->pose.position);

        length = std::max(resolution, 0.5 * (obst + voro_dist));

        // iterate over the gears
        for (unsigned int i = 0; i < NumGears; i++) {

            // casting the gears
            Gear gear = static_cast<Gear>(i);

            // get the children nodes by expanding all gears and steering
            HybridAstarNodeArrayPtr children = GetChidlren(n->pose, goal_pose, gear, length);

            // reference, just a syntactic sugar
            std::vector<HybridAstarNodePtr> &nodes(children->nodes);

            // iterate over the current node's children
            for (std::vector<HybridAstarNodePtr>::iterator it = nodes.begin(); it != nodes.end(); ++it) {

                // avoid a lot of indirect access
                HybridAstarNodePtr child = *it;

                // find the appropriated location in the grid
                // reusing the same "c" pointer declared above
                c = grid_map.PoseToCell(child->pose);

                // we must avoid node->cell = nullptr inside the HybridAstarNode
                if (nullptr != c && ExploredNode != c->status) {

                    if (nullptr != child->action) {

                        double path_cost = PathCost(n->action->gear, child->pose, gear, length);
                        // we a have a valid action, conventional expanding
                        tentative_g = n->g + path_cost;

                    } else if (0 < child->action_set->Size()) {

                        double action_path_cost = child->action_set->CalculateCost(vehicle.min_turn_radius, reverse_factor, gear_switch_cost);

                        // we have a valid action set, it was a Reeds-Shepp analytic expanding
                        tentative_g = n->g + action_path_cost;

                    } else {

                        // we don't have any valid action, it's a bad error
                        // add to the invalid nodes set
                        invalid.push_back(child);

                        // jump to the next iteration
                        continue;
                        // throw std::exception();

                    }

                    // update the heuristic contribution
                    double h = heuristic.GetHeuristicValue(child->pose, goal_pose);

                    tentative_f = tentative_g + h;

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

                    } else if (tentative_f < c->node->f) {

                        // the cell has a node but the current child is a better one

                        // update the node at the cell
                        HybridAstarNodePtr current = c->node;

                        // the old node is updated but not the corresponding Handle/Key in the priority queue
                        current->UpdateValues(*child);

                        // decrease the key at the priority queue
                        open.DecreaseKey(current->handle, tentative_f);

                    }

                }

                // add the node to the discovered set
                discovered.push_back(child);

            }

            // delete the children vector
            delete children;

        }

    }

    // the A* could no find a valid path to the goal
    RemoveAllNodes();

    return new StateArray();

}

// get the path cost
double HybridAstar::PathCost(
    astar::Gear start_gear,
    const astar::Pose2D& goal_pose,
    astar::Gear next_gear,
    double length)
{
    //
    double reverse_cost = 0.0;

    // reverse penalty
    if (BackwardGear == next_gear)
        reverse_cost =  length * reverse_factor;

    // gear change penalty
    if (start_gear != next_gear)
        reverse_cost += gear_switch_cost;

    // compute and return the cost
    return reverse_cost + length + length * voronoi_field_factor * grid.GetPathCost(goal_pose.position);
}
