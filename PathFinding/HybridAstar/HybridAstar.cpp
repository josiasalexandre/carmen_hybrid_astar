#include "HybridAstar.hpp"

#include "../VehicleModel/VehicleModel.hpp"

#include <thread>

using namespace astar;

HybridAstar::HybridAstar() : grid(nullptr), reverse_factor(4), gear_switch_cost(8) {}

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
PoseListPtr
HybridAstar::ReBuildPath(HybridAstarNodePtr n)
{

    // create the list
    PoseListPtr path = new PoseList();

    // auxiliar node ptr
    HybridAstarNodePtr tmp = nullptr;

    // counter
    unsigned int subpathSize = 0;

    // reference syntatic sugar
    std::list<Pose2D> &poses(path->poses);

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // save the current pose to the list
            poses.push_front(n.pose);

        } else if (nullptr != n->action_set) {

            // get the subpath provided by the action set discretization
            PoseArrayPtr subpath = ReedsSheppModel::Discretize(n->parent->pose, n->action_set, vehicle.default_turn_radius, grid->inverse_resolution);

            // get the path size
            subpathSize = subpath->poses.size();

            // prepend the resulting subpath to the curren path
            for (unsigned int i = subpathSize - 1; i >= 0; i--) {

                // save the current pose to the list
                poses.push_front(subpath->poses[i]);

            }

        }

        // move to the parent node
        n = n.parent;

    }

    // move every node's gear to it's parent node
    if (0 < poses.size()) {

        // get the end
        std::list<Pose2D>::iterator last = poses.end() - 1;

        // helper
        std::list<Pose2D>::iterator tmp;
        for (std::list<Pose2D>::iterator it = poses.begin(); it < last; ++it) {

            // get the next node
            tmp = it + 1;

            // copy the next gear
            it->gear = tmp->gear;

        }

    }

    return path;


}

// get the Reeds-Shepp path to the goal and return the appropriated HybridAstarNode
HybridAstarNodePtr HybridAstar::GetReedsSheppChild(const Pose2D &start, const Pose2D &goal) {

    // get the resulting set of actions
    ReedsSheppActionSetPtr action_set = rs.Solve(start, goal, vehicle.default_turn_radius);

    if (0 < action_set->actions.size()) {

        // flag to validate the Reeds-Shep curve
        bool safe = true;

        // get the poses in the Reeds-Shepp curve
        PoseArrayPtr path = rs.Discretize(start, action_set, vehicle.default_turn_radius, grid->inverse_resolution);

        // a reference helper, just in case
        std::vector<Pose2D>& poses(path->poses);

        // iterate over the pose list
        for (std::vector<Pose2D>::iterator it = poses.begin(); it < poses.end(); ++it) {

            if (!grid->isValidPoint(it->position) || !grid->isSafePlace(*it)) {

                // not a good pose
                safe = false;

                break;

            }

        }

        if (safe) {

            // return the HybridAstarNode on the goal pose
            return new HybridAstarNode(goal, path, nullptr, 0, 0, nullptr);

        }

    }

    return nullptr;

}

// get the children nodes by expanding all gears and steeering
HybridAstarNodeArrayPtr HybridAstar::GetChidlren(const Pose2D &start, const Pose2D &goal, Gear gear, double length) {

    // build the vector
    HybridAstarNodeArrayPtr children = new HybridAstarNodeArray();

    // a reference syntatic sugar
    std::vector<HybridAstarNodePtr> &nodes(children->nodes);

    // a pose helper, the current child
    Pose2D child;

    // iterate over the steering moves
    for (unsigned int j = 0; j < astar::NumSteering; j++) {

        // casting the steering
        Steer s = static_cast<Steer>(j);

        // get the next pose
        child = vehicle.NextPose(start, s, gear, length, vehicle.default_turn_radius);

        // set the gear
        child.gear = gear;

        // verify the safety condition and the grid boundary
        if (grid->isValidPoint(child.position) && grid->isSafePlace(child)) {

            // append to the children list
            nodes.push_back(new HybridAstarNode(child, ReedsSheppAction(s, gear, length), nullptr, 0, 0, nullptr));

        }

    }

    // expanding Reeds-Shepp curves now

    // Reeds-Shepp curve threshold value
    double threshold = heuristic.GetHeuristicValue(start, goal);
    double inverseThreshold = 10.0/(threshold*threshold);

    // quadratic fallof
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

// receives the grid, start and goal poses and find a path, if possible
PoseListPtr HybridAstar::FindPath(InternalGridMap &grid_map, const Pose2D& start, const Pose2D& goal) {

    // get the grid map pointer
    // useful inside others methods, just to avoid passing the parameter constantly
    // now, all HybridAstar methods have access to the same grid pointer
    grid = &grid_map;

    // update the heuristic to the new goal
    heuristic.UpdateGoal(grid_map, goal);

    // the start pose heuristic value
    double heuristic_value = heuristic.GetHeuristicValue(grid_map, start, goal);

    // find the current cell
    CellPtr c = grid_map.PoseToCell(start, false);

    // the available space around the vehicle
    // provides the total length between the current pose and the node's children
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

        // get the hight priority node
        n = open.DeleteMin();

        // is it the desired goal?
        if (goal == n->pose) {

            // rebuild the entire path
            PoseListPtr resulting_path = ReBuildPath(n, vehicle.default_turn_radius, grid_map.inverse_resolution);

            // clear all opened and expanded nodes
            std::thread clear_nodes(astar::HybridAstar::RemoveAllNodes(), this);

            // detach the current thread from the child one
            clear_nodes.detach();

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

            // get the children nodes by expanding all gears and steeering
            HybridAstarNodeArrayPtr children = GetChidlren(n->pose, goal, gear, length);

            // reference syntatic sugar
            std::vector<HybridAstarNodePtr> &nodes(children->nodes);

            // iterate over the entire array
            // reverse ordering - easier element erasing
            for (std::vector<HybridAstarNodePtr>::iterator it = nodes.begin(); it < nodes.end(); ++it) {

                // find the appropiated location in the grid
                // reusing the same "c" pointer declared above
                c = grid_map.PoseToCell(it->pose, reverse_gear);

                // we must avoid cell = nullptr inside the HybridAstarNode
                if (nullptr != c) {

                    if (ExploredNode != c->status) {

                        if (nullptr != it->action) {

                            // we a have a valid action, conventional expanding
                            tentative_g = n->g + PathCost(n->pose, it->pose, length, reverse_gear);

                        } else if (0 < it->action_set->size()) {

                            // we have a valid action set, it was a Reeds-Shepp analytic expanding
                            tentative_g = n->g + it->action_set->CalculateCost(vehicle.default_turn_radius, reverse_factor, gear_switch_cost);

                        } else {

                            // we don't have any valid action, it's a bad error
                            // add to the invalid nodes set
                            invalid.push_back(it);

                            // jump to the next iteration
                            continue;
                            // throw std::exception();

                        }

                        // get the estimated cost
                        tentative_f = tentative_g + heuristic.GetHeuristicValue(it->pose, goal);

                        // update the cost
                        it->g = tentative_g;

                        // update the total value / g cost plus heuristic value
                        it->f = tentative_f;

                        // set the parent
                        it->parent = n;

                        // is it a not opened node?
                        if (UnknownNode == c->status) {

                            // the cell is empty and doesn't have any node

                            // set the cell pointer
                            it->cell = c;

                            // update the cell pointer
                            c->node = it;

                            // udpate the cell status
                            c->status = OpenedNode;

                            // add to the open set
                            it->handle = open.Add(it, tentative_f);

                        } else if (OpenedNode == c->status && tentative_f < c->node->f) {

                            // the cell has a node but the current child is a better one

                            // update the node at the cell
                            // copy the handle to the current child, just to avoid losing that information
                            it->handle = c->node->handle;

                            // the old node is updated but not the Key in the priority queue
                            // the handle is not not erased
                            *(c->node) = *it;

                            // udpate the cell status
                            c->status = OpenedNode;

                            // decrease the key at the priority queue
                            open.DecreaseKey(c->node->handle, tentative_f);

                        }

                    }

                }

                // add the node to the discovered set
                discovered.push_back(it);

            }

        }

    }

    // the A* could no find a valid path to the goal
    return nullptr;

}
