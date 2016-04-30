#include "HybridAstar.hpp"

#include "../VehicleModel/VehicleModel.hpp"

using namespace astar;

HybridAstar::HybridAstar() : grid(nullptr) {}

// PRIVATE METHODS

// remove all nodes
void HybridAstar::removeAllNodes() {

    HybridAstarNodePtr tmp;

    // clear the open set
    while(!open.empty()) {

        // get the top element
        tmp = open.top();

        // remove the node form the set
        open.pop();

        // delete the node
        delete(tmp);

    }

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
PoseListPtr HybridAstar::rebuildPath(HybridAstarNodePtr n) {

    // create the list
    PoseListPtr path = new PoseList();

    // auxiliar node ptr
    HybridAstarNodePtr tmp = nullptr;

    // counter
    unsigned int subPathSize = 0;

    // reference syntatic sugar
    std::list<Pose2D> &poses(path->poses);

    // building the path
    while(nullptr != n) {

        // is there a single action?
        if (nullptr != n->action) {

            // save the current pose to the list
            poses.push_front(n.pose);

        } else if (nullptr != n->actionSet) {

            // get the subpath provided by the action set discretization
            PoseArrayPtr subPath = ReedsSheppModel::discretize(n->parent->pose, n->actionSet, vehicle.defaultTurnRadius, grid->inverseResolution);

            // get the path size
            subPathSize = subPath->poses.size();

            // prepend the resulting subPath to the curren path
            for (unsigned int i = subPathSize - 1; i >= 0; i--) {

                // save the current pose to the list
                poses.push_front(subPath->poses[i]);

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
HybridAstarNodePtr HybridAstar::getReedsSheppChild(const Pose2D &start, const Pose2D &goal) {

    // get the resulting set of actions
    ReedsSheppActionSetPtr actionSet = rs.solve(start, goal, vehicle.defaultTurnRadius);

    if (0 < actionSet->actions.size()) {

        // flag to validate the Reeds-Shep curve
        bool safe = true;

        // get the poses in the Reeds-Shepp curve
        PoseArrayPtr path = rs.discretize(start, actionSet, vehicle.defaultTurnRadius, grid->inverseResolution);

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
HybridAstarNodeArrayPtr HybridAstar::getChildren(const Pose2D &start, const Pose2D &goal, Gear gear, double length) {

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
        child = vehicle.nextPose(start, s, gear, length, vehicle.defaultTurnRadius);

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
    double threshold = heuristic.getHeuristicValue(start, goal);
    double inverseThreshold = 10.0/(threshold*threshold);

    // quadratic fallof
    if (10 > threshold || inverseThreshold < rand()) {

        // the a new HybridAstarNode based on ReedsSheppModel
        HybridAstarNodePtr rsNode = getReedsSheppChild(start, goal);

        if (nullptr != rsNode) {

            // append to the children list
            nodes.push_back(rsNode);

        }
    }

    return children;

}

// PUBLIC METHODS

// receives the grid, start and goal poses and find a path, if possible
PoseListPtr HybridAstar::findPath(InternalGridMap &gridMap, const Pose2D& start, const Pose2D& goal) {

    // get the grid map pointer
    // useful inside others methods, just to avoid passing the parameter constantly
    // now, all HybridAstar methods have access to the same grid pointer
    grid = &gridMap;

    // update the heuristic to the new goal
    heuristic.updateGoal(gridMap, goal);

    // helpers
    double heuristicValue = heuristic.getHeuristicValue(gridMap, start, goal);

    // find the current cell
    CellPtr c = gridMap.poseToCell(start);

    // the available space around the vehicle
    // provides the total length between the current pose and the node's children
    double length;

    // the simple case of length
    // length = gridMap.resolution

    // the simple case of dt
    // dt = gridMap.resolution/vehicle.defaultSpeed

    // create a new Node
    HybridAstarNodePtr n = new HybridAstarNode(start, new ReedsSheppAction(astar::RSStraight, ForwardGear, 0), c, 0, heuristicValue, nullptr);

    // push the start node to the queue
    open.push(n);

    // push the start node to the discovered set
    discovered.push_back(n);

    // reverse flag
    bool reverseGear;

    // the cost from the start to the current position
    double tentativeG;

    // the total estimated cost
    // from the start to the goal
    double tentativeF;

    // the actual A* algorithm
    while(!open.empty()) {

        // get the hight priority node
        n = open.top();

        // remove it from the open set
        open.pop();

        // is it the desired goal?
        if (goal == n->pose) {

            // rebuild the entire path
            PoseListPtr resultingPath = rebuildPath(n, vehicle.defaultTurnRadius, gridMap.inverseResolution);

            // clear all opened and expanded nodes nodes
            removeAllNodes();

            // return the path
            return resultingPath;

        }

        // add to the explored set
        n->cell->status = ExploredNode;

        // get the length based on the environment
        length = std::max(gridMap.resolution, 0.5 * (gridMap.getObstacleDistance(n->pose.position) + gridMap.getVoronoiDistance(n->pose.position)));

        // iterate over the gears
        for (unsigned int i = 0; i < NumGears; i++) {

            // casting the gears
            Gear gear = static_cast<Gear>(i);

            // get the children nodes by expanding all gears and steeering
            HybridAstarNodeArrayPtr children = getChildren(n->pose, goal, gear, length);

            // reference syntatic sugar
            std::vector<HybridAstarNodePtr> &nodes(children->nodes);

            // iterate over the entire array
            // reverse ordering - easier element erasing
            for (std::vector<HybridAstarNodePtr>::iterator it = nodes.begin(); it < nodes.end(); ++it) {

                // find the appropiated location in the grid
                c = gridMap.poseToCell(it->pose);

                if (ExploredNode != c->status) {

                    // reverse?
                    reverseGear = (ForwardGear == gear);

                    if (nullptr != it->action) {

                        // we a have a valid action, conventional expanding
                        tentativeG = n->g + pathCost(n->pose, it->pose, length, reverseGear);

                    } else if (0 < it->actionSet->size()) {

                        // we have a valid action set, Reeds-Shepp analytic expanding
                        tentativeG = n->g + it->actionSet->calculateCost(vehicle.defaultTurnRadius, reverseFactor, gearSwitchCost);

                    } else {

                        // we don't have any valid action, it's a bad error
                        // add to the invalid nodes set
                        invalid.push_back(it);

                        continue;
                        // throw std::exception();

                    }

                    // get the estimated cost
                    tentativeF = tentativeG + heuristic.getHeuristicValue(it->pose, goal);

                    // update the cost
                    it->g = tentativeG;

                    // update the heuristic value
                    it->f = tentativeF;

                    // set the parent
                    it->parent = n;

                    // is a not opened node?
                    if (UnknownNode == c->status) {

                        // the cell is empty and doesn't have any node

                        // set the cell pointer
                        it->cell = c;

                        // update the cell pointer
                        c->node = it;

                        // udpate the cell status
                        c->status = OpenedNode;

                        // update the cell value, same value used in the node
                        c->value = tentativeF;

                        // add to the open set
                        open.push(it);

                    } else if (OpenedNode == c->status && tentativeF < c->value) {

                        // the cell has a node but the current child is a better one

                        // update the node at the cell
                        *(c->node) = *it;

                        // update the node's MapCellPtr'
                        it->cell = c;

                        // udpate the cell status
                        c->status = OpenedNode;

                        // update the cell value, same value used in the node
                        c->value = tentativeF;

                        // pryority queue consolidation
                        open.consolidate();

                    }

                }

                // add the node to the discovered set set
                discovered.push_back(it);

            }

        }


    }

}
