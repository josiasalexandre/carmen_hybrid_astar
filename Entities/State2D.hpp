#ifndef STATE_2D_HPP
#define STATE_2D_HPP

#include <list>
#include <vector>

#include "../Helpers/wrap2pi.hpp"
#include "Vector2D.hpp"
#include "ReedsShepp/ReedsSheppAction.hpp"

namespace astar {

class State2D {

    public:

        // PUBLIC ATTRIBUTES

        // the position, x and y coordinates
        astar::Vector2D<double> position;

        // the heading
        double orientation;

        // registering the wheel angle
        double wheel_angle;

        // the speed at the current pose
        double v;

        // how much time
        double time;

        // the gear
        astar::Gear gear;

        // simple constructor
        State2D();

        // basic constructor, only position and orientation are required
        State2D(const astar::Vector2D<double> &pos, double o, double w_angle = 0, double vel = 0, double dt = 0, astar::Gear g = ForwardGear);

        // most explicit constructor
        State2D(double x_, double y_, double o, double w_angle = 0, double vel = 0, double dt = 0, astar::Gear g = ForwardGear);

        // copy constructor
        State2D(const State2D&);

        // distance between two poses
        double Distance(const State2D&);

        // distance squared between two poses
        double Distance2(const State2D&);

        // get difference between orientations (yaw)
        double GetOrientationDiff(const State2D&);

        // get difference between orientations (yaw), overloaded version
        double GetOrientationDiff(double);

        // the assignment operator
        void operator=(const State2D&);

        // == operator overloading
        bool operator==(const State2D&);

        // != operator overloading
        bool operator!=(const State2D&);

};

// a helper class to avoid copies
class StateList {

public:

    // EVERYTHING PUBLIC
    std::list<State2D> states;

};

typedef StateList* StateListPtr;

//
class StateArray {

public:

    // EVERYTHING PUBLIC
    std::vector<State2D> states;

};

typedef StateArray* StateArrayPtr;

}

#endif
