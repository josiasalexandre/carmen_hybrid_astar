#ifndef STATE_2D_HPP
#define STATE_2D_HPP

#include <list>
#include <vector>

#include "../Helpers/wrap2pi.hpp"
#include "Vector2D.hpp"
#include "Pose2D.hpp"
#include "ReedsShepp/ReedsSheppAction.hpp"

namespace astar {

class State2D : public astar::Pose2D {

    public:

        // PUBLIC ATTRIBUTES

    // the current gear
        astar::Gear gear;

        // the desired speed
        double v;

        // the steering angle
        double phi;

        // the command time
        double t;

        // distance to the last cuscp
        double last_cusp_dist;

        // coming to stop flag
        bool coming_to_stop;

        // simple constructor
        State2D();

        // simple constructor, the input is a pose
        State2D(
                const astar::Pose2D&, astar::Gear g = astar::ForwardGear, double vel = 0.0, double wheel_angle = 0.0,
                double t_ = 0.0, double lcd = 0, bool stop = false);

        // copy constructor
        State2D(const astar::State2D&);

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
