#include "StanleyController.hpp"
#include <array>

using namespace astar;

// basic constructor
StanleyController::StanleyController(const VehicleModel &vehicle) :
        vehicle_model(vehicle), cs(CSStandby), next_waypoint(0), prev_waypoint(0),
        dt(0.1), reverse_mode(false), front_axle(), fake_front_axle(), closest_point(),
        left(), right(), raw_path(), forward_path(), reverse_path(), consolidated_path(false),
        raw_path_size(0), raw_path_last_index(0), stopping(), prev_wheel_angle_error(0), vpasterror(0.0), last_cusp(0) {}

// update the path around stopping points
void StanleyController::UpdateLowSpeedRegions(const VehicleModel &vehicle) {

    // get the stopping points vector size
    unsigned int s_size = stopping.size();

    double acceleration_constraint;
    int prev_index, next_index, current_stop;

    State2D &prev, &next;

    for (unsigned int i = 0; i < s_size; i++) {

        // get the path index
        prev_index = next_index = current_stop = stopping[i];

        // go to the left
        prev_index -= 1;
        if (0 <= prev_index) {

            if (0.0 == raw_path[next_index].v) {

                // the speed should be low around the stopping point
                raw_path[prev_index].v = forward_path[prev_index].v = reverse_path[prev_index].v = 0.2;

                // this is a coming to stop point
                raw_path[prev_index].coming_to_stop = forward_path[prev_index].coming_to_stop = reverse_path[i].coming_to_stop = true;

                // update the next index
                next_index = prev_index;

                // update the prev index
                prev_index -= 1;

            }

            while (0 <= prev_index) {

                // get the desired references
                prev = raw_path[prev_index];
                next = raw_path[next_index];

                if (prev.v > next.v) {

                    acceleration_constraint = vehicle_model.GetDecelerationConstraint(next.v, prev.position.Distance(next.position), prev.gear);

                } else {

                    acceleration_constraint = vehicle_model.GetAccelerationConstraint(prev.v, prev.position.Distance(next.position), prev.gear);

                }

                // update the current speed
                if (acceleration_constraint < prev.v) {

                    raw_path[prev_index].v = forward_path[prev_index].v = reverse_path[prev_index].v = acceleration_constraint;

                } else {

                    break;

                }

                // update the indexes
                next_index = prev_index;
                prev_index -= 1;

            }

        }

        // reset the indexes
        prev_index = next_index = current_stop;

        // got the right
        next_index += 1;
        if (raw_path_size > next_index) {

            // the speed should be low around the stopping point
            raw_path[next_index].v = forward_path[next_index].v = reverse_path[next_index].v = 0.2;

            // update the prev index
            prev_index = next_index;

            // update the next index
            next_index += 1;

            while(raw_path_size > next_index) {

                // get the desired references
               prev = raw_path[prev_index];
               next = raw_path[next_index];

               acceleration_constraint = vehicle_model.GetDecelerationConstraint(next.v, prev.position.Distance(next.position), prev.gear);

               // update the current speed
               if (acceleration_constraint < raw_path[next_index].v) {

                   raw_path[next_index].v = forward_path[next_index].v = reverse_path[next_index].v = acceleration_constraint;

               } else {

                   break;

               }

                // update the indexes
                prev_index = next_index;
                next_index += 1;

            }

        }

    }

}

// the first and the last states remains the same
bool StanleyController::ConsolidateStateList(StateListPtr input_path) {

    // syntactic sugar
    std::list<State2D> &input(input_path->states);

    // clear the internal paths
    raw_path.clear();
    forward_path.clear();
    reverse_path.clear();

    // clear the stopping list
    stopping.clear();

    // get the limit iterators
    std::list<State2D>::iterator end = input.end();
    std::list<State2D>::iterator last = --input.end();

    // get the list iterators
    std::list<State2D>::iterator prev = input.begin();
    std::list<State2D>::iterator current = std::advance(prev, 1);
    std::list<State2D>::iterator next = std::advance(current, 1);

    // save the first state to the internal state lists
    raw_path.push_back(*prev);
    forward_path.push_back(vehicle_model.GetFrontAxleState(*prev));
    reverse_path.push_back(vehicle_model.GetFakeFrontAxleState(*prev));

    // register the stopping points index
    unsigned int index = 0;

    // is the car stopped?
    if (0.0 == prev->v) {

       // save to the stopping points list
        stopping.push_back(index);

        // set the car controller state
        cs = CSStopped;

    } else {

        if (ForwardGear == prev->gear) {

            // set the car controller state
            cs = CSForwardDrive;

        } else {

            // set the car controller state
            cs = CSReverseDrive;

        }

    }

    // get the desired command between the current and the next point
    prev->phi = vehicle_model.GetDesiredWheelAngle(*prev, *current);

    // update the index
    index += 1;

    // iterate the entire list but the second last element
    while (next != end) {

        // the common case
        if (current->gear == prev->gear) {

            // get the desired wheel angle
            current->phi = vehicle_model.GetDesiredWheelAngle(*current, *next);

            if (ForwardGear == current->gear) {

                // get the desired orientation at this point
                current->orientation = vehicle_model.GetForwardOrientation(*prev, *current, *next);

                // get the max speed
                current->v = vehicle_model.GetForwardSpeed(*prev, *current, *next);


            } else {

                // get the desired orientation at this point
                current->orientation = vehicle_model.GetBackwardOrientation(*prev, *current, *next);

                // get the max speed
                current->v = vehicle_model.GetBackwardSpeed(*prev, *current, *next);

                // get the desired wheel angle
                current->phi = -current->phi;

            }


        } else {

            // the speed should be zero around any stopping point
            current->v = 0.0;

            // save to the stopping points list
            stopping.push_back(index);

        }

        // append to the current raw path
        raw_path.push_back(*current);

        // append to the current forward path
        forward_path.push_back(vehicle_model.GetFrontAxleState(*current));

        // append to the current internal path
        reverse_path.push_back(vehicle_model.GetFakeFrontAxleState(*current));

        // update the index
        index += 1;

        // update the iterators
        prev = current;
        current = next;
        ++next;

    }

    // the last point has the goal speed
    // we don't need to change that speed
    // append to the current raw path
    raw_path.push_back(*last);

    // append to the current forward path
    forward_path.push_back(vehicle_model.GetFrontAxleState(*last));

    // append to the current internal path
    reverse_path.push_back(vehicle_model.GetFakeFrontAxleState(*last));

    // verify the parking mode
    // save to the stopping points list
    stopping.push_back(index);

    // update the reference sizes
    // get the input path size and the last index
    raw_path_size = raw_path.size();
    raw_path_last_index = raw_path_size - 1;

    // get the required speed around curves and stopping points
    // TODO -> we need to get slower speed next to obstacles
    UpdateLowSpeedRegions(vehicle);

    return true;

}

// how far the car has moved between two points in the path
double StanleyController::HowFarAlong(const Pose2D &current, const Pose2D &prev, const Pose2D &next) {

    Vector2D<double> r = current.position - prev.position;
    Vector2D<double> d = next.position - prev.position;

    double dxy = (d.x * d.x + d.y * d.y);

    return ((r.x * d.x + r.y * d.y)/dxy);

}

// find the previous and next index in the path
void StanleyController::Localize(const astar::State2D &s, unsigned int &prev_index, unsigned int &next_index) {

    // Find closest path point to back axle
    double inf = std::numeric_limits<double>::infinity();
    double bestd = inf;
    int start = std::max<int>(last_cusp, std::max<int>(0, next_waypoint - 2));
    int end = std::min<int>(raw_path_last_index, next_waypoint + 2);
    int besti = start;

    for (int i = start + 1; i < end; i++) {

        if (raw_path[i-1].coming_to_stop)
            break;

        float d = s.position.Distance(raw_path[i].position);
        if (d < bestd) {

            bestd = d;
            besti = i;

        } else if (d != inf && d > bestd) {
            break;
        }

    }

    next_index = besti;

    if (raw_path_size <= besti + 1) {

        prev_index = besti - 1;

    } else if (0 > besti - 1) {

        prev_index = next_index;
        next_index = besti + 1;

    } else if (besti == last_cusp) {

        next_index = besti + 1;
        prev_index = besti;

    } else {

        Vector2D<double> prevmaybe(raw_path[besti - 1].position);
        Vector2D<double> nextmaybe(raw_path[besti + 1].position);

        if (s.position.Distance2(prevmaybe) < s.position.Distance2(nextmaybe)) {

            prev_index = besti - 1;

        } else {

            prev_index = next_index;
            next_index = besti + 1;

        }

    }

    if (raw_path[prev_index - 1].coming_to_stop && prev_index != prev_waypoint) {

        prev_index--;
        next_index--;

    }

}

// find the closest point next to the front axle, fake or not
State2D StanleyController::FindClosesPoint(const astar::State2D &s, const astar::State2D &prev, const astar::State2D &next) {

    State2D state(s);

    if (next.position.x == prev.position.x) {

        // Avoid division by zero
        state.position.x = next.position.x;
        state.position.y = s.position.y;

    } else {

        double m = (next.position.y - prev.position.y) / (next.position.x - prev.position.x);
        double m2 = m * m;
        double b = next.position.y - m * next.position.x;

        state.position.x = (m * s.position.y + s.position.x - m * b) / (m2 + 1);
        state.position.y = (m2 * s.position.y + m * s.position.x + b) / (m2 + 1);

    }

    return state;

}

// the correct action at some stopping point
State2D StanleyController::Stopped(const State2D &s) {

    double phi_error;
    double desired_heading;
    ControllerState next_controller_state;

    State2D state(s);

    // syntactic sugar
    std::vector<State2D> &states = (ForwardGear == forward_path[prev_waypoint].gear) ? forward_path : reverse_path;

    State2D &next(states[next_waypoint]);
    State2D &prev(states[prev_waypoint]);

    if (raw_path_last_index == next_waypoint && raw_path_last_index == prev_waypoint) {

        desired_heading = s.orientation;

        // mission complete
        next_controller_state = CSComplete;

    } else {

        unsigned int last_waypoint = states.size() - 1;

        Vector2D<double> heading = next.position - prev.position;

        desired_heading = std::atan2(heading.y, heading.x);

        next_controller_state = (ForwardGear == prev.gear) ? CSForwardDrive : CSReverseDrive;

    }

    phi_error = -mrpt::math::wrapToPi<double>(s.orientation - desired_heading) - s.phi;

    if (0.001 > std::fabs(phi_error) || 0.1 > std::fabs(std::fabs(s.phi) - vehicle_model.max_wheel_deflection)) {

        // start to move
        cs = next_controller_state;

        // what is desired speed?
        states[prev_waypoint].v = states[next_waypoint].v;

    }

    // update the command values
    state.v = 0.0;
    state.phi = phi_error * 8.0;
    state.t = dt;

    return state;

}

// the main Stanley method
State2D StanleyController::ForwardDrive(const State2D &s) {

    unsigned int prev_index, next_index;

    // find the previous and next index in the path
    Localize(s, prev_index, next_index);

    // update the way points
    prev_waypoint = prev_index;
    next_waypoint = next_index;

    // copy the input state
    State2D state(s);

    // set the reverse mode flag
    reverse_mode = (BackwardGear == forward_path[prev_index].gear);

    std::vector<State2D> &the_path = reverse_mode ? reverse_path : forward_path;

    // get the previous and next states
    State2D &prev(the_path[prev_index]);
    State2D &next(the_path[prev_index]);

    fake_front_axle = vehicle_model.GetFakeFrontAxleState(s);
    front_axle = reverse_mode ? fake_front_axle : vehicle_model.GetFrontAxleState(s);
    closest_point = FindClosesPoint(front_axle, prev, next);

    double how_far = std::min(std::max(0.0 , HowFarAlong(s, prev, next)), 1.0);

    Vector2D<double> prev_point(prev.position);
    Vector2D<double> next_point(next.position);

    Vector2D<double> heading = (0 < prev_index) ? next_point - the_path[prev_index - 1].position : next_point - prev_point;
    double desired_heading = std::atan2(heading.y, heading.x);

    double next_heading;

    if (s.coming_to_stop) {

        next_heading = next.orientation + (reverse_mode ? M_PI : 0);

    } else {

        Vector2D<double> nhv = the_path[next_index + 1].position - prev.position;
        next_heading = std::atan2(nhv.y, nhv.x);

    }

    // update the desired heading
    desired_heading += mrpt::math::wrapToPi<double>(next_heading - desired_heading)*how_far;

    // Is the path to our left or right?
    Vector2D<double> norm = next_point - prev_point;
    norm.Normalize();

    left = right = norm;

    left.RotateZ(M_PI_2);
    left.Scale(2.0);
    left.Add(closest_point.position);

    right.RotateZ(-M_PI_2);
    right.Scale(2.0);
    right.Add(closest_point.position);

    double direction = 1;

    if (left.Distance2(front_axle.position) < right.Distance2(front_axle.position)) {

        direction = -1;

    }

    double k = 1.5;
    double dist = front_axle.position.Distance(closest_point.position);
    double d_theta = mrpt::math::wrapToPi<double>(s.orientation - desired_heading + (reverse_mode ? M_PI : 0));

    if (reverse_mode) {

        d_theta = -d_theta;
        direction = -direction;
    }

    double phi;
    double inverse_speed = (16 < s.v) ? 1.0/s.v: 1.0;
    if (s.coming_to_stop) {

        phi = std::atan(4 * k * direction * dist * inverse_speed);

    } else {

        phi = -d_theta + std::atan(k * direction * dist * inverse_speed);

    }

    double phi_error = phi - s.phi;
    double d_phi_error = (phi_error - prev_wheel_angle_error) / dt;
    double steer = phi_error * 8f + d_phi_error * 0.5f;

    prev_wheel_angle_error = phi_error;

    // velocity
    double vp = 0.5;
    double vi = 0.00001;
    double dv = 0;

    // s.v + lerp(v0, v1, how_far)
    double verror = s.v - ((1 - how_far)*prev.v + how_far*next.v);

    vpasterror = verror * dt;
    double v_total_error = vp * verror + vi*vpasterror;

    dv = -v_total_error;

    if (reverse_mode)
        dv = -dv;

    if (s.coming_to_stop && 1.0 == how_far) {

        last_cusp = next_index;

        if (next_index < raw_path_last_index) {

            next_waypoint += 1;
            prev_waypoint += 1;

        } else {

            next_waypoint = prev_waypoint = raw_path_last_index;

        }

        cs = CSStopped;

        // update the command to stop next point
        state.v = 0.0;
        state.phi = steer;
        state.t = dt;

        return state;

    }

    // update the command
    state.v += dv;
    state.phi = steer;
    state.t = dt;

    return state;

}

// travel along the path - forward
State2D StanleyController::ReverseDrive(const State2D &s) {

    return ForwardDrive(s);

}

// path following simulation
StateArrayPtr StanleyController::FollowPathSimulation() {

    // resulting command list
    StateArrayPtr command_array = new StateArray();

    // syntactic sugar
    std::vector<State2D> &commands(command_array->states);

    // the car initial state
    car = raw_path.front();

    while (CSComplete != cs) {

        switch (cs) {

            case CSForwardDrive:

                // get the next control
                car = ForwardDrive(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSReverseDrive:

                // get the next control
                car = ReverseDrive(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSStopped:

                // get the next control and pose
                car = Stopped(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSStandby:

                cs = CSStopped;

                break;

            default:
                break;

        }

        // update the car pose
        car = vehicle_model.NextState(car);

    }

    // reset the controller state
    cs = CSStandby;

    return commands;

}

// follow a given path
StateArrayPtr StanleyController::FollowPath(const State2D &start) {

    // resulting command list
    StateArrayPtr command_array = new StateArray();

    // syntactic sugar
    std::vector<State2D> &commands(command_array->states);

    // set the car pose
    car = start;

    while (CSComplete != cs) {

        switch (cs) {

            case CSForwardDrive:

                // get the next control
                car = ForwardDrive(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSReverseDrive:

                // get the next control
                car = ReverseDrive(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSStopped:

                // get the next control and pose
                car = Stopped(car);

                // save to the command list
                commands.push_back(car);

                break;

            case CSStandby:

                cs = CSStopped;

                break;

            default:
                break;

        }

        // update the car pose
        car = NextState();

    }

    // reset the current state

    return new command_array();

}

// get a Command list to follow a given path
StateArrayPtr StanleyController::BuildAndFollowPath(astar::StateListPtr input_path) {

    if (5 < input_path->states.size()) {

        consolidated_path = ConsolidateStateList(input_path);

        return FollowPath(input_path->states.front());

    }

    consolidated_path = false;

    return new StateArray();

}

