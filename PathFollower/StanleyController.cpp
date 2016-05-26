#include "StanleyController.hpp"

using namespace astar;

// set the appropriated low speeds around the stopping points
void StanleyController::UpdateLowSpeedRegions(
        StateListPtr state_list,
        std::vector<std::list<State2D>::iterator> &stopping,
        VehicleModel &vehicle)
{

    // get the stopping points vector size
    unsigned int s_size = stopping.size();
    double acceleration_constraint, max_speed;
    double current_speed;

    if (0 < s_size) {

        std::list<State2D>::iterator left, right, current;
        std::list<State2D>::iterator begin = state_list->states.begin();
        std::list<State2D>::iterator end = state_list->states.end();

        // iterate over the stopping list
        for (unsigned int i = 0; i < s_size; i++) {

            // get the iterator stored inside the stopping vector
            current = stopping[i];

            left = right = current;

            // go to the left
            --left;
            if (left != end) {

                // the speed should be low around the stopping point
                left->v = 0.2;

                // update the right pointer
                right = left;

                // update the left pointer
                --left;
                while (left != end) {

                    acceleration_constraint = std::sqrt(
                                right->v * right->v +
                                2 * vehicle.desired_deceleration * (right->position - left->position).Norm());

                    // update the current speed
                    if (acceleration_constraint < current->v) {

                        current->v = acceleration_constraint;

                    } else {

                        break;

                    }

                    // update the pointers
                    right = left;
                    --left;

                }

            }

            // reset the pointers
            left = right = current;

            // go to the right
            ++right;
            if (right != begin) {

                // the speed should be low around the stopping point
                right->v = 0.2;

                // update the left pointer
                left = right;

                // update the right pointer
                ++right;

                while(right != begin) {

                    acceleration_constraint = std::sqrt(
                                left->v * left->v +
                                2 * vehicle.max_forward_acceleration * (right->position - left->position).Norm());

                    // update the current speed
                    if (acceleration_constraint < current->v) {

                        current->v = acceleration_constraint;

                    } else {

                        break;

                    }

                    // update the pointers
                    left = right;
                    ++right;

                }

            }

        }

    }

}

// get the appropriated orientations
void StanleyController::UpdatePath(StateListPtr state_list, VehicleModel &vehicle) {

    // get the acceleration constraint
        double dec_constraint = std::sqrt(next.v*next.v + 2*desired_deceleration*next_v.Norm());

}

// get the desired command given two states
// the first and the last states remains the same
StateListPtr StanleyController::ConsolidateStateList(StateListPtr state_list, VehicleModel &vehicle) {

    // the stopping points vector
    std::vector<std::list<State2D>::iterator> stopping;

    if (5 < state_list->states.size()) {

        // helpers
        astar::Vector2D<double> displacement;
        double max_speed;

        // syntactic sugar
        std::list<State2D> &states(state_list->states);

        // get the limit iterators
        std::list<State2D>::iterator end = states.end();
        std::list<State2D>::iterator begin = states.begin();

        // get the list iterators
        std::list<State2D>::iterator prev = begin;
        std::list<State2D>::iterator current = std::advance(prev, 1);
        std::list<State2D>::iterator next = std::advance(current, 1);

        // is the car stopped?
        if (0.0 == begin->v) {

            // save to the stopping points list
            stopping.push_back(begin);

        }

        // iterate the entire list but the second last element
        while (current != end) {

            // the common case
            if (current->gear == prev->gear) {

                // get the desired orientation at this point
                current->orientation = vehicle.GetDesiredOrientation(*prev, *current, *next);

                // get the max speed
                current->v = vehicle.GetDesiredSpeed(*prev, *current, *next);

            } else {

                // the speed should be zero at any stopping point
                current->v = 0.0;

                // save to the stopping points list
                stopping.push_back(current);

            }

        }

        // verify the parking mode
        if (0.0 == )

    }

}
