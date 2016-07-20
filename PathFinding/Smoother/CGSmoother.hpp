#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <vector>

#include "../../VehicleModel/VehicleModel.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "../../Entities/State2D.hpp"

namespace astar {

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

		// the main function to be minimized
		double CostFunction(astar::StateArrayPtr);

		bool Iterate();

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // smooth a given path
        astar::StateArrayPtr Smooth(astar::InternalGridMap&, astar::VehicleModel&, astar::StateArrayPtr);

};

}

#endif
