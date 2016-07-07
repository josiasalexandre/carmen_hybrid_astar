#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include "InternalGridMap.hpp"
#include "../State2D.hpp"

namespace astar {

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // smooth a given path
        void Smooth(astar::InternalGridMap&, astar::StateArrayPtr);

};

}

#endif
