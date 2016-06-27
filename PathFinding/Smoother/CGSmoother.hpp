#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <list>

#include "InternalGridMap.hpp"
#include "../State2D.hpp"

namespace astar {

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS
        void GetDesiredOrientations();

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // smooth a given path
        astar::StateListPtr Smooth(astar::InternalGridMap&, astar::StateListPtr);

};

}

#endif
