#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <list>

#include "InternalGridMap.hpp"
#include "../Entities/Pose2D.hpp"

namespace astar {

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // smooth a given path
        std::list<astar::Pose2D> smooth(std::list<astar::Pose2D>&, astar::InternalGridMap&);

};

}

#endif
