#ifndef GENERAL_VORONOY_DIAGRAM_HPP
#define GENERAL_VORONOY_DIAGRAM_HPP

#include "InternalGridMap.hpp"

namespace astar {

// Based on http://www.first-mm.eu/files/lau10iros.pdf
// Thanks to Boris Lau, Christoph Sprunk and Wolfram Burgard
class GVDLau {

    private:

        // PRIVATE ATTRIBUTES
        astar::InternalGridMapRef map;


        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // basic constructor
        GVDLau(astar::InternalGridMapRef obstacle_map);

        // initialize the grid map

        // set a given cell as obstacle
        void SetObstacle(int x, int y);

        // set a given cell as free space
        void UnsetObstacle(int x, int y);

};

}

#endif
