#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include "GVDLau.hpp"

namespace astar {

class InternalGridMap {

    private:

        // private members

        // the voronoy fiedl
        astar::GVDLau voronoyField;

        // private methods

    public:

        // public members

        // public methods

        // basic constructor
        InternalGridMap();

        // update the GridMap
        void updateGridMap();

};

}

#endif
