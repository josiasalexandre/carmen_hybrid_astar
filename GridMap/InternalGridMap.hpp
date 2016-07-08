#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include "GVDLau.hpp"
#include "GridMapCell.hpp"
#include "../../Entities/State2D.hpp"

namespace astar {

class InternalGridMap {

    private:

        // PRIVATE ATTRIBUTES

        // the grid map width
        unsigned int width, width_2;

        // the grid map height
        unsigned int height, height_2;

        // the grid map resolution
        double resolution, inverse_resolution;

        // the grid map origin
        astar::Vector2D<double> origin;

        // the grid map orientation
        double orientation;

        // the current grid map
        GridMapCellPtr grid_map;

        // the voronoy field  map
        astar::GVDLau voronoy;

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES


        // PUBLIC METHODS

};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;
typedef InternalGridMap& InternalGridMapRef;

// i is the row and j is the column ;-)
#define GRID_MAP_INDEX(j, i) ((j)*width + i)

}

#endif
