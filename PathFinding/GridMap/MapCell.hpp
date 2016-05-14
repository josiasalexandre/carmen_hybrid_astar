#ifndef INTERNAL_GRID_MAP_CELL_HPP
#define INTERNAL_GRID_MAP_CELL_HPP

#include "../HybridAstar/HybridAstarNode.hpp"

namespace astar {

// define the enum status
enum CellStatus {UnknownNode, OpenedNode, ExploredNode};

class MapCell {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // the occupancy
        char occupancy;

        // the cell status
        CellStatus status;

        // the robot state
        astar::HybridAstarNodeArrayPtr node;

        // PUBLIC METHODS

};

// define a reference pointer
typedef MapCell* MapCellPtr;

}



#endif
