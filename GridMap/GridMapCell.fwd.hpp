#ifndef GRID_MAP_CELL_FORWARD_DECLARATION_HPP
#define GRID_MAP_CELL_FORWARD_DECLARATION_HPP

namespace astar {

// class forward declaration
class GridMapCell;

// define a reference pointer
typedef GridMapCell* GridMapCellPtr;

// define a reference
typedef GridMapCell& GridMapCellRef;

typedef GridMapCell** GridMap;

}

#endif
