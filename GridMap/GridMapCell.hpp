/*
// Author: Josias Alexandre Oliveira
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef INTERNAL_GRID_MAP_CELL_HPP
#define INTERNAL_GRID_MAP_CELL_HPP

#include "GridMapCell.fwd.hpp"
#include "../PathFinding/HybridAstar/HybridAstarNode.fwd.hpp"

namespace astar {

    // define the enumeration status
    enum CellStatus {UnknownNode, OpenedNode, ExploredNode};

    class GridMapCell
    {
        public:

            // PUBLIC ATTRIBUTES

            // the occupancy
            int occupancy;

            // the cell status
            CellStatus status;

            // the robot state
            astar::HybridAstarNodePtr node;

            // the corridor flag
            bool is_corridor;

            // PUBLIC METHODS
            GridMapCell() : occupancy(0), status(astar::UnknownNode), node(nullptr), is_corridor(false) {}

            // the overloading operator
            void operator=(const GridMapCell &c)
            {
                occupancy = c.occupancy;
                status = c.status;
                node = nullptr;
                is_corridor = c.is_corridor;
            }

    };

}

#endif
