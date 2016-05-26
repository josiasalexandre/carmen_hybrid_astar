#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include "GVDLau.hpp"
#include "GridMapCell.hpp"
#include "../State2D.hpp"
#include "../../KDTree/KDTree.hpp"

namespace astar {

class InternalGridMap {

    private:

        // PRIVATE ATTRIBUTES

        // the grid resolution
        double resolution;

        // inverse_resolution
        double inverse_resolution;

        // the grid width
        unsigned int width, width_2;

        // the grid height
        unsigned int height, height_2;

        // the size
        unsigned int size;

        // the obstacle grid map
        astar::GridMapCellPtr grid_map;

        // the map origin
        astar::Vector2D<double> origin;

        // the voronoy field
        astar::GVDLau *voronoy_field;

        // the obstacle distance KDTree
        astar::KDTree<double, 2> obstacle_kdtree;

        // the voronoy Distance KDTree
        astar::KDTree<double, 2> voronoy_kdtree;

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES


        // PUBLIC METHODS

        // basic constructor
        InternalGridMap();

        // update the GridMap
        void UpdateGridMap();

        // occupy a given cell
        void OccupyCell(astar::GridMapCellPtr c);

        // clear a given cell
        void ClearCell(astar::GridMapCellPtr c);

        // initialize the entire grid map
        bool InitializeGridMap(unsigned int w, unsigned int h, double res, const astar::Vector2D<double>& origin, double orientation);

        // find a cell which a given pose is localized
        GridMapCellPtr StateToCell(const astar::State2D&);

        // get the distance to the nearest obstacle
        double GetObstacleDistance(const astar::Vector2D<double>&);

        // get the distance to the nearest voronoy edge
        double GetVoronoiDistance(const astar::Vector2D<double>&);

        // is a valid point?
        bool isValidPoint(const astar::Vector2D<double>&);

        // is a safe place?
        bool isSafePlace(const astar::State2D&);
};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;
typedef InternalGridMap& InternalGridMapRef;

// i is the row and j is the column ;-)
#define GRID_MAP_INDEX(j, i) ((j)*width + i)

}

#endif
