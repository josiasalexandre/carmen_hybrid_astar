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

        // the grid map origin
        astar::Vector2D<double> origin;

        // the obstacle grid map
        astar::GridMapCellPtr grid_map;

        // the map origin
        astar::Vector2D<double> origin;

        // the map orientation
        double orientation;

        // the voronoy field
        astar::GVDLau *voronoy_field;

        // PRIVATE METHODS
        // initialize the entire voronoy field map
        void BuildGVD();

    public:

        // PUBLIC ATTRIBUTES


        // PUBLIC METHODS

        // basic constructor
        InternalGridMap();

        bool InitializeGridMap(
            unsigned int w,
            unsigned int h,
            double res,
            const astar::Vector2D<double> &_origin,
            double _orientation
        );

        // is empty?
        bool isEmpty();

        // update the GridMap and the Voronoy Field
        void Update();

        // occupy a given cell
        void OccupyCell(astar::GridMapCellPtr c);

        // clear a given cell
        void ClearCell(astar::GridMapCellPtr c);

        // find a cell which a given pose is localized
        GridMapCellPtr PoseToCell(const astar::Pose2D&);

        // get the distance to the nearest obstacle
        double GetObstacleDistance(const astar::Vector2D<double>&);

        // get the distance to the nearest voronoy edge
        double GetVoronoiDistance(const astar::Vector2D<double>&);

        // is a valid point?
        bool isValidPoint(const astar::Vector2D<double>&);

        // is a safe place?
        bool isSafePlace(const astar::Pose2D&);

        // is a safe place?
        bool isSafePlace(const astar::Vector2D<double> &position, double orientation);
};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;
typedef InternalGridMap& InternalGridMapRef;

// i is the row and j is the column ;-)
#define GRID_MAP_INDEX(j, i) ((j)*width + i)

}

#endif
