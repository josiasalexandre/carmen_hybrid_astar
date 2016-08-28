#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include <mutex>

#include "GVDLau.hpp"
#include "GridMapCell.hpp"
#include "../Entities/State2D.hpp"
#include "../Entities/Circle.hpp"

namespace astar {

class InternalGridMap {

    private:

        // PRIVATE ATTRIBUTES

        // the grid map width
        unsigned int width, width_2;

        // the grid map height
        unsigned int height, height_2;

        // the grid map size
        unsigned int size;

        // the map resolution
        double resolution, inverse_resolution;

        // the grid map resolution
        double diagonal_resolution;

        // the grid map origin
        astar::Vector2D<double> origin;

        // has the grid map changed?
        bool has_changed;

        // the corridor counter
        unsigned int corridor;

        // the Voronoi field  map
        astar::GVDLau voronoi;

        // PRIVATE METHODS

        // remove the current grid map
        void RemoveGridMap();


    public:

        // PUBLIC ATTRIBUTES

        // the current grid map
        astar::GridMap grid_map;

        // PUBLIC METHODS

        // basic constructor
        InternalGridMap();

        // basic destructor
        ~InternalGridMap();

        // process the voronoi diagram
        void ProcessVoronoiDiagram();

        // initialize the grid map given the map dimensions
        void UpdateGridMap(unsigned int w, unsigned int h, double res, const astar::Vector2D<double> &_origin, double *map);

        // verify if the current grid map has changed
        bool HasChanged() const;

        // Expand the region around the RDDF point
        void ExpandRegion(astar::GridCellIndex &index, unsigned int distance);

        // update the corridor
        void UpdateCorridor(const std::vector<astar::Vector2D<double>> &rddf, unsigned int distance);

        // get the corridor indexes
        unsigned int GetCorridorIndexes();

        // is a valid cell?
        bool isValidPoint(const astar::Vector2D<double>&);

        // verify if a given pose is a valid one
        bool isSafePlace(const std::vector<astar::Circle> &body, double safety_factor);

        // get the grid cell index from any position
        astar::GridCellIndex PoseToIndex(const astar::Vector2D<double>&) const;

        // return a cell given a pose
        GridMapCellPtr PoseToCell(const astar::Pose2D&);

        // get the cell given a cell index
        GridMapCellPtr IndexToCell(const astar::GridCellIndex &index);

        // occupy a given cell
        void SetSimpleObstacle(int row, int col);

        // clear a given cell
        void SetSimpleFreeSpace(int row, int col);

        // occupy a given cell
        void OccupyCell(int row, int col);

        // clear a given cell
        void ClearCell(int row, int col);

        // get the current grid map
        astar::GVDLau* GetGVD();

        // get the nearest obstacle position
        astar::Vector2D<double> GetObstaclePosition(const astar::Vector2D<double>&);

        // indirect obstacle distance
        double GetObstacleDistance(const astar::Vector2D<double>&);

        // get the nearest voronoi edge position
        astar::Vector2D<double> GetVoronoiPosition(const astar::Vector2D<double>&);

        // indirect voronoi edge distance
        double GetVoronoiDistance(const astar::Vector2D<double>&);

        // get the path cost
        double GetPathCost(const astar::Vector2D<double>&);

        // get the current map
        unsigned char* GetGridMap();

        // get the current path cost map
        unsigned char* GetPathCostMap();

        // get the current obstacle distance map
        unsigned char* GetObstacleDistanceMap();

        // get the corridor map
        unsigned char* GetCorridorMap();

        // get the current map width
        unsigned int GetWidth() { return width; }

        // get the current map height
        unsigned int GetHeight() { return height; }

        // get the map resolution
        double GetResolution() { return resolution; }

        // get the inverse resolution
        double GetInverseResolution() { return inverse_resolution; }

        // get the map origin
        astar::Vector2D<double> GetOrigin() { return origin; }

};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;
typedef InternalGridMap& InternalGridMapRef;

// i is the row and j is the column ;-)
#define GRID_MAP_INDEX(j, i) ((j)*width + i)

}

#endif
