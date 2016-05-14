#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include "GVDLau.hpp"
#include "../../Entities/Pose2D.hpp"
#include "MapCell.hpp"

#include "../../KDTree/KDTree.hpp"

namespace astar {

class InternalGridMap {

    private:

        // PRIVATE ATTRIBUTES

        // the grid resolution and inverse_resolution
        double resolution, inverse_resolution;

        // the grid width
        unsigned int width, width_2;

        // the grid height
        unsigned int height, height_2;

        // the size
        unsigned int size;

        // the obstacle grid map
        astar::MapCellPtr obstacle_map;

        // the map origin
        astar::Vector2D origin;

        // the voronoy fiedl
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

        // find a cell which a given pose is localized
        MapCellPtr PoseToCell(const astar::Pose2D&);

        // get the distance to the nearest obstacle
        double GetObstacleDistance(const astar::Vector2D&);

        // get the distance to the nearest voronoy edge
        double GetVoronoiDistance(const astar::Vector2D&);

        // is a valid point?
        bool isValidPoint(const astar::Vector2D&);

        // is a safe place?
        bool isSafePlace(const astar::Pose2D&);
};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;
typedef InternalGridMap& InternalGridMapRef;

}

// i is the row and j is the collumn ;-)
#define GRID_MAP_INDEX(i, j) ((i)*width + j)

#endif
