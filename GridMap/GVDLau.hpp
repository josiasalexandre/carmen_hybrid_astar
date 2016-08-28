#ifndef GENERAL_VORONOI_DIAGRAM_HPP
#define GENERAL_VORONOI_DIAGRAM_HPP

#include <string>
#include <vector>
#include <array>

#include "BucketedQueue.hpp"
#include "../Entities/Pose2D.hpp"
#include "../KDTree/KDTree.hpp"

#include "GridMapCell.hpp"

namespace astar {

// Based on http://www.first-mm.eu/files/lau10iros.pdf
// Thanks to Boris Lau, Christoph Sprunk and Wolfram Burgard
class GVDLau {

    private:

        struct DataCell {

            // the obstacle distance map variables
            int sqdist;
            double dist;
            astar::GridCellIndex nearest_obstacle;
            bool to_raise;
            bool to_process;
            bool voro;

            // the voronoi distance map variables
            int voro_sqdist;
            double voro_dist;
            astar::GridCellIndex nearest_voro;
            bool voro_to_raise;
            bool voro_to_process;

            double path_cost;

        };

        typedef DataCell* DataCellPtr;
        typedef DataCell& DataCellRef;

        // the diagram
        DataCell **data, **next_data;

        // the diagram parameters
        unsigned int height;
        int heightminus1;
        unsigned int width;
        int widthminus1;

        // the general parameters
        double alpha;
        double max_dist, max_sqdist;

        // double max value
        double max_double;

        // is it allocated?
        bool initialized;

        //queues
        astar::BucketPrioQueue<astar::GridCellIndex> open;
        astar::BucketPrioQueue<astar::GridCellIndex> voro_open;

        // the voro edges kdtre
        astar::KDTree<unsigned int, 2> edges;

        // the voro add list
        std::vector<astar::PointT<unsigned int, 2>> add_list;

        // PRIVATE METHODS

        // remove the entire allocated diagram
        void RemoveDiagram();

        // verify a given index against the map dimensions
        bool isValidIndex(const astar::GridCellIndexRef) const;

        // get the squared distance between two cells
        int DistanceSquared(const astar::GridCellIndexRef, const astar::GridCellIndexRef);

        // verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
        inline bool isOccupied(const astar::GridCellIndexRef) const;

        // verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
        // overloaded version
        inline bool isOccupied(const astar::GridCellIndexRef, const DataCellRef);

        // verify if a given is voro occupied, wich means that the nearest voro is the give cell
        bool isVoroOccupied(const astar::GridCellIndexRef);

        // verify if a given is voro occupied, wich means that the nearest voro is the give cell
        bool isVoroOccupied(const astar::GridCellIndexRef, const DataCellRef);

        // set a given cell as voro
        void SetVoro(const astar::GridCellIndexRef);

        // unset a voro cell
        void UnsetVoro(const astar::GridCellIndexRef);

        // check the voro case
        void CheckVoro(astar::GridCellIndexRef, astar::GridCellIndexRef);

        // update the distance map
        void UpdateDistanceMap();

        // update the voronoi distance map
        void UpdateVoronoiMap();

        // update the path cost map
        void UpdatePathCostMap();

    public:

        // basic constructor
        GVDLau();

        // basic destructor
        ~GVDLau();

        // Initialize the GVD
        void InitializeEmpty(unsigned int h, unsigned int w);

        // initialize the GVD with a given map
        void InitializeMap(unsigned int h, unsigned int w, bool **map);

        // restart the GVD
        void RestartVoronoiDiagram();

        // set a given cell as an obstacle
        void SetSimpleFreeSpace(unsigned int row, unsigned int col);

        // set a given cell as an obstacle
        void SetSimpleObstacle(unsigned int row, unsigned int col);

        // set a given cell as an obstacle
        void SetObstacle(unsigned int row, unsigned int col);

        // set a given cell as a free space
        void RemoveObstacle(unsigned int row, unsigned int col);

        // update the entire GVD
        bool Update();

        // get the nearest obstacle distance
        double GetObstacleDistance(unsigned int row, unsigned int col);

        // get the nearest obstacle index
        GridCellIndex GetObstacleIndex(unsigned int row, unsigned int col);

        // get the nearest voronoi edge distance
        double GetVoronoiDistance(unsigned int row, unsigned int col);

        // get the nearest voronoi edge distance given a valid
        // unsigned int overloaded, KDTree usage
        GridCellIndex GetVoronoiIndex(unsigned int row, unsigned int col);

        // get the path cost index
        double GetPathCost(unsigned int row, unsigned int col);

        // save the voronoi field map to an external file
        void Visualize(std::string);

        // get the current path cost map
        unsigned char* GetPathCostMap();

        // get the current obstacle distance map
        unsigned char* GetObstacleDistanceMap();
};

}

#endif
