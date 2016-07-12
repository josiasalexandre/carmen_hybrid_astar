#ifndef GENERAL_VORONOI_DIAGRAM_HPP
#define GENERAL_VORONOI_DIAGRAM_HPP

#include "BucketedQueue.hpp"
#include <string>

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
		DataCell **data;

		// the diagram parameters
		int height, heightminus1;
		int width, widthminus1;

		// the general parameters
		double alpha;
		double max_dist, max_sqdist;

		// double max value
		double max_double;

		//queues
		astar::BucketPrioQueue<astar::GridCellIndex> open;
		astar::BucketPrioQueue<astar::GridCellIndex> voro_open;

		// PRIVATE METHODS

		// remove the entire allocated diagram
		void RemoveDiagram();

		// verify a given index against the map dimensions
		bool isValidIndex(const astar::GridCellIndexRef);

		// get the squared distance between two cells
		int DistanceSquared(const astar::GridCellIndexRef, const astar::GridCellIndexRef);

		// verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
		inline bool isOccupied(const astar::GridCellIndexRef);

		// verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
		// overloaded version
		inline bool isOccupied(const astar::GridCellIndexRef, const DataCellRef);

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
		void InitializeEmpty(int h, int w);

		// initialize the GVD with a given map
		void InitializeMap(int h, int w, bool **map);

		// set a given cell as an obstacle
		void SetObstacle(int row, int col);

		// set a given cell as a free space
		void RemoveObstacle(int row, int col);

		// update the entire GVD
		void Update();

		// get the nearest obstacle distance
		double GetObstacleDistance(int row, int col);

		// get the nearest voronoi edge distance
		double GetVoronoiDistance(int row, int col);

		// save the voronoi field map to an external file
		void Visualize(std::string);
};

}

#endif
