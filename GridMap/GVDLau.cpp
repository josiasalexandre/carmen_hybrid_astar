#include "GVDLau.hpp"

#include <limits>
#include <climits>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace astar;

// basic constructor
GVDLau::GVDLau() :
	data(nullptr),
	height(0),
	heightminus1(0),
	width(0),
	widthminus1(0),
	alpha(20.0),
	max_dist(30.0),
	max_sqdist(900), max_double(std::numeric_limits<double>::max()) {}

// basic destructor
GVDLau::~GVDLau() {

	RemoveDiagram();

}

// Remove the entire allocated diagram
void GVDLau::RemoveDiagram() {

	if (nullptr != data) {

		for (int i = 0; i < height; ++i) {
			delete [] data[i];
		}

		delete [] data;
	}
}

// verify a given index against the map dimensions
bool GVDLau::isValidIndex(const astar::GridCellIndexRef index) {

	return (0 < index.row && heightminus1 > index.row && 0 < index.col && widthminus1 > index.col);
}

// get the squared distance between two cells
int GVDLau::DistanceSquared(const astar::GridCellIndexRef a, const astar::GridCellIndexRef b) {

	int dr = a.row - b.row;
	int dc = a.col - b.col;
	return (dr*dr + dc*dc);

}

// verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
bool GVDLau::isOccupied(const astar::GridCellIndexRef index) {

	// get the cell reference
	GVDLau::DataCellRef c(data[index.row][index.col]);

	return (index.col == c.nearest_obstacle.col  && index.row == c.nearest_obstacle.row);
}

// verify if a given cell is occupied, wich means that the nearest obstacle is the given cell
// overloaded version
bool GVDLau::isOccupied(const astar::GridCellIndexRef index, const DataCellRef c) {

	return (index.row == c.nearest_obstacle.row  && index.col == c.nearest_obstacle.col);

}

// verify if a given is voro occupied, wich means that the nearest voro is the give cell
// overloaded version
bool GVDLau::isVoroOccupied(const GridCellIndexRef s, const DataCellRef c) {

	return (c.nearest_voro.row == s.row && c.nearest_voro.col == s.col);

}

// set a given cell as voro
void GVDLau::SetVoro(const GridCellIndexRef index) {

	// get the current DataCell
	GVDLau::DataCellRef s(data[index.row][index.col]);

	// update the cell values
	s.nearest_voro = index;
	s.voro_dist = 0.0;
	s.voro_sqdist = 0;
	s.voro_to_process = true;

	// add the current cell to the voro open queue
	voro_open.Push(0, GridCellIndex(index));

}

// unset a voro cell
void GVDLau::UnsetVoro(const GridCellIndexRef index) {

	// get the current DataCell
	GVDLau::DataCellRef s(data[index.row][index.col]);

	// update the cell values
	s.voro_dist = max_double;
	s.voro_sqdist = INT_MAX;
	s.nearest_voro.row = s.nearest_voro.col = -1;
	s.voro_to_raise = true;
	s.voro_to_process = true;

	// add the current cell to the voro open queue
	voro_open.Push(INT_MAX, GridCellIndex(index));

}

// check the voro case
void GVDLau::CheckVoro(GridCellIndexRef indexS, GridCellIndexRef indexN) {

	// get the actual cells
	GVDLau::DataCellRef s(data[indexS.row][indexS.col]);
	GVDLau::DataCellRef n(data[indexN.row][indexN.col]);

	if (!isValidIndex(s.nearest_obstacle) || !isValidIndex(n.nearest_obstacle)) return;

	// get the nearest obstacle indexes
	GridCellIndexRef noS(s.nearest_obstacle);
	GridCellIndexRef noN(n.nearest_obstacle);

	if ((1 < s.sqdist || 1 < n.sqdist)) {

		if (1 < std::abs(noS.row - noN.row) || 1 < std::abs(noS.col - noN.col)) {

			int sObstN = DistanceSquared(indexS, noN);
			int nObstS = DistanceSquared(indexN, noS);

			int sStability = sObstN - s.sqdist;
			int nStability = nObstS - n.sqdist;

            if (sStability < 0 || nStability < 0)
                return;

            if (sStability <= nStability)
            {
            	s.voro = true;
                SetVoro(indexS);
            }
            if (nStability <= sStability)
            {
            	n.voro = true;
                SetVoro(indexN);
            }
		}
	}
}

// update the distance map
void GVDLau::UpdateDistanceMap() {

	while (!open.Empty()) {

		// get the cell indexes
		GridCellIndex index = open.Pop();

		// syntactic sugar
		int row = index.row;
		int col = index.col;

		// get the current cell
		GVDLau::DataCellRef s(data[row][col]);

		if (!s.to_process) continue;

		if (s.to_raise) {

			// raise
			// get the 8 neighbors
			for (int drow = -1; drow < 2; ++drow) {

				// get the vertical displacement
				int nrow = row + drow;

				// verify the limits
				if (0 >= nrow || heightminus1 <= nrow) continue;

				for (int dcol = -1; dcol < 2; ++dcol) {

					// the current cell
					if (!drow && !dcol) continue;

					// get the vertical displacement
					int ncol = col + dcol;

					// verify the limits
					if (0 >= ncol || widthminus1 <= ncol) continue;

					// valid cell, so let's process

					// get the current neighbor
					GVDLau::DataCellRef nc(data[nrow][ncol]);

					if (-1 != nc.nearest_obstacle.col && !nc.to_raise) {

						if (!isOccupied(nc.nearest_obstacle, data[nc.nearest_obstacle.row][nc.nearest_obstacle.col])) {

							// update the neighbor values
							nc.sqdist = INT_MAX;
							nc.dist = max_double;
							nc.nearest_obstacle.row = nc.nearest_obstacle.col = -1;
							nc.to_raise = true;

						}

						// set the neighbor to process
						nc.to_process = true;

						// add to the open queue
						open.Push(nc.sqdist, GridCellIndex(nrow, ncol));

					}
				}
			}

			// unset the raise flag
			s.to_raise = false;

		} else if (isOccupied(s.nearest_obstacle, data[s.nearest_obstacle.row][s.nearest_obstacle.col])) {

			// udpate the current voro values
			s.voro = false;
			UnsetVoro(index);
			s.to_process = false;

			// lower

			// get the 8 neighbors
			for (int dcol = -1; dcol < 2; ++dcol) {

				// get the current horizontal displacement
				int ncol = col + dcol;

				// verify the limits
				if (0 >= ncol || widthminus1 <= ncol) continue;

				for (int drow = -1; drow < 2; ++drow) {

					// the current cell
					if (!drow && !dcol) continue;

					// get the current vertical displacement
					int nrow = row + drow;

					// verify the limits
					if (0 >= nrow || heightminus1 <= nrow) continue;

					// valid cell, so let's process

					// get the neighbor cell
					GVDLau::DataCellRef nc(data[nrow][ncol]);

					if (!nc.to_raise) {

						//
						GridCellIndex nindex(nrow, ncol);

						int d = DistanceSquared(s.nearest_obstacle, nindex);

						if (d < nc.sqdist) {

							// update the neighbor values
							nc.sqdist = d;
							nc.dist = std::sqrt(d);
							nc.nearest_obstacle = s.nearest_obstacle;
							nc.to_process = true;

							// add the current neighbor to the open queue
							open.Push(d, nindex);

						} else {

							// check the voro case
							CheckVoro(index, nindex);

						}
					}
				}
			}
		}
	}
}

// update the voronoi distance map
void GVDLau::UpdateVoronoiMap() {

	while (!voro_open.Empty()) {

		// get the current index
		GridCellIndex index = voro_open.Pop();

		// syntactic sugar
		int row = index.row;
		int col = index.col;

		// get the actual DataCell
		GVDLau::DataCellRef s(data[row][col]);

		if (s.voro_to_process) {

			if (s.voro_to_raise) {

				// raise

				// get the 8 neighbors
				for (int drow = -1; drow < 2; ++drow) {

					// get the vertical displacement
					int nrow = row + drow;

					// verify the limits
					if (0 >= nrow || heightminus1 <= nrow) continue;

					for (int dcol = -1; dcol < 2; ++dcol) {

						// the current cell
						if (!drow && !dcol) continue;

						// get the horizontal displacement
						int ncol = col + dcol;

						// verify the limits
						if (0 >= ncol || widthminus1 <= ncol) continue;

						// valid cell, let's process

						// get the actual cell
						GVDLau::DataCellRef nc(data[nrow][ncol]);

						if (-1 != nc.nearest_voro.col && !nc.voro_to_raise) {

							if (!isVoroOccupied(nc.nearest_voro, data[nc.nearest_voro.row][nc.nearest_voro.col])) {

								// update the neighbor values
								nc.dist = max_double;
								nc.sqdist = INT_MAX;
								nc.nearest_voro.row = nc.nearest_voro.col = -1;
								nc.voro_to_raise = true;

							}

							// set the voro to process flag
							nc.voro_to_process = true;

							// add the current neighbor index to the voro open queue
							voro_open.Push(nc.voro_sqdist, GridCellIndex(nrow, ncol));

						}

					}

				}

				// update the voro to raise flag
				s.voro_to_raise = false;

			} else if (isVoroOccupied(s.nearest_voro, data[s.nearest_voro.row][s.nearest_voro.col])) {

				// reset the voro_to_process flag
				s.voro_to_process = false;

				// lower

				// get the 8 neighbors
				for (int drow = -1; drow < 2; ++drow) {

					// get the vertical displacement
					int nrow = row + drow;

					// verify the limits
					if (0 >= nrow || heightminus1 <= nrow) continue;

					for (int dcol = -1; dcol < 2; ++dcol) {

						// the current cell
						if (!drow && !dcol) continue;

						// get the horizontal displacement
						int ncol = col + dcol;

						// verify the limits
						if (0 >= ncol || widthminus1 <= ncol) continue;

						// valid cell, let's process

						// get the actual neighbor cell
						GVDLau::DataCellRef nc(data[nrow][ncol]);

						if (!nc.voro_to_raise) {

							GridCellIndex ngc(nrow, ncol);

							int d = DistanceSquared(s.nearest_voro, ngc);

							if (d < nc.voro_sqdist) {

								// update the neighbor values
								nc.voro_sqdist = d;
								nc.voro_dist = std::sqrt(d);
								nc.nearest_voro = s.nearest_voro;
								nc.voro_to_process = true;
								// add the current cell to the voro open queue
								voro_open.Push(d, ngc);
							}
						}
					}
				}
			}
		}
	}
}

// update the path cost map
void GVDLau::UpdatePathCostMap() {

	for (int r = 0; r < height; ++r) {

		for (int c = 0; c < width; ++c) {

			// get the current data cell
			GVDLau::DataCellRef s(data[r][c]);

			if (max_dist <= s.dist || !(s.voro_dist == s.voro_dist)) {
				s.path_cost = 0.0;
			} else {
				s.path_cost = (alpha / (alpha + s.dist)) * (s.voro_dist / (s.dist + s.voro_dist)) * ((max_dist - s.dist) * (max_dist - s.dist) / (max_sqdist));
			}
		}
	}
}

// Initialize the GVD
void GVDLau::InitializeEmpty(int h, int w) {

	if (0 < h && 0 < w) {

		if (height != h || width != w) {

			// remove the old diagram
			RemoveDiagram();

			// update the dimension parameters
			height = h;
			heightminus1 = h - 1;
			width = w;
			widthminus1 = w - 1;

			// allocate a new diagram
			data = new GVDLau::DataCellPtr[height];

			for (int i = 0; i < height; ++i) {

				data[i] = new GVDLau::DataCell[width];

			}
		}

		// clear the entire diagram
		GVDLau::DataCell c;
		c.sqdist = INT_MAX;
		c.dist = max_double;
		c.nearest_obstacle.row = c.nearest_obstacle.col = -1;
		c.to_raise = c.to_process = false;
		c.voro = true;
		c.voro_sqdist = INT_MAX;
		c.voro_dist = max_double;
		c.nearest_voro.row = c.nearest_voro.col = -1;
		c.voro_to_raise = c.voro_to_process = false;
		c.path_cost = 0;

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				data[i][j] = c;
			}
		}
	}
}

// initialize the GVD with a given map
void GVDLau::InitializeMap(int h, int w, bool **map) {

	if (nullptr != map) {

		if (height != h || width != w) {


			// remove the old diagram
			RemoveDiagram();

			// copy the new parameters
			height = h;
			heightminus1 = h - 1;
			width = w;
			widthminus1 = w - 1;

			// allocate a new diagram
			data = new GVDLau::DataCellPtr[height];

			for (int i = 0; i < height; ++i) {
				data[i] = new GVDLau::DataCell[width];
			}
		}


		// copy the given map
		for (int r = 0; r < h; ++r) {

			for (int c = 0; c < w; ++c) {

				if (map[r][c]) {

					// get the current DataCell
					GVDLau::DataCellRef s(data[r][c]);

					// build the current index
					GridCellIndex index(r, c);

					if (!isOccupied(index, s)) {

						bool isSurrounded = true;

						// get the 8 neighbors
						for (int dr = -1; dr < 2; ++dr) {

							// get the vertical displacement
							int ndr = r + dr;

							// verify the limit
							if (0 >= ndr || heightminus1 <= ndr) continue;

							for (int dc = -1; dc < 2; ++dc) {

								// the current cell
								if (!dr && !dc) continue;

								// get the horizontal displacement
								int ndc = c + dc;

								// verify the limit
								if (0 >= ndc || widthminus1 <= ndc) continue;

								// valid cell, let's process

								if(!map[ndr][ndc]) {
									isSurrounded = false;
								}
							}
						}

						// is it surrounded?
						if (isSurrounded) {

							// update the grid cell values
							s.dist = 0.0;
							s.sqdist = 0;
							s.voro = false;
							s.to_raise = false;
							s.to_process = false;
							s.voro_to_raise = false;
							s.voro_to_process = false;
							s.nearest_obstacle = index;

						} else {
							SetObstacle(r, c);
						}
					}
				}
			}
		}
	}
}

// set a given cell as an obstacle
void GVDLau::SetObstacle(int row, int col) {

	// get the current DataCell
	GVDLau::DataCellRef c(data[row][col]);

	// update the current cell values
	c.dist = 0.0;
	c.sqdist = 0;
	c.nearest_obstacle.row = row;
	c.nearest_obstacle.col = col;
	c.to_process = true;

	// add to the open prio queue
	open.Push(0, GridCellIndex (row, col));

}

// set a given cell as a free space
void GVDLau::RemoveObstacle(int row, int col) {

	// get the current DataCell
	GVDLau::DataCellRef c(data[row][col]);

	// update the current cell values
	c.dist = max_double;
	c.sqdist = INT_MAX;
	c.nearest_obstacle.row = c.nearest_obstacle.col = -1;
	c.to_raise = true;
	c.to_process = true;

	// add to the open prio queue
	open.Push(INT_MAX, GridCellIndex (row, col));

}

// update the entire GVD
void GVDLau::Update() {

	UpdateDistanceMap();
	UpdateVoronoiMap();
	UpdatePathCostMap();

}

// get the nearest obstacle distance
double GVDLau::GetObstacleDistance(int row, int col) {
	if (0 < row && height > row && 0 < col && width > col)
		return data[row][col].dist;
	else
		return max_double;
}

// get the nearest voronoi edge distance
double GVDLau::GetVoronoiDistance(int row, int col) {
	if (0 < row && height > row && 0 < col && width > col)
		return data[row][col].voro_dist;
	else
		return max_double;
}

// save the voronoi field map to an external file
void GVDLau::Visualize(std::string filename) {

	if (filename.size()) {

		// write ppm files
		FILE* F = fopen(filename.c_str(), "w");
		if (!F) {
			std::cout << "could not open 'result.pgm' for writing!\n";
			return;
		}

		fprintf(F, "P6\n#\n");
		fprintf(F, "%d %d\n255\n", width, height);

		for(int r = heightminus1; r >= 0; --r){
			for(int c = 0; c < width; ++c){

				unsigned char ch = 0;

				// get the actual cell
				GVDLau::DataCellRef s(data[r][c]);

				if (s.voro) {
					fputc( 0, F );
					fputc( 0, F );
					fputc( 255, F );
				} else if (0 == s.sqdist) {
					fputc( 0, F );
					fputc( 0, F );
					fputc( 0, F );
				} else {
					float f = 80 + (sqrt(s.sqdist)*10);
					if (f > 255) f=255;
					if (f < 0) f = 0;
					ch = (unsigned char)f;
					fputc( ch, F );
					fputc( ch, F );
					fputc( ch, F );
				}
			}
		}
		fclose(F);
	}
}
