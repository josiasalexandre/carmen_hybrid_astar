#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cmath>
#include "../Entities/Pose2D.cpp"
#include "GVDLau.hpp"

// load the PGM file
void loadPGM(std::istream &is, int *sizeX, int *sizeY, bool ***map) {

	std::string tag;

	is >> tag;
	if (tag!="P5") {
		std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
		exit(-1);
	}

	while (is.peek()==' ' || is.peek()=='\n') is.ignore();
	while (is.peek()=='#') is.ignore(255, '\n');
	is >> *sizeX;
	while (is.peek()=='#') is.ignore(255, '\n');
	is >> *sizeY;
	while (is.peek()=='#') is.ignore(255, '\n');
	is >> tag;
	if (tag!="255") {
		std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
		exit(-1);
	}
	is.ignore(255, '\n');

	*map = new bool*[*sizeY];

	for (int y = 0; y < *sizeY; ++y) {
		(*map)[y] = new bool[*sizeX];
	}

	for (int y = *sizeY-1; y >= 0; --y) {

		for (int x = 0; x < *sizeX; ++x) {

			int c = is.get();

			if ((double) c < 255-255*0.2) (*map)[y][x] = true; // cell is occupied
			else (*map)[y][x] = false; // cell is free

			if (!is.good()) {
				std::cerr << "Error reading pgm map.\n";
				exit(-1);
			}
		}
	}
}

int main (int argc, char **argv) {

	if(argc < 2 || argc > 3) {
	    std::cerr<<"usage: "<< argv[0] << " <pgm map> \n";
	    exit(-1);
	}

	// allocate a new GVD object in stack
	std::cout << "\nBuild a GVDLau object ...\n";
	astar::GVDLau gvd;
	std::cout << "Done!\n";

	// LOAD PGM MAP AND INITIALIZE THE VORONOI
	std::ifstream is(argv[1]);
	if (!is) {
		std::cerr << "Could not open map file for reading.\n";
		exit(-1);
	}

	// auxiliar vars
	bool **map = nullptr;
	int width, height;
	std::string filename = "initial.ppm";

	// load the image
	std::cout << "\nLoading the input image: " << argv[1] << "\n";
	loadPGM(is, &width, &height, &map);
	std::cout << "Done!\n";

	// initialize the map
	std::cout << "\nInitializing a " << width << "x" << height << " map\n";
	gvd.InitializeEmpty(height, width);
	std::cout << "Done!\n";

	// initialize the map
	//std::cout << "\nInitializing a " << width << "x" << height << " map\n";
	//gvd.InitializeMap(height, width, map);
	//std::cout << "Done!\n";


	// hard update
	for (int h = 0; h < height; ++h) {

		for (int c = 0; c < width; ++c) {

			if (map[h][c]) {
				gvd.SetObstacle(h, c);
			}
		}

	}
	// update
	std::cout << "\nUpdating the voronoi map ...\n";
	//int t1 = std::clock();
	gvd.Update();
	std::cout << "Done!\n";

	// initialize the map
	/*std::cout << "\n------------->Re Initializing a " << width << "x" << height << " map\n";
	gvd.InitializeMap(height, width, map);
	std::cout << "Done!\n";

	// update
	std::cout << "\n------------>Updating the voronoi map again ...\n";
	gvd.Update();
	std::cout << "Done!\n";*/

	unsigned int r = 167;
	unsigned int c = 500;
	// test the nearest obstacle query
	std::cout << "\nNearest obstacle distance query for (" << c << ", " << r << "), it should be: inf! ...\n";
	double dist = gvd.GetObstacleDistance(c, r);
	std::cout << "Done! Result dist(" << c << ", " << r << "): " << dist << "\n";

	std::cout << "\nNearest obstacle distance query for (" << r << ", " << c << "), it should not be: inf! ...\n";
	dist = gvd.GetObstacleDistance(r, c);
	std::cout << "Done! Result dist(" << r << ", " << c << "): " << dist << "\n";

	// test the nearest voro edge query
	std::cout << "\nNearest voronoi edge distance query for (" << c << ", " << r << "), it should be: inf! ...\n";
	double voro_dist = gvd.GetVoronoiDistance(c, r);
	std::cout << "Done! Result voro_dist(" << c << ", " << r << "): " << voro_dist << "\n";

	std::cout << "\nNearest voronoi edge distance query for (" << r << ", " << c << "), it should not be: inf! ...\n";
	voro_dist = gvd.GetVoronoiDistance(r, c);
	std::cout << "Done! Result voro_dist(" << r << ", " << c << "): " << voro_dist << "\n";

	std::cout << "\nGet the current pathcost query for (" << r << ", " << c << "), it should not be: inf! ...\n";
	double cost = gvd.GetPathCost(r, c);
	std::cout << "Done! Result cost(" << r << ", " << c << "): " << cost << "\n";

	std::cout << "........................................\n";
	std::cout << "........................................\n";

	// test the nearest voro edge query
	std::cout << "\nKDtree based Nearest voronoi edge distance query for (" << r << ", " << c << "), it should not be: inf! ...\n";
	astar::GridCellIndex index_c(gvd.GetVoronoiIndex(r, c));
	std::cout << "Done! Result voro_dist(" << r << ", " << c << "): (" << index_c.row << ", " << index_c.col << ")" << "\n";

	int dr = r - index_c.row;
	int dc = c - index_c.col;

	std:: cout << "\n The euclidian distance: " << std::sqrt(dr*dr + dc*dc);

	// save the voronoi map to the output file
	std::cout << "\nSaving the voronoi field map to the output file ...\n";
	gvd.Visualize(filename);
	std::cout << "Done\n";

	// delete the allocated map
	if (nullptr != map) {

		for (int r = 0; r < height; ++r) {

			// remove the current row
			delete [] map[r];

		}

		// remove the map pointer
		delete [] map;

	}

	return 0;
}
