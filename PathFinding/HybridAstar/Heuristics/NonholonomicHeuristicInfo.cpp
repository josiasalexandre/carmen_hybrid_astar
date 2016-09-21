#include "NonholonomicHeuristicInfo.hpp"

#include <exception>
#include <limits>
#include <fstream>
#include <cmath>
#include <iostream>

using namespace astar;

// default constructor
NonholonomicHeuristicInfo::NonholonomicHeuristicInfo() :
		        neighborhood_size(0), num_cells(0), resolution(0.0), position_offset(0), orientations(0), orientation_offset(0), heuristic(nullptr), max_heuristic_value(std::numeric_limits<double>::max()) {}

// default destructor
NonholonomicHeuristicInfo::~NonholonomicHeuristicInfo() {

    // clear the allocated heuristic table
    Clear();

}

// alloc a new heuristic table
void NonholonomicHeuristicInfo::Build() {

    if (0 < num_cells && 0 < orientations) {

        // clear any old heuristic table
        Clear();

        // allocate
        heuristic = new double**[num_cells];

        for (unsigned int r = 0; r < num_cells; ++r) {

            heuristic[r] = new double*[num_cells];

            for (unsigned int c = 0; c < num_cells; ++c) {

                heuristic[r][c] = new double[orientations];

            }

        }

    }

}

// clear the entire heuristic table
void NonholonomicHeuristicInfo::Clear() {

    if (nullptr != heuristic && 0 != num_cells && 0 != orientations) {

        for (unsigned int r = 0; r < num_cells; ++r) {
            for (unsigned int c = 0; c < num_cells; ++c) {

                // remove the orientation collumn
                delete [] heuristic[r][c];
            }

            delete [] heuristic[r];
        }

        delete [] heuristic;
    }

}

// save the current heuristic to external file
void NonholonomicHeuristicInfo::Save(const NonholonomicHeuristicInfo &info, std::string filename) {

    if (nullptr != info.heuristic && 0 != info.num_cells && 0 != info.orientations && 0 < filename.size()) {

        // open the external file
        std::ofstream file(filename);

        // save the neighborhood size int the first line
        file << info.neighborhood_size << std::endl;

        // save the resolution in the second line
        file << info.resolution << std::endl;

        // save the orientations in the third line
        file << info.orientations << std::endl;

        // write the current heuristic table to output file
        for (unsigned int r = 0; r < info.num_cells; ++r) {

            for (unsigned int c = 0; c < info.num_cells; ++c) {

                double *cost = info.heuristic[r][c];

                for (unsigned int o = 0; o < info.orientations; ++o) {

                    file << cost[o] << " ";

                }

            }

        }

        file << std::endl;

        // close the file
        file.close();

    }

}

// load the external heuristic file
void NonholonomicHeuristicInfo::Load(NonholonomicHeuristicInfo &info, std::string filename) {

    if (0 < filename.size()) {

        std::string line;

        // open the external file
        std::ifstream file(filename);

        // verify if the file is open
        if (!file.is_open()) {

            //
            std::cout << "Could no open the file: " << filename << "\n";

            // just a standard exception
            throw std::exception();

        }

        // clear the old heuristic table, if any
        info.Clear();

        // get the neighborhood size in the first line
        file >> info.neighborhood_size;

        // get the resolution in the second line
        file >> info.resolution;

        // get the orientations in the third line
        file >> info.orientations;

        // compute the number of cells
        // get the orientations offsets
        info.orientation_offset = 2.0 * M_PI / info.orientations;

        // how many cells??
        info.num_cells = std::ceil(info.neighborhood_size / info.resolution);

        // get odd number of cells
        if (0 == info.num_cells % 2) {

            info.num_cells += 1;

        }

        // compute the offset
        info.position_offset = std::floor(info.num_cells * 0.5) * info.resolution;

        // the max heuristic value
        info.max_heuristic_value = std::numeric_limits<double>::min();

        // clear the heuristic

        // allocate
        info.Build();

        // read the current heuristic table from the input file
        for (unsigned int r = 0; r < info.num_cells; ++r) {

            for (unsigned int c = 0; c < info.num_cells; ++c) {

                double *cost = info.heuristic[r][c];

                for (unsigned int o = 0; o < info.orientations; ++o) {

                    file >> cost[o];

                }

            }

        }

        // close the file
        file.close();

    }

}
