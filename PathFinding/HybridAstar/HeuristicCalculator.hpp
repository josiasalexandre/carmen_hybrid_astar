#ifndef HYBRID_ASTAR_HEURISTIC_CALCULATOR_HPP
#define HYBRID_ASTAR_HEURISTIC_CALCULATOR_HPP

#include <cmath>
#include <limits>
#include <iostream>

#include "../../ReedsShepp/ReedsSheppModel.hpp"
#include "NonholonomicHeuristicInfo.hpp"


namespace astar {

class HeuristicCalculator {

	private:

		// PRIVATE ATTRIBUTES

		// PRIVATE METHODS

	public:

		// compute the non-holonomic without obstacles heuristic around a given neihgborhood
		static astar::NonholonomicHeuristicInfo* Calculate(double neighborhoodSize, double resolution, int orientations, double vehicle_turn_radius) {

			//build a RS Model
			astar::ReedsSheppModel rs;

			// creates a new heuristic info
			astar::NonholonomicHeuristicInfo *info = new astar::NonholonomicHeuristicInfo;

			// save the neighborhood size
			info->neighborhood_size = neighborhoodSize;

			// save the orientations
			info->orientations = orientations;

			// get the orientations offsets
			info->orientation_offset = 2.0 * M_PI / ((double) orientations);

			// save the resolution
			info->resolution = resolution;

			// how many cells??
			info->num_cells = std::ceil(neighborhoodSize / resolution);

			// get odd number of cells
			if (0 == info->num_cells % 2) {

				info->num_cells += 1;

			}

			// the offset
			info->position_offset = std::floor(info->num_cells * 0.5) * resolution;

			// the max heuristic value
			info->max_heuristic_value = std::numeric_limits<double>::min();

			// allocate the heuristic table
			info->heuristic = new double**[info->num_cells];

			for (unsigned int r = 0; r < info->num_cells; ++r) {

				info->heuristic[r] = new double*[info->num_cells];

				for (unsigned int c = 0; c < info->num_cells; ++c) {

					info->heuristic[r][c] = new double[info->orientations];

				}

			}

			// temp pose
			astar::Pose2D pose;

			// the goal pose at the origin
			astar::Pose2D goal(0.0, 0.0, 0.0);

			// compute the heuristic
			for (unsigned int r = 0; r < info->num_cells; ++r) {

				for (unsigned int c = 0; c < info->num_cells; ++c) {

					double *cost = info->heuristic[r][c];

					for (unsigned int o = 0; o < info->orientations; ++o) {

						// get the pose values
						pose.position.x = c * resolution - info->position_offset;
						pose.position.y = r * resolution - info->position_offset;
						pose.orientation = o * info->orientation_offset;

						// Reeds-Shepp heuristic computation
						astar::ReedsSheppActionSetPtr action_set = rs.Solve(pose, goal, vehicle_turn_radius);

						if (0 < action_set->actions.size()) {

							cost[o] = action_set->CalculateCost(vehicle_turn_radius, 1.0, 0.0);

							if (info->max_heuristic_value < action_set->length) {

								info->max_heuristic_value = action_set->length;

							}

						}

						// remove the action set
						delete action_set;

					}

				}

			}

			// save the current heuristic to the external file
			NonholonomicHeuristicInfo::Save(*info, "heuristic_info.txt");

			return info;

		}


};


}

#endif
