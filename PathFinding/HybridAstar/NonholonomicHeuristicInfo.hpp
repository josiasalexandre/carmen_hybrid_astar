#ifndef HYBRID_ASTAR_NON_HOLOMIC_HEURISTIC_INFO_HPP
#define HYBRID_ASTAR_NON_HOLOMIC_HEURISTIC_INFO_HPP

#include <string>

namespace astar {

class NonholonomicHeuristicInfo {

	public:

		// default parameters
		double neighborhood_size;
		unsigned int num_cells;
		double resolution;
		double position_offset;
		unsigned int orientations;
		double orientation_offset;
		double ***heuristic;
		double max_heuristic_value;

		// default constructor
		NonholonomicHeuristicInfo();

		// default destructor
		~NonholonomicHeuristicInfo();

		// build a new heuristic table
		void Build();

		// clear the entire table
		void Clear();

		// save the current heuristic to external file
		static void Save(const NonholonomicHeuristicInfo&, std::string);

		// load the external heuristic file
		static void Load(NonholonomicHeuristicInfo&, std::string);
};

}


#endif
