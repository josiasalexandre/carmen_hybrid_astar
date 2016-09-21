/*
// Author: josiasalexandre@gmail.com

// Based on the Matt Bradley's Masters Degree thesis and work
// Copyright (c) 2012 Matt Bradley
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
