#ifndef ASTAR_VECTOR_2D_ARRAY_HPP
#define ASTAR_VECTOR_2D_ARRAY_HPP

#include <vector>

#include "Vector2D.hpp"

namespace astar {

template<typename T>
class Vector2DArray {

public:

	// the current array
	std::vector<astar::Vector2D<T>> vs;

	// compare two Vector2DArray
	bool EqualsTo(const astar::Vector2DArray<T> &b) {

		if (vs.size() == b.vs.size()) {

			// get the current size
			unsigned int v_size = vs.size();

			// get the direct access
			const std::vector<astar::Vector2D<T>> &vectors(b.vs);

			for (unsigned int i = 0; i < v_size; ++i) {

				if (vectors[i] != vs[i]) {

					return false;

				}

			}

		}

		return false;

	}
	// == operator overloading

};

template<typename T>
using Vector2DArrayPtr = Vector2DArray<double>*;

}

#endif
