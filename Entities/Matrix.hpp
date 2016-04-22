#ifndef MATRIX_TEMPLATE_HPP
#define MATRIX_TEMPLATE_HPP

#include <array>

namespace astar {

template<typename T, unsigned int rows, unsigned int cols>
class MatrixT {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // the actual matrix
        std::array<std::array<T, cols>, rows> m;


        // PUBLIC METHODS

        //basic constructor
        MatrixT() {}

    };

}

#endif
