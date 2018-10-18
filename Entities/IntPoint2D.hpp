#ifndef INT_POINT_2D_HPP
#define INT_POINT_2D_HPP

namespace astar {

	/*! A light-weight integer point with fields x,y */
	class IntPoint2D {

	    public:

	        // public attributes
	        int x, y;

	        // basic constructors
	        IntPoint2D() : x(0), y(0) {}
	        IntPoint2D(int _x, int _y) : x(_x), y(_y) {}

	};

}

#endif
