#ifndef VECTOR_TEMPLATE_HPP
#define VECTOR_TEMPLATE_HPP

#include <array>

#include "Matrix.hpp"

namespace astar {

class Vector2D {

    private:

        // PRIVATE ATTRIBUTES

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES

        // the x and y coordinates
        double x, y;

        // PUBLIC METHODS

        // basic constructor
        Vector2D();

        // copy constructor
        Vector2D(const Vector2D&);

        // explicit constructor
        Vector2D(double, double);

        // distance between two vectors
        double distance(const Vector2D&);

        // distance between two vectors
        double distance2(const Vector2D&);

        // norm
        double norm();

        // add two vectors
        void add(const Vector2D&);

        // add a given scalar to each axis values - translation
        void add(double);

        // add the scalars to each axis values - translation
        void add(double, double);

        // subtract a given vector from the current vector - translation
        void subtract(const Vector2D&);

        // subtract a given scalar from the current vector - translation
        void subtract(double);

        // subtract the scalar values from the the current vector - translation
        void subtract(double, double);

        // multiply the current vector by another vector
        void multiply(const Vector2D&);

        // scale the current vector by a given scalar
        void scale(double);

        // scale the current vector by scalar values in each axis, overloading
        void scale(double, double);

        // the dot product between the two vectors
        double dot(const Vector2D&);

        // translate the current vector by another vector
        void translate(Vector2D&);

        // translation overloading
        void translate(double, double);

        // translate the current vector by a scalar
        void translate(double);

        // rotate the current vector around a the z axis by a given angle
        void rotateZ(double);

        // rotate the current vector around a given point and by an angle
        void rotateZ(const Vector2D&, double);

        // transform the vector by a given Matrix
        void transform(const astar::MatrixT<double, 2, 2>&);

        // OPERATOR OVERLOADING

        // assignment
        void operator=(const Vector2D&);

        // equals comparison
        bool operator==(const Vector2D&);

        // different comparator
        bool operator!=(const Vector2D&);

        // + operator, add a given vector to the current one, self.add
        Vector2D operator+(const Vector2D&);

        // + operator, add a given scalar value to the current one, self.add
        Vector2D operator+(double);

        // - operator, subtract a given vector from the current one values, self.subtract
        Vector2D operator-(const Vector2D&);

        // + operator, add a given scalar value to the current one, self.add
        Vector2D operator-(double);

        // * operator, multiply/scale the current vector by another one, self multiply
        Vector2D operator*(const Vector2D&);

        // * operator, multiply/scale the current vector by a given value, self multiply
        Vector2D operator*(double);

        // / operator, divide the current vector by another one
        Vector2D operator/(double);

};

}
#endif
