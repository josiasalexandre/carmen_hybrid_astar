#ifndef VECTOR_TEMPLATE_HPP
#define VECTOR_TEMPLATE_HPP

#include <array>
#include <cmath>

#include "Matrix.hpp"

namespace astar {

template<typename T>
class Vector2D {

private:

    // PRIVATE ATTRIBUTES

    // PRIVATE METHODS

public:

    // PUBLIC ATTRIBUTES
    T x, y;

    // basic constructor'   '
    Vector2D() : x(0.0), y(0.0) {}

    // copy constructor
    Vector2D(const Vector2D &v) : x(v.x), y(v.y) {}

    // explicit constructor
    Vector2D(T x_, T y_) : x(x_), y(y_) {}

    // distance between two vectors
    T Distance(const Vector2D &v)
    {
        return std::sqrt((x - v.x)*(x - v.x) + (y - v.y)*(y - v.y));
    }

    // squared distance between two vectors
    T Distance2(const Vector2D &v)
    {
        return (x - v.x)*(x - v.x) + (y - v.y)*(y - v.y);
    }

    // norm
    T Norm()
    {
        return std::sqrt(x*x + y*y);
    }

    // add two vectors
    void Add(const Vector2D &v)
    {
        x += v.x;
        y += v.y;
    }

    // add a given scalar to each axis values
    void Add(T value)
    {
        x += value;
        y += value;
    }

    // add the scalars to each axis values
    void Add(T v_x, T v_y)
    {
        x += v_x;
        y += v_y;
    }

    // subtract a given vector from the current vector
    void Subtract(const Vector2D &v)
    {
        x -= v.x;
        y -= v.y;
    }

    // subtract a given scalar from the current vector - translation
    void Subtract(T value)
    {
        x -= value;
        y -= value;
    }

    // subtract the scalar values from the the current vector - translation
    void Subtract(T v_x, T v_y)
    {
        x -= v_x;
        y -= v_y;
    }

    // scale the current vector by a scalar
    void Scale(T value)
    {
        x *= value;
        y *= value;
    }

    // scale the current vector by scalar in each axis, overloading
    void Scale(T v_x, T v_y)
    {
        x *= v_x;
        y *= v_y;
    }

    // multiply the current vector by another vector
    void Multiply(const Vector2D &v)
    {
        x *= v.x;
        y *= v.y;
    }

    // the dot product between the two vectors
    T Dot(const Vector2D &v)
    {
        return x*v.x + y*v.y;
    }

    // translate the current vector by another vector
    void Translate(Vector2D &vec)
    {
        Add(vec);
    }

    // translation overloading
    void Translate(T x_, T y_)
    {
        Add(x_, y_);
    }

    // translate the current vector by a scalar
    void Translate(T value)
    {
        Add(value);
    }

    // rotate the current vector around a the z axis by a given angle
    void RotateZ(T angle)
    {
        T oldX = x, oldY = y;
        x = oldX*std::cos(angle) - oldY*std::sin(angle);
        y = oldX*std::sin(angle) + oldY*std::cos(angle);
    }

    // rotate the current vector around a given point and by an angle
    void RotateZ(const Vector2D &v, T angle)
    {
        // bring to origin
        Subtract(v);

        // rotate around the z axis
        RotateZ(angle);

        // move back to the appropriate position
        Add(v);
    }

    // transform the vector by a given Matrix
    void Transform(const MatrixT<T, 2, 2>& matrix)
    {
        T oldX = x, oldY = y;
        x = matrix.m[0][0]*oldX + matrix.m[0][1]*oldY;
        y = matrix.m[1][0]*oldX + matrix.m[1][1]*oldY;
    }

    // OPERATOR OVERLOADING

    // assignment
    void operator=(const Vector2D &v)
    {
        x = v.x;
        y = v.y;
    }

    // equals comparison
    bool operator==(const Vector2D &v)
    {
        return (std::fabs(x - v.x) < 0.0001 && std::fabs(y - v.y) < 0.0001);
    }

    // different comparator
    bool operator!=(const Vector2D &v)
    {
        return (std::fabs(x - v.x) > 0.0001 || std::fabs(y - v.y) > 0.0001);
    }

    // + operator, add a given vector to the current one, self.add
    Vector2D operator+(const Vector2D &v)
    {
        return Vector2D(x + v.x, y + v.y);
    }

    // + operator, add a given scalar value to the current one, self.add
    Vector2D operator+(T value)
    {
        return Vector2D(x + value, y + value);
    }

    // - operator, subtract a given vector from the current one values, self.subtract
    Vector2D operator-(const Vector2D &v)
    {
        return Vector2D(x - v.x, y - v.y);
    }

    // + operator, add a given scalar value to the current one, self.add
    Vector2D operator-(T value)
    {
        return Vector2D(x - value, y - value);
    }

    // * operator, multiply/scale the current vector by another one
    Vector2D operator*(const Vector2D &v)
    {
        return Vector2D(x*v.x, y*v.y);
    }

    // * operator, multiply/scale the current vector by another one
    Vector2D operator*(T value)
    {
        return Vector2D(x*value, y*value);
    }

    // * operator, multiply/scale the current vector by another one
    Vector2D operator/(T value)
    {
        return Vector2D(x/value, y/value);
    }

};

}
#endif
