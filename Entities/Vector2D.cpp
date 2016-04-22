#include "Vector2D.hpp"

#include <cmath>

using namespace astar;

// basic constructor
Vector2D::Vector2D() : x(0.0), y(0.0) {}

// copy constructor
Vector2D::Vector2D(const Vector2D &v) : x(v.x), y(v.y) {}

// explicit constructor
Vector2D::Vector2D(double x_, double y_) : x(x_), y(y_) {}

// distance between two vectors

double Vector2D::distance(const Vector2D &v) {

    double dx = x - v.x;
    double dy = y - v.y;

    return std::sqrt(dx*dx + dy*dy);

}

// squared distance between two vectors
double Vector2D::distance2(const Vector2D &v) {

    double dx = x - v.x;
    double dy = y - v.y;

    return dx*dx + dy*dy;

}

// norm
double Vector2D::norm() {

    return std::sqrt(x*x + y*y);

}

// add two vectors
void Vector2D::add(const Vector2D &v) {

    // update the values
    x += v.x;
    y += v.y;

    return;

}

// add a given scalar to each axis values
void Vector2D::add(double value) {

    // update the values
    x += value;
    y += value;

    return;

}

// add the scalars to each axis values
void Vector2D::add(double v_x, double v_y) {

    // update the values
    x += v_x;
    y += v_y;

    return;

}

// subtract a given vector from the current vector
void Vector2D::subtract(const Vector2D &v) {

    // update the values
    x -= v.x;
    y -= v.y;

    return;

}

// subtract a given scalar from the current vector - translation
void Vector2D::subtract(double value) {

    // update the values
    x -= value;
    y -= value;

    return;

}

// subtract the scalar values from the the current vector - translation
void Vector2D::subtract(double v_x, double v_y) {

    // update the values
    x -= v_x;
    y -= v_y;

    return;

}

// scale the current vector by a scalar
void Vector2D::scale(double value) {

    // update the values
    x *= value;
    y *= value;

    return;

}

// scale the current vector by scalar in each axis, overloading
void Vector2D::scale(double v_x, double v_y) {

    // update the values
    x *= v_x;
    y *= v_y;

    return;;

}

// multiply the current vector by another vector
void Vector2D::multiply(const Vector2D &v) {

    // update the values
    x *= v.x;
    y *= v.y;

    return;

}

// the dot product between the two vectors
// the dot product between the two vectors

double Vector2D::dot(const Vector2D &v) {

    return x*v.x + y*v.y;

}

// translate the current vector by another vector
void Vector2D::translate(Vector2D &vec) {

    add(vec);

    return;

}

// translation overloading
void Vector2D::translate(double x_, double y_) {

    add(x_, y_);

    return;

}

// translate the current vector by a scalar
void Vector2D::translate(double value) {

    add(value);

    return;

}

// rotate the current vector around a the z axis by a given angle
void Vector2D::rotateZ(double angle) {

    // update the values
    double oldX = x, oldY = y;
    x = oldX*std::cos(angle) - oldY*std::sin(angle);
    y = oldX*std::sin(angle) + oldY*std::cos(angle);

    return;

}

// rotate the current vector around a given point and by an angle
void Vector2D::rotateZ(const Vector2D &v, double angle) {

    // bring to origin
    subtract(v);

    // rotate around the z axis
    rotateZ(angle);

    // move back to the appropriate position
    add(v);

    return;
    
}

// transform the vector by a given Matrix
void Vector2D::transform(const MatrixT<double, 2, 2>& matrix) {

    // matrix vector multiply
    double oldX = x, oldY = y;
    x = matrix.m[0][0]*oldX + matrix.m[0][1]*oldY;
    y = matrix.m[1][0]*oldX + matrix.m[1][1]*oldY;

    return;

}

// OPERATOR OVERLOADING

// assignment
void Vector2D::operator=(const Vector2D &v) {

    // get the external values
    x = v.x;
    y = v.y;

}

// equals comparison
bool Vector2D::operator==(const Vector2D &v) {

    return (std::fabs(x - v.x) < 0.0001 && std::fabs(y - v.y) < 0.0001);

}

// different comparator
bool Vector2D::operator!=(const Vector2D &v) {

    return (std::fabs(x - v.x) > 0.0001 || std::fabs(y - v.y) > 0.0001);

}

// + operator, add a given vector to the current one, self.add
Vector2D Vector2D::operator+(const Vector2D &v) {

    return Vector2D(x + v.x, y + v.y);

}

// + operator, add a given scalar value to the current one, self.add
Vector2D Vector2D::operator+(double value) {

    return Vector2D(x + value, y + value);

}

// - operator, subtract a given vector from the current one values, self.subtract
Vector2D Vector2D::operator-(const Vector2D &v) {

    return Vector2D(x - v.x, y - v.y);

}

// + operator, add a given scalar value to the current one, self.add
Vector2D Vector2D::operator-(double value) {

    return Vector2D(x - value, y - value);

}

// * operator, multiply/scale the current vector by another one
Vector2D Vector2D::operator*(const Vector2D &v) {

    return Vector2D(x*v.x, y*v.y);

}

// * operator, multiply/scale the current vector by another one
Vector2D Vector2D::operator*(double value) {

    return Vector2D(x*value, y*value);

}

// * operator, multiply/scale the current vector by another one
Vector2D Vector2D::operator/(double value) {

    // update the values
    return Vector2D(x/value, y/value);

}
