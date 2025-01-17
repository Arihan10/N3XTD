// Vector3.cpp
#include "Vector3.h"
#include <cmath>
#include <iostream>

Vector3::Vector3() : x(0), y(0), z(0), w(1) {}

Vector3::Vector3(double x, double y, double z, double w)
    : x(x), y(y), z(z), w(w) {}

Vector3::Vector3(const Vector3& vec)
    : x(vec.x), y(vec.y), z(vec.z), w(vec.w) {}

Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z, w + other.w);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z, w - other.w);
}

Vector3 Vector3::operator*(const Vector3& other) const {
    return Vector3(x * other.x, y * other.y, z * other.z, w);
}

Vector3 Vector3::operator*(double scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar, w * scalar);
}

Vector3 Vector3::operator/(const Vector3& other) const {
    return Vector3(x / other.x, y / other.y, z / other.z, w / other.w);
}

Vector3 Vector3::operator/(double scalar) const {
    return Vector3(x / scalar, y / scalar, z / scalar, w / scalar);
}

double Vector3::dot(const Vector3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
        1.0);
}

double Vector3::length() const {
    return std::sqrt(dot(*this));
}

Vector3 Vector3::normalize() const {
    double len = length();
    return Vector3(x / len, y / len, z / len, w / len);
}

Vector3 Vector3::multiplyMatrix(const double matrix[4][4]) const {
    double resultX = x * matrix[0][0] + y * matrix[0][1] + z * matrix[0][2] + w * matrix[0][3];
    double resultY = x * matrix[1][0] + y * matrix[1][1] + z * matrix[1][2] + w * matrix[1][3];
    double resultZ = x * matrix[2][0] + y * matrix[2][1] + z * matrix[2][2] + w * matrix[2][3];
    double resultW = x * matrix[3][0] + y * matrix[3][1] + z * matrix[3][2] + w * matrix[3][3];
    return Vector3(resultX, resultY, resultZ, resultW);
}

Vector3 Vector3::getMidpoint(const Vector3& one, const Vector3& two, const Vector3& three) {
    return Vector3(
        (one.x + two.x + three.x) / 3.0,
        (one.y + two.y + three.y) / 3.0,
        (one.z + two.z + three.z) / 3.0,
        1.0);
}

Vector3 Vector3::intersectPlane(const Vector3& plane_p, const Vector3& plane_n, const Vector3& lineStart, const Vector3& lineEnd) {
    double plane_d = -plane_n.dot(plane_p);
    double ad = lineStart.dot(plane_n);
    double bd = lineEnd.dot(plane_n);
    double t = (-plane_d - ad) / (bd - ad);
    Vector3 lineStartToEnd = lineEnd - lineStart;
    Vector3 lineToIntersect = lineStartToEnd * t;
    return lineStart + lineToIntersect;
}

double Vector3::signedDistance(const Vector3& p, const Vector3& plane_n, const Vector3& plane_p) {
    return plane_n.dot(p) - plane_n.dot(plane_p);
}

int Vector3::clipTriangleAgainstPlane(const Vector3& plane_p, const Vector3& plane_n, Vector3 triIn[3], Vector3 clipped[2][3]) {
    // Make sure plane normal is normalized
    Vector3 planeNormal = plane_n.normalize();

    // Distance of points from plane
    double distances[3];
    for (int i = 0; i < 3; i++) {
        distances[i] = signedDistance(triIn[i], planeNormal, plane_p);
    }

    // Classify points
    Vector3* insidePoints[3];
    int nInsidePointCount = 0;
    Vector3* outsidePoints[3];
    int nOutsidePointCount = 0;

    for (int i = 0; i < 3; i++) {
        if (distances[i] >= 0) {
            insidePoints[nInsidePointCount++] = &triIn[i];
        }
        else {
            outsidePoints[nOutsidePointCount++] = &triIn[i];
        }
    }

    // Now classify the triangle
    if (nInsidePointCount == 0) {
        // No points inside, so no triangle
        return 0;
    }

    if (nInsidePointCount == 3) {
        // All points inside, so return original triangle
        for (int i = 0; i < 3; i++) {
            clipped[0][i] = triIn[i];
        }
        return 1;
    }

    if (nInsidePointCount == 1 && nOutsidePointCount == 2) {
        // Triangle becomes smaller
        clipped[0][0] = *insidePoints[0];
        clipped[0][1] = intersectPlane(plane_p, planeNormal, *insidePoints[0], *outsidePoints[0]);
        clipped[0][2] = intersectPlane(plane_p, planeNormal, *insidePoints[0], *outsidePoints[1]);
        return 1;
    }

    if (nInsidePointCount == 2 && nOutsidePointCount == 1) {
        // Triangle becomes a quad (split into two triangles)
        clipped[0][0] = *insidePoints[0];
        clipped[0][1] = *insidePoints[1];
        clipped[0][2] = intersectPlane(plane_p, planeNormal, *insidePoints[0], *outsidePoints[0]);

        clipped[1][0] = *insidePoints[1];
        clipped[1][1] = clipped[0][2];
        clipped[1][2] = intersectPlane(plane_p, planeNormal, *insidePoints[1], *outsidePoints[0]);
        return 2;
    }

    return 0;
}

void Vector3::flipZ() {
    z = -z;
}

void Vector3::print(const std::string& prefix) const {
    std::cout << prefix << x << " " << y << " " << z << " " << w << std::endl;
}
