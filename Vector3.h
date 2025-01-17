// Vector3.h
#ifndef VECTOR3_H
#define VECTOR3_H

#include <string>

class Vector3 {
public:
    double x;
    double y;
    double z;
    double w;

    Vector3();
    Vector3(double x, double y, double z, double w = 1.0);
    Vector3(const Vector3& vec);

    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(const Vector3& other) const; // Element-wise multiplication
    Vector3 operator*(double scalar) const;
    Vector3 operator/(const Vector3& other) const; // Element-wise division
    Vector3 operator/(double scalar) const;

    double dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
    double length() const;
    Vector3 normalize() const;

    Vector3 multiplyMatrix(const double matrix[4][4]) const;

    static Vector3 getMidpoint(const Vector3& one, const Vector3& two, const Vector3& three);
    static Vector3 intersectPlane(const Vector3& plane_p, const Vector3& plane_n, const Vector3& lineStart, const Vector3& lineEnd);
    static double signedDistance(const Vector3& p, const Vector3& plane_n, const Vector3& plane_p);
    static int clipTriangleAgainstPlane(const Vector3& plane_p, const Vector3& plane_n, Vector3 triIn[3], Vector3 clipped[2][3]);

    void print(const std::string& prefix) const;
    void flipZ();
};

#endif // VECTOR3_H
