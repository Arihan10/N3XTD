// Cube.h
#ifndef CUBE_H
#define CUBE_H

#include "Shape3D.h"

class Cube : public Shape3D {
public:
    Cube(const Vector3& pos, const Vector3& size);
};

#endif // CUBE_H
