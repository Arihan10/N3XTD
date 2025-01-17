// Cube.cpp
#include "Cube.h"

Cube::Cube(const Vector3& pos, const Vector3& size) {
    name = "Cube";
    this->pos = pos;
    this->scale = size;

    // Vertices
    vertsOG = {
        Vector3(-1, -1, -1, 1),
        Vector3(1, -1, -1, 1),
        Vector3(-1, -1, 1, 1),
        Vector3(1, -1, 1, 1),
        Vector3(-1, 1, -1, 1),
        Vector3(1, 1, -1, 1),
        Vector3(-1, 1, 1, 1),
        Vector3(1, 1, 1, 1)
    };

    // Triangles
    tris = {
        0, 4, 5, 0, 5, 1,
        1, 5, 7, 1, 7, 3,
        2, 0, 1, 2, 1, 3,
        2, 6, 4, 2, 4, 0,
        4, 6, 7, 4, 7, 5,
        3, 7, 6, 3, 6, 2
    };

    updateTrans();
}
