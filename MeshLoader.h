// MeshLoader.h
#ifndef MESHLOADER_H
#define MESHLOADER_H

#include <string>
#include <vector>
#include "Vector3.h"

class MeshLoader {
public:
    static bool LoadFromFile(const std::string& filename,
        std::vector<Vector3>& vertices,
        std::vector<int>& triangles);
};

#endif