// MeshLoader.cpp
#include "stdafx.h"
#include "MeshLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool MeshLoader::LoadFromFile(const std::string& filename,
    std::vector<Vector3>& vertices,
    std::vector<int>& triangles) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return false;
    }

    vertices.clear();
    triangles.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string type;
        ss >> type;

        if (type == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            vertices.emplace_back(x, y, z, 1);
        }
        else if (type == "f") {
            int idx1, idx2, idx3;
            ss >> idx1 >> idx2 >> idx3;
            triangles.push_back(idx3 - 1);
            triangles.push_back(idx2 - 1);
            triangles.push_back(idx1 - 1);
        }
    }

    file.close();
    return true;
}