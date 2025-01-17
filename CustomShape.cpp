#include "stdafx.h"

// CustomShape.cpp
#include "CustomShape.h"
#include <fstream>
#include <sstream>
#include <iostream>

CustomShape::CustomShape(const std::string& filename, const Vector3& pos, const Vector3& size)
    : Shape3D() {  // Call default Shape3D constructor for random color
    LoadFromFile(filename, pos, size);
}

CustomShape::CustomShape(const std::string& filename, const Vector3& pos, const Vector3& size, const Color& col)
    : Shape3D(col) {  // Call Shape3D constructor with specified color
    LoadFromFile(filename, pos, size);
}

void CustomShape::LoadFromFile(const std::string& filename, const Vector3& pos, const Vector3& size) {
    this->pos = pos;
    this->scale = size;
    name = filename.substr(0, filename.length() - 4);

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue; 

        std::stringstream ss(line);
        std::string type;
        ss >> type; 

        if (type == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            vertsOG.emplace_back(x, y, z, 1);
        }
        else if (type == "f") {
            int idx1, idx2, idx3;
            char slash; // To skip over slashes in face definitions
            ss >> idx1 >> idx2 >> idx3;
            tris.push_back(idx3 - 1);
            tris.push_back(idx2 - 1);
            tris.push_back(idx1 - 1);
        }
    }

    file.close();
    updateTrans();
}