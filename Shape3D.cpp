#include "stdafx.h"

// Shape3D.cpp
#include "Shape3D.h"
#include <random>

const Color Shape3D::randomColors[6] = {
    Color(1.0f, 0.0f, 0.0f),  // Red
    Color(0.0f, 1.0f, 0.0f),  // Green
    Color(0.0f, 0.0f, 1.0f),  // Blue
    Color(1.0f, 1.0f, 0.0f),  // Yellow
    Color(0.0f, 1.0f, 1.0f),  // Cyan
    Color(1.0f, 0.0f, 1.0f)   // Magenta
};

Shape3D::Shape3D() {
    name = "Obj";

    std::random_device rd; // seed
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 5);

    int randomIndex = dist(gen);

    color = randomColors[randomIndex]; 

    calculateNormals(); 
}

Shape3D::Shape3D(const Color& col) {
    name = "Obj";

    std::random_device rd; // seed
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 5);

    int randomIndex = dist(gen);

    color = col;

    calculateNormals();
}

void Shape3D::updateTrans() {
    verts.resize(vertsOG.size());
    for (size_t i = 0; i < vertsOG.size(); ++i) {
        verts[i] = vertsOG[i] * (scale * 50.0) + pos;
    }
    calculateNormals(); 
}

void Shape3D::calculateNormals() {
    normals.clear();
    normals.resize(tris.size() / 3);  // One normal per face

    for (size_t i = 0; i < tris.size(); i += 3) {
        Vector3 v0 = vertsOG[tris[i]];
        Vector3 v1 = vertsOG[tris[i + 1]];
        Vector3 v2 = vertsOG[tris[i + 2]];

        Vector3 normal = (v1 - v0).cross(v2 - v0).normalize();
        normals[i / 3] = normal;
    }
}

const std::vector<Vector3>& Shape3D::getVerts() const {
    return verts;
}

const std::vector<int>& Shape3D::getTris() const {
    return tris;
}

const std::vector<Vector3>& Shape3D::getNormals() const {
    return normals;
}

void Shape3D::setColor(const Color& color) {
    this->color = color;
}

const Color& Shape3D::getColor() const {
    return color;
}
