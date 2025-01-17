// Shape3D.cpp
#include "Shape3D.h"
#include <random> 

const sf::Color Shape3D::randomColors[6] = {
    sf::Color::Red,
    sf::Color::Green,
    sf::Color::Blue,
    sf::Color::Yellow,
    sf::Color::Cyan,
    sf::Color::Magenta,
};

Shape3D::Shape3D() {
    name = "Obj";

    std::random_device rd; // seed
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 5);

    int randomIndex = dist(gen);

    color = randomColors[randomIndex];
}

void Shape3D::updateTrans() {
    verts.resize(vertsOG.size());
    for (size_t i = 0; i < vertsOG.size(); ++i) {
        verts[i] = vertsOG[i] * (scale * 50.0) + pos;
    }
}

const std::vector<Vector3>& Shape3D::getVerts() const {
    return verts;
}

const std::vector<int>& Shape3D::getTris() const {
    return tris;
}

void Shape3D::setColor(const sf::Color& color) {
    this->color = color;
}

const sf::Color& Shape3D::getColor() const {
    return color;
}
