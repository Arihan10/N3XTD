// Shape3D.h
#ifndef SHAPE3D_H
#define SHAPE3D_H

#include <vector>
#include "Vector3.h"
#include <SFML/Graphics/Color.hpp>

class Shape3D {
protected:
    std::vector<Vector3> vertsOG;
    std::vector<Vector3> verts;
    std::vector<int> tris;
    sf::Color color;

    static const sf::Color randomColors[6];

public:
    std::string name;
    Vector3 pos;
    Vector3 scale;

    Shape3D();
    virtual ~Shape3D() {}

    virtual void updateTrans();

    const std::vector<Vector3>& getVerts() const;
    const std::vector<int>& getTris() const;
    void setColor(const sf::Color& color);
    const sf::Color& getColor() const;
};

#endif // SHAPE3D_H
