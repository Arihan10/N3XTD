// Shape3D.h
#ifndef SHAPE3D_H
#define SHAPE3D_H

#include <vector>
#include "Vector3.h"

struct Color {
    float r, g, b;
    Color(float r = 1.0f, float g = 1.0f, float b = 1.0f) : r(r), g(g), b(b) {}
};

class Shape3D {
protected:
    std::vector<Vector3> vertsOG;
    std::vector<Vector3> verts;
    std::vector<int> tris; 
    std::vector<Vector3> normals; 
    Color color;

    static const Color randomColors[6]; 

public:
    std::string name;
    Vector3 pos;
    Vector3 scale;

    Shape3D();
    Shape3D(const Color& col);
    virtual ~Shape3D() {}

    virtual void updateTrans();

    void calculateNormals(); 

    const std::vector<Vector3>& getVerts() const;
    const std::vector<int>& getTris() const; 
    const std::vector<Vector3>& getNormals() const; 
    void setColor(const Color& color);
    const Color& getColor() const;
};

#endif // SHAPE3D_H
