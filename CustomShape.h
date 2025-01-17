// CustomShape.h
#ifndef CUSTOMSHAPE_H
#define CUSTOMSHAPE_H

#include "Shape3D.h"
#include <string>

class CustomShape : public Shape3D {
public:
    CustomShape(const std::string& filename, const Vector3& pos, const Vector3& size);
};

#endif // CUSTOMSHAPE_H
