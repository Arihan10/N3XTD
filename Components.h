// Components.h
#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Vector3.h"
#include <string>
#include <vector>

struct TransformComponent {
    Vector3 position;
    Vector3 scale;

    TransformComponent(const Vector3& pos = Vector3(), const Vector3& scl = Vector3(1, 1, 1))
        : position(pos), scale(scl) {}
};

struct NameComponent {
    std::string name;

    NameComponent(const std::string& n = "Entity") : name(n) {}
};

struct ColorComponent {
    float r, g, b;

    ColorComponent(float r = 1.0f, float g = 1.0f, float b = 1.0f)
        : r(r), g(g), b(b) {}
};

struct MeshComponent {
    std::vector<Vector3> vertices;
    std::vector<int> triangles;
    std::vector<Vector3> normals;
    std::vector<Vector3> originalVertices;

    void calculateNormals() {
        normals.clear();
        normals.resize(triangles.size() / 3);

        for (size_t i = 0; i < triangles.size(); i += 3) {
            Vector3 v0 = originalVertices[triangles[i]];
            Vector3 v1 = originalVertices[triangles[i + 1]];
            Vector3 v2 = originalVertices[triangles[i + 2]];
            Vector3 normal = (v1 - v0).cross(v2 - v0).normalize();
            normals[i / 3] = normal;
        }
    }

    void updateTransform(const Vector3& position, const Vector3& scale) {
        vertices.resize(originalVertices.size());
        for (size_t i = 0; i < originalVertices.size(); ++i) {
            vertices[i] = originalVertices[i] * (scale) + position;
        }
        calculateNormals();
    }
};

struct ColliderComponent {
    enum Type {
        SPHERE,
        BOX
    };
    Type type;
    Vector3 size; // For box: half-extents, For sphere: x = radius
    Vector3 offset; // Offset from transform position

    ColliderComponent(Type t = SPHERE, const Vector3& s = Vector3(1, 1, 1), const Vector3& off = Vector3())
        : type(t), size(s), offset(off) {}
};

struct RigidbodyComponent {
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 angularVelocity;    // Added for ball spin
    float mass;
    float restitution;          // Bounciness
    float friction;             // Ground friction coefficient
    float airResistance;        // Air drag coefficient
    bool isStatic;             // Static objects don't move

    RigidbodyComponent(
        float m = 1.0f,
        float r = 0.5f,
        float f = 0.1f,
        float air = 0.02f,
        bool stat = false
    ) : velocity(0, 0, 0, 0),
        acceleration(),
        angularVelocity(0, 0, 0, 0),
        mass(m),
        restitution(r),
        friction(f),
        airResistance(air),
        isStatic(stat) {}
};

#endif