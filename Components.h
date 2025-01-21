// Components.h
#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Vector3.h"
#include <string>
#include <vector>

struct TransformComponent {
    Vector3 position;
    Vector3 rotation;  // Rotation in radians (x=pitch, y=yaw, z=roll)
    Vector3 scale;

    TransformComponent(
        const Vector3& pos = Vector3(0,0,0),
        const Vector3& rot = Vector3(0,0,0),
        const Vector3& scl = Vector3(1, 1, 1)
    ) : position(pos), rotation(rot), scale(scl) {}

    Vector3 getForward() const {
        // Convert degrees to radians
        const double DEG_TO_RAD = 3.14159265359 / 180.0;

        // Get rotation angles and convert to radians
        double pitch = rotation.x * DEG_TO_RAD;  // X rotation
        double yaw = rotation.y * DEG_TO_RAD;    // Y rotation
        double roll = rotation.z * DEG_TO_RAD;   // Z rotation

        // Calculate forward vector components
        // We only need yaw and pitch for forward vector (roll doesn't affect forward direction)
        double forwardX = sin(yaw) * cos(pitch);
        double forwardY = -sin(pitch);  // Negative because pitch up gives negative Y
        double forwardZ = cos(yaw) * cos(pitch);

        // Return normalized forward vector
        return Vector3(forwardX, forwardY, forwardZ, 0.0).normalize();
    }

    Vector3 getRight() const {
        Vector3 forward = getForward();
        Vector3 worldUp(0.0, 1.0, 0.0);
        // Right is cross product of forward and up
        return forward.cross(worldUp).normalize();
    }

    Vector3 getUp() const {
        // Right vector cross forward vector gives up
        return getRight().cross(getForward()).normalize();
    }
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
    int zIndex = 0; 

    MeshComponent(int zIdx = 0) : zIndex(zIdx) {}  // Constructor with default z-index

    void calculateNormals() {
        normals.clear();
        normals.resize(triangles.size() / 3);
        for (size_t i = 0; i < triangles.size(); i += 3) {
            /*Vector3 v0 = originalVertices[triangles[i]];
            Vector3 v1 = originalVertices[triangles[i + 1]];
            Vector3 v2 = originalVertices[triangles[i + 2]];*/
            Vector3 v0 = vertices[triangles[i]]; 
            Vector3 v1 = vertices[triangles[i + 1]]; 
            Vector3 v2 = vertices[triangles[i + 2]]; 
            Vector3 normal = (v1 - v0).cross(v2 - v0).normalize();
            normals[i / 3] = normal;
        }
    }

    void updateTransform(const Vector3& position, const Vector3& rotation, const Vector3& scale) {
        vertices.resize(originalVertices.size());

        // Convert degrees to radians
        const double DEG_TO_RAD = 3.14159265359 / 180.0;
        Vector3 rotationRad = rotation * DEG_TO_RAD;

        // Create rotation matrices (using radians)
        double rotX[4][4] = {
            {1, 0, 0, 0},
            {0, cos(rotationRad.x), -sin(rotationRad.x), 0},
            {0, sin(rotationRad.x), cos(rotationRad.x), 0},
            {0, 0, 0, 1}
        };

        double rotY[4][4] = {
            {cos(rotationRad.y), 0, sin(rotationRad.y), 0},
            {0, 1, 0, 0},
            {-sin(rotationRad.y), 0, cos(rotationRad.y), 0},
            {0, 0, 0, 1}
        };

        double rotZ[4][4] = {
            {cos(rotationRad.z), -sin(rotationRad.z), 0, 0},
            {sin(rotationRad.z), cos(rotationRad.z), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };

        for (size_t i = 0; i < originalVertices.size(); ++i) {
            // Start with scaled vertex
            Vector3 transformed = originalVertices[i] * scale;

            // Apply rotations in order: Z, X, Y
            transformed = transformed.multiplyMatrix(rotZ);
            transformed = transformed.multiplyMatrix(rotX);
            transformed = transformed.multiplyMatrix(rotY);

            // Finally apply translation
            vertices[i] = transformed + position;
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