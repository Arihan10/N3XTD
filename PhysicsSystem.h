// PhysicsSystem.h
#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "World.h"
#include "Vector3.h"

class PhysicsSystem {
private:
    World& world;
    float gravity = -9.81f;

    bool checkSphereSphereCollision(
        const Vector3& pos1, float radius1,
        const Vector3& pos2, float radius2,
        Vector3& normal, float& depth);

    bool checkBoxBoxCollision(
        const Vector3& pos1, const Vector3& size1,
        const Vector3& pos2, const Vector3& size2,
        Vector3& normal, float& depth);

    bool checkSphereBoxCollision(
        const Vector3& spherePos, float radius,
        const Vector3& boxPos, const Vector3& boxSize,
        Vector3& normal, float& depth);

public:
    PhysicsSystem(World& w) : world(w) {}
    void update(float deltaTime);
};

#endif