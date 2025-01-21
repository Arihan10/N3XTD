#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H
#include "World.h"
#include "Vector3.h"
#include "Entity.h"
#include "Components.h"
#include <functional>

// Callback type for trigger events
using TriggerCallback = std::function<void(Entity*, Entity*)>;

// Structure to represent an oriented box for collision
struct OrientedBox {
    Vector3 center;
    Vector3 halfExtents;
    Vector3 axisX;  // Local right axis after rotation
    Vector3 axisY;  // Local up axis after rotation
    Vector3 axisZ;  // Local forward axis after rotation
    OrientedBox(const Vector3& pos, const Vector3& size, const Vector3& rotation);
    Vector3 getVertex(int i) const;  // Get box vertex in world space
    void getMinMaxOnAxis(const Vector3& axis, float& min, float& max) const;
};

class PhysicsSystem {
private:
    World& world;
    float gravity = -9.81f * 4;
    TriggerCallback triggerCallback;  // Callback for trigger events

    // Helper methods for collision detection
    bool checkSphereSphereCollision(
        const Vector3& pos1, float radius1,
        const Vector3& pos2, float radius2,
        Vector3& normal, float& depth);

    bool checkSphereOBBCollision(
        const Vector3& spherePos, float radius,
        const OrientedBox& box,
        Vector3& normal, float& depth);

    bool checkOBBOBBCollision(
        const OrientedBox& box1,
        const OrientedBox& box2,
        Vector3& normal, float& depth);

    // Separating Axis Test helper
    bool testAxis(
        const Vector3& axis,
        const OrientedBox& box1,
        const OrientedBox& box2,
        float& depth,
        bool& shouldFlip);

    // Collision response helpers
    void handleStaticCollision(
        TransformComponent* transform,
        RigidbodyComponent* rb,
        ColliderComponent* collider,
        const Vector3& normal,
        float depth);

    void resolveCollision(
        TransformComponent* trans1, RigidbodyComponent* rb1, ColliderComponent* col1,
        TransformComponent* trans2, RigidbodyComponent* rb2, ColliderComponent* col2,
        const Vector3& normal, float depth);

public:
    PhysicsSystem(World& w) : world(w) {}

    void setTriggerCallback(TriggerCallback callback) {
        triggerCallback = callback;
    }

    void update(float deltaTime);
    void applyForce(Entity* entity, const Vector3& force, const Vector3& point);
    float getGravity() const { return gravity; }
    void setGravity(float g) { gravity = g; }
    void resetPhysicsState(Entity* entity);
};

#endif // PHYSICS_SYSTEM_H