// PhysicsSystem.cpp
#include "stdafx.h"
#include "PhysicsSystem.h"
#include <algorithm>
#include <cmath>

OrientedBox::OrientedBox(const Vector3& pos, const Vector3& size, const Vector3& rotation) {
    center = pos;
    halfExtents = size;

    // Create rotation matrices
    const double DEG_TO_RAD = 3.14159265359 / 180.0;
    Vector3 rotRad = rotation * DEG_TO_RAD;

    // Initial axes
    axisX = Vector3(1, 0, 0);
    axisY = Vector3(0, 1, 0);
    axisZ = Vector3(0, 0, 1);

    // Rotation matrices
    double rotX[4][4] = {
        {1, 0, 0, 0},
        {0, cos(rotRad.x), -sin(rotRad.x), 0},
        {0, sin(rotRad.x), cos(rotRad.x), 0},
        {0, 0, 0, 1}
    };

    double rotY[4][4] = {
        {cos(rotRad.y), 0, sin(rotRad.y), 0},
        {0, 1, 0, 0},
        {-sin(rotRad.y), 0, cos(rotRad.y), 0},
        {0, 0, 0, 1}
    };

    double rotZ[4][4] = {
        {cos(rotRad.z), -sin(rotRad.z), 0, 0},
        {sin(rotRad.z), cos(rotRad.z), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // Apply rotations in order: Z, X, Y
    axisX = axisX.multiplyMatrix(rotZ).multiplyMatrix(rotX).multiplyMatrix(rotY);
    axisY = axisY.multiplyMatrix(rotZ).multiplyMatrix(rotX).multiplyMatrix(rotY);
    axisZ = axisZ.multiplyMatrix(rotZ).multiplyMatrix(rotX).multiplyMatrix(rotY);
}

Vector3 OrientedBox::getVertex(int i) const {
    Vector3 direction;

    // Convert index to vertex position
    if (i & 1) direction = direction + axisX * halfExtents.x;
    else direction = direction - axisX * halfExtents.x;

    if (i & 2) direction = direction + axisY * halfExtents.y;
    else direction = direction - axisY * halfExtents.y;

    if (i & 4) direction = direction + axisZ * halfExtents.z;
    else direction = direction - axisZ * halfExtents.z;

    return center + direction;
}

void OrientedBox::getMinMaxOnAxis(const Vector3& axis, float& min, float& max) const {
    // First vertex as initial values
    Vector3 vertex = getVertex(0);
    float dot = vertex.dot(axis);
    min = max = dot;

    // Check remaining vertices
    for (int i = 1; i < 8; i++) {
        vertex = getVertex(i);
        dot = vertex.dot(axis);

        if (dot < min) min = dot;
        if (dot > max) max = dot;
    }
}

bool PhysicsSystem::checkSphereOBBCollision(
    const Vector3& spherePos,
    float radius,
    const OrientedBox& box,
    Vector3& normal,
    float& depth) {

    // Get vector from box center to sphere center
    Vector3 localPos = spherePos - box.center;

    // Find closest point on box
    float dotX = localPos.dot(box.axisX);
    float dotY = localPos.dot(box.axisY);
    float dotZ = localPos.dot(box.axisZ);

    // Clamp to box bounds
    float clampedX = std::max((float)-box.halfExtents.x, std::min(dotX, (float)box.halfExtents.x));
    float clampedY = std::max((float)-box.halfExtents.y, std::min(dotY, (float)box.halfExtents.y));
    float clampedZ = std::max((float)-box.halfExtents.z, std::min(dotZ, (float)box.halfExtents.z));

    // Construct closest point using clamped distances along each axis
    Vector3 closest = box.center;
    closest = closest + box.axisX * clampedX;
    closest = closest + box.axisY * clampedY;
    closest = closest + box.axisZ * clampedZ;

    // Check if sphere intersects with closest point
    Vector3 delta = spherePos - closest;
    float distanceSquared = delta.dot(delta);

    if (distanceSquared > radius * radius) {
        return false;
    }

    float distance = sqrt(distanceSquared);

    if (distance < 0.0001f) {
        normal = box.axisY; // Default to up if sphere is exactly on surface
        depth = radius;
    }
    else {
        normal = delta / distance;
        depth = radius - distance;
    }

    return true;
}

bool PhysicsSystem::testAxis(
    const Vector3& axis,
    const OrientedBox& box1,
    const OrientedBox& box2,
    float& depth,
    bool& shouldFlip) {

    float min1, max1, min2, max2;
    box1.getMinMaxOnAxis(axis, min1, max1);
    box2.getMinMaxOnAxis(axis, min2, max2);

    if (max1 < min2 || max2 < min1) {
        return false;
    }

    float d1 = max2 - min1;
    float d2 = max1 - min2;

    depth = std::min(d1, d2);
    shouldFlip = d1 < d2;

    return true;
}

bool PhysicsSystem::checkOBBOBBCollision(
    const OrientedBox& box1,
    const OrientedBox& box2,
    Vector3& normal,
    float& depth) {

    float minDepth = FLT_MAX;
    Vector3 bestAxis;
    bool shouldFlip = false;

    // Test box1's axes
    Vector3 axes[3] = { box1.axisX, box1.axisY, box1.axisZ };
    for (int i = 0; i < 3; i++) {
        float axisDepth;
        bool flip;
        if (!testAxis(axes[i], box1, box2, axisDepth, flip)) {
            return false;
        }
        if (axisDepth < minDepth) {
            minDepth = axisDepth;
            bestAxis = axes[i];
            shouldFlip = flip;
        }
    }

    // Test box2's axes
    axes[0] = box2.axisX;
    axes[1] = box2.axisY;
    axes[2] = box2.axisZ;

    for (int i = 0; i < 3; i++) {
        float axisDepth;
        bool flip;
        if (!testAxis(axes[i], box1, box2, axisDepth, flip)) {
            return false;
        }
        if (axisDepth < minDepth) {
            minDepth = axisDepth;
            bestAxis = axes[i];
            shouldFlip = flip;
        }
    }

    // Test cross products of edges
    Vector3 box1Axes[3] = { box1.axisX, box1.axisY, box1.axisZ };
    Vector3 box2Axes[3] = { box2.axisX, box2.axisY, box2.axisZ };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vector3 axis = box1Axes[i].cross(box2Axes[j]);
            float length = axis.length();
            if (length > 0.001f) {
                axis = axis * (1.0f / length);  // Normalize
                float axisDepth;
                bool flip;
                if (!testAxis(axis, box1, box2, axisDepth, flip)) {
                    return false;
                }
                if (axisDepth < minDepth) {
                    minDepth = axisDepth;
                    bestAxis = axis;
                    shouldFlip = flip;
                }
            }
        }
    }

    depth = minDepth;
    normal = shouldFlip ? bestAxis * -1.0f : bestAxis;
    return true;
}

void PhysicsSystem::resolveCollision(
    TransformComponent* trans1, RigidbodyComponent* rb1,
    TransformComponent* trans2, RigidbodyComponent* rb2,
    const Vector3& normal, float depth) {

    if (!rb1->isStatic && !rb2->isStatic) {
        float totalMass = rb1->mass + rb2->mass;
        float ratio1 = rb1->mass / totalMass;
        float ratio2 = rb2->mass / totalMass;

        // Move objects out of collision
        trans1->position = trans1->position + normal * depth * ratio2;
        trans2->position = trans2->position - normal * depth * ratio1;

        // Calculate relative velocity
        Vector3 relativeVel = rb1->velocity - rb2->velocity;
        float velAlongNormal = relativeVel.dot(normal);

        // Only resolve if objects are moving towards each other
        if (velAlongNormal < 0) {
            float restitution = std::min(rb1->restitution, rb2->restitution);

            // Reduce restitution at low speeds
            float speedThreshold = 2.0f;
            if (std::abs(velAlongNormal) < speedThreshold) {
                restitution *= std::abs(velAlongNormal) / speedThreshold;
            }

            float impulse = -(1 + restitution) * velAlongNormal;
            impulse /= 1 / rb1->mass + 1 / rb2->mass;

            Vector3 impulseVec = normal * impulse;
            rb1->velocity = rb1->velocity + impulseVec / rb1->mass;
            rb2->velocity = rb2->velocity - impulseVec / rb2->mass;

            // Calculate and apply friction
            Vector3 tangent = relativeVel - normal * velAlongNormal;
            float tangentLength = tangent.length();

            if (tangentLength > 0.0001f) {
                tangent = tangent * (1.0f / tangentLength);
                float frictionImpulse = -relativeVel.dot(tangent);
                frictionImpulse /= 1 / rb1->mass + 1 / rb2->mass;

                float combinedFriction = (rb1->friction + rb2->friction) * 0.5f;
                float maxFriction = std::abs(impulse) * combinedFriction;
                frictionImpulse = std::max(-maxFriction, std::min(frictionImpulse, maxFriction));

                Vector3 frictionVec = tangent * frictionImpulse;
                rb1->velocity = rb1->velocity + frictionVec / rb1->mass;
                rb2->velocity = rb2->velocity - frictionVec / rb2->mass;
            }
        }
    }
    else if (!rb1->isStatic) {
        handleStaticCollision(trans1, rb1, normal, depth);
    }
    else if (!rb2->isStatic) {
        handleStaticCollision(trans2, rb2, normal * -1.0f, depth);
    }
}

void PhysicsSystem::handleStaticCollision(
    TransformComponent* transform,
    RigidbodyComponent* rb,
    const Vector3& normal,
    float depth) {

    // Move object out of collision
    transform->position = transform->position + normal * depth;

    // Calculate velocity reflection
    float velDotNormal = rb->velocity.dot(normal);

    // Only reflect if moving towards surface
    if (velDotNormal < 0) {
        // Reduce restitution at low speeds to encourage rolling
        float speedThreshold = 2.0f;
        float restitution = rb->restitution;
        if (std::abs(velDotNormal) < speedThreshold) {
            restitution *= std::abs(velDotNormal) / speedThreshold;
        }

        // Calculate reflection vector
        Vector3 normalVel = normal * velDotNormal;
        Vector3 tangentVel = rb->velocity - normalVel;

        // Apply restitution to normal component
        Vector3 newVel = tangentVel - normalVel * restitution;

        // Apply friction to tangential component
        float tangentSpeed = tangentVel.length();
        if (tangentSpeed > 0.0001f) {
            float frictionDecel = rb->friction * std::abs(gravity);
            float newSpeed = std::max(0.0f, tangentSpeed - frictionDecel);

            if (tangentSpeed > 0) {
                newVel = newVel * (newSpeed / tangentSpeed);
            }
        }

        rb->velocity = newVel;

        // Update angular velocity for rolling on surface
        if (tangentSpeed > 0.0001f) {
            Vector3 rotationAxis = normal.cross(tangentVel).normalize();
            float rotationSpeed = tangentSpeed / (2.0f * 3.14159f);  // Approximate radius as 1
            rb->angularVelocity = rotationAxis * rotationSpeed;
        }
    }
}

void PhysicsSystem::applyForce(Entity* entity, const Vector3& force, const Vector3& point) {
    auto rb = entity->getComponent<RigidbodyComponent>();
    auto transform = entity->getComponent<TransformComponent>();

    if (!rb || !transform || rb->isStatic) return;

    // Apply linear force
    Vector3 acceleration = force * (1.0f / rb->mass);
    rb->velocity = rb->velocity + acceleration;

    // Calculate and apply torque
    Vector3 torque = (point - transform->position).cross(force);

    // Convert torque to angular acceleration (simplified - assuming uniform mass distribution)
    float momentOfInertia = rb->mass * 0.4f; // Simplified moment of inertia for a sphere
    Vector3 angularAccel = torque * (1.0f / momentOfInertia);

    rb->angularVelocity = rb->angularVelocity + angularAccel;
}

void PhysicsSystem::update(float deltaTime) {
    auto entities = world.getEntities();

    // Update velocities and positions
    for (auto& entity : entities) {
        auto rb = entity->getComponent<RigidbodyComponent>();
        auto transform = entity->getComponent<TransformComponent>();

        if (!rb || !transform || rb->isStatic) continue;

        // Apply gravity
        rb->acceleration.y = gravity;

        // Apply air resistance
        Vector3 airResistance = rb->velocity * -rb->airResistance * rb->velocity.length();
        rb->acceleration = rb->acceleration + airResistance;

        // Update angular velocity (reduced by air resistance)
        rb->angularVelocity = rb->angularVelocity * (1.0f - rb->airResistance * deltaTime);

        // Update rotation based on angular velocity
        transform->rotation = transform->rotation + rb->angularVelocity * deltaTime;

        // Update linear velocity and position
        rb->velocity = rb->velocity + rb->acceleration * deltaTime;
        transform->position = transform->position + rb->velocity * deltaTime;
    }

    // Check for collisions
    for (size_t i = 0; i < entities.size(); i++) {
        for (size_t j = i + 1; j < entities.size(); j++) {
            auto entity1 = entities[i];
            auto entity2 = entities[j];

            auto rb1 = entity1->getComponent<RigidbodyComponent>();
            auto rb2 = entity2->getComponent<RigidbodyComponent>();
            auto col1 = entity1->getComponent<ColliderComponent>();
            auto col2 = entity2->getComponent<ColliderComponent>();
            auto trans1 = entity1->getComponent<TransformComponent>();
            auto trans2 = entity2->getComponent<TransformComponent>();

            if (!rb1 || !rb2 || !col1 || !col2 || !trans1 || !trans2) continue;

            Vector3 normal;
            float depth;
            bool collision = false;

            // Create oriented boxes if needed
            OrientedBox box1(trans1->position + col1->offset, col1->size, trans1->rotation);
            OrientedBox box2(trans2->position + col2->offset, col2->size, trans2->rotation);

            if (col1->type == ColliderComponent::SPHERE && col2->type == ColliderComponent::SPHERE) {
                collision = checkSphereSphereCollision(
                    trans1->position + col1->offset, col1->size.x,
                    trans2->position + col2->offset, col2->size.x,
                    normal, depth);
            }
            else if (col1->type == ColliderComponent::SPHERE && col2->type == ColliderComponent::BOX) {
                collision = checkSphereOBBCollision(
                    trans1->position + col1->offset, col1->size.x,
                    box2, normal, depth);
            }
            else if (col1->type == ColliderComponent::BOX && col2->type == ColliderComponent::SPHERE) {
                collision = checkSphereOBBCollision(
                    trans2->position + col2->offset, col2->size.x,
                    box1, normal, depth);
                normal = normal * -1.0f;
            }
            else if (col1->type == ColliderComponent::BOX && col2->type == ColliderComponent::BOX) {
                collision = checkOBBOBBCollision(box1, box2, normal, depth);
            }

            if (collision) {
                resolveCollision(trans1, rb1, trans2, rb2, normal, depth);
            }
        }
    }
}

bool PhysicsSystem::checkSphereSphereCollision(
    const Vector3& pos1, float radius1,
    const Vector3& pos2, float radius2,
    Vector3& normal, float& depth) {

    Vector3 delta = pos2 - pos1;
    float distance = delta.length();
    float minDistance = radius1 + radius2;

    if (distance >= minDistance) return false;

    normal = delta * (1.0f / distance); // Normalize
    depth = minDistance - distance;
    return true;
}