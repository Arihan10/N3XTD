// PhysicsSystem.cpp
#include "stdafx.h"
#include "PhysicsSystem.h"
#include "Components.h"
#include <iostream>

void PhysicsSystem::update(float deltaTime) {
    auto entities = world.getEntities();

    // Update velocities and positions
    for (auto& entity : entities) {
        auto rb = entity->getComponent<RigidbodyComponent>();
        auto transform = entity->getComponent<TransformComponent>();

        if (!rb || !transform || rb->isStatic) continue;

        // Apply gravity
        rb->acceleration.y = gravity;

        // Update velocity
        rb->velocity = rb->velocity + rb->acceleration * deltaTime;

        // Update position
        transform->position = transform->position + rb->velocity * deltaTime;

        rb->velocity.print("");
        // std::cout << deltaTime << std::endl;
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

            // Determine collision type and check
            if (col1->type == ColliderComponent::SPHERE && col2->type == ColliderComponent::SPHERE) {
                collision = checkSphereSphereCollision(
                    trans1->position + col1->offset, col1->size.x,
                    trans2->position + col2->offset, col2->size.x,
                    normal, depth);
            }
            else if (col1->type == ColliderComponent::SPHERE && col2->type == ColliderComponent::BOX) {
                collision = checkSphereBoxCollision(
                    trans1->position + col1->offset, col1->size.x,
                    trans2->position + col2->offset, col2->size,
                    normal, depth);
            }
            else if (col1->type == ColliderComponent::BOX && col2->type == ColliderComponent::SPHERE) {
                // Swap order and negate normal if box comes first
                collision = checkSphereBoxCollision(
                    trans2->position + col2->offset, col2->size.x,
                    trans1->position + col1->offset, col1->size,
                    normal, depth);
                normal = normal * -1.0f;
            }
            // Add other collision checks here

            if (collision) {
                // Resolve collision
                if (!rb1->isStatic && !rb2->isStatic) {
                    float totalMass = rb1->mass + rb2->mass;
                    float ratio1 = rb1->mass / totalMass;
                    float ratio2 = rb2->mass / totalMass;

                    // Move objects out of collision
                    trans1->position = trans1->position + normal * depth * ratio2;
                    trans2->position = trans2->position - normal * depth * ratio1;

                    // Calculate impulse
                    Vector3 relativeVel = rb1->velocity - rb2->velocity;
                    float velAlongNormal = relativeVel.dot(normal);

                    float restitution = std::min(rb1->restitution, rb2->restitution);
                    float impulse = -(1 + restitution) * velAlongNormal;
                    impulse /= 1 / rb1->mass + 1 / rb2->mass;

                    Vector3 impulseVec = normal * impulse;
                    rb1->velocity = rb1->velocity + impulseVec / rb1->mass;
                    rb2->velocity = rb2->velocity - impulseVec / rb2->mass;
                }
                else if (!rb1->isStatic) {
                    trans1->position = trans1->position + normal * depth;
                    rb1->velocity = rb1->velocity - normal * 2 * rb1->velocity.dot(normal) * rb1->restitution;
                }
                else if (!rb2->isStatic) {
                    trans2->position = trans2->position - normal * depth;
                    rb2->velocity = rb2->velocity - normal * 2 * rb2->velocity.dot(normal) * rb2->restitution;
                }
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

    normal = delta.normalize();
    depth = minDistance - distance;
    return true;
}

bool PhysicsSystem::checkSphereBoxCollision(
    const Vector3& spherePos, float radius,
    const Vector3& boxPos, const Vector3& boxHalfSize,
    Vector3& normal, float& depth) {

    // Find the closest point on the box to the sphere center
    Vector3 closestPoint;

    // Clamp sphere center to box bounds on each axis
    closestPoint.x = std::max(boxPos.x - boxHalfSize.x,
        std::min(spherePos.x, boxPos.x + boxHalfSize.x));
    closestPoint.y = std::max(boxPos.y - boxHalfSize.y,
        std::min(spherePos.y, boxPos.y + boxHalfSize.y));
    closestPoint.z = std::max(boxPos.z - boxHalfSize.z,
        std::min(spherePos.z, boxPos.z + boxHalfSize.z));

    Vector3 delta = spherePos - closestPoint;
    float distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

    // If distance is greater than radius, no collision
    if (distanceSquared > radius * radius) return false;

    float distance = sqrt(distanceSquared);

    // Avoid division by zero if sphere is exactly on surface
    if (distance == 0.0f) {
        // Sphere center is exactly on box surface - choose arbitrary normal
        normal = Vector3(0, 1, 0);
        depth = radius;
    }
    else {
        normal = delta / distance; // Normalize
        depth = radius - distance;
    }

    return true;
}