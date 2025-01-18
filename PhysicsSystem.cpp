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

        // Apply air resistance
        Vector3 airResistance = rb->velocity * -rb->airResistance * rb->velocity.length();
        rb->acceleration = rb->acceleration + airResistance;

        // Update angular velocity (reduced by air resistance)
        rb->angularVelocity = rb->angularVelocity * (1.0f - rb->airResistance * deltaTime);

        // Update linear velocity with acceleration
        rb->velocity = rb->velocity + rb->acceleration * deltaTime;

        // Update position
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

            // Collision detection (existing code)
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
                collision = checkSphereBoxCollision(
                    trans2->position + col2->offset, col2->size.x,
                    trans1->position + col1->offset, col1->size,
                    normal, depth);
                normal = normal * -1.0f;
            }

            if (collision) {
                // Resolve collision with rolling friction
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
                        // Calculate restitution
                        float restitution = std::min(rb1->restitution, rb2->restitution);

                        // Reduce restitution based on impact velocity (softer bounces at low speeds)
                        float speedThreshold = 2.0f; // Adjust this value to control when rolling starts
                        if (std::abs(velAlongNormal) < speedThreshold) {
                            restitution *= std::abs(velAlongNormal) / speedThreshold;
                        }

                        // Calculate impulse scalar
                        float impulse = -(1 + restitution) * velAlongNormal;
                        impulse /= 1 / rb1->mass + 1 / rb2->mass;

                        // Apply impulse along normal
                        Vector3 impulseVec = normal * impulse;
                        rb1->velocity = rb1->velocity + impulseVec / rb1->mass;
                        rb2->velocity = rb2->velocity - impulseVec / rb2->mass;

                        // Calculate and apply friction
                        Vector3 tangent = relativeVel - normal * velAlongNormal;
                        float tangentLength = tangent.length();

                        if (tangentLength > 0.0001f) {
                            tangent = tangent / tangentLength; // Normalize
                            float frictionImpulse = -relativeVel.dot(tangent);
                            frictionImpulse /= 1 / rb1->mass + 1 / rb2->mass;

                            // Combine friction coefficients
                            float combinedFriction = (rb1->friction + rb2->friction) * 0.5f;

                            // Clamp friction impulse
                            float maxFriction = std::abs(impulse) * combinedFriction;
                            frictionImpulse = std::max(-maxFriction, std::min(frictionImpulse, maxFriction));

                            // Apply friction impulse
                            Vector3 frictionVec = tangent * frictionImpulse;
                            rb1->velocity = rb1->velocity + frictionVec / rb1->mass;
                            rb2->velocity = rb2->velocity - frictionVec / rb2->mass;

                            // Update angular velocity based on friction (for spheres)
                            if (col1->type == ColliderComponent::SPHERE) {
                                float radius = col1->size.x;
                                rb1->angularVelocity = rb1->angularVelocity +
                                    (frictionVec.cross(normal) / (radius * rb1->mass));
                            }
                            if (col2->type == ColliderComponent::SPHERE) {
                                float radius = col2->size.x;
                                rb2->angularVelocity = rb2->angularVelocity -
                                    (frictionVec.cross(normal) / (radius * rb2->mass));
                            }
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
        }
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

        // Calculate reflection vector with restitution
        Vector3 reflection = rb->velocity - normal * (1 + restitution) * velDotNormal;

        // Apply friction to the parallel component
        Vector3 parallel = reflection - normal * reflection.dot(normal);
        float parallelSpeed = parallel.length();

        if (parallelSpeed > 0.0001f) {
            // Calculate friction deceleration
            float frictionDecel = rb->friction * std::abs(gravity);
            float newSpeed = std::max(0.0f, parallelSpeed - frictionDecel);

            if (parallelSpeed > 0) {
                parallel = parallel * (newSpeed / parallelSpeed);
            }

            // Combine normal and friction-affected parallel components
            rb->velocity = normal * reflection.dot(normal) + parallel;
        }
        else {
            rb->velocity = reflection;
        }
    }
}

void PhysicsSystem::applyForce(Entity* entity, const Vector3& force, const Vector3& point) {
    auto rb = entity->getComponent<RigidbodyComponent>();
    auto transform = entity->getComponent<TransformComponent>();

    if (!rb || !transform || rb->isStatic) return;

    // Apply linear force
    Vector3 acceleration = force / rb->mass;
    rb->velocity = rb->velocity + acceleration;

    // Calculate torque and angular acceleration
    Vector3 torque = (point - transform->position).cross(force);
    rb->angularVelocity = rb->angularVelocity + torque / rb->mass;
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