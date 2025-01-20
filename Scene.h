// Scene.h
#ifndef SCENE_H
#define SCENE_H

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "Entity.h"
#include "World.h"

class Scene {
private:
    std::string name;
    std::vector<std::shared_ptr<Entity>> sceneEntities;
    World& world;
    bool isEnabled;

public:
    Scene(const std::string& sceneName, World& worldRef);

    // Add an entity to the scene
    void addEntity(std::shared_ptr<Entity> entity);

    // Create and add a new entity
    std::shared_ptr<Entity> createEntity();

    // Enable/disable all entities in the scene
    void setEnabled(bool enabled);

    // Get scene enabled state
    bool getEnabled() const;

    // Get all entities in the scene
    const std::vector<std::shared_ptr<Entity>>& getEntities() const;

    // Get scene name
    const std::string& getName() const;

    // Remove an entity from the scene
    void removeEntity(std::shared_ptr<Entity> entity);

    // Clear all entities from the scene
    void clear();
};

#endif // SCENE_H