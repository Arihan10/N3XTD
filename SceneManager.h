// SceneManager.h
#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include <memory>
#include <string>
#include <unordered_map>
#include "Scene.h"
#include "World.h"

class SceneManager {
private:
    std::unordered_map<std::string, std::shared_ptr<Scene>> scenes;
    World& world;

public:
    SceneManager(World& worldRef);

    // Create a new scene
    std::shared_ptr<Scene> createScene(const std::string& name);

    // Get a scene by name
    std::shared_ptr<Scene> getScene(const std::string& name);

    // Remove a scene
    void removeScene(const std::string& name);

    // Clear all scenes
    void clear();
};

#endif // SCENE_MANAGER_H