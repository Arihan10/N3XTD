// SceneManager.cpp
#include <stdafx.h>
#include "SceneManager.h"

SceneManager::SceneManager(World& worldRef)
    : world(worldRef)
{
}

std::shared_ptr<Scene> SceneManager::createScene(const std::string& name) {
    auto scene = std::make_shared<Scene>(name, world);
    scenes[name] = scene;
    return scene;
}

std::shared_ptr<Scene> SceneManager::getScene(const std::string& name) {
    auto it = scenes.find(name);
    if (it != scenes.end()) {
        return it->second;
    }
    return nullptr;
}

void SceneManager::removeScene(const std::string& name) {
    scenes.erase(name);
}

void SceneManager::clear() {
    scenes.clear();
}