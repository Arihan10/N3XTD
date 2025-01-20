// Scene.cpp
#include <stdafx.h>
#include "Scene.h"

Scene::Scene(const std::string& sceneName, World& worldRef)
    : name(sceneName)
    , world(worldRef)
    , isEnabled(true)
{
}

void Scene::addEntity(std::shared_ptr<Entity> entity) {
    sceneEntities.push_back(entity);
}

std::shared_ptr<Entity> Scene::createEntity() {
    auto entity = world.createEntity();
    sceneEntities.push_back(entity);
    return entity;
}

void Scene::setEnabled(bool enabled) {
    isEnabled = enabled;
    for (auto& entity : sceneEntities) {
        entity->setEnabled(enabled);
    }
}

bool Scene::getEnabled() const {
    return isEnabled;
}

const std::vector<std::shared_ptr<Entity>>& Scene::getEntities() const {
    return sceneEntities;
}

const std::string& Scene::getName() const {
    return name;
}

void Scene::removeEntity(std::shared_ptr<Entity> entity) {
    auto it = std::find(sceneEntities.begin(), sceneEntities.end(), entity);
    if (it != sceneEntities.end()) {
        sceneEntities.erase(it);
    }
}

void Scene::clear() {
    sceneEntities.clear();
}