// World.h
#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include "Entity.h"

class World {
private:
    std::vector<std::shared_ptr<Entity>> entities;

public:
    std::shared_ptr<Entity> createEntity() {
        auto entity = std::make_shared<Entity>();
        entities.push_back(entity);
        return entity;
    }

    const std::vector<std::shared_ptr<Entity>>& getEntities() const {
        return entities;
    }

    void clear() {
        entities.clear();
    }
};

#endif