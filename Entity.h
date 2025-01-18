// Entity.h
#ifndef ENTITY_H
#define ENTITY_H

#include <memory>
#include <unordered_map>
#include <typeindex>

class Entity {
private:
    std::unordered_map<std::type_index, std::shared_ptr<void>> components;

public:
    template<typename T>
    void addComponent(const T& component) {
        components[std::type_index(typeid(T))] = std::make_shared<T>(component);
    }

    template<typename T>
    T* getComponent() {
        auto it = components.find(std::type_index(typeid(T)));
        if (it != components.end()) {
            return static_cast<T*>(it->second.get());
        }
        return nullptr;
    }

    template<typename T>
    bool hasComponent() {
        return components.find(std::type_index(typeid(T))) != components.end();
    }
};

#endif