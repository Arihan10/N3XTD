#include "stdafx.h"
#include <windows.h>
#include <math.h>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <string>
#include <functional>
#include <algorithm>
#include "app/app.h"
#include "Vector3.h"
#include "Components.h"
#include "World.h"
#include "MeshLoader.h"
#include "PhysicsSystem.h"
#include "KeyTracker.h"
#include <Scene.h>
#include <SceneManager.h>

#include <stdio.h>
#include <fcntl.h>

void createConsoleWindow() {
    // Allocate a console
    AllocConsole();

    // Redirect standard output streams to the console
    FILE* fileStream;
    freopen_s(&fileStream, "CONOUT$", "w", stdout); // Redirect stdout
    freopen_s(&fileStream, "CONOUT$", "w", stderr); // Redirect stderr
    freopen_s(&fileStream, "CONIN$", "r", stdin);  // Redirect stdin

    // Set the console title
    SetConsoleTitle(L"Debug Console");

    std::cout << "Console window successfully created!" << std::endl;
}

struct ScanlineTriangle {
    Vector3 points[3]; 
    double depth; 
    float r, g, b; 
    int zIndex; 
};

/*struct CompareTriangles {
    bool operator()(const ScanlineTriangle& a, const ScanlineTriangle& b) {
        return a.depth < b.depth; // Reverse order for back-to-front
    }
};*/

struct CompareTriangles {
    bool operator()(const ScanlineTriangle& a, const ScanlineTriangle& b) {
        if (a.zIndex != b.zIndex) {
            return a.zIndex > b.zIndex;  // Higher z-index renders later - on top
        }
        // If z-indices are equal, fall back to depth comparison
        return a.depth < b.depth;
    }
};

extern int WINDOW_WIDTH;
extern int WINDOW_HEIGHT;

// Camera and lighting
Vector3 camPos(0.0, 0.0, 0.0);
Vector3 lightDir(0.0, -1.0, -0.5);

// Camera rotation angles
double angleY = 0.0;
double angleX = 0.0;
double FoV = 25.0;
double zFar = 100;
double zNear = 0.1;
double moveSpeed = 20.0;
double mouseSensitivityX = 0.004;
double mouseSensitivityY = 0.003;
double deltaX = 0.0;
double deltaY = 0.0;
float lastMouseX = 0;
float lastMouseY = 0;

// Single press keys
KeyTracker keyTracker; 

// World instance
World world;

// Physics system
PhysicsSystem physicsSystem(world); 

// Create a scene manager
SceneManager sceneManager(world); 

void loadMesh(std::shared_ptr<Entity> entity, const std::string& filename, int zIndex = 0) {
    // MeshComponent mesh; 
    MeshComponent mesh(zIndex); // I love the stack
    MeshLoader::LoadFromFile(filename, mesh.originalVertices, mesh.triangles); 

    auto transform = entity->getComponent<TransformComponent>();
    if (transform) {
        mesh.updateTransform(transform->position, transform->rotation, transform->scale); 
    }

    entity->addComponent(mesh);
}

void createStaticObject(std::shared_ptr<Entity> entity, const std::string& meshFile, const std::string& name, const Vector3& position, const Vector3& rotation, const Vector3& scale, const Vector3& color, Scene* scene = nullptr, int zIndex = 0) {
    entity = world.createEntity(); 
    
    entity->addComponent(TransformComponent(position, rotation, scale));
    entity->addComponent(NameComponent(name));
    entity->addComponent(ColorComponent(color.x, color.y, color.z));

    loadMesh(entity, meshFile); 

    if (scene) {
        scene->addEntity(entity);
    }
}

void createPhysicsObject(std::shared_ptr<Entity> entity, const std::string& meshFile, const std::string& name, const Vector3& position, const Vector3& rotation, const Vector3& scale, const Vector3& color, float mass, float restitution, float friction, float angularDamping, bool isStatic, Scene* scene = nullptr, int zIndex = 0) {
    entity = world.createEntity(); 
    
    entity->addComponent(TransformComponent(position, rotation, scale));
    entity->addComponent(NameComponent(name));
    entity->addComponent(ColorComponent(color.x, color.y, color.z));

    entity->addComponent(RigidbodyComponent(mass, restitution, friction, angularDamping, isStatic));
    entity->addComponent(ColliderComponent(ColliderComponent::BOX, scale)); 

    loadMesh(entity, meshFile); 

    if (scene) {
        scene->addEntity(entity); 
    }
}

Scene* scenes[2] = { nullptr, nullptr }; 

CSimpleSprite* bgSprite; 

// Global game object declarations
std::shared_ptr<Entity> ball; 
std::shared_ptr<Entity> arrows; 

// Level 0
std::shared_ptr<Entity> suzanne;
std::shared_ptr<Entity> fireAxe;
std::shared_ptr<Entity> ak; 
std::shared_ptr<Entity> ground;
std::shared_ptr<Entity> ground2;
std::shared_ptr<Entity> ground3;
std::shared_ptr<Entity> box1;
std::shared_ptr<Entity> box2;

// Level 1
std::shared_ptr<Entity> box3; 
std::shared_ptr<Entity> guard1; 
std::shared_ptr<Entity> guard2; 
std::shared_ptr<Entity> guard3; 

void CreateLevel0() {
    // Create Suzanne
    createStaticObject(suzanne, "Suzanne.obj", "Suzanne",
        Vector3(0, 0, 10), Vector3(0, 0, 0), Vector3(3, 3, 3),
        Vector3(1.0f, 1.0f, 0.0f), scenes[0]); 

    // Create Fire Axe
    createStaticObject(fireAxe, "Fire_Axe_Test_3.obj", "Fire_Axe",
        Vector3(7, 0, -9), Vector3(0, 0, 0), Vector3(1, 1, 1),
        Vector3(1.0, 0.0, 0.0), scenes[0]); 

    // Create AK
    createStaticObject(ak, "AK_1.obj", "AK_1",
        Vector3(3, 0, -7), Vector3(0, 0, 0), Vector3(3, 3, 3),
        Vector3(0.0, 0.0, 1.0), scenes[0]); 

    // Create ground
    createPhysicsObject(ground, "Cube.obj", "Ground",
        Vector3(0, -25, 0), Vector3(40, 0, 0), Vector3(15, 0.1, 15),
        Vector3(0.0f, 1.0f, 0.0f),
        1.0f, 0.5f, 0.02f, 0.001f, true, scenes[0]);

    // Create ground2
    createPhysicsObject(ground2, "Cube.obj", "Ground",
        Vector3(0, -30, 25), Vector3(-70, 0, 0), Vector3(15, 0.1, 15),
        Vector3(0.0f, 1.0f, 0.0f),
        1.0f, 0.5f, 0.002f, 0.001f, true, scenes[0]);

    // Create ground3
    createPhysicsObject(ground3, "Cube_Top_Sub.obj", "Ground",
        Vector3(0, -50, 0), Vector3(0, 0, 0), Vector3(100, 0.1, 100),
        Vector3(0.3f, 0.5f, 1.0f),
        1.0f, 0.5f, 0.002f, 0.001f, true, scenes[0]);

    // Create box1
    createPhysicsObject(box1, "Cube.obj", "Box1",
        Vector3(0, -45, 90), Vector3(0, 20, 0), Vector3(5, 5, 5),
        Vector3(0.5, 0.5, 0.5),
        1.0f, 0.5f, 0, 0, true, scenes[0]);

    // Create box2
    createPhysicsObject(box2, "Cube.obj", "Box2",
        Vector3(0, -53.5, 50), Vector3(-20, 0, 0), Vector3(5, 5, 5),
        Vector3(0.5, 0.5, 0.5),
        1.0f, 0.5f, 0, 0, true, scenes[0]); 
}

void CreateLevel1() {
    // Ground
    createPhysicsObject(box3, "Cube.obj", "Box3",
        Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(14, 0.1, 40),
        Vector3(0, 1, 0),
        1.0f, 0.5f, 0, 0, true, scenes[1]);

    // Left guard
    createPhysicsObject(guard1, "Cube.obj", "Guard1",
        Vector3(-15, 0, 0), Vector3(0, 0, 0), Vector3(1, 3, 40),
        Vector3(0.5, 0.5, 0.5),
        1.0f, 0.5f, 0, 0, true, scenes[1]);

    // Right guard
    createPhysicsObject(guard2, "Cube.obj", "Guard2",
        Vector3(15, 0, 0), Vector3(0, 0, 0), Vector3(1, 3, 40),
        Vector3(0.5, 0.5, 0.5),
        1.0f, 0.5f, 0, 0, true, scenes[1]);

    // Front guard
    createPhysicsObject(guard3, "Cube.obj", "Guard3",
        Vector3(0, 0, 40), Vector3(0, 0, 0), Vector3(14, 3, 1),
        Vector3(0.5, 0.5, 0.5),
        1.0f, 0.5f, 0, 0, true, scenes[1]);
}

// Golf specific declarations
Vector3 ballPositions[2] = { Vector3(0, 25, 8), Vector3(0, 25, -36) }; 

int currentLevel = 0; 

float swingTime = -1, fullSwingTime = 5.0f, fullSwingForce = 150; 

void LoadScene(int sceneID) {
    size_t length = sizeof(scenes) / sizeof(Scene*);
    for (int i = 0; i < length; ++i) {
        if (i != sceneID) scenes[i]->setEnabled(false);
        else scenes[i]->setEnabled(true);
    }

    ball->getComponent<TransformComponent>()->position = ballPositions[sceneID]; 
    physicsSystem.resetPhysicsState(ball.get()); 

    currentLevel = sceneID; 
}

void Init() {
    createConsoleWindow(); 
    std::cout << "Initialization complete!" << std::endl; 

    bgSprite = App::CreateSprite("bg_1.png", 1, 1);
    bgSprite->SetPosition(WINDOW_WIDTH/2, WINDOW_HEIGHT/2); 
    bgSprite->SetScale(1); 

    ball = world.createEntity();
    ball->addComponent(TransformComponent(ballPositions[currentLevel], Vector3(0, 0, 0), Vector3(1.5, 1.5, 1.5))); 
    ball->addComponent(NameComponent("Ball")); 
    ball->addComponent(ColorComponent(1.0f, 1.0f, 1.0f)); 
    ball->addComponent(RigidbodyComponent(1.0f, 0.6f, 0.002f, 0.00001f)); // Mass 1, high bounce
    ball->addComponent(ColliderComponent(ColliderComponent::SPHERE, Vector3(1.5, 1.5, 1.5))); 
    loadMesh(ball, "Sphere_Ico.obj"); 

    arrows = world.createEntity(); 
    arrows->addComponent(TransformComponent(Vector3(0, 2, -36), Vector3(0, 1, 0), Vector3(1, 1, 1)));
    arrows->addComponent(NameComponent("Ball")); 
    arrows->addComponent(ColorComponent(1.0f, 1.0f, 1.0f)); 
    loadMesh(arrows, "Arrows_1.obj"); 
    arrows->setEnabled(false); 

    scenes[0] = new Scene("Demo", world); 
    scenes[1] = new Scene("Level1", world); 

    CreateLevel0(); 
    CreateLevel1(); 

    LoadScene(1); 
}

void Update(const float deltaTime) {
    float _deltaTime = deltaTime / 1000.0; 

    bool movingCamera = App::IsKeyPressed(VK_SHIFT); 

    if (App::IsKeyPressed(VK_ESCAPE)) {
        glutLeaveMainLoop();
    }

    // Input handling
    if (movingCamera) {
        float mouseX, mouseY;
        App::GetMousePos(mouseX, mouseY);

        deltaX = (mouseX - lastMouseX) * WINDOW_WIDTH / 2;
        deltaY = (mouseY - lastMouseY) * WINDOW_HEIGHT / 2;

        lastMouseX = mouseX;
        lastMouseY = mouseY;

        angleY -= deltaX * _deltaTime * mouseSensitivityX;
        angleX += deltaY * _deltaTime * mouseSensitivityY;

        if (App::IsKeyPressed('E')) camPos.y += moveSpeed * _deltaTime; // up
        if (App::IsKeyPressed('Q')) camPos.y -= moveSpeed * _deltaTime; // down
        if (App::IsKeyPressed('W')) {
            camPos.x += sin(-angleY) * moveSpeed * _deltaTime;
            camPos.z += cos(-angleY) * moveSpeed * _deltaTime;
        }
        if (App::IsKeyPressed('S')) {
            camPos.x -= sin(-angleY) * moveSpeed * _deltaTime;
            camPos.z -= cos(-angleY) * moveSpeed * _deltaTime;
        }
        if (App::IsKeyPressed('A')) {
            camPos.x -= sin(-angleY + 3.14 / 2) * moveSpeed * _deltaTime;
            camPos.z -= cos(-angleY + 3.14 / 2) * moveSpeed * _deltaTime;
        }
        if (App::IsKeyPressed('D')) {
            camPos.x += sin(-angleY + 3.14 / 2) * moveSpeed * _deltaTime;
            camPos.z += cos(-angleY + 3.14 / 2) * moveSpeed * _deltaTime;
        }
    }
    else {
        deltaX = 0;
        deltaY = 0;
        App::GetMousePos(lastMouseX, lastMouseY);
    }

    // Update mesh transforms
    for (const auto& entity : world.getEntities()) {
        auto transform = entity->getComponent<TransformComponent>(); 
        auto mesh = entity->getComponent<MeshComponent>();

        if (transform && mesh) {
            mesh->updateTransform(transform->position, transform->rotation, transform->scale); 
        }
    }

    if (App::IsKeyPressed('F')) {
        Vector3 hitPoint = ball->getComponent<TransformComponent>()->position + Vector3(0, -0.2, 0); // Slightly below center
        // Vector3 clubForce = Vector3(forwardDir.x * power, upAngle * power, forwardDir.z * power); 
        Vector3 clubForce = Vector3(2, 2, 0);
        physicsSystem.applyForce(ball.get(), clubForce, hitPoint);
    } else if (App::IsKeyPressed('T')) {
        // ball->getComponent<TransformComponent>()->position = Vector3(0, 25, 8); 
        // ball->getComponent<RigidbodyComponent>()->velocity = Vector3(0, 0, 0); 
        LoadScene(currentLevel); 
    }

    float force = 50;
    if (keyTracker.IsKeyDown(VK_LEFT)) physicsSystem.applyForce(ball.get(), Vector3(-force, 0, 0), ball->getComponent<TransformComponent>()->position); 
    if (keyTracker.IsKeyDown(VK_RIGHT)) physicsSystem.applyForce(ball.get(), Vector3(force, 0, 0), ball->getComponent<TransformComponent>()->position); 
    if (keyTracker.IsKeyDown(VK_UP)) physicsSystem.applyForce(ball.get(), Vector3(0, 0, force), ball->getComponent<TransformComponent>()->position); 
    if (keyTracker.IsKeyDown(VK_DOWN)) physicsSystem.applyForce(ball.get(), Vector3(0, 0, -force), ball->getComponent<TransformComponent>()->position); 
    /*if (keyTracker.IsKeyDown(VK_SPACE)) physicsSystem.applyForce(ball.get(), Vector3(0, force, 0), ball->getComponent<TransformComponent>()->position);
    if (keyTracker.IsKeyDown('C')) LoadScene(currentLevel + 1);*/

    float swingHandle = (::std::min<float>(swingTime, fullSwingTime)) / fullSwingTime;

    if (keyTracker.IsKeyDown(VK_SPACE)) {
        // swinging = true; 
        ball->getComponent<MeshComponent>()->zIndex = 1; 
        arrows->getComponent<MeshComponent>()->zIndex = 1; 
        arrows->getComponent<TransformComponent>()->position = ball->getComponent<TransformComponent>()->position; 
        arrows->getComponent<TransformComponent>()->scale = Vector3(0, 0, 0); 
        arrows->setEnabled(true); 

        swingTime = 0; 
    } else if (keyTracker.IsKeyUp(VK_SPACE)) {
        // swinging = false; 
        ball->getComponent<MeshComponent>()->zIndex = 0; 
        arrows->getComponent<MeshComponent>()->zIndex = 0; 
        arrows->setEnabled(false); 

        float swingForce = swingHandle * fullSwingForce; 
        physicsSystem.applyForce(ball.get(), Vector3(0, 0, swingForce), ball->getComponent<TransformComponent>()->position); 

        swingTime = -1; 
    }

    if (swingTime >= 0) {
        swingTime += fullSwingTime * _deltaTime; 

        arrows->getComponent<TransformComponent>()->scale = Vector3(1, 1, 1) * swingHandle; 

        std::cout << swingTime << std::endl; 
    }

    physicsSystem.update(_deltaTime); 
    keyTracker.Update(); 
}

void Render() {
    bgSprite->Draw(); 

    // Rotation matrices
    double rotationX[4][4] = {
        {1, 0, 0, 0},
        {0, cos(angleX), -sin(angleX), 0},
        {0, sin(angleX), cos(angleX), 0},
        {0, 0, 0, 1}
    };

    double rotationY[4][4] = {
        {cos(angleY), 0, sin(angleY), 0},
        {0, 1, 0, 0},
        {-sin(angleY), 0, cos(angleY), 0},
        {0, 0, 0, 1}
    };

    // Projection matrix
    double FoVScale = 1 / tan(FoV * 3.14 / 180.0 / 2.0); 
    double normalization = zFar / (zFar - zNear);
    double lambda = (-zFar * zNear) / (zFar - zNear);
    double perspective[4][4] = {
        {FoVScale * 100, 0, 0, 0},
        {0, FoVScale * 100, 0, 0},
        {0, 0, normalization, lambda},
        {0, 0, 1, 0}
    };

    std::priority_queue<ScanlineTriangle, std::vector<ScanlineTriangle>, CompareTriangles> triangleQueue;

    // Clipping arrays
    Vector3 clipped[2][3];

    // Render each entity with required components
    for (const auto& entity : world.getEntities()) {
        auto mesh = entity->getComponent<MeshComponent>();
        auto color = entity->getComponent<ColorComponent>();

        if (!mesh || !color || !entity->isEnabled) continue;

        const auto& verts = mesh->vertices;
        const auto& tris = mesh->triangles;
        const auto& normals = mesh->normals;

        for (size_t j = 0; j < tris.size(); j += 3) {
            Vector3 projectedVerts3D[3], translatedVertices[3];

            // Transform and project vertices
            for (int k = 0; k < 3; ++k) {
                int vertIndex = tris[j + k];
                Vector3 vert = verts[vertIndex];

                // Translate by camera position
                vert = vert - camPos;
                translatedVertices[k] = vert;

                // Rotate
                vert = vert.multiplyMatrix(rotationY);
                vert = vert.multiplyMatrix(rotationX);

                // Store for clipping
                projectedVerts3D[k] = vert;
            }

            // Fetch normals
            Vector3 worldNormal = normals[j / 3];

            Vector3 viewNormal = worldNormal;
            viewNormal = viewNormal.multiplyMatrix(rotationY);
            viewNormal = viewNormal.multiplyMatrix(rotationX);
            viewNormal = viewNormal.normalize();

            // Backface culling
            Vector3 midPoint = (projectedVerts3D[0] + projectedVerts3D[1] + projectedVerts3D[2]) / 3.0; 
            Vector3 viewDir = midPoint.normalize(); 
            if (viewNormal.dot(viewDir) < 0) continue; 
            // if (viewNormal.dot(projectedVerts3D[0]) < 0) continue; 

            // Lighting calculation
            Vector3 light = lightDir.normalize(); 
            double dp = (worldNormal.dot(light) + 1) / 2.0; 

            // Indirect (ambient) lighting
            const float ambientStrength = 0; 
            const Vector3 ambientColor(0.2, 0.3, 0.4); 
            Vector3 finalLighting = ambientColor * ambientStrength + Vector3(1.0, 1.0, 1.0) * (1.0f - ambientStrength) * dp; 

            // Calculate midpoint for depth sorting
            // Vector3 midPoint = Vector3::getMidpoint(translatedVertices[0], translatedVertices[1], translatedVertices[2]);
            double depth = midPoint.x * midPoint.x + midPoint.y * midPoint.y + midPoint.z * midPoint.z; 
            // double depth = Vector3::getTriangleDepth(translatedVertices[0], translatedVertices[1], translatedVertices[2]); 

            // Clip against near plane
            int clippedTrianglesCount = Vector3::clipTriangleAgainstPlane(Vector3(0, 0, zNear), Vector3(0, 0, 1), projectedVerts3D, clipped);

            // Process clipped triangles
            for (int k = 0; k < clippedTrianglesCount; ++k) {
                Vector3* currTri = clipped[k];
                ScanlineTriangle tri;

                // Project and convert to screen space
                for (int l = 0; l < 3; ++l) {
                    Vector3 vert = currTri[l];
                    vert = vert.multiplyMatrix(perspective);

                    if (vert.w != 0) {
                        vert.x /= (vert.w);
                        vert.y /= (vert.w);
                    }

                    vert.x += WINDOW_WIDTH / 2.0;
                    vert.y += WINDOW_HEIGHT / 2.0;

                    tri.points[l] = vert;
                }

                // Set triangle properties
                tri.depth = depth;
                // tri.r = dp * color->r; 
                // tri.g = dp * color->g; 
                // tri.b = dp * color->b; 
                tri.r = finalLighting.x * color->r; 
                tri.g = finalLighting.y * color->g; 
                tri.b = finalLighting.z * color->b; 
                tri.zIndex = mesh->zIndex; 

                triangleQueue.push(tri);
            }
        }
    }

    // Render triangles back to front
    while (!triangleQueue.empty()) {
        const ScanlineTriangle& tri = triangleQueue.top();
        Vector3 screenPoints[3] = { tri.points[0], tri.points[1], tri.points[2] };

        // Sort points by Y coordinate for scanline
        if (screenPoints[0].y > screenPoints[1].y) std::swap(screenPoints[0], screenPoints[1]);
        if (screenPoints[1].y > screenPoints[2].y) std::swap(screenPoints[1], screenPoints[2]);
        if (screenPoints[0].y > screenPoints[1].y) std::swap(screenPoints[0], screenPoints[1]);

        // Scanline fill algorithm
        // Top part of triangle
        float slope1 = (screenPoints[1].y - screenPoints[0].y) != 0 ?
            (screenPoints[1].x - screenPoints[0].x) / (screenPoints[1].y - screenPoints[0].y) : 0;
        float slope2 = (screenPoints[2].y - screenPoints[0].y) != 0 ?
            (screenPoints[2].x - screenPoints[0].x) / (screenPoints[2].y - screenPoints[0].y) : 0;

        float x1 = screenPoints[0].x;
        float x2 = screenPoints[0].x;

        // Scan top half of triangle
        for (int y = (int)screenPoints[0].y; y < (int)screenPoints[1].y; y++) {
            if (y >= 0 && y < WINDOW_HEIGHT) {
                float startX = x1 < x2 ? x1 : x2;
                float endX = x1 > x2 ? x1 : x2;
                App::DrawLine(startX, y, endX, y, tri.r, tri.g, tri.b);
            }
            x1 += slope1;
            x2 += slope2;
        }

        // Bottom part of triangle
        slope1 = (screenPoints[2].y - screenPoints[1].y) != 0 ?
            (screenPoints[2].x - screenPoints[1].x) / (screenPoints[2].y - screenPoints[1].y) : 0;
        x1 = screenPoints[1].x;

        // Scan bottom half of triangle
        for (int y = (int)screenPoints[1].y; y < (int)screenPoints[2].y; y++) {
            if (y >= 0 && y < WINDOW_HEIGHT) {
                float startX = x1 < x2 ? x1 : x2;
                float endX = x1 > x2 ? x1 : x2;
                App::DrawLine(startX, y, endX, y, tri.r, tri.g, tri.b);
            }
            x1 += slope1;
            x2 += slope2;
        }

        triangleQueue.pop();
    }
}

void Shutdown()
{
    world.clear();
}