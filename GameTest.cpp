///////////////////////////////////////////////////////////////////////////////
// GameTest.cpp - 3D Wireframe Renderer
///////////////////////////////////////////////////////////////////////////////
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
#include "app\app.h"
#include "Vector3.h"
#include "Shape3D.h"
#include "Cube.h"
#include "CustomShape.h"

struct ScanlineTriangle {
    Vector3 points[3];
    double depth;
    float r, g, b;
};

struct CompareTriangles {
    bool operator()(const ScanlineTriangle& a, const ScanlineTriangle& b) {
        return a.depth < b.depth; // Reverse order for back-to-front
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
double FoV = 20.0;
double zFar = 80.0;
double zNear = 2.0;
double moveSpeed = 2.0;
double mouseSensitivityX = 0.000004;
double mouseSensitivityY = 0.000003;
double deltaX = 0.0;
double deltaY = 0.0;
float lastMouseX = 0;
float lastMouseY = 0;

// Directional keys and mouse control
bool dirs[10] = { false };
bool movingCamera = false;

// Objects in the scene
std::vector<Shape3D*> objects;

void Init()
{
    objects.push_back(new CustomShape("Suzanne.obj", Vector3(0, 0, 1000), Vector3(3, 3, 3)));
    objects.push_back(new CustomShape("Fire_Axe_Test_3.obj", Vector3(700, 0, -900), Vector3(1, 1, 1)));
    objects.push_back(new CustomShape("AK_1.obj", Vector3(300, 0, -700), Vector3(3, 3, 3)));
    objects.push_back(new CustomShape("Spaceship.obj", Vector3(100, -200, 200), Vector3(2, 2, 2)));

}

void Update(const float deltaTime)
{
    // Input handling
    dirs[2] = App::IsKeyPressed('W'); 
    dirs[3] = App::IsKeyPressed('S'); 
    dirs[4] = App::IsKeyPressed('A'); 
    dirs[5] = App::IsKeyPressed('D'); 
    dirs[0] = App::IsKeyPressed('E'); // up
    dirs[1] = App::IsKeyPressed('Q'); // down

    bool currentMovingCamera = App::IsKeyPressed(VK_SHIFT);
    if (currentMovingCamera && !movingCamera) {
        // First frame of camera movement
        movingCamera = true;
    }
    movingCamera = currentMovingCamera;

    if (App::IsKeyPressed(VK_ESCAPE)) {
        glutLeaveMainLoop();
    }

    // Mouse handling
    if (movingCamera) {
        float mouseX, mouseY;
        App::GetMousePos(mouseX, mouseY);

        // Calculate delta from last frame's position
        deltaX = (mouseX - lastMouseX) * WINDOW_WIDTH / 2;
        deltaY = (mouseY - lastMouseY) * WINDOW_HEIGHT / 2;

        // Store current position for next frame
        lastMouseX = mouseX;
        lastMouseY = mouseY;

        // Camera movement
        angleY -= deltaX * deltaTime * mouseSensitivityX;
        angleX += deltaY * deltaTime * mouseSensitivityY;

        if (dirs[0]) camPos.y += moveSpeed * deltaTime;
        if (dirs[1]) camPos.y -= moveSpeed * deltaTime;
        if (dirs[2]) {
            camPos.x += sin(-angleY) * moveSpeed * deltaTime;
            camPos.z += cos(-angleY) * moveSpeed * deltaTime;
        }
        if (dirs[3]) {
            camPos.x -= sin(-angleY) * moveSpeed * deltaTime;
            camPos.z -= cos(-angleY) * moveSpeed * deltaTime;
        }
        if (dirs[4]) {
            camPos.x -= sin(-angleY + 3.14 / 2) * moveSpeed * deltaTime;
            camPos.z -= cos(-angleY + 3.14 / 2) * moveSpeed * deltaTime;
        }
        if (dirs[5]) {
            camPos.x += sin(-angleY + 3.14 / 2) * moveSpeed * deltaTime;
            camPos.z += cos(-angleY + 3.14 / 2) * moveSpeed * deltaTime;
        }
    }
    else {
        // Reset deltas when not moving camera
        deltaX = 0;
        deltaY = 0;

        // Get current mouse position so we don't get a huge delta
        // when starting camera movement
        App::GetMousePos(lastMouseX, lastMouseY);
    }
}

void Render()
{
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
        {FoVScale, 0, 0, 0},
        {0, FoVScale, 0, 0},
        {0, 0, normalization, lambda},
        {0, 0, 1, 0}
    };

    // First calculate plane normals from projection/FoV
    double fovRad = FoV * 3.14 / 180.0;
    double tanFov = tan(fovRad / 2.0);
    Vector3 nearPlaneN(0, 0, 1);  // Looking down +Z
    Vector3 leftPlaneN(tanFov, 0, 1);
    leftPlaneN.normalize();  // Angled left plane
    Vector3 rightPlaneN(-tanFov, 0, 1);
    rightPlaneN.normalize(); // Angled right plane
    Vector3 topPlaneN(0, -tanFov, 1);
    topPlaneN.normalize();   // Angled top plane
    Vector3 bottomPlaneN(0, tanFov, 1);
    bottomPlaneN.normalize(); // Angled bottom plane

    // Sort triangles by depth buffer
    std::priority_queue<ScanlineTriangle, std::vector<ScanlineTriangle>, CompareTriangles> triangleQueue; 

    // Clipping arrays
    Vector3 clipped[2][3];

    // Render each object
    for (Shape3D* object : objects) {
        const std::vector<int>& tris = object->getTris();
        const std::vector<Vector3>& verts = object->getVerts();

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

            // Calculate normal for backface culling and lighting
            Vector3 normal = (projectedVerts3D[1] - projectedVerts3D[0]).cross(projectedVerts3D[2] - projectedVerts3D[0]).normalize();
            if (normal.dot(projectedVerts3D[0]) < 0) continue;

            // Lighting calculation
            Vector3 light = lightDir.normalize();
            double dp = (normal.dot(light) + 1) / 2.0;

            // Calculate midpoint for depth sorting
            Vector3 midPoint = Vector3::getMidpoint(translatedVertices[0], translatedVertices[1], translatedVertices[2]);
            double depth = midPoint.x * midPoint.x + midPoint.y * midPoint.y + midPoint.z * midPoint.z;

            // Clip against all frustum planes
            Vector3 clippedOutput[2][2][3];  // Two buffers of [2][3] arrays
            int currentBuffer = 0;

            // Near plane clip
            int clippedTrianglesCount = Vector3::clipTriangleAgainstPlane(
                Vector3(0, 0, zNear), nearPlaneN,
                projectedVerts3D, clippedOutput[currentBuffer]);
            if (clippedTrianglesCount == 0) continue;

            // Left plane clip
            currentBuffer = 1 - currentBuffer;
            int newClippedCount = 0;
            for (int i = 0; i < clippedTrianglesCount; i++) {
                int newTriangles = Vector3::clipTriangleAgainstPlane(
                    Vector3(0, 0, 0), leftPlaneN,
                    clippedOutput[1 - currentBuffer][i], clippedOutput[currentBuffer]);
                newClippedCount += newTriangles;
            }
            clippedTrianglesCount = newClippedCount;
            if (clippedTrianglesCount == 0) continue;

            // Right plane clip
            currentBuffer = 1 - currentBuffer;
            newClippedCount = 0;
            for (int i = 0; i < clippedTrianglesCount; i++) {
                int newTriangles = Vector3::clipTriangleAgainstPlane(
                    Vector3(0, 0, 0), rightPlaneN,
                    clippedOutput[1 - currentBuffer][i], clippedOutput[currentBuffer]);
                newClippedCount += newTriangles;
            }
            clippedTrianglesCount = newClippedCount;
            if (clippedTrianglesCount == 0) continue;

            // Top plane clip
            currentBuffer = 1 - currentBuffer;
            newClippedCount = 0;
            for (int i = 0; i < clippedTrianglesCount; i++) {
                int newTriangles = Vector3::clipTriangleAgainstPlane(
                    Vector3(0, 0, 0), topPlaneN,
                    clippedOutput[1 - currentBuffer][i], clippedOutput[currentBuffer]);
                newClippedCount += newTriangles;
            }
            clippedTrianglesCount = newClippedCount;
            if (clippedTrianglesCount == 0) continue;

            // Bottom plane clip (final output goes to original clipped array)
            currentBuffer = 1 - currentBuffer;
            newClippedCount = 0;
            for (int i = 0; i < clippedTrianglesCount; i++) {
                int newTriangles = Vector3::clipTriangleAgainstPlane(
                    Vector3(0, 0, 0), bottomPlaneN,
                    clippedOutput[1 - currentBuffer][i], clipped);
                newClippedCount += newTriangles;
            }
            clippedTrianglesCount = newClippedCount;
            if (clippedTrianglesCount == 0) continue;

            // Process clipped triangles
            for (int k = 0; k < clippedTrianglesCount; ++k) {
                Vector3* currTri = clipped[k];
                ScanlineTriangle tri;

                // Project and convert to screen space
                for (int l = 0; l < 3; ++l) {
                    Vector3 vert = currTri[l];
                    vert = vert.multiplyMatrix(perspective);

                    if (vert.w != 0) {
                        vert.x /= (vert.w / 100.0);
                        vert.y /= (vert.w / 100.0);
                    }

                    vert.x += WINDOW_WIDTH / 2.0;
                    vert.y += WINDOW_HEIGHT / 2.0;

                    tri.points[l] = vert;
                }

                // Set triangle properties
                tri.depth = depth;
                tri.r = dp * object->getColor().r / 255.0f;
                tri.g = dp * object->getColor().g / 255.0f;
                tri.b = dp * object->getColor().b / 255.0f;

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
    for (Shape3D* obj : objects) {
        delete obj;
    }
}