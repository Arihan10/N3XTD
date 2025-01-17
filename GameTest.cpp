///////////////////////////////////////////////////////////////////////////////
// Filename: GameTest.cpp
// Provides a demo of how to use the API
///////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------
#include "stdafx.h"
//------------------------------------------------------------------------
#include <windows.h>
#include <math.h>

#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <string>
#include <functional>
#include <algorithm>
//------------------------------------------------------------------------
#include "app\app.h"

extern int WINDOW_WIDTH;
extern int WINDOW_HEIGHT;

#include "Vector3.h"
#include "Shape3D.h"
#include "Cube.h"
#include "CustomShape.h"
//------------------------------------------------------------------------

// Example data....
CSimpleSprite *testSprite;
enum
{
	ANIM_FORWARDS,
	ANIM_BACKWARDS,
	ANIM_LEFT,
	ANIM_RIGHT,
};

// Camera and lighting
Vector3 camPos(0.0, 0.0, 0.0);
Vector3 lightDir(0.0, -1.0, -0.5);
sf::Color bgCol(0, 120, 220);

// Camera rotation angles
double angleY = 0.0;
double angleX = 0.0;
double FoV = 20.0; // 28
double zFar = 80.0;
double zNear = 2.0;
double moveSpeed = 200.0;
double mouseSensitivityX = 0.4;
double mouseSensitivityY = 0.3;
double deltaX = 0.0;
double deltaY = 0.0;

// Directional keys
bool dirs[10] = { false };

// Mouse control
bool movingCamera = false;

// Objects in the scene
std::vector<Shape3D*> objects;

// Start
void Init()
{
	// Add initial objects
	// objects.push_back(new Cube(Vector3(0, 0, -10), Vector3(3, 3, 3)));
	objects.push_back(new CustomShape("Suzanne.obj", Vector3(700, -100, -500), Vector3(3, 3, 3)));
	objects.push_back(new CustomShape("Fire_Axe_Test_3.obj", Vector3(700, 0, -900), Vector3(1, 1, 1)));
	objects.push_back(new CustomShape("AK_1.obj", Vector3(300, 0, -700), Vector3(3, 3, 3))); 
}

// Update
void Update(const float deltaTime)
{
	// Check WASD movement
	dirs[2] = App::IsKeyPressed('W'); 
	dirs[3] = App::IsKeyPressed('S'); 
	dirs[4] = App::IsKeyPressed('A'); 
	dirs[5] = App::IsKeyPressed('D'); 
	// Check Q/E vertical movement
	dirs[0] = App::IsKeyPressed('E'); 
	dirs[1] = App::IsKeyPressed('Q'); 
	// Check camera movement (Shift)
	bool currentMovingCamera = App::IsKeyPressed(VK_SHIFT); 
	// Check escape
	if (App::IsKeyPressed(VK_ESCAPE)) glutLeaveMainLoop(); 

	if (movingCamera) {
		float mouseX, mouseY;
		App::GetMousePos(mouseX, mouseY);

		// The API gives us normalized coordinates (-1 to 1)
		// Convert to pixel coordinates relative to center
		float centerX = 0.0f; // Center is 0 in normalized coords
		float centerY = 0.0f;

		deltaX = (mouseX - centerX) * WINDOW_WIDTH / 2;  // Scale back to pixels
		deltaY = (mouseY - centerY) * WINDOW_HEIGHT / 2;
	}
}

// Add your display calls here (DrawLine,Print, DrawSprite.) 
// See App.h 
void Render()
{	
	// Example Sprite Code
	testSprite->Draw();

	// Example Text
	App::Print(100, 100, "Sample Text");

	// Example Line Drawing
	static float a = 0.0f;
	const float r = 1.0f;
	float g = 1.0f;
	float b = 1.0f;
	a += 0.1f;
	for (int i = 0; i < 20; i++)
	{

		const float sx = 200 + sinf(a + i * 0.1f) * 60.0f;
		const float sy = 200 + cosf(a + i * 0.1f) * 60.0f;
		const float ex = 700 - sinf(a + i * 0.1f) * 60.0f;
		const float ey = 700 - cosf(a + i * 0.1f) * 60.0f;
		g = (float)i / 20.0f;
		b = (float)i / 20.0f;
		App::DrawLine(sx, sy, ex, ey, r, g, b);
	}
}
// OnDestroy
void Shutdown()
{
	delete testSprite;
}