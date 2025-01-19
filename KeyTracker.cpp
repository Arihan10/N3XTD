#include "stdafx.h"
#include "KeyTracker.h"
#include "app/app.h"
#include <memory.h>

KeyTracker::KeyTracker()
{
    memset(previousKeyState, 0, sizeof(previousKeyState));
}

void KeyTracker::Update()
{
    for (int i = 0; i < 256; i++)
    {
        previousKeyState[i] = App::IsKeyPressed(i);
    }
}

bool KeyTracker::IsKeyUp(int key)
{
    return previousKeyState[key] && !App::IsKeyPressed(key);
}

bool KeyTracker::IsKeyDown(int key)
{
    return !previousKeyState[key] && App::IsKeyPressed(key);
}