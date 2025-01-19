#pragma once

#include <stdafx.h>

class KeyTracker
{
public:
    KeyTracker();

    // Call this at the end of each frame to update previous key states
    void Update();

    // Returns true only on the frame when the key is released
    bool IsKeyUp(int key);

    // Returns true only on the frame when the key is first pressed
    bool IsKeyDown(int key);

private:
    bool previousKeyState[256];  // Store previous frame's key states
};