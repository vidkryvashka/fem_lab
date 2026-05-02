#include "raylib.h"
#include "raygui.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    bool showEdges;
    bool showVertexes;
} BodyDrawOptions;

Vector3 TransformPoint(double p[3], Vector3 origin) {
    return (Vector3){ 
        (float)p[0] - origin.x, 
        (float)p[2] - origin.y, 
        (float)p[1] - origin.z 
    };
}

int main() {
    InitWindow(1280, 720, "fem lab");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    char youngModulusText[128] = "4.0";
    bool youngEditMode = false;

    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            UpdateCameraPro(&camera, 
                (Vector3){ 0, 0, 0 }, 
                (Vector3){ delta.x * 0.1f, delta.y * 0.1f, 0 }, 
                GetMouseWheelMove() * 2.0f);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
            DrawGrid(32, 1.0f);
            // Here 'd be DrawBody
        EndMode3D();

        if (GuiTextBox((Rectangle){ 20, 20, 120, 30 }, youngModulusText, 128, youngEditMode)) {
            youngEditMode = !youngEditMode;
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}