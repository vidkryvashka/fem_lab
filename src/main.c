#include "raylib.h"
#include "raymath.h"
#include "raygui.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "input_value.h"
#include "math_utils.h"
#include "fem.h"
#include <stdlib.h>

int main(void) {
	CalculateDFIABG(); // Ініціалізація математики Гаусса

	InitWindow(1280, 720, "FEM Lab");
	SetTargetFPS(60);

	Camera3D camera = { 0 };
	camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
	camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	// Створюємо Input елементи за допомогою нашого інструментарію
	InputValueFloat youngInput = NewInputValueFloat(4.0f);
	InputValueFloat poissonInput = NewInputValueFloat(0.3f);
	InputValueFloat pressureInput = NewInputValueFloat(2.0f);

	float bodySize[3] = {4.0f, 5.0f, 3.0f};
	int bodySplit[3] = {2, 2, 2};

	FEM fem = {0};
	BuildElements(&fem, bodySize, bodySplit);
	
	// Початкові ГУ
	for (int i = 0; i < bodySplit[0] * bodySplit[1]; i++) {
		fem.zu_flags[i * 6 + 4] = true; 
		fem.zp_flags[(fem.numElements - 1 - i) * 6 + 5] = true;
	}

	Vector3 *deformedNodes = NULL;
	BodyDrawOptions drawOpt = { .showEdges = true, .showVertexes = true };

	while (!WindowShouldClose()) {
		// Оновлення камери
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
			Vector2 delta = GetMouseDelta();
			UpdateCameraPro(&camera,
							(Vector3){0},
							(Vector3){delta.x * 0.1f, delta.y * 0.1f, 0},
							GetMouseWheelMove() * 2.0f);
		}

		// Рендер
		BeginDrawing();
		ClearBackground(RAYWHITE);

		BeginMode3D(camera);
			DrawGrid(20, 1.0f);
			Vector3 origin = (Vector3){ bodySize[0]*0.5f, 0, bodySize[2]*0.5f };

			DrawBodyMesh(&fem, NULL, origin, GRAY, BLUE, drawOpt);
			if (deformedNodes) {
				DrawBodyMesh(&fem, deformedNodes, origin, RED, GREEN, drawOpt);
			}
		EndMode3D();

		// Інтерфейс (Raygui)
		DrawRectangle(10, 10, 310, 180, ColorAlpha(LIGHTGRAY, 0.8f));
		
		GuiLabel((Rectangle){ 20, 20, 140, 20 }, "Young's Modulus:");
		if (GuiTextBox((Rectangle){ 160, 20, 140, 20 }, youngInput.text, 32, youngInput.editMode)) {
			youngInput.editMode = !youngInput.editMode;
			if(!youngInput.editMode) UpdateInputValueFloat(&youngInput);
		}

		GuiLabel((Rectangle){ 20, 50, 140, 20 }, "Poisson's Ratio:");
		if (GuiTextBox((Rectangle){ 160, 50, 140, 20 }, poissonInput.text, 32, poissonInput.editMode)) {
			poissonInput.editMode = !poissonInput.editMode;
			if(!poissonInput.editMode) UpdateInputValueFloat(&poissonInput);
		}

		GuiLabel((Rectangle){ 20, 80, 140, 20 }, "Pressure:");
		if (GuiTextBox((Rectangle){ 160, 80, 140, 20 }, pressureInput.text, 32, pressureInput.editMode)) {
			pressureInput.editMode = !pressureInput.editMode;
			if(!pressureInput.editMode) UpdateInputValueFloat(&pressureInput);
		}

		if (GuiButton((Rectangle){ 20, 120, 280, 40 }, "RUN FEM ANALYSIS")) {
			if (deformedNodes) free(deformedNodes);
			ApplyForcesFEM(&fem, youngInput.value, poissonInput.value, pressureInput.value, &deformedNodes);
		}

		EndDrawing();
	}

	// Чистимо пам'ять
	if (deformedNodes) free(deformedNodes);
	FreeFEM(&fem);

	CloseWindow();
	return 0;
}
