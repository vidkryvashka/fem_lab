#include <stdlib.h>
#include "raylib.h"
// #include "raymath.h"

#include "input_value.h"
#include "math_utils.h"
#include "fem.h"


void ResetBoundaryConditions(FEM *f, int split[3]) {
	for (int i = 0; i < f->numElements * 6; i++) {
		f->zu_flags[i] = false;
		f->zp_flags[i] = false;
	}
	for (int i = 0; i < split[0] * split[1]; i++) {
		f->zu_flags[i * 6 + 4] = true; 
		f->zp_flags[(f->numElements - 1 - i) * 6 + 5] = true;
	}
}


void update_mesh(input_interface_t *ii, FEM *fem, float bodySize[3], int bodySplit[3], Vector3 *deformedNodes) {
	if (!ii->splitXInput.editMode && !ii->splitYInput.editMode && !ii->splitZInput.editMode) {
		if (bodySplit[0] != ii->splitXInput.value || bodySplit[1] != ii->splitYInput.value || bodySplit[2] != ii->splitZInput.value) {
			
			if (ii->splitXInput.value < 1) ii->splitXInput.value = 1;
			if (ii->splitYInput.value < 1) ii->splitYInput.value = 1;
			if (ii->splitZInput.value < 1) ii->splitZInput.value = 1;

			bodySplit[0] = ii->splitXInput.value;
			bodySplit[1] = ii->splitYInput.value;
			bodySplit[2] = ii->splitZInput.value;

			bodySize[0] = bodySplit[0] * CUBE_SIZE;
			bodySize[1] = bodySplit[1] * CUBE_SIZE;
			bodySize[2] = bodySplit[2] * CUBE_SIZE;

			if (deformedNodes) { free(deformedNodes); deformedNodes = NULL; }

			BuildElements(fem, bodySize, bodySplit);
			ResetBoundaryConditions(fem, bodySplit);
		}
	}
}


int main(void) {
	CalculateDFIABG(); // Gauss math init

	InitWindow(1280, 720, "FEM Lab");
	SetTargetFPS(60);

	Camera3D camera = {
		.position = (Vector3){ 10.0f, 10.0f, 10.0f },
		.target = (Vector3){ 0.0f, 0.0f, 0.0f },
		.up = (Vector3){ 0.0f, 1.0f, 0.0f },
		.fovy = 45.0f,
		.projection = CAMERA_PERSPECTIVE
	};

	input_interface_t ii = (input_interface_t){
		.splitXInput = NewInputValueInt(5),
		.splitYInput = NewInputValueInt(1),
		.splitZInput = NewInputValueInt(1),
		.youngInput = NewInputValueFloat(4.0f),
		.poissonInput = NewInputValueFloat(0.3f),
		.pressureInput = NewInputValueFloat(100.0f)
	};

	float bodySize[3] = {
		ii.splitXInput.value * CUBE_SIZE,
		ii.splitYInput.value * CUBE_SIZE,
		ii.splitYInput.value * CUBE_SIZE
	};
	int bodySplit[3] = {
		ii.splitXInput.value,
		ii.splitYInput.value,
		ii.splitYInput.value
	};

	FEM fem = {0};
	BuildElements(&fem, bodySize, bodySplit);

	ResetBoundaryConditions(&fem, bodySplit);

	Vector3 *deformedNodes = NULL;
	BodyDrawOptions drawOpt = {
		.showEdges = true,
		.showVertexes = true
	};

	while (!WindowShouldClose()) {
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
			UpdateCamera(&camera, CAMERA_THIRD_PERSON);
		// else
			// UpdateCamera(&camera, CAMERA_ORBITAL);

		// Render
		BeginDrawing();
		ClearBackground(RAYWHITE);

		BeginMode3D(camera);
		DrawGrid(20, 1.0f);
			Vector3 origin = (Vector3){ bodySize[0] * 0.5f, 0.0f, bodySize[2] * 0.5f };
			camera.target = (Vector3){ 0.0f, bodySize[1] * 0.5f, 0.0f };
			DrawBodyMesh(&fem, NULL, origin, GRAY, BLUE, drawOpt);
			if (deformedNodes) {
				DrawBodyMesh(&fem, deformedNodes, origin, RED, GREEN, drawOpt);
			}
		EndMode3D();

		show_input_interface(&ii);

		if (run_fem_button()) {
			if (deformedNodes) { free(deformedNodes); deformedNodes = NULL; }
			ApplyForcesFEM(&fem, ii.youngInput.value, ii.poissonInput.value, ii.pressureInput.value, &deformedNodes);
		}

		update_mesh(&ii, &fem, bodySize, bodySplit, deformedNodes);

		EndDrawing();
	}

	if (deformedNodes) free(deformedNodes);
	FreeFEM(&fem);

	CloseWindow();
	return 0;
}
