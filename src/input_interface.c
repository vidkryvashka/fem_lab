#include "input_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "raylib.h"
#include "math_utils.h"
#include "raygui.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

InputField NewInputFloat(float val) {
	InputField in = { .value = val, .editMode = false };
	snprintf(in.text, sizeof(in.text), "%.2f", val);
	return in;
}

InputField NewInputInt(int val) {
	InputField in = { .value = (float)val, .editMode = false };
	snprintf(in.text, sizeof(in.text), "%d", val);
	return in;
}

void UpdateInputValueFloat(InputField *input) {
	input->value = strtof(input->text, NULL);
}

void UpdateInputValueInt(InputField *input) {
	input->value = (int)strtol(input->text, NULL, 10);
}


FaceHitResult CheckFaceClick(FEM *fem, Ray ray, Vector3 origin) {
	FaceHitResult result = { .elementIdx = -1, .sideIdx = -1, .hit = false };
	float closestDist = FLT_MAX;

	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			Vector3 p0 = TransformPoint(fem->elements[el][fem->faceCorners[side][0]], origin);
			Vector3 p1 = TransformPoint(fem->elements[el][fem->faceCorners[side][1]], origin);
			Vector3 p2 = TransformPoint(fem->elements[el][fem->faceCorners[side][2]], origin);
			Vector3 p3 = TransformPoint(fem->elements[el][fem->faceCorners[side][3]], origin);

			RayCollision c1 = GetRayCollisionTriangle(ray, p0, p1, p2);
			RayCollision c2 = GetRayCollisionTriangle(ray, p0, p2, p3);

			RayCollision c1_rev = GetRayCollisionTriangle(ray, p0, p2, p1);
			RayCollision c2_rev = GetRayCollisionTriangle(ray, p0, p3, p2);

			if (c1.hit && c1.distance < closestDist) {
				closestDist = c1.distance; result.elementIdx = el; result.sideIdx = side; result.hit = true;
			}
			if (c2.hit && c2.distance < closestDist) {
				closestDist = c2.distance; result.elementIdx = el; result.sideIdx = side; result.hit = true;
			}
			if (c1_rev.hit && c1_rev.distance < closestDist) {
				closestDist = c1_rev.distance; result.elementIdx = el; result.sideIdx = side; result.hit = true;
			}
			if (c2_rev.hit && c2_rev.distance < closestDist) {
				closestDist = c2_rev.distance; result.elementIdx = el; result.sideIdx = side; result.hit = true;
			}
		}
	}
	return result;
}


void mouce_click_register(Camera3D *camera, FEM *fem, float bodySize[3], Vector3 **deformedNodes, int *bcTypeMode) {
	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
		Ray ray = GetScreenToWorldRay(GetMousePosition(), *camera);
		Vector3 origin = (Vector3){ bodySize[0] * 0.5f, 0.0f, bodySize[2] * 0.5f };

		FaceHitResult hitRes = CheckFaceClick(fem, ray, origin);

		if (hitRes.hit) {
			int idx = hitRes.elementIdx * 6 + hitRes.sideIdx;

			if (*bcTypeMode == 0) {
				fem->zu_flags[idx] = !fem->zu_flags[idx];
				if (fem->zu_flags[idx]) fem->zp_flags[idx] = false; 
			} else {
				fem->zp_flags[idx] = !fem->zp_flags[idx];
				if (fem->zp_flags[idx]) fem->zu_flags[idx] = false;
			}
			
			if (*deformedNodes) { free(*deformedNodes); *deformedNodes = NULL; }
		}
	}
}


// ПОВЕРНЕНО СТАРУ НАДІЙНУ ЛОГІКУ, АЛЕ В КОМПАКТНОМУ ВИГЛЯДІ
void DrawGuiInputField(Rectangle rect, const char *label, InputField *field, bool isInt) {
	GuiLabel((Rectangle){ rect.x, rect.y, 140, rect.height }, label);

	// Точно так само, як у вашому першому робочому коді:
	if (GuiTextBox((Rectangle){ rect.x + 140, rect.y, rect.width - 140, rect.height }, field->text, 32, field->editMode)) {
		field->editMode = !field->editMode;
		
		// Оновлюємо значення тільки тоді, коли користувач вийшов з режиму редагування
		if (!field->editMode) {
			if (isInt) {
				UpdateInputValueInt(field);
				snprintf(field->text, sizeof(field->text), "%d", (int)field->value);
			} else {
				UpdateInputValueFloat(field);
			}
		}
	}
}


void render_input_interface(input_interface_t *ii, int *bcTypeMode) {
	// --- БЛОК 1: ГЕОМЕТРІЯ ---
	DrawRectangle(10, 10, 310, 150, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 10, 310, 150, GRAY);
	GuiLabel((Rectangle){ 20, 15, 280, 20 }, "[ GEOMETRY CONFIGURATION ]");

	DrawGuiInputField((Rectangle){ 20, 40, 280, 20 }, "Blocks X (Length):",  &ii->splitXInput, true);
	DrawGuiInputField((Rectangle){ 20, 70, 280, 20 }, "Blocks Y (Width):",   &ii->splitYInput, true);
	DrawGuiInputField((Rectangle){ 20, 100, 280, 20 }, "Blocks Z (Height):", &ii->splitZInput, true);

	// --- БЛОК 2: ФІЗИЧНІ ПАРАМЕТРИ ---
	DrawRectangle(10, 170, 310, 190, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 170, 310, 190, GRAY);

	DrawGuiInputField((Rectangle){ 20, 180, 280, 20 }, "Young's Modulus:", &ii->youngInput,   false);
	DrawGuiInputField((Rectangle){ 20, 210, 280, 20 }, "Poisson's Ratio:", &ii->poissonInput, false);
	DrawGuiInputField((Rectangle){ 20, 240, 280, 20 }, "Pressure:",        &ii->pressureInput, false);

	// --- БЛОК 3: ГРАНИЧНІ УМОВИ ---
	DrawRectangle(10, 370, 310, 90, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 370, 310, 90, GRAY);

	GuiToggleGroup((Rectangle){ 20, 380, 140, 25 }, "SET FIXATION (ZU);SET PRESSURE (ZP)", bcTypeMode);
	GuiLabel((Rectangle){ 20, 415, 120, 40 }, "Right Click on a 3D Face to toggle boundary\ncondition. Blue = Fixed, Orange = Loaded.");
}
