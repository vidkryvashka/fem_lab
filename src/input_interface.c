#include "input_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "raylib.h"
#include "math_utils.h"
#include "raygui.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

InputValueFloat NewInputValueFloat(float val) {
	InputValueFloat input;
	input.value = val;
	input.editMode = false;
	snprintf(input.text, sizeof(input.text), "%.2f", val);
	return input;
}

InputValueInt NewInputValueInt(int val) {
	InputValueInt input;
	input.value = val;
	input.editMode = false;
	snprintf(input.text, sizeof(input.text), "%d", val);
	return input;
}

void UpdateInputValueFloat(InputValueFloat *input) {
	input->value = strtof(input->text, NULL);
}

void UpdateInputValueInt(InputValueInt *input) {
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


void render_input_interface(input_interface_t *ii, int *bcTypeMode) {
	DrawRectangle(10, 10, 310, 150, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 10, 310, 150, GRAY);
	GuiLabel((Rectangle){ 20, 15, 280, 20 }, "[ GEOMETRY CONFIGURATION ]");

	GuiLabel((Rectangle){ 20, 40, 140, 20 }, "Blocks X (Length):");
	if (GuiTextBox((Rectangle){ 160, 40, 140, 20 }, ii->splitXInput.text, 32, ii->splitXInput.editMode)) {
		ii->splitXInput.editMode = !ii->splitXInput.editMode;
		if (!ii->splitXInput.editMode) UpdateInputValueInt(&ii->splitXInput);
	}

	GuiLabel((Rectangle){ 20, 70, 140, 20 }, "Blocks Y (Width):");
	if (GuiTextBox((Rectangle){ 160, 70, 140, 20 }, ii->splitYInput.text, 32, ii->splitYInput.editMode)) {
		ii->splitYInput.editMode = !ii->splitYInput.editMode;
		if (!ii->splitYInput.editMode) UpdateInputValueInt(&ii->splitYInput);
	}

	GuiLabel((Rectangle){ 20, 100, 140, 20 }, "Blocks Z (Height):");
	if (GuiTextBox((Rectangle){ 160, 100, 140, 20 }, ii->splitZInput.text, 32, ii->splitZInput.editMode)) {
		ii->splitZInput.editMode = !ii->splitZInput.editMode;
		if (!ii->splitZInput.editMode) UpdateInputValueInt(&ii->splitZInput);
	}


	DrawRectangle(10, 170, 310, 190, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 170, 310, 190, GRAY);

	GuiLabel((Rectangle){ 20, 180, 140, 20 }, "Young's Modulus:");
	if (GuiTextBox((Rectangle){ 160, 180, 140, 20 }, ii->youngInput.text, 32, ii->youngInput.editMode)) {
		ii->youngInput.editMode = !ii->youngInput.editMode;
		if(!ii->youngInput.editMode) UpdateInputValueFloat(&ii->youngInput);
	}

	GuiLabel((Rectangle){ 20, 210, 140, 20 }, "Poisson's Ratio:");
	if (GuiTextBox((Rectangle){ 160, 210, 140, 20 }, ii->poissonInput.text, 32, ii->poissonInput.editMode)) {
		ii->poissonInput.editMode = !ii->poissonInput.editMode;
		if(!ii->poissonInput.editMode) UpdateInputValueFloat(&ii->poissonInput);
	}

	GuiLabel((Rectangle){ 20, 240, 140, 20 }, "Pressure:");
	if (GuiTextBox((Rectangle){ 160, 240, 140, 20 }, ii->pressureInput.text, 32, ii->pressureInput.editMode)) {
		ii->pressureInput.editMode = !ii->pressureInput.editMode;
		if(!ii->pressureInput.editMode) UpdateInputValueFloat(&ii->pressureInput);
	}

	DrawRectangle(10, 370, 310, 90, ColorAlpha(LIGHTGRAY, 0.8f));
	DrawRectangleLines(10, 370, 310, 90, GRAY);

	if (GuiToggleGroup((Rectangle){ 20, 380, 140, 25 }, "SET FIXATION (ZU);SET PRESSURE (ZP)", bcTypeMode)) {
		// changed mode
	}
	GuiLabel((Rectangle){ 20, 415, 120, 40 }, "Right Click on a 3D Face to toggle boundary\ncondition. Blue = Fixed, Orange = Loaded.");
}
