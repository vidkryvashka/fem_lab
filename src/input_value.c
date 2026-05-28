#include "input_value.h"
#include <stdio.h>
#include <stdlib.h>
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


void show_input_interface(input_interface_t *ii) {
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
}
