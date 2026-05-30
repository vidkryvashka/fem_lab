#ifndef INPUT_INTERFACE_H
#define INPUT_INTERFACE_H

#include <stdbool.h>
#include "raylib.h"
#include "raygui.h"
#include "fem.h"

#define CUBE_SIZE 1.0f

typedef struct {
	float value;
	char text[32];
	bool editMode;
} InputValueFloat;

typedef struct {
	int value;
	char text[32];
	bool editMode;
} InputValueInt;

typedef struct {
	InputValueInt splitXInput;
	InputValueInt splitYInput;
	InputValueInt splitZInput;
	InputValueFloat youngInput;
	InputValueFloat poissonInput;
	InputValueFloat pressureInput;
} input_interface_t;

InputValueFloat NewInputValueFloat(float val);
InputValueInt NewInputValueInt(int val);

void UpdateInputValueFloat(InputValueFloat *input);
void UpdateInputValueInt(InputValueInt *input);

void mouce_click_register(Camera3D *camera, FEM *fem, float bodySize[3], Vector3 **deformedNodes, int *bcTypeMode);

inline bool run_fem_button() {
	return GuiButton((Rectangle){ 20, 290, 280, 45 }, "RUN FEM ANALYSIS");
}

void render_input_interface(input_interface_t *ii, int *bcTypeMode);

#endif
