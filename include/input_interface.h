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
} InputField;

typedef struct {
	InputField splitXInput;
	InputField splitYInput;
	InputField splitZInput;
	InputField youngInput;
	InputField poissonInput;
	InputField pressureInput;
} input_interface_t;

InputField NewInputFloat(float val);
InputField NewInputInt(int val);

void UpdateInputValueFloat(InputField *input);
void UpdateInputValueInt(InputField *input);

void mouce_click_register(Camera3D *camera, FEM *fem, float bodySize[3], Vector3 **deformedNodes, int *bcTypeMode);

inline bool run_fem_button() {
	return GuiButton((Rectangle){ 20, 290, 280, 45 }, "RUN FEM ANALYSIS");
}

void render_input_interface(input_interface_t *ii, int *bcTypeMode);

#endif
