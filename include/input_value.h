#ifndef INPUT_VALUE_H
#define INPUT_VALUE_H

#include <stdbool.h>
#include "raylib.h"
#include "raygui.h"

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

// Конструктори (аналоги NewInputValue у Go)
InputValueFloat NewInputValueFloat(float val);
InputValueInt NewInputValueInt(int val);

// Синхронізація тексту з числовим значенням після редагування
void UpdateInputValueFloat(InputValueFloat *input);
void UpdateInputValueInt(InputValueInt *input);

inline bool run_fem_button() {
	return GuiButton((Rectangle){ 20, 290, 280, 45 }, "RUN FEM ANALYSIS");
}

void show_input_interface(input_interface_t *ii);

#endif // INPUT_VALUE_H
