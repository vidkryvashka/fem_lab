#include "input_value.h"
#include <stdio.h>
#include <stdlib.h>

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
