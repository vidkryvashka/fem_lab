#ifndef INPUT_VALUE_H
#define INPUT_VALUE_H

#include <stdbool.h>

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

// Конструктори (аналоги NewInputValue у Go)
InputValueFloat NewInputValueFloat(float val);
InputValueInt NewInputValueInt(int val);

// Синхронізація тексту з числовим значенням після редагування
void UpdateInputValueFloat(InputValueFloat *input);
void UpdateInputValueInt(InputValueInt *input);

#endif // INPUT_VALUE_H
