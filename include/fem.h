#ifndef FEM_H
#define FEM_H

#include "raylib.h"
#include <stdbool.h>

typedef struct {
	bool showEdges;
	bool showVertexes;
} BodyDrawOptions;

typedef struct {
	double (*elements)[20][3]; 
	double (*akt)[3];		   
	int (*nt)[20];			   
	bool *zu_flags;			   
	bool *zp_flags;			   
	int numElements;
	int numNodes;
	int numEquations;
} FEM;

// Ініціалізація та очищення структури (Керування пам'яттю)
void FreeFEM(FEM *fem);
void BuildElements(FEM *fem, float bodySize[3], int bodySplit[3]);

// Обчислення МСЕ розрахунку сил
void ApplyForcesFEM(FEM *fem, float young, float poisson, float pressure, Vector3 **outDeformed);

// Функція рендерингу отриманої сітки елементів
void DrawBodyMesh(FEM *fem, Vector3 *customNodes, Vector3 origin, Color edgeColor, Color vertexColor, BodyDrawOptions opt);

#endif // FEM_H
