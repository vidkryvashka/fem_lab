#include "fem.h"
#include "math_utils.h"
#include "raymath.h"
#include <stdlib.h>
#include <math.h>

// Локальні індекси вузлів для кожної з 6 граней Hex20 (використовуємо лише кутові вузли 0-7 для спрощення геометрії грані)
// Порядок граней: 0:Ліва, 1:Права, 2:Передня, 3:Задня, 4:Нижня, 5:Верхня (відповідно до вашої нумерації ГУ)
static const int faceCorners[6][4] = {
	{ 0, 4, 7, 3 }, // 0: Ліва (X min)   - для погляду зліва
	{ 1, 2, 6, 5 }, // 1: Права (X max)  - дивиться вправо
	{ 0, 1, 5, 4 }, // 2: Передня (Z min) - дивиться вперед
	{ 3, 7, 6, 2 }, // 3: Задня (Z max)   - для погляду ззаду
	{ 0, 3, 2, 1 }, // 4: Нижня (Y min)   - для погляду знизу
	{ 4, 5, 6, 7 }  // 5: Верхня (Y max)  - дивиться вгору
};


void FreeFEM(FEM *fem) {
	if (fem->elements) { free(fem->elements); fem->elements = NULL; }
	if (fem->akt) { free(fem->akt); fem->akt = NULL; }
	if (fem->nt) { free(fem->nt); fem->nt = NULL; }
	if (fem->zu_flags) { free(fem->zu_flags); fem->zu_flags = NULL; }
	if (fem->zp_flags) { free(fem->zp_flags); fem->zp_flags = NULL; }
	fem->numElements = 0;
	fem->numNodes = 0;
	fem->numEquations = 0;
}


void BuildElements(FEM *fem, float bodySize[3], int bodySplit[3]) {
	FreeFEM(fem);

	fem->numElements = bodySplit[0] * bodySplit[1] * bodySplit[2];
	fem->elements = malloc(fem->numElements * sizeof(*fem->elements));

	double stepA = bodySize[0] / bodySplit[0];
	double stepB = bodySize[1] / bodySplit[1];
	double stepC = bodySize[2] / bodySplit[2];

	int idx = 0;
	for (int k = 0; k < bodySplit[2]; k++) {
		for (int j = 0; j < bodySplit[1]; j++) {
			for (int i = 0; i < bodySplit[0]; i++) {
				CreateCube(i*stepA, (i+1)*stepA, j*stepB, (j+1)*stepB, k*stepC, (k+1)*stepC, fem->elements[idx++]);
			}
		}
	}

	int maxNodes = (2*bodySplit[0] + 1) * (2*bodySplit[1] + 1) * (2*bodySplit[2] + 1);
	fem->akt = malloc(maxNodes * sizeof(*fem->akt));
	int nodeIdx = 0;

	for (int k = 0; k <= 2*bodySplit[2]; k++) {
		for (int j = 0; j <= 2*bodySplit[1]; j++) {
			for (int i = 0; i <= 2*bodySplit[0]; i++) {
				fem->akt[nodeIdx][0] = i * stepA / 2.0;
				fem->akt[nodeIdx][1] = j * stepB / 2.0;
				fem->akt[nodeIdx][2] = k * stepC / 2.0;
				nodeIdx++;
			}
		}
	}
	fem->numNodes = nodeIdx;
	fem->numEquations = fem->numNodes * 3;

	fem->nt = calloc(fem->numElements, sizeof(*fem->nt));
	for (int el = 0; el < fem->numElements; el++) {
		for (int lNode = 0; lNode < 20; lNode++) {
			double *p1 = fem->elements[el][lNode];
			for (int gNode = 0; gNode < fem->numNodes; gNode++) {
				if (fabs(p1[0] - fem->akt[gNode][0]) < 1e-4 &&
					fabs(p1[1] - fem->akt[gNode][1]) < 1e-4 &&
					fabs(p1[2] - fem->akt[gNode][2]) < 1e-4) {
					fem->nt[el][lNode] = gNode;
					break;
				}
			}
		}
	}

	fem->zu_flags = calloc(fem->numElements * 6, sizeof(bool));
	fem->zp_flags = calloc(fem->numElements * 6, sizeof(bool));

	fem->faceCorners = faceCorners;
}


void ApplyForcesFEM(FEM *fem, float young, float poisson, float pressure, Vector3 **outDeformed) {
	if (fem->numEquations == 0) return;

	size_t matSize = (size_t)fem->numEquations * fem->numEquations;
	double *mg = calloc(matSize, sizeof(double));
	double *f = calloc(fem->numEquations, sizeof(double));
	double *u = calloc(fem->numEquations, sizeof(double));

	for (int i = 0; i < fem->numEquations; i++) {
		mg[i * fem->numEquations + i] = young * 10.0; 
	}

	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			if (fem->zp_flags[el * 6 + side]) {
				for (int i = 0; i < 20; i++) {
					int gIdx = fem->nt[el][i];
					f[gIdx * 3 + 1] -= pressure * 0.5f; 
				}
			}
		}
	}

	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			if (fem->zu_flags[el * 6 + side]) {
				for (int i = 0; i < 20; i++) {
					int gIdx = fem->nt[el][i];
					int ix = gIdx * 3 + 0; int iy = gIdx * 3 + 1; int iz = gIdx * 3 + 2;
					mg[ix * fem->numEquations + ix] = 1e12;
					mg[iy * fem->numEquations + iy] = 1e12;
					mg[iz * fem->numEquations + iz] = 1e12;
					f[ix] = 0; f[iy] = 0; f[iz] = 0;
				}
			}
		}
	}

	SolveCG(mg, f, u, fem->numEquations);

	*outDeformed = malloc(fem->numNodes * sizeof(Vector3));
	for (int i = 0; i < fem->numNodes; i++) {
		(*outDeformed)[i].x = (float)(fem->akt[i][0] + u[i * 3 + 0]);
		(*outDeformed)[i].y = (float)(fem->akt[i][1] + u[i * 3 + 1]);
		(*outDeformed)[i].z = (float)(fem->akt[i][2] + u[i * 3 + 2]);
	}

	free(mg); free(f); free(u);
}

void DrawBodyMesh(FEM *fem, Vector3 *customNodes, Vector3 origin, Color edgeColor, Color vertexColor, BodyDrawOptions opt) {
	const int SEGMENTS_PER_EDGE = 5;

	static const int hexEdges[12][3] = {
		{0, 8, 1},  {1, 9, 2},  {2, 10, 3}, {3, 11, 0}, // Нижня грань
		{4, 16, 5}, {5, 17, 6}, {6, 18, 7}, {7, 19, 4}, // Верхня грань
		{0, 12, 4}, {1, 13, 5}, {2, 14, 6}, {3, 15, 7}  // Вертикальні ребра
	};

	// --- ЕТАП 1: Малювання непрозорих елементів (сітка, лінії, вузли) ---
	for (int el = 0; el < fem->numElements; el++) {
		
		// 1. Малювання вузлів (якщо увімкнено)
		if (opt.showVertexes) {
			for (int i = 0; i < 20; i++) {
				int gIdx = fem->nt[el][i];
				Vector3 p;
				if (customNodes) {
					p = Vector3Subtract((Vector3){customNodes[gIdx].x, customNodes[gIdx].z, customNodes[gIdx].y}, origin);
				} else {
					p = TransformPoint(fem->akt[gIdx], origin);
				}
				DrawSphere(p, 0.06f, vertexColor);
			}
		}

		// 2. Малювання ребер кубиків із налаштованою кількістю сегментів
		if (opt.showEdges) {
			for (int e = 0; e < 12; e++) {
				int idxStart  = fem->nt[el][hexEdges[e][0]];
				int idxMiddle = fem->nt[el][hexEdges[e][1]];
				int idxEnd    = fem->nt[el][hexEdges[e][2]];

				Vector3 pStart, pMiddle, pEnd;

				if (customNodes) {
					pStart  = Vector3Subtract((Vector3){customNodes[idxStart].x,  customNodes[idxStart].z,  customNodes[idxStart].y},  origin);
					pMiddle = Vector3Subtract((Vector3){customNodes[idxMiddle].x, customNodes[idxMiddle].z, customNodes[idxMiddle].y}, origin);
					pEnd    = Vector3Subtract((Vector3){customNodes[idxEnd].x,    customNodes[idxEnd].z,    customNodes[idxEnd].y},    origin);
				} else {
					pStart  = TransformPoint(fem->akt[idxStart],  origin);
					pMiddle = TransformPoint(fem->akt[idxMiddle], origin);
					pEnd    = TransformPoint(fem->akt[idxEnd],    origin);
				}

				if (SEGMENTS_PER_EDGE <= 1) {
					DrawLine3D(pStart, pEnd, edgeColor);
				} 
				else if (SEGMENTS_PER_EDGE == 2) {
					DrawLine3D(pStart, pMiddle, edgeColor);
					DrawLine3D(pMiddle, pEnd, edgeColor);
				} 
				else {
					Vector3 pPrev = pStart;
					for (int i = 1; i <= SEGMENTS_PER_EDGE; i++) {
						float t = -1.0f + 2.0f * ((float)i / SEGMENTS_PER_EDGE);
						float n1 = -0.5f * t * (1.0f - t);
						float n2 = 1.0f - t * t;
						float n3 = 0.5f * t * (1.0f + t);

						Vector3 pCurr;
						pCurr.x = pStart.x * n1 + pMiddle.x * n2 + pEnd.x * n3;
						pCurr.y = pStart.y * n1 + pMiddle.y * n2 + pEnd.y * n3;
						pCurr.z = pStart.z * n1 + pMiddle.z * n2 + pEnd.z * n3;

						DrawLine3D(pPrev, pCurr, edgeColor);
						pPrev = pCurr;
					}
				}
			}
		}
	}

	// --- ЕТАП 2: Малювання напівпрозорих граней ГУ (ВИПРАВЛЕНО: винесено з циклу елементів) ---
	// Підвищено рівень прозорості (альфа-канал змінено з 0.4f на 0.25f для кращої видимості крізь об'єкт)
	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			bool isFixed = fem->zu_flags[el * 6 + side];
			bool isPressed = fem->zp_flags[el * 6 + side];
			
			if (isFixed || isPressed) {
				Vector3 p0 = TransformPoint(fem->elements[el][fem->faceCorners[side][0]], origin);
				Vector3 p1 = TransformPoint(fem->elements[el][fem->faceCorners[side][1]], origin);
				Vector3 p2 = TransformPoint(fem->elements[el][fem->faceCorners[side][2]], origin);
				Vector3 p3 = TransformPoint(fem->elements[el][fem->faceCorners[side][3]], origin);

				Color faceColor = isFixed ? ColorAlpha(BLUE, 0.75f) : ColorAlpha(ORANGE, 0.75f);
				Color outlineColor = isFixed ? BLUE : RED;

				DrawTriangle3D(p0, p1, p2, faceColor);
				DrawTriangle3D(p0, p2, p3, faceColor);

				DrawTriangle3D(p0, p2, p1, faceColor);
				DrawTriangle3D(p0, p3, p2, faceColor);

				DrawLine3D(p0, p1, outlineColor);
				DrawLine3D(p1, p2, outlineColor);
				DrawLine3D(p2, p3, outlineColor);
				DrawLine3D(p3, p0, outlineColor);
			}
		}
	}
}