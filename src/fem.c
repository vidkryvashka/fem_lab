#include "fem.h"
#include "math_utils.h"
#include "raymath.h"
#include <stdlib.h>
#include <math.h>

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
	for (int el = 0; el < fem->numElements; el++) {
		for (int i = 0; i < 20; i++) {
			int gIdx = fem->nt[el][i];
			Vector3 p1;
			if (customNodes) {
				p1 = Vector3Subtract((Vector3){customNodes[gIdx].x, customNodes[gIdx].z, customNodes[gIdx].y}, origin);
			} else {
				p1 = TransformPoint(fem->akt[gIdx], origin);
			}

			if (opt.showVertexes) DrawSphere(p1, 0.06f, vertexColor);

			if (opt.showEdges && i < 19) {
				int nextGIdx = fem->nt[el][i + 1];
				Vector3 p2;
				if (customNodes) {
					p2 = Vector3Subtract((Vector3){customNodes[nextGIdx].x, customNodes[nextGIdx].z, customNodes[nextGIdx].y}, origin);
				} else {
					p2 = TransformPoint(fem->akt[nextGIdx], origin);
				}
				DrawLine3D(p1, p2, edgeColor);
			}
		}
	}
}
