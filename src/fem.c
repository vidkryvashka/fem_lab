#include "fem.h"
#include "math_utils.h"
#include "raymath.h"
#include <stdlib.h>
#include <math.h>

// Local node indices for each of the 6 Hex20 faces (we only use corner nodes 0-7 to simplify the face geometry)
// according to your GU numbering
static const int faceCorners[6][4] = {
	{ 0, 4, 7, 3 }, // 0: left (X min)
	{ 1, 2, 6, 5 }, // 1: right (X max)
	{ 0, 1, 5, 4 }, // 2: front (Z min)
	{ 3, 7, 6, 2 }, // 3: back (Z max)
	{ 0, 3, 2, 1 }, // 4: low (Y min)
	{ 4, 5, 6, 7 }  // 5: up (Y max)
};

static const int sideNodeIndices[6][8] = {
	{3, 0, 4, 7, 11, 12, 19, 15}, // 0: left
	{1, 2, 6, 5, 9,  14, 17, 13}, // 1: right
	{0, 1, 5, 4, 8,  13, 16, 12}, // 2: front
	{2, 3, 7, 6, 10, 15, 18, 14}, // 3: back
	{3, 2, 1, 0, 10, 9,  8,  11}, // 4: low
	{4, 5, 6, 7, 16, 17, 18, 19}  // 5: up
};


static Color GetHeatmapColor(double value, double minVal, double maxVal) {
	if (maxVal <= minVal) return GRAY;
	double t = (value - minVal) / (maxVal - minVal);
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;

	double r = 0.0, g = 0.0, b = 0.0;
	if (t < 0.25) { // 0.00 - 0.25: Blue -> Cyan
		r = 0.0;
		g = t * 4.0;
		b = 1.0;
	} else if (t < 0.50) { // 0.25 - 0.50: Cyan -> Green
		r = 0.0;
		g = 1.0;
		b = 1.0 - (t - 0.25) * 4.0;
	} else if (t < 0.75) { // 0.50 - 0.75: Green -> Yellow
		r = (t - 0.50) * 4.0;
		g = 1.0;
		b = 0.0;
	} else { // 0.75 - 1.00: Yellow -> Red
		r = 1.0;
		g = 1.0 - (t - 0.75) * 4.0;
		b = 0.0;
	}

	return (Color){
		(unsigned char)(r * 255.0),
		(unsigned char)(g * 255.0),
		(unsigned char)(b * 255.0),
		255
	};
}


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


void ApplyForcesFEM(FEM *fem, float young, float poisson, float pressure, Vector3 **outDeformed, double **outStresses) {
	if (fem->numEquations == 0) return;

	size_t matSize = (size_t)fem->numEquations * fem->numEquations;
	double *mg = calloc(matSize, sizeof(double));
	double *f = calloc(fem->numEquations, sizeof(double));
	double *u = calloc(fem->numEquations, sizeof(double));

	// Lamé coefficients (from Young's modulus and Poisson's coefficient)
	double lambda = (young * poisson) / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
	double mu = young / (2.0 * (1.0 + poisson));

	// 1. FORMATION OF THE GLOBAL STIFFNESS MATRIX (MGE -> MG)
	for (int el = 0; el < fem->numElements; el++) {
		double mge[60][60] = {0};
		double dfixyz[27][20][3] = {0};
		double djDet[27] = {0};

		// 1.1 Calculation of Jacobian and derivatives in global coordinates
		for (int gp = 0; gp < 27; gp++) {
			double J[3][3] = {0};
			for (int k = 0; k < 20; k++) {
				int gNode = fem->nt[el][k];
				for (int i = 0; i < 3; i++) {
					J[i][0] += dfiabg[gp][k][i] * fem->akt[gNode][0];
					J[i][1] += dfiabg[gp][k][i] * fem->akt[gNode][1];
					J[i][2] += dfiabg[gp][k][i] * fem->akt[gNode][2];
				}
			}

			djDet[gp] = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1])
					  - J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0])
					  + J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);

			if (fabs(djDet[gp]) < 1e-12) continue;

			double invJ[3][3];
			invJ[0][0] =  (J[1][1]*J[2][2] - J[1][2]*J[2][1]) / djDet[gp];
			invJ[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) / djDet[gp];
			invJ[0][2] =  (J[0][1]*J[1][2] - J[0][2]*J[1][1]) / djDet[gp];
			invJ[1][0] = -(J[1][0]*J[2][2] - J[1][2]*J[2][0]) / djDet[gp];
			invJ[1][1] =  (J[0][0]*J[2][2] - J[0][2]*J[2][0]) / djDet[gp];
			invJ[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) / djDet[gp];
			invJ[2][0] =  (J[1][0]*J[2][1] - J[1][1]*J[2][0]) / djDet[gp];
			invJ[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) / djDet[gp];
			invJ[2][2] =  (J[0][0]*J[1][1] - J[0][1]*J[1][0]) / djDet[gp];

			for (int k = 0; k < 20; k++) {
				for (int i = 0; i < 3; i++) {
					dfixyz[gp][k][i] = invJ[i][0]*dfiabg[gp][k][0] +
									   invJ[i][1]*dfiabg[gp][k][1] +
									   invJ[i][2]*dfiabg[gp][k][2];
				}
			}
		}

		// 1.2 Compilation of the local matrix MGE[60][60]
		int index = 0;
		for (int k1 = 0; k1 < 3; k1++) {
			for (int k2 = 0; k2 < 3; k2++) {
				for (int k3 = 0; k3 < 3; k3++) {
					double weight = gaussianConst[k1] * gaussianConst[k2] * gaussianConst[k3] * djDet[index];
					for (int i = 0; i < 20; i++) {
						for (int j = 0; j < 20; j++) {
							double d00 = dfixyz[index][i][0]*dfixyz[index][j][0];
							double d11 = dfixyz[index][i][1]*dfixyz[index][j][1];
							double d22 = dfixyz[index][i][2]*dfixyz[index][j][2];
							
							double a11 = weight * (lambda*(1-poisson)*d00 + mu*(d11+d22));
							double a22 = weight * (lambda*(1-poisson)*d11 + mu*(d00+d22));
							double a33 = weight * (lambda*(1-poisson)*d22 + mu*(d00+d11));
							
							double a12 = weight * (lambda*poisson*dfixyz[index][i][0]*dfixyz[index][j][1] + mu*dfixyz[index][i][1]*dfixyz[index][j][0]);
							double a13 = weight * (lambda*poisson*dfixyz[index][i][0]*dfixyz[index][j][2] + mu*dfixyz[index][i][2]*dfixyz[index][j][0]);
							double a23 = weight * (lambda*poisson*dfixyz[index][i][1]*dfixyz[index][j][2] + mu*dfixyz[index][i][2]*dfixyz[index][j][1]);

							mge[i][j] += a11;          mge[i][20+j] += a12;       mge[i][40+j] += a13;
							mge[20+i][j] += a12;       mge[20+i][20+j] += a22;    mge[20+i][40+j] += a23;
							mge[40+i][j] += a13;       mge[40+i][20+j] += a23;    mge[40+i][40+j] += a33;
						}
					}
					index++;
				}
			}
		}

		// 1.3 Assembly into the global MG matrix
		for (int i = 0; i < 60; i++) {
			int gNodeI = fem->nt[el][i % 20];
			int dofI = i / 20;
			int mgI = gNodeI * 3 + dofI;
			for (int j = 0; j < 60; j++) {
				int gNodeJ = fem->nt[el][j % 20];
				int dofJ = j / 20;
				int mgJ = gNodeJ * 3 + dofJ;
				mg[mgJ * fem->numEquations + mgI] += mge[j][i];
			}
		}
	}

	// 2. VECTOR OF PRESSURE FORCES (ZP - Surface integration)
	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			if (fem->zp_flags[el * 6 + side]) {
				double zpPoints[8][3];
				for (int i = 0; i < 8; i++) {
					int gNode = fem->nt[el][sideNodeIndices[side][i]];
					zpPoints[i][0] = fem->akt[gNode][0];
					zpPoints[i][1] = fem->akt[gNode][1];
					zpPoints[i][2] = fem->akt[gNode][2];
				}

				double fe1[8] = {0}, fe2[8] = {0}, fe3[8] = {0};
				int idx = 0;
				for (int k1 = 0; k1 < 3; k1++) {
					for (int k2 = 0; k2 < 3; k2++) {
						double sumXEta=0, sumYEta=0, sumZEta=0;
						double sumXTau=0, sumYTau=0, sumZTau=0;
						for (int j = 0; j < 8; j++) {
							sumXEta += zpPoints[j][0] * dpsite[idx][j][0];
							sumYEta += zpPoints[j][1] * dpsite[idx][j][0];
							sumZEta += zpPoints[j][2] * dpsite[idx][j][0];
							sumXTau += zpPoints[j][0] * dpsite[idx][j][1];
							sumYTau += zpPoints[j][1] * dpsite[idx][j][1];
							sumZTau += zpPoints[j][2] * dpsite[idx][j][1];
						}
						
						for (int i = 0; i < 8; i++) {
							double weight = gaussianConst[k1] * gaussianConst[k2] * pressure * dpsiteXYZdeNT[idx][i];
							fe1[i] += weight * (sumZTau * sumYEta - sumYTau * sumZEta);
							fe2[i] += weight * (sumXTau * sumZEta - sumZTau * sumXEta);
							fe3[i] += weight * (sumYTau * sumXEta - sumXTau * sumYEta);
						}
						idx++;
					}
				}

				// Distribution of local forces on global face nodes
				double* feTarget = (side == 0 || side == 1) ? fe1 : (side == 2 || side == 3) ? fe2 : fe3;
				int dof = (side == 0 || side == 1) ? 0 : (side == 2 || side == 3) ? 1 : 2;

				for (int i = 0; i < 8; i++) {
					int gNode = fem->nt[el][sideNodeIndices[side][i]];
					f[gNode * 3 + dof] += feTarget[i];
				}
			}
		}
	}

	// 3. APPLICATION OF BORDER CONDITIONS (Fine method - ZU)
	for (int el = 0; el < fem->numElements; el++) {
		for (int side = 0; side < 6; side++) {
			if (fem->zu_flags[el * 6 + side]) {
				for (int i = 0; i < 8; i++) {
					int gIdx = fem->nt[el][sideNodeIndices[side][i]];
					mg[(gIdx * 3 + 0) * fem->numEquations + (gIdx * 3 + 0)] = 1e16;
					mg[(gIdx * 3 + 1) * fem->numEquations + (gIdx * 3 + 1)] = 1e16;
					mg[(gIdx * 3 + 2) * fem->numEquations + (gIdx * 3 + 2)] = 1e16;
					f[gIdx * 3 + 0] = 0;
					f[gIdx * 3 + 1] = 0;
					f[gIdx * 3 + 2] = 0;
				}
			}
		}
	}

	// 4. SOLUTION OF SLIE
	SolveCG(mg, f, u, fem->numEquations);

	// 5. UPDATE OF COORDINATES AND VOLTAGE
	if (*outDeformed) { free(*outDeformed); *outDeformed = NULL; }
	*outDeformed = malloc(fem->numNodes * sizeof(Vector3));
	for (int i = 0; i < fem->numNodes; i++) {
		(*outDeformed)[i].x = (float)(fem->akt[i][0] + u[i * 3 + 0]);
		(*outDeformed)[i].y = (float)(fem->akt[i][1] + u[i * 3 + 1]);
		(*outDeformed)[i].z = (float)(fem->akt[i][2] + u[i * 3 + 2]);
	}

	// --- CALCULATION OF VOLTAGE ---
	if (*outStresses) { free(*outStresses); *outStresses = NULL; }
	*outStresses = calloc(fem->numElements, sizeof(double));
	
	// Gaussian center point index (alpha=0, beta=0, gamma=0) in dfiabg array
	int centerIdx = 1 * 9 + 1 * 3 + 1; // 13

	for (int el = 0; el < fem->numElements; el++) {
		// 1. Formation of the Jacobian matrix for the center of the element
		double J[3][3] = {0};
		for (int k = 0; k < 20; k++) {
			int gNode = fem->nt[el][k];
			for (int i = 0; i < 3; i++) {
				J[i][0] += dfiabg[centerIdx][k][i] * fem->akt[gNode][0];
				J[i][1] += dfiabg[centerIdx][k][i] * fem->akt[gNode][1];
				J[i][2] += dfiabg[centerIdx][k][i] * fem->akt[gNode][2];
			}
		}

		// 2. Calculation of the determinant and the inverse Jacobian matrix (invJ)
		double det = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1])
				   - J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0])
				   + J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);

		if (fabs(det) < 1e-12) continue; // Avoiding division by zero

		double invJ[3][3];
		invJ[0][0] =  (J[1][1]*J[2][2] - J[1][2]*J[2][1]) / det;
		invJ[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) / det;
		invJ[0][2] =  (J[0][1]*J[1][2] - J[0][2]*J[1][1]) / det;
		invJ[1][0] = -(J[1][0]*J[2][2] - J[1][2]*J[2][0]) / det;
		invJ[1][1] =  (J[0][0]*J[2][2] - J[0][2]*J[2][0]) / det;
		invJ[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) / det;
		invJ[2][0] =  (J[1][0]*J[2][1] - J[1][1]*J[2][0]) / det;
		invJ[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) / det;
		invJ[2][2] =  (J[0][0]*J[1][1] - J[0][1]*J[1][0]) / det;

		// 3. Derivatives of shape functions in global coordinates x, y, z
		double dN_dx[20][3] = {0};
		for (int k = 0; k < 20; k++) {
			for (int i = 0; i < 3; i++) {
				dN_dx[k][i] = invJ[i][0]*dfiabg[centerIdx][k][0] +
							  invJ[i][1]*dfiabg[centerIdx][k][1] +
							  invJ[i][2]*dfiabg[centerIdx][k][2];
			}
		}

		// 4. Calculation of deformations based on the displacement vector u
		double exx=0, eyy=0, ezz=0, gxy=0, gxz=0, gyz=0;
		for (int k = 0; k < 20; k++) {
			int gNode = fem->nt[el][k];
			double ux = u[gNode*3 + 0], uy = u[gNode*3 + 1], uz = u[gNode*3 + 2];
			
			exx += dN_dx[k][0] * ux;
			eyy += dN_dx[k][1] * uy;
			ezz += dN_dx[k][2] * uz;
			gxy += (dN_dx[k][1] * ux + dN_dx[k][0] * uy); // engineering shear deformation
			gxz += (dN_dx[k][2] * ux + dN_dx[k][0] * uz);
			gyz += (dN_dx[k][2] * uy + dN_dx[k][1] * uz);
		}

		// 5. Components of the stress tensor according to Hooke's law
		double sxx = lambda*(exx+eyy+ezz) + 2.0*mu*exx;
		double syy = lambda*(exx+eyy+ezz) + 2.0*mu*eyy;
		double szz = lambda*(exx+eyy+ezz) + 2.0*mu*ezz;
		double sxy = mu*gxy;
		double sxz = mu*gxz;
		double syz = mu*gyz;

		// 6. Equivalent stress according to Mises (Von Mises)
		double vmStress = sqrt(0.5 * (pow(sxx-syy,2) + pow(syy-szz,2) + pow(szz-sxx,2) + 6.0*(sxy*sxy + sxz*sxz + syz*syz)));
		(*outStresses)[el] = vmStress;
	}

	free(mg); free(f); free(u);
}


void DrawBodyMesh(FEM *fem, Vector3 *customNodes, double *stresses, Vector3 origin, Color edgeColor, Color vertexColor, BodyDrawOptions opt) {
	const int SEGMENTS_PER_EDGE = 5;

	static const int hexEdges[12][3] = {
		{0, 8, 1},  {1, 9, 2},  {2, 10, 3}, {3, 11, 0}, 
		{4, 16, 5}, {5, 17, 6}, {6, 18, 7}, {7, 19, 4}, 
		{0, 12, 4}, {1, 13, 5}, {2, 14, 6}, {3, 15, 7}  
	};

	// Finding the minimum and maximum stress for the scale
	double minStress = 0.0, maxStress = 1e-6;
	if (stresses) {
		minStress = stresses[0];
		maxStress = stresses[0];
		for (int i = 1; i < fem->numElements; i++) {
			if (stresses[i] < minStress) minStress = stresses[i];
			if (stresses[i] > maxStress) maxStress = stresses[i];
		}
	}

	// --- STAGE 1: Drawing opaque elements (grid, lines, nodes) ---
	for (int el = 0; el < fem->numElements; el++) {
		Color currentEdgeColor = edgeColor;
		if (stresses)
			currentEdgeColor = GetHeatmapColor(stresses[el], minStress, maxStress);

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

						DrawLine3D(pPrev, pCurr, currentEdgeColor);
						pPrev = pCurr;
					}
				}
			}
		}
	}

	// --- STAGE 2: Drawing the translucent faces of the GU ---
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
