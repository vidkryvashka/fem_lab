#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "raylib.h"

// Global integration constants
extern const double gaussianCoords[3];
extern const double gaussianConst[3];
extern const double localPoints3D[20][3];

// Global 3D array for derived shape functions
extern double dfiabg[27][20][3];

// Initialization of form function matrices (called once in main)
void CalculateDFIABG(void);

// Transformation of coordinates taking into account the specifics of the axes (replacement of Y and Z by places)
Vector3 TransformPoint(double p[3], Vector3 origin);

// Function to create a local 20-node finite element (cube)
void CreateCube(double aStart, double aEnd, double bStart, double bEnd, double cStart, double cEnd, double outCube[20][3]);

// Iterative SLIE solver (Conjugate gradient method)
void SolveCG(double *A, double *b, double *x, int n);

extern double dpsite[9][8][2];
extern double dpsiteXYZdeNT[9][8];

void CalculateDPSITE(void);
void CalculateDPsiteXYZdeNT(void);

#endif // MATH_UTILS_H
