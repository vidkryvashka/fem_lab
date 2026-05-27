#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "raylib.h"

// Глобальні константи інтегрування
extern const double gaussianCoords[3];
extern const double gaussianConst[3];
extern const double localPoints3D[20][3];

// Глобальний тривимірний масив для похідних функцій форми
extern double dfiabg[27][20][3];

// Ініціалізація матриць функцій форми (викликається один раз в main)
void CalculateDFIABG(void);

// Трансформація координат з урахуванням специфіки осей (заміна Y та Z місцями)
Vector3 TransformPoint(double p[3], Vector3 origin);

// Функція створення локального 20-вузлового скінченного елемента (куба)
void CreateCube(double aStart, double aEnd, double bStart, double bEnd, double cStart, double cEnd, double outCube[20][3]);

// Ітераційний СЛАР солвер (Метод спряжених градієнтів)
void SolveCG(double *A, double *b, double *x, int n);

#endif // MATH_UTILS_H
