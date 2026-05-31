#include "math_utils.h"
#include <math.h>
#include <stdlib.h>


const double gaussianCoords[3] = {-0.7745966692414834, 0.0, 0.7745966692414834};
const double gaussianConst[3]  = {5.0/9.0, 8.0/9.0, 5.0/9.0};

double dfiabg[27][20][3];
double dpsite[9][8][2];
double dpsiteXYZdeNT[9][8];


static const double localPoints2D[8][2] = {
	{-1, -1}, {1, -1}, {1, 1}, {-1, 1},
	{0, -1}, {1, 0}, {0, 1}, {-1, 0}
};


const double localPoints3D[20][3] = {
	{-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1},
	{-1, 1, 1}, {1, 1, 1}, {1, -1, 1}, {-1, -1, 1},
	{0, 1, -1}, {1, 0, -1}, {0, -1, -1}, {-1, 0, -1},
	{-1, 1, 0}, {1, 1, 0}, {1, -1, 0}, {-1, -1, 0},
	{0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1}
};


static void psint14der(double eta, double tau, double x, double y, double out[2]) {
	out[0] = (1.0 / 4.0) * (tau * y + 1.0) * (x * (x * eta + y * tau - 1.0) + x * (x * eta + 1.0));
	out[1] = (1.0 / 4.0) * (x * eta + 1.0) * (y * (x * eta + y * tau - 1.0) + y * (y * tau + 1.0));
}


static void psint57der(double eta, double tau, double x, double y, double out[2]) {
	out[0] = (-tau * y - 1.0) * eta;
	out[1] = (1.0 / 2.0) * (1.0 - eta * eta) * y;
}


static void psint68der(double eta, double tau, double x, double y, double out[2]) {
	out[0] = (1.0 / 2.0) * (1.0 - tau * tau) * x;
	out[1] = (-eta * x - 1.0) * tau;
}


void CalculateDPSITE(void) {
	for (int k1 = 0; k1 < 3; k1++) {
		double eta = gaussianCoords[k1];
		for (int k2 = 0; k2 < 3; k2++) {
			double tau = gaussianCoords[k2];
			int idx = k1 * 3 + k2;
			for (int i = 0; i < 8; i++) {
				if (i < 4) psint14der(eta, tau, localPoints2D[i][0], localPoints2D[i][1], dpsite[idx][i]);
				else if (i == 4 || i == 6) psint57der(eta, tau, localPoints2D[i][0], localPoints2D[i][1], dpsite[idx][i]);
				else if (i == 5 || i == 7) psint68der(eta, tau, localPoints2D[i][0], localPoints2D[i][1], dpsite[idx][i]);
			}
		}
	}
}


void CalculateDPsiteXYZdeNT(void) {
	for (int k1 = 0; k1 < 3; k1++) {
		double eta = gaussianCoords[k1];
		for (int k2 = 0; k2 < 3; k2++) {
			double tau = gaussianCoords[k2];
			int idx = k1 * 3 + k2;
			for (int i = 0; i < 8; i++) {
				if (i < 4) dpsiteXYZdeNT[idx][i] = (1.0/4.0)*(tau*localPoints2D[i][1]+1.0)*(eta*localPoints2D[i][0]+1.0)*(eta*localPoints2D[i][0]+localPoints2D[i][1]*tau-1.0);
				else if (i == 4 || i == 6) dpsiteXYZdeNT[idx][i] = (1.0/2.0)*(-eta*eta+1.0)*(localPoints2D[i][1]*tau+1.0);
				else if (i == 5 || i == 7) dpsiteXYZdeNT[idx][i] = (1.0/2.0)*(-tau*tau+1.0)*(localPoints2D[i][0]*eta+1.0);
			}
		}
	}
}


static double dfiabg18(double alpha, double beta, double gamma, double x, double y, double z) {
	return (1.0 / 8.0) * (1.0 + beta*y) * (1.0 + gamma*z) * (x*(-2.0 + alpha*x + gamma*z + beta*y) + x*(1.0 + alpha*x));
}


static double dfiabg14(double alpha, double beta, double gamma, double alphaI, double betaI, double gammaI) {
	double term1 = -betaI*betaI*gammaI*gammaI*alpha*alpha - beta*beta*gammaI*gammaI*alphaI*alphaI - betaI*betaI*gamma*gamma*alphaI*alphaI + 1.0;
	return (1.0 / 4.0) * (1.0 + beta*betaI) * (1.0 + gamma*gammaI) * (alphaI * term1 - (2.0 * betaI*betaI*gammaI*gammaI*alpha) * (alpha*alphaI + 1.0));
}


void CalculateDFIABG(void) {
	for (int k1 = 0; k1 < 3; k1++) {
		double gamma = gaussianCoords[k1];
		for (int k2 = 0; k2 < 3; k2++) {
			double beta = gaussianCoords[k2];
			for (int k3 = 0; k3 < 3; k3++) {
				double alpha = gaussianCoords[k3];
				int idx = k1 * 9 + k2 * 3 + k3;
				for (int i = 0; i < 20; i++) {
					if (i <= 7) {
						dfiabg[idx][i][0] = dfiabg18(alpha, beta, gamma, localPoints3D[i][0], localPoints3D[i][1], localPoints3D[i][2]);
						dfiabg[idx][i][1] = dfiabg18(beta, alpha, gamma, localPoints3D[i][1], localPoints3D[i][0], localPoints3D[i][2]);
						dfiabg[idx][i][2] = dfiabg18(gamma, beta, alpha, localPoints3D[i][2], localPoints3D[i][1], localPoints3D[i][0]);
					} else {
						dfiabg[idx][i][0] = dfiabg14(alpha, beta, gamma, localPoints3D[i][0], localPoints3D[i][1], localPoints3D[i][2]);
						dfiabg[idx][i][1] = dfiabg14(beta, alpha, gamma, localPoints3D[i][1], localPoints3D[i][0], localPoints3D[i][2]);
						dfiabg[idx][i][2] = dfiabg14(gamma, beta, alpha, localPoints3D[i][2], localPoints3D[i][1], localPoints3D[i][0]);
					}
				}
			}
		}
	}
}


Vector3 TransformPoint(double p[3], Vector3 origin) {
	return (Vector3){
		(float)p[0] - origin.x,
		(float)p[2] - origin.y,
		(float)p[1] - origin.z
	};
}


void CreateCube(double aStart, double aEnd, double bStart, double bEnd, double cStart, double cEnd, double outCube[20][3]) {
	double aSize = aEnd - aStart;
	double bSize = bEnd - bStart;
	double cSize = cEnd - cStart;

	double x[20] = {aStart, aEnd, aEnd, aStart, aStart, aEnd, aEnd, aStart, aStart + aSize/2, aEnd, aStart + aSize/2, aStart, aStart, aEnd, aEnd, aStart, aStart + aSize/2, aEnd, aStart + aSize/2, aStart};
	double y[20] = {bStart, bStart, bEnd, bEnd, bStart, bStart, bEnd, bEnd, bStart, bStart + bSize/2, bEnd, bStart + bSize/2, bStart, bStart, bEnd, bEnd, bStart, bStart + bSize/2, bEnd, bStart + bSize/2};
	double z[20] = {cStart, cStart, cStart, cStart, cEnd, cEnd, cEnd, cEnd, cStart, cStart, cStart, cStart, cStart + cSize/2, cStart + cSize/2, cStart + cSize/2, cStart + cSize/2, cEnd, cEnd, cEnd, cEnd};

	for (int i = 0; i < 20; i++) {
		outCube[i][0] = x[i];
		outCube[i][1] = y[i];
		outCube[i][2] = z[i];
	}
}


void SolveCG(double *A, double *b, double *x, int n) {
	double *r = calloc(n, sizeof(double));
	double *p = calloc(n, sizeof(double));
	double *Ap = calloc(n, sizeof(double));
	
	for (int i = 0; i < n; i++) {
		double Ax = 0;
		for (int j = 0; j < n; j++) Ax += A[i * n + j] * x[j];
		r[i] = b[i] - Ax;
		p[i] = r[i];
	}

	double rsold = 0;
	for (int i = 0; i < n; i++) rsold += r[i] * r[i];

	for (int iter = 0; iter < 500; iter++) {
		if (sqrt(rsold) < 1e-6) break;

		for (int i = 0; i < n; i++) {
			Ap[i] = 0;
			for (int j = 0; j < n; j++) Ap[i] += A[i * n + j] * p[j];
		}

		double pAp = 0;
		for (int i = 0; i < n; i++) pAp += p[i] * Ap[i];
		
		double alpha = rsold / (pAp + 1e-20);

		for (int i = 0; i < n; i++) {
			x[i] += alpha * p[i];
			r[i] -= alpha * Ap[i];
		}

		double rsnew = 0;
		for (int i = 0; i < n; i++) rsnew += r[i] * r[i];
		for (int i = 0; i < n; i++) p[i] = r[i] + (rsnew / (rsold + 1e-20)) * p[i];
		rsold = rsnew;
	}

	free(r); free(p); free(Ap);
}
