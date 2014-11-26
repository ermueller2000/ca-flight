/* Multilinear interpolation function to support collision avoidance flight testing
   Eric Mueller
   11/5/14
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "interpolate.h"

double interpN(int N, double *x, double **neighX, double *neighY)
{
// Given an N-dimensional vector, x, 
// its 2^N neighbors (each of dimension N, so N by 2^N) neighX, 
// and the neighbors' values (length 2^N) neighY, 
// return the value of  the function at x.

	double maxvec[N], minvec[N], vecfar[N], vecnear[N];
	int i, j, numVerticies;
	double denNorm, y, coefnum;

	numVerticies = pow(2,N);

	// Find the maximum and minimum values of the neighbor x coordinates:
	// I probably don't need to look at every vertex since there should only
	// be two values in each dimension (assuming we're doing "square" interpolation)
	for (i=0;i<N;i++) {
		maxvec[i] = neighX[i][0];
		minvec[i] = neighX[i][0];
		for (j=1;j<numVerticies;j++) {
			if (neighX[i][j]>maxvec[i])
				maxvec[i] = neighX[i][j];
			if (neighX[i][j]<minvec[i])
				minvec[i] = neighX[i][j];
		}
	}

	for (i=0;i<N;i++) {
		vecfar[i] = maxvec[i]-x[i];
		vecnear[i] = x[i]-minvec[i];
	}
	
	denNorm=1;
	for (i=0;i<N;i++) {
		denNorm *= maxvec[i]-minvec[i];  // The normalization factor
	}

	// Calculate the coefficients
	y=0;
	for (i=0;i<numVerticies;i++) {
		coefnum = 1;
		for (j=0; j<N; j++) {
			if (neighX[j][i] == maxvec[j]) 
			{
				coefnum *= vecnear[j];
			}
			else
			{
				coefnum *= vecfar[j];
			}
		}
		// Accumulate terms as we go:
		y += neighY[i]*coefnum;
	}

	// Normalize the result
	y /= denNorm;

	return y;

}