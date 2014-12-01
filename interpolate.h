/* Multilinear interpolation function to support collision avoidance flight testing
   Eric Mueller
   11/5/14
*/

// grid module was sourced from https://github.com/personalrobotics/or_cdchomp/

#include <stdlib.h>
#include"grid.h"

#define MAXDISC 40  // Used as the maximum length of the fractional index lookup

int ind2subs(int *dims, int numDims, int index, int *subs);

double interpN(int N, double *x, double **neighX, double *neighY, int EXTRAPMODE);

// Returns the fractional index of value within array of values vararray[]
double getFracIndex(double vararray[MAXDISC], double value, int numElem);

// Returns a fractional index of the current state
int getIndicies(double *indicies, double *currentState, int numDims, int *elementsDim, double **discMat);

// Returns the state values of each neighbor of "indicies" and the value of Q(s,a) at that grid point:
int getNeighbors(double *indicies, struct cd_grid **gridInst, double **neighX, double *neighY, int numDims, double **discMat);

// Test program
void test_ind2sub(void);
