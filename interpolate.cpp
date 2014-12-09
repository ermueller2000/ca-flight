/* Multilinear interpolation function to support collision avoidance flight testing
   Eric Mueller
   11/5/14
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "interpolate.h"

int ind2subs(int *dims, int numDims, int index, int *subs)
// Given a set of dimensions (dims), the number of dimensions, and an index, returns
// the subscripts corresponding to the index in subs.
{
  int i, vi, vj;
  int *cumprod;

  cumprod = (int *)malloc(numDims*sizeof(int));

  cumprod[0] = 1;
  //printf("CumProd = [%d ", cumprod[0]);
  for (i=1; i<numDims; i++) {
    cumprod[i] = cumprod[i-1] * dims[i-1];
    //printf("%d ", cumprod[i]);
  }
  //printf("]\n");

  for (i=numDims-1;i>=0;i--){
    vi = index % cumprod[i];
    //printf("vi=%d\n", vi);
    vj = (index-vi)/cumprod[i];
    //printf("vj=%d\n",vj);
    subs[i] = vj;
    //printf("subs[i]=%d\n", subs[i]);
    index = vi;
    //printf("index=%d\n", index);
  }
//printf("here\n");
  return 0;
}

double interpN(int N, double *xInit, double **neighX, double *neighY, int EXTRAPMODE)
{
// Given an N-dimensional vector, x, 
// its 2^N neighbors (each of dimension N, so N by 2^N) neighX, 
// and the neighbors' values (length 2^N) neighY, 
// return the value of  the function at x.
// EXTRAPMODE allows the user to determine how to handle out-of-index interpolation values:
//    0 returns the nearest neighbor value (i.e. x[i]>max(neighX[i,:]) uses max(neighX[i,:]) )  
//      1 allows each dimension to linearly extrapolate beyond the edge of neighX 
//        (though the functionality hasn't been verified)

  double maxvec[N], minvec[N], vecfar[N], vecnear[N];
  int i, j, numVerticies;
  double denNorm, y, coefnum;
  double *x;

  x = (double *)malloc(N*sizeof(double));
  for(i=0;i<N;i++){
	  x[i]=xInit[i];
  }

  numVerticies = pow(2,N);
  x = (double *)malloc(N*sizeof(double));
  for (i=0;i<N;i++) {
    x[i] = xInit[i];
  }

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

  // Clamp values of x if EXTRAPMODE==0
  if (EXTRAPMODE == 0) {
    for (i=0;i<N;i++) {
      if (x[i] > maxvec[i])
        x[i] = maxvec[i];
      if (x[i] < minvec[i])
          x[i] = minvec[i];
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
  free(x);
  return y;

}

double getFracIndex(double vararray[MAXDISC], double value, int numElem)
// Returns the fractional index of value within array of values vararray[]
{
  int i;

  if (value <= vararray[0]) {
    return 0;
  }

  if (value >= vararray[numElem-1]) {
    // We return numElem-2 because we want the neighbor lookup function to examine the
    // two grid points bracketing the current value. If our current value is greater than
    // or equal to the max value, we use the highest two grid points for the interpolation.
    return (numElem-2);
  }

  i=0;
  while (value > vararray[i+1]) {
    i++;
  }

  return (i+(value-vararray[i])/(vararray[i+1] - vararray[i]));
}

int getIndicies(double *indicies, double *currentState, int numDims, int *elementsDim, double **discMat)
// Returns a fractional index of the current state
{
  int i, j;
  double discVec[MAXDISC];

  for (i=0; i<numDims; i++)
  {
    for (j=0; j<elementsDim[i]; j++) {
      discVec[j] = discMat[i][j];
    }

    indicies[i] = getFracIndex(discVec, currentState[i], elementsDim[i]);
  }

  return 0;
}

int getNeighbors(double *indicies, struct cd_grid **gridInst, double **neighX, double *neighY, int numDims, double **discMat)
// Returns the state values of each neighbor of "indicies" and the value of Q(s,a) at that grid point:
{
  int i,j,k,l,m,n,o,p;
  int vertexInd;
  int subs[numDims];

  vertexInd = 0;
  switch (numDims)
  {
    case 3:
      for (i=floor(indicies[0]); i<=floor(indicies[0])+1; i++) {
        subs[0] = i;
        for (j=floor(indicies[1]); j<=floor(indicies[1])+1; j++) {
          subs[1] = j;
          for (k=floor(indicies[2]); k<=floor(indicies[2])+1; k++)
          {
            subs[2] = k;
            neighX[0][vertexInd] = discMat[0][i];
            neighX[1][vertexInd] = discMat[1][j];
            neighX[2][vertexInd] = discMat[2][k];
            neighY[vertexInd] = *(double *)cd_grid_get_subs(*gridInst, subs);
            vertexInd++;
          }
        }
      }
      
    break;

    case 6:
      for (i=floor(indicies[0]); i<=floor(indicies[0])+1; i++) {
        subs[0] = i;
        for (j=floor(indicies[1]); j<=floor(indicies[1])+1; j++) {
          subs[1] = j;
          for (k=floor(indicies[2]); k<=floor(indicies[2])+1; k++) {
            subs[2] = k;
            for (l=floor(indicies[3]); l<=floor(indicies[3])+1; l++) {
              subs[3] = l;
              for (m=floor(indicies[4]); m<=floor(indicies[4])+1; m++) {
                subs[4] = m;
                for (n=floor(indicies[5]); n<=floor(indicies[5])+1; n++)
                {
                  subs[5] = n;
                  neighX[0][vertexInd] = discMat[0][i];
                  neighX[1][vertexInd] = discMat[1][j];
                  neighX[2][vertexInd] = discMat[2][k];
                  neighX[3][vertexInd] = discMat[3][l];
                  neighX[4][vertexInd] = discMat[4][m];
                  neighX[5][vertexInd] = discMat[5][n];
                  neighY[vertexInd] = *(double *)cd_grid_get_subs(*gridInst, subs);
                  vertexInd++;
                }
              }
            }
          }
        }
      }
      
    break;

    case 8:
      for (i=floor(indicies[0]); i<=floor(indicies[0])+1; i++) {
        subs[0] = i;
        for (j=floor(indicies[1]); j<=floor(indicies[1])+1; j++) {
          subs[1] = j;
          for (k=floor(indicies[2]); k<=floor(indicies[2])+1; k++) {
            subs[2] = k;
            for (l=floor(indicies[3]); l<=floor(indicies[3])+1; l++) {
              subs[3] = l;
              for (m=floor(indicies[4]); m<=floor(indicies[4])+1; m++) {
                subs[4] = m;
                for (n=floor(indicies[5]); n<=floor(indicies[5])+1; n++) {
                  subs[5] = n;
                  for (o=floor(indicies[6]); o<=floor(indicies[6])+1; o++) {
                    subs[6] = o;
                    for (p=floor(indicies[7]); p<=floor(indicies[7])+1; p++)
                    {
                      subs[7] = p;
                      neighX[0][vertexInd] = discMat[0][i];
                      neighX[1][vertexInd] = discMat[1][j];
                      neighX[2][vertexInd] = discMat[2][k];
                      neighX[3][vertexInd] = discMat[3][l];
                      neighX[4][vertexInd] = discMat[4][m];
                      neighX[5][vertexInd] = discMat[5][n];
                      neighX[6][vertexInd] = discMat[6][o];
                      neighX[7][vertexInd] = discMat[7][p];
                      neighY[vertexInd] = *(double *)cd_grid_get_subs(*gridInst, subs);
                      vertexInd++;
                    }
                  }
                }
              }
            }
          }
        }
      }
      
    break;


    default:
      printf("Non supported number of dimensions\n");
      return 1;
  }

  return 0;

}

void test_ind2sub(void)
{
  int *dims,  *subs; 
  int numDims, index, numElements, i;

  numDims = 3;
  dims = (int *)malloc(numDims*sizeof(int));
  subs = (int *)malloc(numDims*sizeof(int));

  dims[0] = 2;
  dims[1] = 3;
  dims[2] = 4;

  numElements = 1;
  for (index=0; index<numDims; index++)
    numElements *= dims[index];

  for (index=0;index<numElements;index++) {
    ind2subs(dims, numDims, index, subs);
    printf("Index %d = [", index);
    for (i=0; i<numDims; i++)
      printf("%d, ", subs[i]);
    printf("]\n");
  }

}

// Testing code:

/*
  printf("VecFar:\n");
  printf("[");
  for (i=0;i<N;i++) {
    printf("%lf, ", vecfar[i]);
  }
  printf("]\n");
  printf("MaxVec:\n");
  printf("[");
  for (i=0;i<N;i++) {
    printf("%lf, ", maxvec[i]);
  }
  printf("]\n");
  */
