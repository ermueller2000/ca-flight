#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include"interpolate.h"


#define VERBOSE 0
#define READSTATELOCATION 0   // 0 is a canned state, 1 is from a file, 2 is from MAVLink
#define EXTRAPMODE 0      // Use 0 for the interpolation function to use nearest neighbor 
                //  beyond the grid

int readState(double *currentState, int numDims)
// States are [rx, ry, vxo, vyo, vxi, vyi, dx, dy]
{

  if (READSTATELOCATION == 0)
  {
    if (numDims==8)
    {
      currentState[0] = 2;  // rx
      currentState[1] = 10; // ry
      currentState[2] = 0;  // vxo
      currentState[3] = 2;  // vyo
      currentState[4] = 0;  // vxi
      currentState[5] = 0;  // vyi
      currentState[6] = 0;  // dx
      currentState[7] = 0;  // dy
    }

    if (numDims==3)
    {
      currentState[0] = 21.1; // rx
      currentState[1] = 21.2; // ry
      currentState[2] = 21; // vxo
    }
  }

  // Read from file
  if (READSTATELOCATION == 1)
  {

  }


  // Read from MAVLink/autopilot
  if (READSTATELOCATION == 2)
  {

  }
  return 0;

}

int writeAction(int actionInd, FILE *fpOut)
// Sends command corresponding to actionInd to autopilot/MAVLink.  
// Actions are the following:
// 0  nothing
// 1  -x
// 2  +x
// 3  -y
// 4  +y
{

switch (actionInd)
  {
    case 0:
      // Send command for zero acceleration
      
    break;

    case 1:
      // Send command for -x acceleration
      
    break;

    case 2:
      // Send command for +x acceleration
      
    break;

    case 3:
      // Send command for -y acceleration
      
    break;

    case 4:
      // Send command for +y acceleration
      
    break;

    default:
      printf("Non supported action\n");
      return 1;
  }

  fprintf(fpOut, "%d\n", actionInd);

  return 0;

}

int writeLogs(FILE *fpLog, int stepCounter, double* currentState, int numDims, int actionInd)
// Writes key states to a log:
// [currentStep, commanded action, states[0:N]]
{
  int i;

  fprintf(fpLog, "%d, ", stepCounter);
  fprintf(fpLog, "%d, ", actionInd);
  for (i=0; i<numDims-1; i++) {
    fprintf(fpLog, "%lf, ", currentState[i]);
  }
  fprintf(fpLog, "%lf\n", currentState[numDims-1]);

  return 0;
}

int main()
{
  /***********************************************************
  ***   The following would be in start_Algorithm()        ***
  ************************************************************/

  // Needed only in start_Algorithm():
  FILE *fpIn, *fpData;
  int numVerticies; 

  // Needed in  decide_Algorithm() but allocated or evaluated 
  // in start_Algorithm():
  int numDims, numActions, *elementsDim;
  FILE *fpOut, *fpLog;
  double *neighY;
  double **discMat, **neighX;
  struct cd_grid ***gridQsa;  // This should hold a vector of grids, one for each action

  // Variables simply used in the algorithm functions, may be redeclared in each function:
  int i, j, k;
  int err, keyinput, exitCondition;

  /* Open and check the files */
  fpIn  = fopen("test.txt","r");
  fpData  = fopen("data.txt","r");
  fpOut = fopen("testOut.txt","w");
  fpLog = fopen("testLog.txt","w");

  /* Check for errors opening files */
  if (fpIn == NULL) {
    fprintf(stderr, "Can't open input file\n");
    exit(1);
  }
  if (fpData == NULL) {
    fprintf(stderr, "Can't open data file\n");
    exit(1);
  }
  if (fpOut == NULL) {
    fprintf(stderr, "Can't open output file\n");
    exit(1);
  }
  if (fpLog == NULL) {
    fprintf(stderr, "Can't open log file\n");
    exit(1);
  }

  /* Read in parameter file */

  // Number of dimensions:
  fscanf(fpIn, "%d", &numDims);

  // Number of grid points (states) in each dimension:
  //numElements = 1;
  elementsDim = (int *)malloc(numDims*sizeof(int));
  discMat = (double **)malloc(numDims*sizeof(double));
  for (i=0; i<numDims; i++) {
    fscanf(fpIn, "%d", &elementsDim[i]);
    //printf("Dimension %d = %d\n", i, elementsDim[i]);
    //numElements *= elementsDim[i];
    discMat[i] = (double *)malloc(elementsDim[i]*sizeof(double));
  }

  // Values of states at each grid point:
  for (i=0; i<numDims; i++) {
    for (j=0; j<elementsDim[i]; j++) {
      fscanf(fpIn, "%lf", &discMat[i][j]);
    }
  }

  // Number of actions:
  fscanf(fpIn, "%d", &numActions);

  fclose(fpIn);
  /* Finished reading parameter file

  /* Create the grid data structure to hold contents as specified in the parameter file */
  //cell_init = (void *)malloc(numElements*sizeof(double));
  // gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
  gridQsa = (struct cd_grid ***)malloc(numActions*sizeof(struct cd_grid));
  if (numDims == 3)
  {
    // **** Need to make these calls to separate functions (perhaps in another utility file?)
    double Q[elementsDim[0]][elementsDim[1]][elementsDim[2]];
    struct cd_grid **gridInst;
    int actionInd;

    /* Using the contents of the parameter file, read and parse the data file */  
    /* Currently sets a max size for Q, should eventually malloc a multi-dimensional Q 
       that is exactly the size defined by elementsDim. Or read directly into grid structure
       For now will just assume it's 3D */  

    for (actionInd=0; actionInd<numActions; actionInd++)
    {
      gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
      for (i=0; i<elementsDim[0]; i++) {
        for (j=0; j<elementsDim[1]; j++) {
          for (k=0; k<elementsDim[2]; k++) {
            fscanf(fpData, "%lf", &Q[i][j][k]);
            if (VERBOSE) {
              printf("Q[%d,%d,%d] = %lf\n", i,j,k,Q[i][j][k]);
            }
            //cd_grid_get_index(struct cd_grid * g, size_t index)
          }
          //printf("%f, ", discMat[i][j]);
        }
        //printf("]\n");
      }
      //printf("Done loading data\n");

      // After the number of dimensions, the size of each dimension must be passed in as a series of arguments.
      cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, elementsDim[0], elementsDim[1], elementsDim[2]);
      gridQsa[actionInd] = gridInst;
    }
    //printf("Grid constructed\n");
  }
  fclose(fpData);

  // Allocate space for the interpolations needed to decide actions:
  numVerticies = pow(2,numDims);
  neighX = malloc(numDims*sizeof(double *));
  if (neighX) {
    for (i=0;i<numDims;i++) {
      neighX[i]=(double *)malloc(numVerticies*sizeof(double));
    }
  }
  neighY = (double *)malloc(numVerticies*sizeof(double)); 

  // Prepare keyboard input, if necessary:
  // init_keyboard();    // Only need this in testing, not flight hardware

  /***********************************************************
  ***   The following would be in decide_Algorithm()       ***
  ************************************************************/
  int bestActionInd;
  double maxQsa;
  int stepCounter;
  double *qVals, *currentState, *indicies;

  // These allocations may be done on every call to decide_Algorithm(), they're not large
  qVals = (double *)malloc(numActions*sizeof(double));
  currentState = (double *)malloc(numDims*sizeof(double));
  indicies = (double *)malloc(numDims*sizeof(double));
  exitCondition = 0;
  stepCounter = 0;
  while (!exitCondition)
  {

    // read the current state from MAVLink (or a file, for now)
    err = readState(currentState, numDims);

    // find indicies into the grid using the current state
    err = getIndicies(indicies, currentState, numDims, elementsDim, discMat);

    // for each action:
    int actionInd;
    for (actionInd=0; actionInd<numActions; actionInd++) {
      // read in the neighbor verticies to the data structure and read in the values at the verticies
      err = getNeighbors(indicies, gridQsa[actionInd], neighX, neighY, numDims, discMat);
    
      // interpolate within the data file using the current state (call interpN()) and store
      qVals[actionInd] = interpN(numDims, currentState, neighX, neighY, EXTRAPMODE);
    }

    // calculate the best action
    bestActionInd = 0;
      maxQsa = qVals[0];
    for (i=1;i<numActions;i++) {
      if (qVals[i] > maxQsa) {
        maxQsa = qVals[i];
        bestActionInd = i;
      }
    }


    // write the action to MAVLink (or a file, for now)
    err = writeAction(bestActionInd, fpOut);

    // log the current state and action to a log file
    err = writeLogs(fpLog, stepCounter, currentState, numDims, bestActionInd);

    // Increment the counter (proxy for the current time when writing to file)
    stepCounter++;

    // Only need the following if we're quitting this loop with a keystroke 
    // (not in flight software)
    // if (kbhit()) {
    //   if (readch()=='q')
    //     exitCondition = 1;
    // }

    // While debugging, stop after a predetermined number of steps:
    if (stepCounter>=10)
      exitCondition = 1;  
  }
  free(currentState);
  free(qVals);
  free(indicies);

  /***********************************************************
  ***   The following would be in stop_Algorithm()         ***
  ************************************************************/

  //close_keyboard();  // This is only necessary if we're using kbhit()

  // Free allocated memory
  for (i=0;i<numActions;i++) {
    cd_grid_destroy(*gridQsa[i]);
  }
  free(gridQsa);
  for (i=0; i<numDims; i++) {
    free(neighX[i]);
    free(discMat[i]);
  }
  free(discMat);
  free(elementsDim);
  free(neighX);
  free(neighY);
  

  /* close the log and output files */
  fclose(fpOut);
  fclose(fpLog);
  printf("Exiting\n");

return 0;
}

/* Fractional index and neighbor lookup test code:

    printf("Indicies:\n");
    printf("[");
    for (i=0;i<numDims;i++) {
      printf("%lf, ", indicies[i]);
    }
    printf("]\n");

    printf("Neighbors:\n");
    printf("[");
    for (i=0;i<numDims;i++) {
      for(j=0;j<numVerticies; j++){
        printf("%lf, ", neighX[i][j]);
      }
      printf("\n");
    }

    printf("Neighbor values:\n");
    printf("[");
    for (i=0;i<numVerticies;i++) {
      printf("%lf, ", neighY[i]);
    }
    printf("]\n");

*/


// Code to check interpolation function (needs adaptation to own interpolation function)
/*

// Check whether things are being stored correctly:
  
  float ii, jj;
  // Now do a test interpolation
  p = (double *)malloc(numElements*sizeof(double));
  valuep = (double *)malloc(sizeof(double));
  p[0] = 0.55;
  p[1] = 0.68;
  p[2] = 0.0;
  printf("Built interp structures\n");

  size_t * index;
  double *value;
  g = *gridInst;
  index = (size_t *) malloc((g->n+1) * sizeof(size_t));
  cd_grid_lookup_index(*gridInst, p, index);
  value = (double *)(g->data + (*index)*g->cell_size);
  printf("%lf\n", *value);

  double interpMat[11][11];
  int indX, indY;
  indX = 1;
  err = 0;
  for (ii=0; ii<=1; ii+=0.2) {
    indY = 1;
    printf("[");
    for (jj=0; jj<=1; jj+=0.2)  {
      // err = cd_grid_double_interp(*gridInst, p, valuep);
      p[0] = ii;
      p[1] = jj;
      err = cd_grid_double_interp(*gridInst, p, &interpMat[indX][indY]);
      // Need to get the following working:
      // double interpN(int N, double *x, double **neighX, double *neighY)
      if (err)
        printf("Got error: %d", err);
      printf("%lf, ", interpMat[indX][indY]);
      // printf("%d\n", jj);
      indY++;
    }
    printf("]\n");
    indX++;
  }

  if (err)
    printf("Error in interpolation\n");

*/

  // The following code is for realtime checking of interpolation functions:
/*
  if (VERBOSE) {
        printf("Indicies:\n");
        printf("[");
        for (i=0;i<numDims;i++) {
          printf("%lf, ", indicies[i]);
        }
        printf("]\n");

        printf("Neighbors:\n");
        printf("[");
        for (i=0;i<numDims;i++) {
          for(j=0;j<numVerticies; j++){
            printf("%lf, ", neighX[i][j]);
          }
          printf("\n");
        }

        printf("Neighbor values:\n");
        printf("[");
        for (i=0;i<numVerticies;i++) {
          printf("%lf, ", neighY[i]);
        }
        printf("]\n");

        printf("Interpolated Value: %lf\n", qVals[actionInd]);
      }
      */

  // cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep)
  //free(p);
  //free(valuep);
  //free(cell_init);