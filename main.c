#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include"grid.h"
#include"interpolate.h"

// grid module was sourced from https://github.com/personalrobotics/or_cdchomp/

// How grids are defined in grid.h
// struct cd_grid
// {
// /* Dimensionality of space */
// int n;
// /* Grid parameters */
// int * sizes;
// size_t ncells;
//  /* The actual data */
// int cell_size;
// char * data;
// /* Actual grid side lengths (1x1x1x... by default) */
// double * lengths;
// };


#define MAXDIMS	13
#define MAXDISC	20
#define READSTATELOCATION 0   // 0 is a canned state, 1 is from a file, 2 is from MAVLink

int readState(double *currentState, int numDims)
// States are [rx, ry, vxo, vyo, vxi, vyi, dx, dy]
{

	if (READSTATELOCATION == 0)
	{
		if (numDims==8)
		{
			currentState[0] = 2;	// rx
			currentState[1] = 10;	// ry
			currentState[2] = 0;	// vxo
			currentState[3] = 2;	// vyo
			currentState[4] = 0;	// vxi
			currentState[5] = 0;	// vyi
			currentState[6] = 0;	// dx
			currentState[7] = 0;	// dy
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

double getFracIndex(double vararray[MAXDISC], double value, int numElem)
{


	return 0;
}

int getIndicies(double *indicies, double *currentState, int numDims, int *elementsDim, double discMat[MAXDIMS][MAXDISC])
// Returns a fractional index of the current state
{
	int i;
	double discVec[MAXDISC];

	for (i=0; i<numDims; i++)
	{
		discVec = discMat[i];
		indicies[i] = getFracIndex();
	}

	return 0;
}


int main()
{
	FILE *fpIn, *fpData, *fpOut, *fpLog;
	int numDims, elementsDim[MAXDIMS]; 
	double discMat[MAXDIMS][MAXDISC];
	int i, j, k;
	float ii, jj;
	struct cd_grid **gridInst;
	struct cd_grid *g;
	void *cell_init;
	int 	numElements, out;
	//double *p;	// The array of indicies to interpolate on, I believe
	//double *valuep;	// Just the output, I believe, of the interpolation
	int err;
	int keyinput, exitCondition;
	double *x, *neighX, *neighY;
	double *currentState, *indicies;

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
	fscanf(fpIn, "%d", &numDims);
	//printf("Dimensions = %d\n", numDims);

	numElements = 1;
	for (i=0; i<numDims; i++) {
		fscanf(fpIn, "%d", &elementsDim[i]);
		//printf("Dimension %d = %d\n", i, elementsDim[i]);
		numElements *= elementsDim[i];
	}
	//printf("Num Elements = %d\n", numElements);
	
	for (i=0; i<numDims; i++) {
		//printf("Dimension %d: [", i);
		for (j=0; j<elementsDim[i]; j++) {
			fscanf(fpIn, "%lf", &discMat[i][j]);
			//printf("%f, ", discMat[i][j]);
		}
		//printf("]\n");
	}
	fclose(fpIn);

	/* Create the grid data structure to hold contents as specified in the parameter file */
	cell_init = (void *)malloc(numElements*sizeof(double));
	gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
	if (numDims == 3)
	{
		// **** Need to make these calls to separate functions (perhaps in another utility file?)
		double Q[elementsDim[0]][elementsDim[1]][elementsDim[2]];
		/* Using the contents of the parameter file, read and parse the data file */	
		/* Currently sets a max size for Q, should eventually malloc a multi-dimensional Q 
		   that is exactly the size defined by elementsDim. Or read directly into grid structure
		   For now will just assume it's 3D */	
		for (i=0; i<elementsDim[0]; i++) {
			for (j=0; j<elementsDim[1]; j++) {
				for (k=0; k<elementsDim[2]; k++) {
					fscanf(fpData, "%lf", &Q[i][j][k]);
					printf("Q[%d,%d,%d] = %lf\n", i,j,k,Q[i][j][k]);
					//cd_grid_get_index(struct cd_grid * g, size_t index)
				}
				//printf("%f, ", discMat[i][j]);
			}
			//printf("]\n");
		}
		//printf("Done loading data\n");

	 	// After the number of dimensions, the size of each dimension must be passed in as a series of arguments.
		cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, elementsDim[0], elementsDim[1], elementsDim[2]);
		//printf("Grid constructed\n");
	}
	fclose(fpData);

	// double interpN(int N, double *x, double **neighX, double *neighY)
	// Given an N-dimensional vector, x, 
	// its 2^N neighbors (each of dimension N, so N by 2^N) neighX, 
	// and the neighbors' values (length 2^N) neighY, 
	// return the value of  the function at x.
	// Allocate space for the interpolations:
	x = (double *)malloc(numDims*sizeof(double));
	neighX = (double *)malloc(numDims*pow(2, numDims)*sizeof(double));
	neighY = (double *)malloc(pow(2, numDims)*sizeof(double));
	currentState = (double *)malloc(numDims*sizeof(double));
	indicies = (double *)malloc(numDims*sizeof(double));

	
	

	// Run flight loop:
	init_keyboard();
	exitCondition = 0;
	while (!exitCondition)
	{

		// read the current state from MAVLink (or a file, for now)
		err = readState(currentState, numDims);

		// find indicies into the grid using the current state
		err = getIndicies(indicies, currentState, numDims, elementsDim, discMat);

		// for each action (one grid per action?):
			// read in the neighbor verticies to the data structure
			// read in the values at the verticies
			// interpolate within the data file using the current state (call interpN())
			// store the value of Q(s,a)

		// calculate the best action

		// write the action to MAVLink (or a file, for now)

		// log the current state and action to a log file


		if (kbhit()) {
			if (readch()=='q')
				exitCondition = 1;
		}

		// Always exit, while debugging
		exitCondition = 1;
	}
	close_keyboard();

/*

// Check whether things are being stored correctly:

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
		for (jj=0; jj<=1; jj+=0.2)	{
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

	// cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep)
	//free(p);
	//free(valuep);
	free(cell_init);
	free(gridInst);
	/* close the log and output files */
	fclose(fpOut);
	fclose(fpLog);
	printf("Exiting\n");
return 0;
}


	// printf("Waiting for keyboard input\n");

	// while( !kbhit()) 	{

	// }
	// printf("Key read = %c\n", readch());

// Old version:
// int main()
// {
// 	FILE *fpIn, *fpData, *fpOut, *fpLog;
// 	int numDims, elementsDim[MAXDIMS]; 
// 	float discMat[MAXDIMS][MAXDISC];
// 	int i, j, k;
// 	float ii, jj;
// 	struct cd_grid **gridInst;
// 	struct cd_grid *g;
// 	void *cell_init;
// 	int 	numElements, out;
// 	double *p;	// The array of indicies to interpolate on, I believe
// 	double *valuep;	// Just the output, I believe, of the interpolation
// 	int err;

// 	/* Open and check the files */
// 	fpIn  = fopen("test.txt","r");
// 	fpData  = fopen("data.txt","r");
// 	fpOut = fopen("testOut.txt","w");
// 	fpLog = fopen("testLog.txt","w");

// 	/* Check for errors opening files */
// 	if (fpIn == NULL) {
// 	  fprintf(stderr, "Can't open input file\n");
// 	  exit(1);
// 	}
// 	if (fpData == NULL) {
// 	  fprintf(stderr, "Can't open data file\n");
// 	  exit(1);
// 	}
// 	if (fpOut == NULL) {
// 	  fprintf(stderr, "Can't open output file\n");
// 	  exit(1);
// 	}
// 	if (fpLog == NULL) {
// 	  fprintf(stderr, "Can't open log file\n");
// 	  exit(1);
// 	}

// 	/* Read in parameter file */
// 	fscanf(fpIn, "%d", &numDims);
// 	//printf("Dimensions = %d\n", numDims);

// 	numElements = 1;
// 	for (i=0; i<numDims; i++) {
// 		fscanf(fpIn, "%d", &elementsDim[i]);
// 		//printf("Dimension %d = %d\n", i, elementsDim[i]);
// 		numElements *= elementsDim[i];
// 	}
// 	//printf("Num Elements = %d\n", numElements);
	
// 	for (i=0; i<numDims; i++) {
// 		//printf("Dimension %d: [", i);
// 		for (j=0; j<elementsDim[i]; j++) {
// 			fscanf(fpIn, "%f", &discMat[i][j]);
// 			//printf("%f, ", discMat[i][j]);
// 		}
// 		//printf("]\n");
// 	}

// 	/* Using the contents of the parameter file, read and parse the data file */
// 	/* Currently sets a max size for Q, should eventually malloc a multi-dimensional Q 
// 	   that is exactly the size defined by elementsDim 
// 	   For now will just assume it's 3D */
	
// 	double Q[2][3][2];
// 	for (i=0; i<elementsDim[0]; i++) {
// 		for (j=0; j<elementsDim[1]; j++) {
// 			for (k=0; k<elementsDim[2]; k++) {
// 				fscanf(fpData, "%lf", &Q[i][j][k]);
// 				printf("Q[%d,%d,%d] = %lf\n", i,j,k,Q[i][j][k]);
// 				//cd_grid_get_index(struct cd_grid * g, size_t index)
// 			}
// 			//printf("%f, ", discMat[i][j]);
// 		}
// 		//printf("]\n");
// 	}
// 	//printf("Done loading data\n");
// 	cell_init = (void *)malloc(numElements*sizeof(double));
// 	//cell_init = (void *)malloc(sizeof(double));
// 	gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
//  	// After the number of dimensions, the size of each dimension must be passed in as a series of arguments.
// 	cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, 2, 3, 2);
// 	//printf("Grid constructed\n");

// 	// Check whether things are being stored correctly:
	

// 	// Now do a test interpolation
// 	p = (double *)malloc(numElements*sizeof(double));
// 	valuep = (double *)malloc(sizeof(double));
// 	p[0] = 0.55;
// 	p[1] = 0.68;
// 	p[2] = 0.0;
// 	printf("Built interp structures\n");

// 	size_t * index;
// 	double *value;
// 	g = *gridInst;
// 	index = (size_t *) malloc((g->n+1) * sizeof(size_t));
// 	cd_grid_lookup_index(*gridInst, p, index);
// 	value = (double *)(g->data + (*index)*g->cell_size);
// 	printf("%lf\n", *value);

// 	double interpMat[11][11];
// 	int indX, indY;
// 	indX = 1;
// 	err = 0;
// 	for (ii=0; ii<=1; ii+=0.2) {
// 		indY = 1;
// 		printf("[");
// 		for (jj=0; jj<=1; jj+=0.2)	{
// 			// err = cd_grid_double_interp(*gridInst, p, valuep);
// 			p[0] = ii;
// 			p[1] = jj;
// 			err = cd_grid_double_interp(*gridInst, p, &interpMat[indX][indY]);
// 			// Need to get the following working:
// 			// double interpN(int N, double *x, double **neighX, double *neighY)
// 			if (err)
// 				printf("Got error: %d", err);
// 			printf("%lf, ", interpMat[indX][indY]);
// 			// printf("%d\n", jj);
// 			indY++;
// 		}
// 		printf("]\n");
// 		indX++;
// 	}

// 	if (err)
// 		printf("Error in interpolation\n");

// 	// cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep)
// 	free(p);
// 	free(valuep);
// 	free(cell_init);
// 	free(gridInst);
// 	/* while !end condition
// 		read the current state from MAVLink (or a file, for now)

// 		interpolate within the data file using the current state

// if (ifp == NULL) {
//   fprintf(stderr, "Can't open input file in.list!\n");
//   exit(1);
// }
// 		calculate the best action

// 		write the action to MAVLink (or a file, for now)

// 		log the current state and action to a log file
// 		*/

// 		/* close the log and output files */
// 	fclose(fpIn);
// 	fclose(fpData);
// 	fclose(fpOut);
// 	fclose(fpLog);
// return 0;
// }