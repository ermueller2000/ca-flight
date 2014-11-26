/* Multilinear interpolation function to support collision avoidance flight testing
   Eric Mueller
   11/5/14
*/

#include <stdlib.h>

struct InterpGrid
{
   /* Dimensionality of space */
   int n;
   /* Grid parameters */
   int * sizes;
   size_t ncells;
   /* The actual data */
   int cell_size;
   char * data;
   /* Actual grid side lengths (1x1x1x... by default) */
   double * lengths;
};

//int gridCreate(struct InterpGrid ** gp, void * cell_init, int cell_size, int n, ...);

double interpN(int N, double *x, double **neighX, double *neighY);