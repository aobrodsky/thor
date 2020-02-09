#include <math.h>
#include <stdlib.h>
#include <unistd.h>

/* This code is based on the Java Random.nextGaussian() code (Java 7), which
 * uses the polar method of G. E. P. Box, M. E. Muller, and G. Marsaglia.
 * This is described by Donald E. Knuth in The Art of Computer Programming, 
 * Volume 3: Seminumerical Algorithms, Section 3.4.1, Subsection C, Algorithm P.
 *
 * Note, as stated in the Java docs, two independent values are generated 
 * for one call to log() and one call to sqrt().
 */

static double next;
static int have_next;

static double next_double() {
  static double max = 0x7fffffff;

  return random() / max;
}

extern double next_gaussian( double mean, double std ) {
  double v1, v2, s, multiplier;

  if( have_next ) {
    have_next = 0;
    return std * next + mean;
  }

  do {
    v1 = 2 * next_double() - 1;   /* between -1.0 and 1.0 */
    v2 = 2 * next_double() - 1;   /* between -1.0 and 1.0 */
    s = v1 * v1 + v2 * v2;
  } while( ( s >= 1 ) || ( s == 0 ) );

  multiplier = sqrt( -2 * log(s) / s );
  next = v2 * multiplier;
  have_next = 1;
  return v1 * multiplier * std + mean;
}
