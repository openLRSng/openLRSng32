/* rscombo.h
 * Copyright Henry Minsky (hqm@alum.mit.edu) 1991-2009
 * (modified for embedded devices)
 * This software library is licensed under terms of the GNU GENERAL
 * PUBLIC LICENSE
 *
 * RSCODE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RSCODE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Rscode.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Source code is available at http://rscode.sourceforge.net
 *
 * RSCOMBO code available at http://relwin.hackhut.com/2011/05/30/reed-solomon-encoderdecoder-for-embedded-devices/
 *
 * Commercial licensing is available under a separate license, please
 * contact author for details.
 *
 */

#define NPAR 4

void  rs_init(void);
void rs_encode_data(unsigned char msg[], int nbytes, unsigned char dst[]);
unsigned char rs_decode_data(unsigned char data[], int nbytes);
int rs_correct_errors_erasures(unsigned char codeword[], int csize, int nerasures, unsigned char erasures[]);

//#define DUMP_ARRAYS       //generate data file, must do if NPAR changed
#define USE_HARDCODED       //use data file


/* exclude encoder/decoder only when using hardcoded data */
//#ifdef COMPILE_TX
//#define USE_ENCODE    //compile encoder routines
//#else
#define USE_DECODE         //compile decoder routines
//#endif

//#define DEBUG     //turns on debug msgs, requires decoder included

#define RS_DATA_FILE "rs_data.h"     //polynomial data file

/****************************************************************/

typedef unsigned char BYTE;

/* Maximum degree of various polynomials. */
#define MAXDEG (NPAR*2)

#ifdef USE_HARDCODED
#include RS_DATA_FILE
#else
BYTE gexp[256 * 2];
BYTE glog[256];
/* generator polynomial */
BYTE genPoly[MAXDEG * 2];
#endif




#ifdef USE_DECODE
/* Decoder syndrome bytes */
BYTE synBytes[MAXDEG];

/* The Error Locator Polynomial, also known as Lambda or Sigma. Lambda[0] == 1 */
static BYTE Lambda[MAXDEG];

/* The Error Evaluator Polynomial */
static BYTE Omega[MAXDEG];


/* error locations found using Chien's search*/
static BYTE ErrorLocs[NPAR];    //expect no more than NPAR errors
static BYTE NErrors;

/* erasure flags */
static BYTE *ErasureLocs;  //points to erasure list
static BYTE NErasures;


/* local ANSI declarations */
static void compute_modified_omega(void);
#endif //decode

/********** polynomial arithmetic *******************/

/* multiplication using logarithms */
BYTE gmult(BYTE a, BYTE b)
{
  BYTE i, j;

  if (a == 0 || b == 0) {
    return (0);
  }

  i = glog[a];
  j = glog[b];
  return (gexp[i + j]);
}


#ifdef USE_DECODE

BYTE ginv(BYTE elt)
{
  return (gexp[255 - glog[elt]]);
}

void add_polys(BYTE dst[], BYTE src[])
{
  BYTE i;

  for (i = 0; i < MAXDEG; i++) {
    dst[i] ^= src[i];
  }
}

//replace with std lib function
#define copy_poly(dst, src) memcpy(dst, src, MAXDEG*sizeof(BYTE))
/*
void copy_poly (BYTE dst[], BYTE src[])
{
  BYTE i;
  for (i = 0; i < MAXDEG; i++) dst[i] = src[i];
}
*/

void scale_poly(int k, BYTE poly[])
{
  BYTE i;

  for (i = 0; i < MAXDEG; i++) {
    poly[i] = gmult(k, poly[i]);
  }
}


//replace with std lib function
#define zero_poly(poly) memset(poly, 0, MAXDEG*sizeof(BYTE))
/*
void zero_poly (BYTE poly[])
{
  BYTE i;
  for (i = 0; i < MAXDEG; i++) poly[i] = 0;
}
*/

/* multiply by z, i.e., shift right by 1 */
static void mul_z_poly(BYTE src[])
{
  BYTE i;

  for (i = MAXDEG - 1; i > 0; i--) {
    src[i] = src[i - 1];
  }

  src[0] = 0;
}



/* polynomial multiplication */
void mult_polys(BYTE dst[], BYTE p1[], BYTE p2[])
{
  int i, j;
  BYTE tmp1[MAXDEG * 2];

  //  for (i=0; i < (MAXDEG*2); i++) dst[i] = 0;
  memset(dst, 0, MAXDEG * 2 * sizeof(BYTE));

  for (i = 0; i < MAXDEG; i++) {
    //for(j=MAXDEG; j<(MAXDEG*2); j++) tmp1[j]=0;
    memset(&tmp1[MAXDEG], 0, sizeof(tmp1) / 2);

    /* scale tmp1 by p1[i] */
    for (j = 0; j < MAXDEG; j++) {
      tmp1[j] = gmult(p2[j], p1[i]);
    }

    /* and mult (shift) tmp1 right by i */
    for (j = (MAXDEG * 2) - 1; j >= i; j--) {
      tmp1[j] = tmp1[j - i];
    }

    for (j = 0; j < i; j++) {
      tmp1[j] = 0;
    }

    /* add into partial product */
    for (j = 0; j < (MAXDEG * 2); j++) {
      dst[j] ^= tmp1[j];
    }
  }
}



/* gamma = product (1-z*a^Ij) for erasure locs Ij */
void init_gamma(BYTE gamma[])
{
  BYTE e;
  BYTE tmp[MAXDEG];

  zero_poly(gamma);
  zero_poly(tmp);
  gamma[0] = 1;

  for (e = 0; e < NErasures; e++) {
    copy_poly(tmp, gamma);
    scale_poly(gexp[ErasureLocs[e]], tmp);
    mul_z_poly(tmp);
    add_polys(gamma, tmp);
  }
}



void compute_next_omega(int d, BYTE A[], BYTE dst[], BYTE src[])
{
  BYTE i;

  for (i = 0; i < MAXDEG;  i++) {
    dst[i] = src[i] ^ gmult(d, A[i]);
  }
}



BYTE compute_discrepancy(BYTE lambda[], BYTE S[], int L, int n)
{
  int i;
  BYTE sum = 0;

  for (i = 0; i <= L; i++) {
    sum ^= gmult(lambda[i], S[n - i]);
  }

  return (sum);
}



/* given Psi (called Lambda in Modified_Berlekamp_Massey) and synBytes,
    compute the combined erasure/error evaluator polynomial as
    Psi*S mod z^4
  */
void compute_modified_omega()
{
  BYTE i;
  BYTE product[MAXDEG * 2];

  mult_polys(product, Lambda, synBytes);
  zero_poly(Omega);

  for (i = 0; i < NPAR; i++) {
    Omega[i] = product[i];
  }

}

/* From  Cain, Clark, "Error-Correction Coding For Digital Communications", pp. 216. */
void Modified_Berlekamp_Massey(void)
{
  int  k,  i;
  BYTE d, n, L, L2;
  BYTE psi[MAXDEG], psi2[MAXDEG], D[MAXDEG];
  BYTE gamma[MAXDEG];

  /* initialize Gamma, the erasure locator polynomial */
  init_gamma(gamma);

  /* initialize to z */
  copy_poly(D, gamma);
  mul_z_poly(D);

  copy_poly(psi, gamma);
  k = -1;
  L = NErasures;

  for (n = NErasures; n < NPAR; n++) {

    d = compute_discrepancy(psi, synBytes, L, n);

    if (d != 0) {

      /* psi2 = psi - d*D */
      for (i = 0; i < MAXDEG; i++) {
        psi2[i] = psi[i] ^ gmult(d, D[i]);
      }


      if (L < (n - k)) {
        L2 = n - k;
        k = n - L;

        /* D = scale_poly(ginv(d), psi); */
        for (i = 0; i < MAXDEG; i++) {
          D[i] = gmult(psi[i], ginv(d));
        }

        L = L2;
      }

      /* psi = psi2 */
      for (i = 0; i < MAXDEG; i++) {
        psi[i] = psi2[i];
      }
    }

    mul_z_poly(D);
  }

  for (i = 0; i < MAXDEG; i++) {
    Lambda[i] = psi[i];
  }

  compute_modified_omega();
}


/* Finds all the roots of an error-locator polynomial with coefficients
 * Lambda[j] by evaluating Lambda at successive values of alpha.
 *
 * This can be tested with the decoder's equations case.
 */


void Find_Roots(void)
{
  int r;
  BYTE sum, k;
  NErrors = 0;

  for (r = 1; r < 256; r++) {
    sum = 0;

    /* evaluate lambda at r */
    for (k = 0; k < NPAR + 1; k++) {
      sum ^= gmult(gexp[(k * r) % 255], Lambda[k]);
    }

    if (sum == 0) {
      if (NErrors < NPAR) {
        ErrorLocs[NErrors] = (255 - r);
      } else {
#ifdef DEBUG
        fprintf(stderr, "Root NErrors>%d\n", NPAR);
#endif
      }

      NErrors++;
#ifdef DEBUG
      fprintf(stderr, "Root found at r = %d, (255-r) = %d\n", r, (255 - r));
#endif
    }
  }
}

/* Combined Erasure And Error Magnitude Computation
 *
 * Pass in the codeword, its size in bytes, as well as
 * an array of any known erasure locations, along the number
 * of these erasures.
 *
 * Evaluate Omega(actually Psi)/Lambda' at the roots
 * alpha^(-i) for error locs i.
 *
 * Returns 1 if everything ok, or 0 if an out-of-bounds error is found
 *
 */

int  rs_correct_errors_erasures(unsigned char codeword[],
                                int csize,
                                int nerasures,
                                unsigned char erasures[])
{
  int i, j;
  BYTE num, denom, err, r;

  /* If you want to take advantage of erasure correction, be sure to
    set NErasures and ErasureLocs[] with the locations of erasures.
    */
  i = NErasures = nerasures;
  //for (i = 0; i < NErasures; i++) ErasureLocs[i] = erasures[i];
  ErasureLocs = erasures;  //save pointer

  Modified_Berlekamp_Massey();
  Find_Roots();


  if ((NErrors <= NPAR) && NErrors > 0) {

    /* first check for illegal error locs */
    for (r = 0; r < NErrors; r++) {
      if (ErrorLocs[r] >= csize) {
#ifdef DEBUG
        fprintf(stderr, "Error loc i=%d outside of codeword length %d\n", i, csize);
#endif
        return (0);
      }
    }

    for (r = 0; r < NErrors; r++) {

      i = ErrorLocs[r];
      /* evaluate Omega at alpha^(-i) */

      num = 0;

      for (j = 0; j < MAXDEG; j++) {
        num ^= gmult(Omega[j], gexp[((255 - i) * j) % 255]);
      }

      /* evaluate Lambda' (derivative) at alpha^(-i) ; all odd powers disappear */
      denom = 0;

      for (j = 1; j < MAXDEG; j += 2) {
        denom ^= gmult(Lambda[j], gexp[((255 - i) * (j - 1)) % 255]);
      }

      err = gmult(num, ginv(denom));
#ifdef DEBUG
      fprintf(stderr, "Error magnitude %#x at loc %d\n", err, csize - i);
#endif

      codeword[csize - i - 1] ^= err;
    }

    return (1);
  } else {
#ifdef DEBUG

    if (NErrors) {
      fprintf(stderr, "Uncorrectable codeword\n");
    }

#endif
    return (0);
  }
}
#endif //decode





#ifndef USE_HARDCODED
static void init_exp_table(void)
{
  int i, z;
  BYTE pinit, p1, p2, p3, p4, p5, p6, p7, p8;

  pinit = p2 = p3 = p4 = p5 = p6 = p7 = p8 = 0;
  p1 = 1;

  gexp[0] = 1;
  gexp[255] = gexp[0];
  glog[0] = 0;          /* shouldn't log[0] be an error? */

  for (i = 1; i < 256; i++) {
    pinit = p8;
    p8 = p7;
    p7 = p6;
    p6 = p5;
    p5 = p4 ^ pinit;
    p4 = p3 ^ pinit;
    p3 = p2 ^ pinit;
    p2 = p1;
    p1 = pinit;
    gexp[i] = p1 + p2 * 2 + p3 * 4 + p4 * 8 + p5 * 16 + p6 * 32 + p7 * 64 + p8 * 128;
    gexp[i + 255] = gexp[i];
  }

  for (i = 1; i < 256; i++) {
    for (z = 0; z < 256; z++) {
      if (gexp[z] == i) {
        glog[i] = z;
        break;
      }
    }
  }
}

#endif



/* debugging routines */
#ifdef DEBUG
void print_parity(void)
{
  int i;
  printf("Parity Bytes: ");
  //for (i = 0; i < NPAR; i++)
  //     printf("[%d]:%x, ",i,pBytes[i]);
  printf("\n");
}


void print_syndrome(void)
{
  int i;
  printf("Syndrome Bytes: ");

  for (i = 0; i < NPAR; i++) {
    printf("[%d]:%x, ", i, synBytes[i]);
  }

  printf("\n");
}

void debug_check_syndrome(void)
{
  int i;

  for (i = 0; i < 3; i++) {
    printf(" inv log S[%d]/S[%d] = %d\n", i, i + 1,
           glog[gmult(synBytes[i], ginv(synBytes[i + 1]))]);
  }
}

#endif


/**********************************************************
 * Reed Solomon Decoder
 *
 * Computes the syndrome of a codeword. Puts the results
 * into the synBytes[] array.
 * returns 0 if syndrome=0
 */
#ifdef USE_DECODE
unsigned char rs_decode_data(unsigned char data[], int nbytes)
{
  int i;
  BYTE sum, j;
  unsigned char r = 0;

  for (j = 0; j < NPAR;  j++) {
    sum    = 0;

    for (i = 0; i < nbytes; i++) {
      sum = data[i] ^ gmult(gexp[j + 1], sum);
    }

    synBytes[j]  = sum;
    r |= sum;
  }

  return (r);
}
#endif //decode




/* Create a generator polynomial for an n byte RS code.
 * The coefficients are returned in the genPoly arg.
 * Make sure that the genPoly array which is passed in is
 * at least n+1 bytes long.
 */
#ifndef USE_HARDCODED
static void compute_genpoly(int nbytes, BYTE genpoly[])
{
  int i;
  BYTE tp[256], tp1[256];

  /* multiply (x + a^n) for n = 1 to nbytes */

  zero_poly(tp1);
  tp1[0] = 1;

  for (i = 1; i <= nbytes; i++) {
    zero_poly(tp);
    tp[0] = gexp[i];       /* set up x+a^n */
    tp[1] = 1;

    mult_polys(genpoly, tp, tp1);
    copy_poly(tp1, genpoly);
  }
}
#endif

/* Simulate a LFSR with generator polynomial for n byte RS code.
 * Pass in a pointer to the data array, and amount of data.
 *
 * The parity bytes are deposited into pBytes[], and the whole message
 * and parity are copied to dest to make a codeword.
 *
 */
#ifdef USE_ENCODE
void rs_encode_data(unsigned char msg[], int nbytes, unsigned char dst[])
{
  int i,  j;
  BYTE LFSR[NPAR + 1], dbyte;

  //for(i=0; i < NPAR+1; i++) LFSR[i]=0;
  memset(LFSR, 0, sizeof(LFSR));

  for (i = 0; i < nbytes; i++) {
    dbyte = msg[i] ^ LFSR[NPAR - 1];

    for (j = NPAR - 1; j > 0; j--) {
      LFSR[j] = LFSR[j - 1] ^ gmult(genPoly[j], dbyte);
    }

    LFSR[0] = gmult(genPoly[0], dbyte);
  }

  /*
    for (i = 0; i < NPAR; i++)
       pBytes[i] = LFSR[i];
    build_codeword(msg, nbytes, dst);
  */
  memcpy(dst, msg, nbytes);   //always in bytes

  for (i = 0; i < NPAR; i++) {
    dst[i + nbytes] = LFSR[NPAR - 1 - i];
  }
}
#endif //encode


#ifdef DUMP_ARRAYS
void data_dump_int(FILE *f, char *name, int *d, int numints)
{
  int i = 0;


  fprintf(f, "%s[%d]={\n", name, numints);

  while (numints-- > 0) {

    fprintf(f, "0x%04X%c", *d++, numints > 0 ? ',' : ' ');
    i++;

    if (i % 8 == 0) {
      fprintf(f, "\n");
    }

  }

  fprintf(f, "};\n");
}

void data_dump_bytes(FILE *f, char *name, unsigned char *d, int numbytes)
{
  int i = 0;


  fprintf(f, "%s[%d]={\n", name, numbytes);

  while (numbytes-- > 0) {

    fprintf(f, "0x%02X%c", *d++, numbytes > 0 ? ',' : ' ');
    i++;

    if (i % 16 == 0) {
      fprintf(f, "\n");
    }

  }

  fprintf(f, "};\n");
}

void data_dump(void)
{
  FILE *f;

  if (!(f = fopen(RS_DATA_FILE, "w+"))) {
    return;
  }

  fprintf(f, "/*NPAR:%d RS_MAX_CODE_SIZE:%d */\n", NPAR, 256);

  data_dump_bytes(f, "const unsigned char gexp", gexp, sizeof(gexp));
  data_dump_bytes(f, "const unsigned char glog", glog, sizeof(glog));

  data_dump_bytes(f, "const unsigned char genPoly",   genPoly , sizeof(genPoly));

  fclose(f);

}
#endif


/* Initialize lookup tables, polynomials, etc.
    Optional if USE_HARDCODED #defined.
*/
void  rs_init(void)
{
#ifndef USE_HARDCODED
  /* initialize the table of powers of alpha */
  init_exp_table();

  /* Compute the encoder generator polynomial */
  compute_genpoly(NPAR, genPoly);
#endif

#ifdef DUMP_ARRAYS
  data_dump();
#endif
}

