// DctQuant.cpp: implementation of the CDctQuant class.
//
//////////////////////////////////////////////////////////////////////

#include "DctQuant.h"
#include <memory.h>


#ifdef EIGHT_BIT_SAMPLES
#define CONST_BITS  13
#define PASS1_BITS  2
#else
#define CONST_BITS  13
#define PASS1_BITS  0		/* lose a little precision to avoid overflow */
#endif

#define ONE	((INT32) 1)

#define CONST_SCALE (ONE << CONST_BITS)

/* Convert a positive real constant to an integer scaled by CONST_SCALE. */

#define FIX(x)	((INT32) ((x) * CONST_SCALE + 0.5))

/* Some C compilers fail to reduce "FIX(constant)" at compile time, thus
 * causing a lot of useless floating-point operations at run time.
 * To get around this we use the following pre-calculated constants.
 * If you change CONST_BITS you may want to add appropriate values.
 * (With a reasonable C compiler, you can just rely on the FIX() macro...)
 */

#if CONST_BITS == 13
#define FIX_0_298631336  ((INT32)  2446)	/* FIX(0.298631336) */
#define FIX_0_390180644  ((INT32)  3196)	/* FIX(0.390180644) */
#define FIX_0_541196100  ((INT32)  4433)	/* FIX(0.541196100) */
#define FIX_0_765366865  ((INT32)  6270)	/* FIX(0.765366865) */
#define FIX_0_899976223  ((INT32)  7373)	/* FIX(0.899976223) */
#define FIX_1_175875602  ((INT32)  9633)	/* FIX(1.175875602) */
#define FIX_1_501321110  ((INT32)  12299)	/* FIX(1.501321110) */
#define FIX_1_847759065  ((INT32)  15137)	/* FIX(1.847759065) */
#define FIX_1_961570560  ((INT32)  16069)	/* FIX(1.961570560) */
#define FIX_2_053119869  ((INT32)  16819)	/* FIX(2.053119869) */
#define FIX_2_562915447  ((INT32)  20995)	/* FIX(2.562915447) */
#define FIX_3_072711026  ((INT32)  25172)	/* FIX(3.072711026) */
#else
#define FIX_0_298631336  FIX(0.298631336)
#define FIX_0_390180644  FIX(0.390180644)
#define FIX_0_541196100  FIX(0.541196100)
#define FIX_0_765366865  FIX(0.765366865)
#define FIX_0_899976223  FIX(0.899976223)
#define FIX_1_175875602  FIX(1.175875602)
#define FIX_1_501321110  FIX(1.501321110)
#define FIX_1_847759065  FIX(1.847759065)
#define FIX_1_961570560  FIX(1.961570560)
#define FIX_2_053119869  FIX(2.053119869)
#define FIX_2_562915447  FIX(2.562915447)
#define FIX_3_072711026  FIX(3.072711026)
#endif


/* Descale and correctly round an INT32 value that's scaled by N bits.
 * We assume RIGHT_SHIFT rounds towards minus infinity, so adding
 * the fudge factor is correct for either sign of X.
 */

//#define DESCALE(x,n)  RIGHT_SHIFT((x) + (ONE << ((n)-1)), n)

#define DESCALE(x,n)  (((x) + (ONE << ((n)-1)))>>(n))

/* Multiply an INT32 variable by an INT32 constant to yield an INT32 result.
 * For 8-bit samples with the recommended scaling, all the variable
 * and constant values involved are no more than 16 bits wide, so a
 * 16x16->32 bit multiply can be used instead of a full 32x32 multiply;
 * this provides a useful speedup on many machines.
 * There is no way to specify a 16x16->32 multiply in portable C, but
 * some C compilers will do the right thing if you provide the correct
 * combination of casts.
 * NB: for 12-bit samples, a full 32-bit multiplication will be needed.
 */

#ifdef EIGHT_BIT_SAMPLES

#ifdef SHORTxSHORT_32		/* may work if 'int' is 32 bits */
#define MULTIPLY(var,const)  (((INT16) (var)) * ((INT16) (const)))
#endif

#ifdef SHORTxLCONST_32		/* known to work with Microsoft C 6.0 */
#define MULTIPLY(var,const)  (((INT16) (var)) * ((INT32) (const)))
#endif
#endif

#ifndef MULTIPLY		/* default definition */
#define MULTIPLY(var,const)  ((var) * (const))
#endif


/* INT32 must hold signed 32-bit values; if your machine happens */
/* to have 64-bit longs, you might want to change this. */
#ifndef XMD_H			/* X11/xmd.h correctly defines INT32 */
typedef long INT32;  //eliu commented for COM DLL
#endif

//for zigzag
static const short ZAG[DCTSIZE2] = 
{
  0,  1,  8, 16,  9,  2,  3, 10,
 17, 24, 32, 25, 18, 11,  4,  5,
 12, 19, 26, 33, 40, 48, 41, 34,
 27, 20, 13,  6,  7, 14, 21, 28,
 35, 42, 49, 56, 57, 50, 43, 36,
 29, 22, 15, 23, 30, 37, 44, 51,
 58, 59, 52, 45, 38, 31, 39, 46,
 53, 60, 61, 54, 47, 55, 62, 63
};

//for quantnize


static short std_luminance_quant_tbl1[DCTSIZE2] = 
{
    16,  11,  12,  14,  12,  10,  16,  14,
    13,  14,  18,  17,  16,  19,  24,  40,
    26,  24,  22,  22,  24,  49,  35,  37,
    29,  40,  58,  51,  61,  60,  57,  51,
    56,  55,  64,  72,  92,  78,  64,  68,
    87,  69,  55,  56,  80, 109,  81,  87,
    95,  98, 103, 104, 103,  62,  77, 113,
    121, 112, 100, 120,  92, 101, 103,  99
};

static short std_chrominance_quant_tbl1[DCTSIZE2] = 
{
    17,  18,  18,  24,  21,  24,  47,  26,
    26,  47,  99,  66,  56,  66,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99,
    99,  99,  99,  99,  99,  99,  99,  99
};


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CDctQuant::CDctQuant()
{
	scale_factor = 100;
	memcpy(std_luminance_quant_tbl, std_luminance_quant_tbl1, sizeof(short)*DCTSIZE2);
	memcpy(std_chrominance_quant_tbl,std_chrominance_quant_tbl1,sizeof(short)*DCTSIZE2);
}

CDctQuant::~CDctQuant()
{

}

void CDctQuant::j_uvy_quant_Init(short factor)
{
	long temp;

	memcpy(std_luminance_quant_tbl, std_luminance_quant_tbl1, sizeof(short)*DCTSIZE2);
	memcpy(std_chrominance_quant_tbl,std_chrominance_quant_tbl1,sizeof(short)*DCTSIZE2);

	scale_factor = j_quality_scaling (factor);
	for(int i = 0; i <64; i++)
	{
			temp = ((long) std_luminance_quant_tbl[i] * scale_factor + 50L) / 100L;
			/* limit the values to the valid range */
			if (temp <= 0L)
			{
				temp = 1L;
			}

		#ifdef EIGHT_BIT_SAMPLES
			if (temp > 32767L) temp = 32767L; /* QUANT_VALs are 'short' */
		#else
			if (temp > 65535L) temp = 65535L; /* QUANT_VALs are 'UINT16' */
		#endif
		std_luminance_quant_tbl[i] = (short) temp;
	}

	for(int i = 0; i <64; i++)
	{
			temp = ((long) std_chrominance_quant_tbl[i] * scale_factor + 50L) / 100L;
			/* limit the values to the valid range */
			if (temp <= 0L)
			{
				temp = 1L;
			}

		#ifdef EIGHT_BIT_SAMPLES
			if (temp > 32767L) temp = 32767L; /* QUANT_VALs are 'short' */
		#else
			if (temp > 65535L) temp = 65535L; /* QUANT_VALs are 'UINT16' */
		#endif
		std_chrominance_quant_tbl[i] = (short) temp;
	}

}

void CDctQuant::j_fwd_dct (DCTBLOCK data)
{
  INT32 tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
  INT32 tmp10, tmp11, tmp12, tmp13;
  INT32 z1, z2, z3, z4, z5;
  register DCTELEM *dataptr;
  int rowctr;
  //SHIFT_TEMPS

  /* Pass 1: process rows. */
  /* Note results are scaled up by sqrt(8) compared to a true DCT; */
  /* furthermore, we scale the results by 2**PASS1_BITS. */

  dataptr = data;
  for (rowctr = DCTSIZE-1; rowctr >= 0; rowctr--) 
  {
    tmp0 = dataptr[0] + dataptr[7];
    tmp7 = dataptr[0] - dataptr[7];
    tmp1 = dataptr[1] + dataptr[6];
    tmp6 = dataptr[1] - dataptr[6];
    tmp2 = dataptr[2] + dataptr[5];
    tmp5 = dataptr[2] - dataptr[5];
    tmp3 = dataptr[3] + dataptr[4];
    tmp4 = dataptr[3] - dataptr[4];
    
    /* Even part per LL&M figure 1 --- note that published figure is faulty;
     * rotator "sqrt(2)*c1" should be "sqrt(2)*c6".
     */
    
    tmp10 = tmp0 + tmp3;
    tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;
    tmp12 = tmp1 - tmp2;
    
    dataptr[0] = (DCTELEM) ((tmp10 + tmp11) << PASS1_BITS);
    dataptr[4] = (DCTELEM) ((tmp10 - tmp11) << PASS1_BITS);
    
    z1 = MULTIPLY(tmp12 + tmp13, FIX_0_541196100);
    dataptr[2] = (DCTELEM) DESCALE(z1 + MULTIPLY(tmp13, FIX_0_765366865),
				   CONST_BITS-PASS1_BITS);
    dataptr[6] = (DCTELEM) DESCALE(z1 + MULTIPLY(tmp12, - FIX_1_847759065),
				   CONST_BITS-PASS1_BITS);
    
    /* Odd part per figure 8 --- note paper omits factor of sqrt(2).
     * cK represents cos(K*pi/16).
     * i0..i3 in the paper are tmp4..tmp7 here.
     */
    
    z1 = tmp4 + tmp7;
    z2 = tmp5 + tmp6;
    z3 = tmp4 + tmp6;
    z4 = tmp5 + tmp7;
    z5 = MULTIPLY(z3 + z4, FIX_1_175875602); /* sqrt(2) * c3 */
    
    tmp4 = MULTIPLY(tmp4, FIX_0_298631336); /* sqrt(2) * (-c1+c3+c5-c7) */
    tmp5 = MULTIPLY(tmp5, FIX_2_053119869); /* sqrt(2) * ( c1+c3-c5+c7) */
    tmp6 = MULTIPLY(tmp6, FIX_3_072711026); /* sqrt(2) * ( c1+c3+c5-c7) */
    tmp7 = MULTIPLY(tmp7, FIX_1_501321110); /* sqrt(2) * ( c1+c3-c5-c7) */
    z1 = MULTIPLY(z1, - FIX_0_899976223); /* sqrt(2) * (c7-c3) */
    z2 = MULTIPLY(z2, - FIX_2_562915447); /* sqrt(2) * (-c1-c3) */
    z3 = MULTIPLY(z3, - FIX_1_961570560); /* sqrt(2) * (-c3-c5) */
    z4 = MULTIPLY(z4, - FIX_0_390180644); /* sqrt(2) * (c5-c3) */
    
    z3 += z5;
    z4 += z5;
    
    dataptr[7] = (DCTELEM) DESCALE(tmp4 + z1 + z3, CONST_BITS-PASS1_BITS);
    dataptr[5] = (DCTELEM) DESCALE(tmp5 + z2 + z4, CONST_BITS-PASS1_BITS);
    dataptr[3] = (DCTELEM) DESCALE(tmp6 + z2 + z3, CONST_BITS-PASS1_BITS);
    dataptr[1] = (DCTELEM) DESCALE(tmp7 + z1 + z4, CONST_BITS-PASS1_BITS);
    
    dataptr += DCTSIZE;		/* advance pointer to next row */
  }

  /* Pass 2: process columns. */
  /* Note that we must descale the results by a factor of 8 == 2**3, */
  /* and also undo the PASS1_BITS scaling. */

  dataptr = data;
  for (rowctr = DCTSIZE-1; rowctr >= 0; rowctr--) 
  {
    tmp0 = dataptr[DCTSIZE*0] + dataptr[DCTSIZE*7];
    tmp7 = dataptr[DCTSIZE*0] - dataptr[DCTSIZE*7];
    tmp1 = dataptr[DCTSIZE*1] + dataptr[DCTSIZE*6];
    tmp6 = dataptr[DCTSIZE*1] - dataptr[DCTSIZE*6];
    tmp2 = dataptr[DCTSIZE*2] + dataptr[DCTSIZE*5];
    tmp5 = dataptr[DCTSIZE*2] - dataptr[DCTSIZE*5];
    tmp3 = dataptr[DCTSIZE*3] + dataptr[DCTSIZE*4];
    tmp4 = dataptr[DCTSIZE*3] - dataptr[DCTSIZE*4];
    
    /* Even part per LL&M figure 1 --- note that published figure is faulty;
     * rotator "sqrt(2)*c1" should be "sqrt(2)*c6".
     */
    
    tmp10 = tmp0 + tmp3;
    tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;
    tmp12 = tmp1 - tmp2;
    
    dataptr[DCTSIZE*0] = (DCTELEM) DESCALE(tmp10 + tmp11, PASS1_BITS+3);
    dataptr[DCTSIZE*4] = (DCTELEM) DESCALE(tmp10 - tmp11, PASS1_BITS+3);
    
    z1 = MULTIPLY(tmp12 + tmp13, FIX_0_541196100);
    dataptr[DCTSIZE*2] = (DCTELEM) DESCALE(z1 + MULTIPLY(tmp13, FIX_0_765366865),
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*6] = (DCTELEM) DESCALE(z1 + MULTIPLY(tmp12, - FIX_1_847759065),
					   CONST_BITS+PASS1_BITS+3);
    
    /* Odd part per figure 8 --- note paper omits factor of sqrt(2).
     * cK represents cos(K*pi/16).
     * i0..i3 in the paper are tmp4..tmp7 here.
     */
    
    z1 = tmp4 + tmp7;
    z2 = tmp5 + tmp6;
    z3 = tmp4 + tmp6;
    z4 = tmp5 + tmp7;
    z5 = MULTIPLY(z3 + z4, FIX_1_175875602); /* sqrt(2) * c3 */
    
    tmp4 = MULTIPLY(tmp4, FIX_0_298631336); /* sqrt(2) * (-c1+c3+c5-c7) */
    tmp5 = MULTIPLY(tmp5, FIX_2_053119869); /* sqrt(2) * ( c1+c3-c5+c7) */
    tmp6 = MULTIPLY(tmp6, FIX_3_072711026); /* sqrt(2) * ( c1+c3+c5-c7) */
    tmp7 = MULTIPLY(tmp7, FIX_1_501321110); /* sqrt(2) * ( c1+c3-c5-c7) */
    z1 = MULTIPLY(z1, - FIX_0_899976223); /* sqrt(2) * (c7-c3) */
    z2 = MULTIPLY(z2, - FIX_2_562915447); /* sqrt(2) * (-c1-c3) */
    z3 = MULTIPLY(z3, - FIX_1_961570560); /* sqrt(2) * (-c3-c5) */
    z4 = MULTIPLY(z4, - FIX_0_390180644); /* sqrt(2) * (c5-c3) */
    
    z3 += z5;
    z4 += z5;
    
    dataptr[DCTSIZE*7] = (DCTELEM) DESCALE(tmp4 + z1 + z3,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*5] = (DCTELEM) DESCALE(tmp5 + z2 + z4,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*3] = (DCTELEM) DESCALE(tmp6 + z2 + z3,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*1] = (DCTELEM) DESCALE(tmp7 + z1 + z4,
					   CONST_BITS+PASS1_BITS+3);
    
    dataptr++;			/* advance pointer to next column */
  }
}



void CDctQuant::j_rev_dct (DCTBLOCK data)
{
  INT32 tmp0, tmp1, tmp2, tmp3;
  INT32 tmp10, tmp11, tmp12, tmp13;
  INT32 z1, z2, z3, z4, z5;
  register DCTELEM *dataptr;
  int rowctr;
 // SHIFT_TEMPS

  /* Pass 1: process rows. */
  /* Note results are scaled up by sqrt(8) compared to a true IDCT; */
  /* furthermore, we scale the results by 2**PASS1_BITS. */

  dataptr = data;
  for (rowctr = DCTSIZE-1; rowctr >= 0; rowctr--) {
    /* Due to quantization, we will usually find that many of the input
     * coefficients are zero, especially the AC terms.  We can exploit this
     * by short-circuiting the IDCT calculation for any row in which all
     * the AC terms are zero.  In that case each output is equal to the
     * DC coefficient (with scale factor as needed).
     * With typical images and quantization tables, half or more of the
     * row DCT calculations can be simplified this way.
     */

    if ((dataptr[1] | dataptr[2] | dataptr[3] | dataptr[4] |
	 dataptr[5] | dataptr[6] | dataptr[7]) == 0) 
	{
      /* AC terms all zero */
      DCTELEM dcval = (DCTELEM) (dataptr[0] << PASS1_BITS);
      
      dataptr[0] = dcval;
      dataptr[1] = dcval;
      dataptr[2] = dcval;
      dataptr[3] = dcval;
      dataptr[4] = dcval;
      dataptr[5] = dcval;
      dataptr[6] = dcval;
      dataptr[7] = dcval;
      
      dataptr += DCTSIZE;	/* advance pointer to next row */
      continue;
    }

    /* Even part: reverse the even part of the forward DCT. */
    /* The rotator is sqrt(2)*c(-6). */

    z2 = (INT32) dataptr[2];
    z3 = (INT32) dataptr[6];

    z1 = MULTIPLY(z2 + z3, FIX_0_541196100);
    tmp2 = z1 + MULTIPLY(z3, - FIX_1_847759065);
    tmp3 = z1 + MULTIPLY(z2, FIX_0_765366865);

    tmp0 = ((INT32) dataptr[0] + (INT32) dataptr[4]) << CONST_BITS;
    tmp1 = ((INT32) dataptr[0] - (INT32) dataptr[4]) << CONST_BITS;

    tmp10 = tmp0 + tmp3;
    tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;
    tmp12 = tmp1 - tmp2;
    
    /* Odd part per figure 8; the matrix is unitary and hence its
     * transpose is its inverse.  i0..i3 are y7,y5,y3,y1 respectively.
     */

    tmp0 = (INT32) dataptr[7];
    tmp1 = (INT32) dataptr[5];
    tmp2 = (INT32) dataptr[3];
    tmp3 = (INT32) dataptr[1];

    z1 = tmp0 + tmp3;
    z2 = tmp1 + tmp2;
    z3 = tmp0 + tmp2;
    z4 = tmp1 + tmp3;
    z5 = MULTIPLY(z3 + z4, FIX_1_175875602); /* sqrt(2) * c3 */
    
    tmp0 = MULTIPLY(tmp0, FIX_0_298631336); /* sqrt(2) * (-c1+c3+c5-c7) */
    tmp1 = MULTIPLY(tmp1, FIX_2_053119869); /* sqrt(2) * ( c1+c3-c5+c7) */
    tmp2 = MULTIPLY(tmp2, FIX_3_072711026); /* sqrt(2) * ( c1+c3+c5-c7) */
    tmp3 = MULTIPLY(tmp3, FIX_1_501321110); /* sqrt(2) * ( c1+c3-c5-c7) */
    z1 = MULTIPLY(z1, - FIX_0_899976223); /* sqrt(2) * (c7-c3) */
    z2 = MULTIPLY(z2, - FIX_2_562915447); /* sqrt(2) * (-c1-c3) */
    z3 = MULTIPLY(z3, - FIX_1_961570560); /* sqrt(2) * (-c3-c5) */
    z4 = MULTIPLY(z4, - FIX_0_390180644); /* sqrt(2) * (c5-c3) */
    
    z3 += z5;
    z4 += z5;
    
    tmp0 += z1 + z3;
    tmp1 += z2 + z4;
    tmp2 += z2 + z3;
    tmp3 += z1 + z4;

    /* Final output stage: inputs are tmp10..tmp13, tmp0..tmp3 */

    dataptr[0] = (DCTELEM) DESCALE(tmp10 + tmp3, CONST_BITS-PASS1_BITS);
    dataptr[7] = (DCTELEM) DESCALE(tmp10 - tmp3, CONST_BITS-PASS1_BITS);
    dataptr[1] = (DCTELEM) DESCALE(tmp11 + tmp2, CONST_BITS-PASS1_BITS);
    dataptr[6] = (DCTELEM) DESCALE(tmp11 - tmp2, CONST_BITS-PASS1_BITS);
    dataptr[2] = (DCTELEM) DESCALE(tmp12 + tmp1, CONST_BITS-PASS1_BITS);
    dataptr[5] = (DCTELEM) DESCALE(tmp12 - tmp1, CONST_BITS-PASS1_BITS);
    dataptr[3] = (DCTELEM) DESCALE(tmp13 + tmp0, CONST_BITS-PASS1_BITS);
    dataptr[4] = (DCTELEM) DESCALE(tmp13 - tmp0, CONST_BITS-PASS1_BITS);

    dataptr += DCTSIZE;		/* advance pointer to next row */
  }

  /* Pass 2: process columns. */
  /* Note that we must descale the results by a factor of 8 == 2**3, */
  /* and also undo the PASS1_BITS scaling. */

  dataptr = data;
  for (rowctr = DCTSIZE-1; rowctr >= 0; rowctr--) 
  {
    /* Columns of zeroes can be exploited in the same way as we did with rows.
     * However, the row calculation has created many nonzero AC terms, so the
     * simplification applies less often (typically 5% to 10% of the time).
     * On machines with very fast multiplication, it's possible that the
     * test takes more time than it's worth.  In that case this section
     * may be commented out.
     */

#ifndef NO_ZERO_COLUMN_TEST
    if ((dataptr[DCTSIZE*1] | dataptr[DCTSIZE*2] | dataptr[DCTSIZE*3] |
	 dataptr[DCTSIZE*4] | dataptr[DCTSIZE*5] | dataptr[DCTSIZE*6] |
	 dataptr[DCTSIZE*7]) == 0) {
      /* AC terms all zero */
      DCTELEM dcval = (DCTELEM) DESCALE((INT32) dataptr[0], PASS1_BITS+3);
      
      dataptr[DCTSIZE*0] = dcval;
      dataptr[DCTSIZE*1] = dcval;
      dataptr[DCTSIZE*2] = dcval;
      dataptr[DCTSIZE*3] = dcval;
      dataptr[DCTSIZE*4] = dcval;
      dataptr[DCTSIZE*5] = dcval;
      dataptr[DCTSIZE*6] = dcval;
      dataptr[DCTSIZE*7] = dcval;
      
      dataptr++;		/* advance pointer to next column */
      continue;
    }
#endif

    /* Even part: reverse the even part of the forward DCT. */
    /* The rotator is sqrt(2)*c(-6). */

    z2 = (INT32) dataptr[DCTSIZE*2];
    z3 = (INT32) dataptr[DCTSIZE*6];

    z1 = MULTIPLY(z2 + z3, FIX_0_541196100);
    tmp2 = z1 + MULTIPLY(z3, - FIX_1_847759065);
    tmp3 = z1 + MULTIPLY(z2, FIX_0_765366865);

    tmp0 = ((INT32) dataptr[DCTSIZE*0] + (INT32) dataptr[DCTSIZE*4]) << CONST_BITS;
    tmp1 = ((INT32) dataptr[DCTSIZE*0] - (INT32) dataptr[DCTSIZE*4]) << CONST_BITS;

    tmp10 = tmp0 + tmp3;
    tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;
    tmp12 = tmp1 - tmp2;
    
    /* Odd part per figure 8; the matrix is unitary and hence its
     * transpose is its inverse.  i0..i3 are y7,y5,y3,y1 respectively.
     */

    tmp0 = (INT32) dataptr[DCTSIZE*7];
    tmp1 = (INT32) dataptr[DCTSIZE*5];
    tmp2 = (INT32) dataptr[DCTSIZE*3];
    tmp3 = (INT32) dataptr[DCTSIZE*1];

    z1 = tmp0 + tmp3;
    z2 = tmp1 + tmp2;
    z3 = tmp0 + tmp2;
    z4 = tmp1 + tmp3;
    z5 = MULTIPLY(z3 + z4, FIX_1_175875602); /* sqrt(2) * c3 */
    
    tmp0 = MULTIPLY(tmp0, FIX_0_298631336); /* sqrt(2) * (-c1+c3+c5-c7) */
    tmp1 = MULTIPLY(tmp1, FIX_2_053119869); /* sqrt(2) * ( c1+c3-c5+c7) */
    tmp2 = MULTIPLY(tmp2, FIX_3_072711026); /* sqrt(2) * ( c1+c3+c5-c7) */
    tmp3 = MULTIPLY(tmp3, FIX_1_501321110); /* sqrt(2) * ( c1+c3-c5-c7) */
    z1 = MULTIPLY(z1, - FIX_0_899976223); /* sqrt(2) * (c7-c3) */
    z2 = MULTIPLY(z2, - FIX_2_562915447); /* sqrt(2) * (-c1-c3) */
    z3 = MULTIPLY(z3, - FIX_1_961570560); /* sqrt(2) * (-c3-c5) */
    z4 = MULTIPLY(z4, - FIX_0_390180644); /* sqrt(2) * (c5-c3) */
    
    z3 += z5;
    z4 += z5;
    
    tmp0 += z1 + z3;
    tmp1 += z2 + z4;
    tmp2 += z2 + z3;
    tmp3 += z1 + z4;

    /* Final output stage: inputs are tmp10..tmp13, tmp0..tmp3 */

    dataptr[DCTSIZE*0] = (DCTELEM) DESCALE(tmp10 + tmp3,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*7] = (DCTELEM) DESCALE(tmp10 - tmp3,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*1] = (DCTELEM) DESCALE(tmp11 + tmp2,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*6] = (DCTELEM) DESCALE(tmp11 - tmp2,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*2] = (DCTELEM) DESCALE(tmp12 + tmp1,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*5] = (DCTELEM) DESCALE(tmp12 - tmp1,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*3] = (DCTELEM) DESCALE(tmp13 + tmp0,
					   CONST_BITS+PASS1_BITS+3);
    dataptr[DCTSIZE*4] = (DCTELEM) DESCALE(tmp13 - tmp0,
					   CONST_BITS+PASS1_BITS+3);
    
    dataptr++;			/* advance pointer to next column */
  }
}



void CDctQuant::j_f_y_quant(DCTBLOCK data)
{
	short i,j;
	short *data_ptr;
	short tmp_data;

	data_ptr=data;
	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{
			tmp_data=data_ptr[i*8+j];
			if (tmp_data < 0) 
			{
				tmp_data = -tmp_data;
				tmp_data += std_luminance_quant_tbl[i*8+j]>>1;
				tmp_data /= std_luminance_quant_tbl[i*8+j];
				tmp_data = -tmp_data;
			} 
			else 
			{
				tmp_data += std_luminance_quant_tbl[i*8+j]>>1;
				tmp_data /= std_luminance_quant_tbl[i*8+j];
			}
			data_ptr[i*8+j]=(short)tmp_data;
		}
	}
}


void CDctQuant::j_i_y_quant(DCTBLOCK data)
{
	short i,j;
	short *data_ptr;
	short tmp_data;

	data_ptr=data;
	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{
			tmp_data=data_ptr[i*8+j];
			if (tmp_data < 0) 
			{
				tmp_data = -tmp_data;
				tmp_data *= std_luminance_quant_tbl[i*8+j];
				tmp_data = -tmp_data;
			} 
			else 
			{
				tmp_data *= std_luminance_quant_tbl[i*8+j];
			}
			data_ptr[i*8+j]=(short)tmp_data;
		}
	}
}


void CDctQuant::j_f_uv_quant(DCTBLOCK data)
{
	short i,j;
	short *data_ptr;
	short tmp_data;

	data_ptr=data;
	for(i=0;i<8;i++)
		for(j=0;j<8;j++)
		{
			tmp_data=data_ptr[i*8+j];
			if (tmp_data < 0) 
			{
				tmp_data = -tmp_data;
				tmp_data += std_chrominance_quant_tbl[i*8+j]>>1;
				tmp_data /= std_chrominance_quant_tbl[i*8+j];
				tmp_data = -tmp_data;
			} 
			else 
			{
				tmp_data += std_chrominance_quant_tbl[i*8+j]>>1;
				tmp_data /= std_chrominance_quant_tbl[i*8+j];
			}
			data_ptr[i*8+j] = (short)tmp_data;
		}

}


void CDctQuant::j_i_uv_quant(DCTBLOCK data)
{
	short i,j;
	short *data_ptr;
	short tmp_data;

	data_ptr=data;
	for(i=0;i<8;i++)
		for(j=0;j<8;j++)
		{
			tmp_data=data_ptr[i*8+j];
			if (tmp_data < 0) 
			{
				tmp_data = -tmp_data;
				tmp_data *= std_chrominance_quant_tbl[i*8+j];
				tmp_data = -tmp_data;
			} 
			else 
			{
				tmp_data *= std_chrominance_quant_tbl[i*8+j];
			}
			data_ptr[i*8+j] = (short)tmp_data;

		}

}



void CDctQuant::zigzag_trans(DCTBLOCK data)
{

	short i;
	
	short tmp_data[64];

	short *dataptr;
	dataptr=data;
	for(i = 0; i<64;i++)
	{
		tmp_data[i]=*(dataptr+ZAG[i]);
	}

	for(i=0;i<64;i++)
	{
		dataptr[i]=tmp_data[i];
	}
	
}


void CDctQuant::zigzag_i_trans(DCTBLOCK data)
{

	short i;
	short tmp_data[64];

	short *dataptr;

	dataptr=data;

	for(i = 0; i<64;i++)
	{
		*(tmp_data+ZAG[i])=dataptr[i];
	}

	for(i=0;i<64;i++)
	{
		dataptr[i]=tmp_data[i];
	}
}


short CDctQuant::j_quality_scaling (short quality)
/* Convert a user-specified quality rating to a percentage scaling factor
 * for an underlying quantization table, using our recommended scaling curve.
 * The input 'quality' factor should be 0 (terrible) to 100 (very good).
 */
{
  /* Safety limit on quality factor.  Convert 0 to 1 to avoid zero divide. */
  if (quality <= 0) quality = 1;
  if (quality > 100) quality = 100;

  /* The basic table is used as-is (scaling 100) for a quality of 50.
   * Qualities 50..100 are converted to scaling percentage 200 - 2*Q;
   * note that at Q=100 the scaling is 0, which will cause j_add_quant_table
   * to make all the table entries 1 (hence, no quantization loss).
   * Qualities 1..50 are converted to scaling percentage 5000/Q.
   */
  if (quality < 50)
    quality = 5000 / quality;
  else
    quality = 200 - quality*2;

  return quality;
}
