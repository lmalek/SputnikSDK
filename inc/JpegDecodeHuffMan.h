// JpegDecodeHuffMan.h: interface for the CJpegDecodeHuffMan class.
//
//////////////////////////////////////////////////////////////////////


#if !defined(AFX_JPEGDECODEHUFFMAN_H__85527877_44F9_4655_BF6E_5F31795AFC8F__INCLUDED_)
#define AFX_JPEGDECODEHUFFMAN_H__85527877_44F9_4655_BF6E_5F31795AFC8F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define HAVE_UNSIGNED_CHAR    // eliu added for COM DLL
#define HAVE_UNSIGNED_SHORT   // eliu added for COM DLL

#include "stddef.h"

#define MEMZERO(target,size)    memset((void *)(target), 0, (size_t)(size))
#define MEMCOPY(dest,src,size)  memcpy((void *)(dest), (const void *)(src), (size_t)(size))

#define DCTSIZE2	64	/* DCTSIZE squared; # of elements in a block */
typedef short JCOEF;
typedef JCOEF JBLOCK[DCTSIZE2];	/* one block of coefficients */

#define NUM_HUFF_TBLS     3	/* Huffman tables are numbered 0..3 */
#define JPEG_BUF_SIZE   1024 /* bytes */
#define MAX_COMPS_IN_SCAN   3	/* JPEG limit on # of components in one scan */

typedef int boolean; //  eliu commented for COM DLL

#undef FALSE             /*  in case these macros already exist */
#undef TRUE
#define FALSE   0         /*   values of boolean  */
#define TRUE    1

/* UINT8 must hold at least the values 0..255. */

#ifdef HAVE_UNSIGNED_CHAR
typedef unsigned char UINT8;
#else /* not HAVE_UNSIGNED_CHAR */
#ifdef CHAR_IS_UNSIGNED
typedef char UINT8;
#else /* not CHAR_IS_UNSIGNED */
typedef short UINT8;
#endif /* CHAR_IS_UNSIGNED */
#endif /* HAVE_UNSIGNED_CHAR */

/* UINT16 must hold at least the values 0..65535. */

#ifdef HAVE_UNSIGNED_SHORT
typedef unsigned short UINT16;
#else /* not HAVE_UNSIGNED_SHORT */
typedef unsigned int UINT16;
#endif /* HAVE_UNSIGNED_SHORT */

/* INT32 must hold signed 32-bit values; if your machine happens */
/* to have 64-bit longs, you might want to change this. */
#ifndef XMD_H			/* X11/xmd.h correctly defines INT32 */
typedef long INT32;   //eliu commented for COM DLL
#endif

/* INT16 must hold at least the values -32768..32767. */
#ifndef XMD_H			/* X11/xmd.h correctly defines INT16 */
typedef short INT16;
#endif

#define EIGHT_BIT_SAMPLES 8 

/* DCT coefficient quantization tables.
 * For 8-bit precision, 'INT16' should be good enough for quantization values;
 * for more precision, we go for the full 16 bits.  'INT16' provides a useful
 * speedup on many machines (multiplication & division of JCOEFs by
 * quantization values is a significant chunk of the runtime).
 * Note: the values in a QUANT_TBL are always given in zigzag order.
 */
#ifdef EIGHT_BIT_SAMPLES
typedef INT16 QUANT_VAL;	/* element of a quantization table */
#else
typedef UINT16 QUANT_VAL;	/* element of a quantization table */
#endif
typedef QUANT_VAL QUANT_TBL[DCTSIZE2];	/* A quantization table */
typedef QUANT_VAL * QUANT_TBL_PTR;	/* pointer to same */


/* Basic info about one component */
/* These values are fixed over the whole image */
typedef struct 
{
	short component_id;	/* identifier for this component (0..255) */

	short dc_tbl_no;	/* DC entropy table selector (0..3) */
	short ac_tbl_no;	/* AC entropy table selector (0..3) */
} jpeg_component_info;

/* A Huffman coding table */
typedef struct 
{		
	/* These two fields directly represent the contents of a JPEG DHT marker */
	UINT8 bits[17];		/* bits[k] = # of symbols with codes of */
				/* length k bits; bits[0] is unused */
	UINT8 huffval[256];	/* The symbols, in order of incr code length */
  
	/* The remaining fields are computed from the above to allow more efficient
	* coding and decoding.  These fields should be considered private to the
	* Huffman compression & decompression modules.
	*/
	/* decoding tables: (element [0] of each array is unused) */
	UINT16 mincode[17];	/* smallest code of length k */
	INT32 maxcode[18];	/* largest code of length k (-1 if none) */
	/* (maxcode[17] is a sentinel to ensure huff_DECODE terminates) */
	short valptr[17];	/* huffval[] index of 1st symbol of length k */

} HUFF_TBL;


/* Working data for decompression */
struct Decompress_info_struct 
{
	char * input_buffer;	/* start of buffer (private to input code) */
	char * next_input_byte;	/* => next byte to read from buffer */
	int bytes_in_buffer;	/* # of bytes remaining in buffer */

	short num_components;	/* # of color components in JPEG image */
	jpeg_component_info * comp_info;
	/* comp_info[i] describes component that appears i'th in SOF */

	HUFF_TBL * dc_huff_tbl_ptrs[NUM_HUFF_TBLS];
	HUFF_TBL * ac_huff_tbl_ptrs[NUM_HUFF_TBLS];

	/* these fields are private data for the entropy encoder */
	JCOEF last_dc_val[MAX_COMPS_IN_SCAN]; /* last DC coef for each comp */
};

typedef struct Decompress_info_struct * decompress_info_ptr;

#define MAXFILELEN  20480

class CHuffManDecoder  
{
public:
	CHuffManDecoder();
	virtual ~CHuffManDecoder();

	boolean Huff_Decode_Init();
	boolean Huff_Decode_Buffer(const char *lpbuffer, int nSize);
	boolean Huff_Decode_ReadFile(const char *szFileName);
	boolean Huff_Decode_MPEGFile(const char *szFileName, bool &bRestart);
	
	void Huff_Decode_One_Block(JBLOCK block, short seq_component);
	void Huff_Decode_Free();

private:
	inline int huff_DECODE (HUFF_TBL * htbl);
	size_t JFREAD(char *dataptr,int datacount);
	int read_jpeg_data ();
	int fill_bit_buffer (int nbits);
	void fix_huff_tbl (HUFF_TBL * htbl);
	void add_huff_table (HUFF_TBL **htblptr, const UINT8 *bits, const UINT8 *val);
	void std_huff_tables (void);

private:
	char chReadArray[MAXFILELEN];
	int  nReadArrayLen;
	int  nHasReadArrayLen;
};

#endif // !defined(AFX_JPEGDECODEHUFFMAN_H__85527877_44F9_4655_BF6E_5F31795AFC8F__INCLUDED_)
