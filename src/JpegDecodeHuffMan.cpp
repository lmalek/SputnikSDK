// JpegDecodeHuffMan.cpp: implementation of the CJpegDecodeHuffMan class.
//
//////////////////////////////////////////////////////////////////////


#include "JpegDecodeHuffMan.h"
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <stdio.h>


struct Decompress_info_struct dcinfo;
static decompress_info_ptr cinfo;

static INT32 get_buffer;	// current bit-extraction buffer 
static int bits_left;		// # of unused bits in it 


#ifdef SLOW_SHIFT_32
#define MIN_GET_BITS  15	/* minimum allowable value */
#else
#undef MIN_GET_BITS
#define MIN_GET_BITS  25	/* max value for 32-bit get_buffer */
#endif
static const int bmask[16] =	/* bmask[n] is mask for n rightmost bits */
  { 0, 0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF,
    0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF };

/* Macros for reading data from the decompression input buffer */

#ifdef CHAR_IS_UNSIGNED
#define JGETC(cinfo)	( --(cinfo)->bytes_in_buffer < 0 ? \
							read_jpeg_data() : \
							(int) (*(cinfo)->next_input_byte++) )
#else
#undef JGETC
#define JGETC(cinfo)	( --(cinfo)->bytes_in_buffer < 0 ? \
							read_jpeg_data() : \
							(int) (*(cinfo)->next_input_byte++) & 0xFF )
#endif

#define JUNGETC(ch,cinfo)  ((cinfo)->bytes_in_buffer++, \
			    *(--((cinfo)->next_input_byte)) = (char) (ch))

#define MIN_UNGET	4	/* may always do at least 4 JUNGETCs */

/* Macros to make things go at some speed! */
/* NB: parameter to get_bits should be simple variable, not expression */

#define get_bits(nbits) \
	(bits_left >= (nbits) ? \
	 ((int) (get_buffer >> (bits_left -= (nbits)))) & bmask[nbits] : \
	 fill_bit_buffer(nbits))

#define get_bit() \
	(bits_left ? \
	 ((int) (get_buffer >> (--bits_left))) & 1 : \
	 fill_bit_buffer(1))

/* Figure F.12: extend sign bit */

#define huff_EXTEND(x,s)  ((x) < extend_test[s] ? (x) + extend_offset[s] : (x))
static const int extend_test[16] =   /* entry n is 2**(n-1) */
  { 0, 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000 };

static const int extend_offset[16] = /* entry n is (-1 << n) + 1 */
  { 0, ((-1)<<1) + 1, ((-1)<<2) + 1, ((-1)<<3) + 1, ((-1)<<4) + 1,
    ((-1)<<5) + 1, ((-1)<<6) + 1, ((-1)<<7) + 1, ((-1)<<8) + 1,
    ((-1)<<9) + 1, ((-1)<<10) + 1, ((-1)<<11) + 1, ((-1)<<12) + 1,
    ((-1)<<13) + 1, ((-1)<<14) + 1, ((-1)<<15) + 1 };

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHuffManDecoder::CHuffManDecoder()
{
	cinfo = &dcinfo;
}

CHuffManDecoder::~CHuffManDecoder()
{
	Huff_Decode_Free();
}

boolean CHuffManDecoder::Huff_Decode_ReadFile(const char *szFileName)
{
	//read data
	FILE *fp = fopen(szFileName, "rb");
	if( fp == NULL ) return FALSE;
	
	memset(chReadArray, 0, MAXFILELEN);

	nReadArrayLen = 0;
	while( !feof(fp) && !ferror(fp) )
	{
		chReadArray[nReadArrayLen] = fgetc(fp);
		nReadArrayLen++;
		if( nReadArrayLen >= MAXFILELEN )  break;
	}
	fclose(fp);

	if( (nReadArrayLen == 0) || (nReadArrayLen >= MAXFILELEN) ) return FALSE;
	nHasReadArrayLen = 0;

	bits_left = 0;
	cinfo->bytes_in_buffer = 0; /* initialize buffer to empty */

	for (int i = 0; i < cinfo->num_components; i++) 
	{
		/* Initialize DC predictions to 0 */
		cinfo->last_dc_val[i] = 0;
	}

	return TRUE;
}

// eliu added
boolean CHuffManDecoder::Huff_Decode_Buffer(const char *lpbuffer, int nSize)
{
	
	memset(chReadArray, 0, MAXFILELEN);

	if ( nSize <=0 || nSize > MAXFILELEN ) return FALSE;

	memcpy(chReadArray,lpbuffer,nSize);
	nReadArrayLen = nSize;

	nHasReadArrayLen = 0;

	bits_left = 0;
	cinfo->bytes_in_buffer = 0; /* initialize buffer to empty */

	for (int i = 0; i < cinfo->num_components; i++) 
	{
		/* Initialize DC predictions to 0 */
		cinfo->last_dc_val[i] = 0;
	}

	return TRUE;
}




boolean CHuffManDecoder::Huff_Decode_MPEGFile(const char *szFileName, bool &bRestart)
{
	//read data
	FILE *fp = fopen(szFileName, "rb");
	if( fp == NULL ) return FALSE;

	// uMarker: indicate if this picture is a difference or orginal picture
	//		  : 0xD7;		//restart marker
	//		  : 0xD6;		//continue marker
	unsigned char uMarker;
	if( fread(&uMarker, 1, 1, fp) != 1 ) 
	{
		fclose(fp);
		return false;
	}

	if( uMarker == 0xD7 )		bRestart = true;
	else if( uMarker == 0xD6 )	bRestart = false;
	else
	{
		rewind(fp);
		bRestart = true;
	}

	nReadArrayLen = 0;
	while( !feof(fp) && !ferror(fp) )
	{
		chReadArray[nReadArrayLen] = fgetc(fp);
		nReadArrayLen++;
		if( nReadArrayLen >= MAXFILELEN)  break;
	}
	fclose(fp);

	if( (nReadArrayLen == 0) || (nReadArrayLen >= MAXFILELEN) ) return FALSE;
	nHasReadArrayLen = 0;

	bits_left = 0;
	cinfo->bytes_in_buffer = 0; /* initialize buffer to empty */

	for (int i = 0; i < cinfo->num_components; i++) 
	{
		/* Initialize DC predictions to 0 */
		cinfo->last_dc_val[i] = 0;
	}

	return TRUE;
}

/*
 * Initialize for a Huffman-compressed scan.
 */
boolean CHuffManDecoder::Huff_Decode_Init()
{
	short i;
  	jpeg_component_info * compptr;

	bits_left = 0;

	/* Initialize pointers as needed to mark stuff unallocated. */
	/* Outer application may fill in default tables for abbreviated files... */
	cinfo->comp_info = NULL;
	for (i = 0; i < NUM_HUFF_TBLS; i++) 
	{
		cinfo->dc_huff_tbl_ptrs[i] = NULL;
		cinfo->ac_huff_tbl_ptrs[i] = NULL;
	}
  
	/* Allocate memory for input buffer, unless outer application provides it. */
	cinfo->input_buffer = (char *) malloc((size_t) (JPEG_BUF_SIZE + MIN_UNGET));
	if( cinfo->input_buffer == NULL )	return FALSE;
	cinfo->bytes_in_buffer = 0; /* initialize buffer to empty */

	std_huff_tables ();   

	cinfo->num_components = 3;
	cinfo->comp_info = (jpeg_component_info *) malloc(3 * sizeof(jpeg_component_info));
	if( cinfo->comp_info == NULL )
	{
		free(cinfo->input_buffer);
		cinfo->input_buffer = NULL;
		return FALSE;
	}
	
	/* luminance Y */
	compptr = &cinfo->comp_info[0];
	compptr->component_id = 1;	
	compptr->dc_tbl_no = 0;
	compptr->ac_tbl_no = 0;
	/* U */
	compptr = &cinfo->comp_info[1];
	compptr->component_id = 2;
	compptr->dc_tbl_no = 1;
	compptr->ac_tbl_no = 1;
	/* V */
	compptr = &cinfo->comp_info[2];
	compptr->component_id = 3;
	compptr->dc_tbl_no = 1;
	compptr->ac_tbl_no = 1;

	for (i = 0; i < cinfo->num_components; i++) 
	{
		compptr = &cinfo->comp_info[i];

		/* Compute derived values for Huffman tables */
		/* We may do this more than once for same table, but it's not a big deal */
		fix_huff_tbl(cinfo->dc_huff_tbl_ptrs[compptr->dc_tbl_no]);
		fix_huff_tbl(cinfo->ac_huff_tbl_ptrs[compptr->ac_tbl_no]);
		
		/* Initialize DC predictions to 0 */
		cinfo->last_dc_val[i] = 0;
	}
	return TRUE;
}



void CHuffManDecoder::Huff_Decode_Free()
{
	if( cinfo->input_buffer != NULL )
	{
		free(cinfo->input_buffer);
		cinfo->input_buffer = NULL;
	}
	if( cinfo->comp_info != NULL )
	{
		free( cinfo->comp_info ); 
		cinfo->comp_info = NULL;
	}

	for (short i = 0; i < NUM_HUFF_TBLS; i++) 
	{
		if( cinfo->dc_huff_tbl_ptrs[i] != NULL )
		{
			free(cinfo->dc_huff_tbl_ptrs[i]);
			cinfo->dc_huff_tbl_ptrs[i] = NULL;
		}
		if(cinfo->ac_huff_tbl_ptrs[i] != NULL)
		{
			free(cinfo->ac_huff_tbl_ptrs[i]);
			cinfo->ac_huff_tbl_ptrs[i] = NULL;
		}
	}

}


/*
 * Decode and return one MCU's worth of Huffman-compressed coefficients.
 * This routine also handles quantization descaling and zigzag reordering
 * of coefficient values.
 *
 * The i'th block of the MCU is stored into the block pointed to by
 * MCU_data[i].  WE ASSUME THIS AREA HAS BEEN ZEROED BY THE CALLER.
 * (Wholesale zeroing is usually a little faster than retail...)
 */

void CHuffManDecoder::Huff_Decode_One_Block(JBLOCK block, short seq_component)
{
	register int s, k, r;
	HUFF_TBL *dctbl;
	HUFF_TBL *actbl;
	jpeg_component_info * compptr;

	memset(block, 0, sizeof(JCOEF)*DCTSIZE2);

	compptr = &cinfo->comp_info[seq_component];
    actbl = cinfo->ac_huff_tbl_ptrs[compptr->ac_tbl_no];
    dctbl = cinfo->dc_huff_tbl_ptrs[compptr->dc_tbl_no];

    /* Decode a single block's worth of coefficients */
    /* Section F.2.2.1: decode the DC coefficient difference */
	s = huff_DECODE(dctbl);
    if (s) 
	{
		r = get_bits(s);
		s = huff_EXTEND(r, s);
    }

    /* Convert DC difference to actual value, update last_dc_val */
    s += cinfo->last_dc_val[seq_component];
    cinfo->last_dc_val[seq_component] = (JCOEF) s;
    
	block[0] = (JCOEF)s;
	/* Descale and output the DC coefficient (assumes ZAG[0] = 0) */
    //(*block)[0] = (JCOEF) (((JCOEF) s) * quanttbl[0]);
    
    /* Section F.2.2.2: decode the AC coefficients */
    /* Since zero values are skipped, output area must be zeroed beforehand */
    for (k = 1; k < DCTSIZE2; k++) 
	{
		r = huff_DECODE(actbl);
      
		s = r & 15;
		r = r >> 4;
      
		if (s) 
		{
			k += r;
			r = get_bits(s);
			s = huff_EXTEND(r, s);
			
			if( k < DCTSIZE2)
				block[k] = (JCOEF)s;
			/* Descale coefficient and output in natural (dezigzagged) order */
			//(*block)[ZAG[k]] = (JCOEF) (((JCOEF) s) * quanttbl[k]);
		 } 
		 else 
		 {
			if (r != 15)	break;
			k += 15;
		 }
	}
}

/* Figure F.16: extract next coded symbol from input stream */
int CHuffManDecoder::huff_DECODE (HUFF_TBL * htbl)
{
	register int l;
	register INT32 code;
  
	code = get_bit();
	l = 1;
	while (code > htbl->maxcode[l]) 
	{
		code = (code << 1) | get_bit();
		l++;
		if( l > 16 ) break;
	}

  /* With garbage input we may reach the sentinel value l = 17. */
	if (l > 16) 
	{
		return 0;			/* fake a zero as the safest result */
	}
	
	int nTemp = htbl->valptr[l] + ((int) (code - htbl->mincode[l]));
	if( nTemp > 255 ) return 0;

	return htbl->huffval[nTemp];
}


/* Load up the bit buffer and do get_bits(nbits) */
/* Attempt to load at least MIN_GET_BITS bits into get_buffer. */
int CHuffManDecoder::fill_bit_buffer (int nbits)
{
	while (bits_left < MIN_GET_BITS) 
	{
		register int c = JGETC(cinfo);
    
		/* If it's 0xFF, check and discard stuffed zero byte */
		if (c == 0xFF) 
		{
			int c2 = JGETC(cinfo);
			if (c2 != 0) 
			{
				/* Oops, it's actually a marker indicating end of compressed data. */
				/* Better put it back for use later */
				JUNGETC(c2,cinfo);
				JUNGETC(c,cinfo);
				
				/* There should be enough bits still left in the data segment; */
				/* if so, just break out of the while loop. */
				if (bits_left >= nbits)	  break;

				/* Uh-oh.  Report corrupted data to user and stuff zeroes into
				 * the data stream, so we can produce some kind of image.
				 * Note that this will be repeated for each byte demanded for the
				 * rest of the segment; this is a bit slow but not unreasonably so.
				 * The main thing is to avoid getting a zillion warnings, hence:
				 */
				c = 0;			/* insert a zero byte into bit buffer */
			}
		}

		/* OK, load c into get_buffer */
		get_buffer = (get_buffer << 8) | c;
		bits_left += 8;
	}

	/* Having filled get_buffer, extract desired bits (this simplifies macros) */
	bits_left -= nbits;
	return ((int) (get_buffer >> bits_left)) & bmask[nbits];
}


int CHuffManDecoder::read_jpeg_data ()
{
	cinfo->next_input_byte = cinfo->input_buffer + MIN_UNGET;
	cinfo->bytes_in_buffer=(int) JFREAD(cinfo->next_input_byte,JPEG_BUF_SIZE);
  
	if (cinfo->bytes_in_buffer <= 0) 
	{
		cinfo->next_input_byte[0] = (char) 0xFF;
		cinfo->next_input_byte[1] = (char) 0xD9; /* EOI marker */
		cinfo->bytes_in_buffer = 2;
	}
	return JGETC(cinfo);
}

/* read from file */
size_t CHuffManDecoder::JFREAD(char *dataptr,int datacount)
{
	int i;
	for (i=0;i<datacount;++i)
	{
		if( nHasReadArrayLen < nReadArrayLen)
		{	
			*dataptr++ = chReadArray[nHasReadArrayLen];
			nHasReadArrayLen++;
		}
		else
			break;
	}

	return(i);
}



void CHuffManDecoder::fix_huff_tbl (HUFF_TBL * htbl)
/* Compute derived values for a Huffman table */
{
  int p, i, l, si;
  char huffsize[257];
  UINT16 huffcode[257];
  UINT16 code;
  
  /* Figure C.1: make table of Huffman code length for each symbol */
  /* Note that this is in code-length order. */

  p = 0;
  for (l = 1; l <= 16; l++) 
  {
    for (i = 1; i <= (int) htbl->bits[l] && p<257; i++)
      huffsize[p++] = (char) l;
  }
  huffsize[p] = 0;
  
  /* Figure C.2: generate the codes themselves */
  /* Note that this is in code-length order. */
  
  code = 0;
  si = huffsize[0];
  p = 0;
  while (huffsize[p] && p<257) 
  {
    while (((int) huffsize[p]) == si && p<257) 
	{
      huffcode[p++] = code;
      code++;
    }
    code <<= 1;
    si++;
  }

  /* We don't bother to fill in the encoding tables ehufco[] and ehufsi[], */
  /* since they are not used for decoding. */

  /* Figure F.15: generate decoding tables */

  p = 0;
  for (l = 1; l <= 16; l++) 
  {
    if (htbl->bits[l]) 
	{
      htbl->valptr[l] = p;	/* huffval[] index of 1st sym of code len l */
      htbl->mincode[l] = huffcode[p]; /* minimum code of length l */
      p += htbl->bits[l];
      htbl->maxcode[l] = huffcode[p-1];	/* maximum code of length l */
    } 
	else 
	{
      htbl->maxcode[l] = -1;
    }
  }
  htbl->maxcode[17] = 0xFFFFFL;	/* ensures huff_DECODE terminates */
}



/* Set up the standard Huffman tables (cf. JPEG standard section K.3) */
/* IMPORTANT: these are only valid for 8-bit data precision! */
void CHuffManDecoder::std_huff_tables (void)
{
  static const UINT8 dc_luminance_bits[17] =
    { /* 0-base */ 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
  static const UINT8 dc_luminance_val[] =
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
  
  static const UINT8 dc_chrominance_bits[17] =
    { /* 0-base */ 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
  static const UINT8 dc_chrominance_val[] =
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
  
  static const UINT8 ac_luminance_bits[17] =
    { /* 0-base */ 0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
  static const UINT8 ac_luminance_val[] =
    { 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
      0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
      0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
      0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
      0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
      0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
      0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
      0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
      0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
      0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
      0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
      0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
      0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
      0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
      0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
      0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
      0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
      0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
      0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
      0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
      0xf9, 0xfa };
  
  static const UINT8 ac_chrominance_bits[17] =
    { /* 0-base */ 0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
  static const UINT8 ac_chrominance_val[] =
    { 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
      0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
      0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
      0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
      0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
      0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
      0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
      0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
      0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
      0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
      0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
      0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
      0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
      0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
      0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
      0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
      0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
      0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
      0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
      0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
      0xf9, 0xfa };
  
  add_huff_table(&cinfo->dc_huff_tbl_ptrs[0], dc_luminance_bits, dc_luminance_val);
  add_huff_table(&cinfo->ac_huff_tbl_ptrs[0], ac_luminance_bits, ac_luminance_val);

  add_huff_table(&cinfo->dc_huff_tbl_ptrs[1], dc_chrominance_bits, dc_chrominance_val);
  add_huff_table(&cinfo->ac_huff_tbl_ptrs[1], ac_chrominance_bits, ac_chrominance_val);
}

/* Define a Huffman table */
void CHuffManDecoder::add_huff_table (HUFF_TBL **htblptr, const UINT8 *bits, const UINT8 *val)
{
  if (*htblptr == NULL)    *htblptr = (HUFF_TBL *) malloc(sizeof(HUFF_TBL));
  
  MEMCOPY((*htblptr)->bits, bits, sizeof((*htblptr)->bits));
  MEMCOPY((*htblptr)->huffval, val, sizeof((*htblptr)->huffval));
}

