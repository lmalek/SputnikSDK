// JpegDecode.c: 
//
//////////////////////////////////////////////////////////////////////
#include "uyvy2rgb.h" //consit a heder for a ANSI C programing
#include <memory.h>
#include <stdlib.h>  // for abs
#include "JpegDecodeHuffMan.h"
#include "DctQuant.h"

#define CENTERJSAMPLE	128
//extern "C" unsigned char UYVY2RGB(int m_nQuality,const unsigned char *lpbuffer,
//				  int nSize, unsigned char *lpRGB, int nRow, 
//				  int nCol);
// added by eliu, most was copied from DecodeImage
unsigned char DecodeUYVY(int m_nQuality,
		     const unsigned char *lpbuffer, int nSize, unsigned char *YData, 
		     unsigned char *UVData, int nRow, int nCol) {
  CHuffManDecoder m_decode;
  CDctQuant       m_dctquant;
  int i,j,m,n;
  int nImageSize = nRow * nCol;

  m_dctquant.j_uvy_quant_Init(m_nQuality);
  if( m_decode.Huff_Decode_Init() == 0 ) return false;

  if( m_decode.Huff_Decode_Buffer((const char*)lpbuffer,nSize) == 0 )  return false;  // eliu changed

  // reset 
  memset(YData,0x00,nImageSize);
  memset(UVData,0x00,nImageSize);

  // Compute Y
  // 18 X 22 blocks
  short block[64];
  int offset = 0;
  
  for( i=0; i< nRow/8; i++) { 
    for( j=0; j<nCol/8; j++){
      memset(block,0,64*sizeof(short));	    // Y block
      m_decode.Huff_Decode_One_Block(block, 0);
      m_dctquant.zigzag_i_trans(block);	    //zigzag
      m_dctquant.j_i_y_quant(block);	    //quantization
      m_dctquant.j_rev_dct(block);	    //DCT
      for( m=0; m<8; m++)	 {
	for(n=0; n<8; n++) {
	  if( block[m*8+n] > 120 )  block[m*8+n] = 120;
	  if( block[m*8+n] < -120 )  block[m*8+n] = -120;
	  YData[(i*8 +m)*nCol+(j*8 + n)] = block[m*8 + n] + CENTERJSAMPLE;
	  offset ++;
	}
	offset += nCol;
      }
      offset += 8;
    }
    offset +=  ( nCol <<  3);
  }
  // compute UV
  offset = 0;
  for( i=0; i< nRow/8; i++)  {
    for( j=0; j<nCol/8; j++) {
      memset(block,0,64*sizeof(short)); 	    // UV block
      m_decode.Huff_Decode_One_Block(block, 1);
      m_dctquant.zigzag_i_trans(block); 	    //zigzag
      m_dctquant.j_i_uv_quant(block);	    //quantization
      m_dctquant.j_rev_dct(block);	    //DCT
      for( m=0; m<8; m++)  {
	for(n=0; n<8; n++) {
	  UVData[(i*8 +m)*nCol+(j*8 + n)] = (unsigned char)block[m*8 + n];
	  offset ++;
	}
	offset += nCol;
      }
      offset += 8;
    }
    offset += ( nCol << 3 ) ;
  }
  m_decode.Huff_Decode_Free();
  return 1;
}


// Added by ELiu, size of lpRGB should be >= ( nRow * nCol * 3 )

// assume nRow*nCol is even number
unsigned char UYVY2RGB(int m_nQuality,const unsigned char *lpbuffer, int nSize, 
		     unsigned char *lpRGB, int nRow, int nCol){

  int nImageSize = nRow * nCol;
  unsigned char *YData = (unsigned char*)calloc(nImageSize,sizeof(unsigned char));
  if ( YData == NULL ) return 0; 
  unsigned char *UVData = (unsigned char*)calloc(nImageSize,sizeof(unsigned char));
  if ( UVData == NULL ) return 0;
  // real rgb
  unsigned char bRet = 0;
  int nTmp;

  if( DecodeUYVY(m_nQuality,lpbuffer, nSize, YData, UVData, nRow, nCol)) {
    // UYVY to RGB
    unsigned char *lp = lpRGB;
    
    unsigned char Y,U,V;
    
    for (int i=0;i<nImageSize-1;i+=2){   // assure even number
      Y = YData[i];
      U = UVData[i];
      V = UVData[i+1];
   
      nTmp = abs((int)(Y + 1.402*(V-128)));
      lp[0] = nTmp>255?255:nTmp;
      nTmp = abs((int)(Y - 0.3441*(U-128) - 0.7139 *(V-128)));
      lp[1] = nTmp>255?255:nTmp;
      nTmp = abs((int)(Y + 1.7718*(U-128) - 0.0012 *(V-128)));
      lp[2] = nTmp>255?255:nTmp;
      lp += 3;
   		
      // next pixel
      Y = YData[i+1];
      
      nTmp = abs((int)(Y + 1.402*(V-128)));
      lp[0] = nTmp>255?255:nTmp;
      nTmp = abs((int)(Y - 0.3441*(U-128) - 0.7139 *(V-128)));
      lp[1] = nTmp>255?255:nTmp;
      nTmp = abs((int)(Y + 1.7718*(U-128) - 0.0012 *(V-128)));
      lp[2] = nTmp>255?255:nTmp;
      lp += 3;
      
    }
    bRet = 1;
  }
  free(YData);
  free(UVData);
  return bRet;

}


