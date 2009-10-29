// uyvy2rgb.h: 
//
//////////////////////////////////////////////////////////////////////

#if !defined(UYVY2RGB_H)
#define UYVY2RGB_H


unsigned char UYVY2RGB(int m_nQuality,const unsigned char *lpbuffer, 
		       int nSize, unsigned char *lpRGB, 
		       int nRow, int nCol);

#endif // !defined(UYVY2RGB_H)
