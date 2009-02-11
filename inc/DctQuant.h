// DctQuant.h: interface for the CDctQuant class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DCTQUANT_H__5EC35C71_6627_4FBC_A677_D9F26DC82618__INCLUDED_)
#define AFX_DCTQUANT_H__5EC35C71_6627_4FBC_A677_D9F26DC82618__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/*
 * Perform the forward DCT on one block of samples.
 */
#define DCTSIZE2 64
#define DCTSIZE 8
typedef short DCTELEM;
typedef DCTELEM DCTBLOCK[DCTSIZE2];

#define EIGHT_BIT_SAMPLES 8 

class CDctQuant  
{
public:
	CDctQuant();
	virtual ~CDctQuant();

	void j_uvy_quant_Init(short factor);

	void j_fwd_dct (DCTBLOCK data);
	void j_rev_dct (DCTBLOCK data);

	void j_f_y_quant(DCTBLOCK data);
	void j_i_y_quant(DCTBLOCK data);

	void j_f_uv_quant(DCTBLOCK data);
	void j_i_uv_quant(DCTBLOCK data);


	void zigzag_trans(DCTBLOCK data);
	void zigzag_i_trans(DCTBLOCK data);

	short j_quality_scaling (short quality);

private:
	short std_luminance_quant_tbl[DCTSIZE2];
	short std_chrominance_quant_tbl[DCTSIZE2];

	short scale_factor;
};

#endif // !defined(AFX_DCTQUANT_H__5EC35C71_6627_4FBC_A677_D9F26DC82618__INCLUDED_)
