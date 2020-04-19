/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/

/* #include "Mp_Precomp.h" */
/* #include "../odm_precomp.h" */

#include <drv_types.h>

#include "HalEfuseMask8192E_SDIO.h"



/******************************************************************************
*                           MSDIO.TXT
******************************************************************************/

u1Byte Array_MP_8192E_MSDIO[] = {
	0xFF,
	0xF3,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x0F,
	0xF1,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,

};

u2Byte EFUSE_GetArrayLen_MP_8192E_MSDIO(VOID)
{
	return sizeof(Array_MP_8192E_MSDIO) / sizeof(u1Byte);
}

VOID EFUSE_GetMaskArray_MP_8192E_MSDIO(pu1Byte Array)
{
	u2Byte len = EFUSE_GetArrayLen_MP_8192E_MSDIO(), i = 0;

	for (i = 0; i < len; ++i)
		Array[i] = Array_MP_8192E_MSDIO[i];
}

BOOLEAN EFUSE_IsAddressMasked_MP_8192E_MSDIO(u2Byte Offset)
{
	int r = Offset / 16;
	int c = (Offset % 16) / 2;
	int result = 0;

	/* RTW_INFO(" %s ,Offset=%x r= %d , c=%d , Array_MP_8192E_MSDIO[r]= %x\n",__func__,Offset,r,c,Array_MP_8192E_MSDIO[r]); */

	if (c < 4) /* Upper double word */
		result = (Array_MP_8192E_MSDIO[r] & (0x10 << c));
	else
		result = (Array_MP_8192E_MSDIO[r] & (0x01 << (c - 4)));

	return (result > 0) ? 0 : 1;
}
