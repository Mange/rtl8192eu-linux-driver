/****************************************************************************** 
* 
* Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved. 
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
* You should have received a copy of the GNU General Public License along with 
* this program; if not, write to the Free Software Foundation, Inc., 
* 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA 
* 
* 
******************************************************************************/

/*Image2HeaderVersion: 2.14*/
#include "mp_precomp.h"
#include "../phydm_precomp.h"

#if (RTL8192E_SUPPORT == 1)
static BOOLEAN
CheckPositive(
	IN  PDM_ODM_T     pDM_Odm,
	IN  const u4Byte  Condition1,
	IN  const u4Byte  Condition2,
	IN	const u4Byte  Condition3,
	IN	const u4Byte  Condition4
)
{
	u1Byte    _BoardType = ((pDM_Odm->BoardType & BIT4) >> 4) << 0 | /* _GLNA*/
				((pDM_Odm->BoardType & BIT3) >> 3) << 1 | /* _GPA*/ 
				((pDM_Odm->BoardType & BIT7) >> 7) << 2 | /* _ALNA*/
				((pDM_Odm->BoardType & BIT6) >> 6) << 3 | /* _APA */
				((pDM_Odm->BoardType & BIT2) >> 2) << 4;  /* _BT*/  

	u4Byte	cond1   = Condition1, cond2 = Condition2, cond3 = Condition3, cond4 = Condition4;
	u4Byte    driver1 = pDM_Odm->CutVersion       << 24 | 
				(pDM_Odm->SupportInterface & 0xF0) << 16 | 
				pDM_Odm->SupportPlatform  << 16 | 
				pDM_Odm->PackageType      << 12 | 
				(pDM_Odm->SupportInterface & 0x0F) << 8  |
				_BoardType;

	u4Byte    driver2 = (pDM_Odm->TypeGLNA & 0xFF) <<  0 |  
				(pDM_Odm->TypeGPA & 0xFF)  <<  8 | 
				(pDM_Odm->TypeALNA & 0xFF) << 16 | 
				(pDM_Odm->TypeAPA & 0xFF)  << 24; 

u4Byte    driver3 = 0;

	u4Byte    driver4 = (pDM_Odm->TypeGLNA & 0xFF00) >>  8 |
				(pDM_Odm->TypeGPA & 0xFF00) |
				(pDM_Odm->TypeALNA & 0xFF00) << 8 |
				(pDM_Odm->TypeAPA & 0xFF00)  << 16;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("===> CheckPositive (cond1, cond2, cond3, cond4) = (0x%X 0x%X 0x%X 0x%X)\n", cond1, cond2, cond3, cond4));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("===> CheckPositive (driver1, driver2, driver3, driver4) = (0x%X 0x%X 0x%X 0x%X)\n", driver1, driver2, driver3, driver4));

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("	(Platform, Interface) = (0x%X, 0x%X)\n", pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_TRACE, 
	("	(Board, Package) = (0x%X, 0x%X)\n", pDM_Odm->BoardType, pDM_Odm->PackageType));


	/*============== Value Defined Check ===============*/
	/*QFN Type [15:12] and Cut Version [27:24] need to do value check*/
	
	if (((cond1 & 0x0000F000) != 0) && ((cond1 & 0x0000F000) != (driver1 & 0x0000F000)))
		return FALSE;
	if (((cond1 & 0x0F000000) != 0) && ((cond1 & 0x0F000000) != (driver1 & 0x0F000000)))
		return FALSE;

	/*=============== Bit Defined Check ================*/
	/* We don't care [31:28] */

	cond1   &= 0x00FF0FFF; 
	driver1 &= 0x00FF0FFF; 

	if ((cond1 & driver1) == cond1) {
		u4Byte bitMask = 0;

		if ((cond1 & 0x0F) == 0) /* BoardType is DONTCARE*/
			return TRUE;

		if ((cond1 & BIT0) != 0) /*GLNA*/
			bitMask |= 0x000000FF;
		if ((cond1 & BIT1) != 0) /*GPA*/
			bitMask |= 0x0000FF00;
		if ((cond1 & BIT2) != 0) /*ALNA*/
			bitMask |= 0x00FF0000;
		if ((cond1 & BIT3) != 0) /*APA*/
			bitMask |= 0xFF000000;

		if (((cond2 & bitMask) == (driver2 & bitMask)) && ((cond4 & bitMask) == (driver4 & bitMask)))  /* BoardType of each RF path is matched*/
			return TRUE;
		else
			return FALSE;
	} else
		return FALSE;
}
static BOOLEAN
CheckNegative(
	IN  PDM_ODM_T     pDM_Odm,
	IN  const u4Byte  Condition1,
	IN  const u4Byte  Condition2
)
{
	return TRUE;
}

/******************************************************************************
*                           RadioA.TXT
******************************************************************************/

u4Byte Array_MP_8192E_RadioA[] = { 
		0x07F, 0x00000082,
		0x081, 0x0003FC00,
		0x000, 0x00030000,
		0x008, 0x00008400,
		0x018, 0x00000407,
		0x019, 0x00000012,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x00000064,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0xA0000000,	0x00000000,
		0x01B, 0x0000394C,
	0xB0000000,	0x00000000,
		0x01E, 0x00080009,
		0x01F, 0x00000880,
		0x02F, 0x0001A060,
		0x03F, 0x00000000,
		0x042, 0x000060C0,
		0x057, 0x000D0000,
		0x058, 0x000BE180,
		0x067, 0x00001552,
		0x083, 0x00000000,
		0x0B0, 0x000FF9F1,
		0x0B1, 0x00055418,
		0x0B2, 0x0008CC00,
		0x0B4, 0x00043083,
		0x0B5, 0x00008166,
		0x0B6, 0x0000803E,
		0x0B7, 0x0001C69F,
		0x0B8, 0x0000407F,
		0x0B9, 0x00090001,
		0x0BA, 0x00040001,
		0x0BB, 0x00000400,
		0x0BC, 0x00000078,
		0x0BD, 0x000B3333,
		0x0BE, 0x00033340,
		0x0BF, 0x00000000,
		0x0C0, 0x00005999,
		0x0C1, 0x00009999,
		0x0C2, 0x00002400,
		0x0C3, 0x00000009,
		0x0C4, 0x00040C91,
		0x0C5, 0x00099999,
		0x0C6, 0x000000A3,
		0x0C7, 0x00088820,
		0x0C8, 0x00076C06,
		0x0C9, 0x00000000,
		0x0CA, 0x00080000,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x01C, 0x00000000,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x01C, 0x00000000,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x01C, 0x00000000,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x01C, 0x00000000,
	0xA0000000,	0x00000000,
		0x01C, 0x00000000,
	0xB0000000,	0x00000000,
		0x0DF, 0x00000180,
		0x0EF, 0x000001A0,
		0x051, 0x00069545,
		0x052, 0x0007E45E,
		0x053, 0x00000071,
		0x056, 0x00051FF3,
		0x035, 0x000000A8,
		0x035, 0x000001E2,
		0x035, 0x000002A8,
		0x036, 0x00001C24,
		0x036, 0x00009C24,
		0x036, 0x00011C24,
		0x036, 0x00019C24,
		0x018, 0x00000C07,
		0x05A, 0x00048000,
		0x019, 0x000739D0,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0xA0000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0xB0000000,	0x00000000,
		0x000, 0x00030159,
		0x084, 0x00068180,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0xA0000000,	0x00000000,
		0x086, 0x0000014E,
	0xB0000000,	0x00000000,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x00049F80,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x00049F80,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0xA0000000,	0x00000000,
		0x087, 0x00049F80,
	0xB0000000,	0x00000000,
		0x08E, 0x00065540,
		0x08F, 0x00088000,
		0x0EF, 0x000020A0,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F52B0,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F02B0,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0xA0000000,	0x00000000,
		0x03B, 0x000F02B0,
	0xB0000000,	0x00000000,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0xA0000000,	0x00000000,
		0x03B, 0x000EF7B0,
		0x03B, 0x000D4FB0,
		0x03B, 0x000CF060,
		0x03B, 0x000B0090,
		0x03B, 0x000A0080,
		0x03B, 0x00090080,
		0x03B, 0x0008F780,
	0xB0000000,	0x00000000,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x00070730,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x00078730,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0xA0000000,	0x00000000,
		0x03B, 0x00078730,
	0xB0000000,	0x00000000,
		0x03B, 0x00060FB0,
		0x03B, 0x0005FFA0,
		0x03B, 0x00040620,
		0x03B, 0x00037090,
		0x03B, 0x00020080,
		0x03B, 0x0001F060,
		0x03B, 0x0000FFB0,
		0x0EF, 0x000000A0,
		0x0FE, 0x00000000,
		0x018, 0x0000FC07,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x01E, 0x00000001,
		0x01F, 0x00080000,
		0x000, 0x00033E70,

};

void
ODM_ReadAndConfig_MP_8192E_RadioA(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i         = 0;
	u1Byte     cCond;
	BOOLEAN bMatched = TRUE, bSkipped = FALSE;
	u4Byte     ArrayLen    = sizeof(Array_MP_8192E_RadioA)/sizeof(u4Byte);
	pu4Byte    Array       = Array_MP_8192E_RadioA;
	
	u4Byte	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8192E_RadioA\n"));

	while ((i + 1) < ArrayLen) {
		v1 = Array[i];
		v2 = Array[i + 1];

		if (v1 & (BIT31 | BIT30)) {/*positive & negative condition*/
			if (v1 & BIT31) {/* positive condition*/
				cCond  = (u1Byte)((v1 & (BIT29|BIT28)) >> 28);
				if (cCond == COND_ENDIF) {/*end*/
					bMatched = TRUE;
					bSkipped = FALSE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ENDIF\n"));
				} else if (cCond == COND_ELSE) { /*else*/
					bMatched = bSkipped?FALSE:TRUE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ELSE\n"));
				}
				else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("IF or ELSE IF\n"));
				}
			} else if (v1 & BIT30) { /*negative condition*/
				if (bSkipped == FALSE) {
					if (CheckPositive(pDM_Odm, pre_v1, pre_v2, v1, v2)) {
						bMatched = TRUE;
						bSkipped = TRUE;
					} else {
						bMatched = FALSE;
						bSkipped = FALSE;
					}
				} else
					bMatched = FALSE;
			}
		} else {
			if (bMatched)
				odm_ConfigRF_RadioA_8192E(pDM_Odm, v1, v2);
		}
		i = i + 2;
	}
}

u4Byte
ODM_GetVersion_MP_8192E_RadioA(void)
{
	   return 51;
}

/******************************************************************************
*                           RadioB.TXT
******************************************************************************/

u4Byte Array_MP_8192E_RadioB[] = { 
		0x07F, 0x00000082,
		0x081, 0x0003FC00,
		0x000, 0x00030000,
		0x008, 0x00008400,
		0x018, 0x00000407,
		0x019, 0x00000012,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x00000064,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x01B, 0x0000146C,
	0xA0000000,	0x00000000,
		0x01B, 0x0000394C,
	0xB0000000,	0x00000000,
		0x01E, 0x00080009,
		0x01F, 0x00000880,
		0x02F, 0x0001A060,
		0x03F, 0x00000000,
		0x042, 0x000060C0,
		0x057, 0x000D0000,
		0x058, 0x000BE180,
		0x067, 0x00001552,
		0x07F, 0x00000082,
		0x081, 0x0003F000,
		0x083, 0x00000000,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x01C, 0x00001C00,
	0xA0000000,	0x00000000,
		0x01C, 0x00000000,
	0xB0000000,	0x00000000,
		0x0DF, 0x00000180,
		0x0EF, 0x000001A0,
		0x051, 0x00069545,
		0x052, 0x0007E42E,
		0x053, 0x00000071,
		0x056, 0x00051FF3,
		0x035, 0x000000A8,
		0x035, 0x000001E0,
		0x035, 0x000002A8,
		0x036, 0x00001CA8,
		0x036, 0x00009C24,
		0x036, 0x00011C24,
		0x036, 0x00019C24,
		0x018, 0x00000C07,
		0x05A, 0x00048000,
		0x019, 0x000739D0,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000A093,
		0x034, 0x0000908F,
		0x034, 0x0000808C,
		0x034, 0x0000704D,
		0x034, 0x0000604A,
		0x034, 0x00005047,
		0x034, 0x0000400A,
		0x034, 0x00003007,
		0x034, 0x00002004,
		0x034, 0x00001001,
		0x034, 0x00000000,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0xA0000000,	0x00000000,
		0x034, 0x0000ADD7,
		0x034, 0x00009DD4,
		0x034, 0x00008DD1,
		0x034, 0x00007DCE,
		0x034, 0x00006DCB,
		0x034, 0x00005DC8,
		0x034, 0x00004DC5,
		0x034, 0x000034CC,
		0x034, 0x0000244F,
		0x034, 0x0000144C,
		0x034, 0x00000014,
	0xB0000000,	0x00000000,
		0x000, 0x00030159,
		0x084, 0x00068180,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x086, 0x0000010E,
	0xA0000000,	0x00000000,
		0x086, 0x000000CE,
	0xB0000000,	0x00000000,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x00049F80,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x00049F80,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x087, 0x000F5F80,
	0xA0000000,	0x00000000,
		0x087, 0x00049F80,
	0xB0000000,	0x00000000,
		0x08E, 0x00065540,
		0x08F, 0x00088000,
		0x0EF, 0x000020A0,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F52B0,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F02B0,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000F0730,
	0xA0000000,	0x00000000,
		0x03B, 0x000F02B0,
	0xB0000000,	0x00000000,
	0x80000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000E0730,
		0x03B, 0x000D0020,
		0x03B, 0x000C0020,
		0x03B, 0x000B0760,
		0x03B, 0x000A0010,
		0x03B, 0x00090000,
		0x03B, 0x00080000,
	0xA0000000,	0x00000000,
		0x03B, 0x000EF7B0,
		0x03B, 0x000D4FB0,
		0x03B, 0x000CF060,
		0x03B, 0x000B0090,
		0x03B, 0x000A0080,
		0x03B, 0x00090080,
		0x03B, 0x0008F780,
	0xB0000000,	0x00000000,
	0x80000400,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x00070730,
	0x90000003,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000003,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000002,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x00078730,
	0x90000001,	0x00000000,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x00000005,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x0000000a,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0x90000001,	0x0000000f,	0x40000000,	0x00000000,
		0x03B, 0x000787B0,
	0xA0000000,	0x00000000,
		0x03B, 0x00078730,
	0xB0000000,	0x00000000,
		0x03B, 0x00060FB0,
		0x03B, 0x0005FFA0,
		0x03B, 0x00040620,
		0x03B, 0x00037090,
		0x03B, 0x00020080,
		0x03B, 0x0001F060,
		0x03B, 0x0000FFB0,
		0x0EF, 0x000000A0,
		0x000, 0x00010159,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x0FE, 0x00000000,
		0x01E, 0x00000001,
		0x01F, 0x00080000,
		0x000, 0x00033E70,

};

void
ODM_ReadAndConfig_MP_8192E_RadioB(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i         = 0;
	u1Byte     cCond;
	BOOLEAN bMatched = TRUE, bSkipped = FALSE;
	u4Byte     ArrayLen    = sizeof(Array_MP_8192E_RadioB)/sizeof(u4Byte);
	pu4Byte    Array       = Array_MP_8192E_RadioB;
	
	u4Byte	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8192E_RadioB\n"));

	while ((i + 1) < ArrayLen) {
		v1 = Array[i];
		v2 = Array[i + 1];

		if (v1 & (BIT31 | BIT30)) {/*positive & negative condition*/
			if (v1 & BIT31) {/* positive condition*/
				cCond  = (u1Byte)((v1 & (BIT29|BIT28)) >> 28);
				if (cCond == COND_ENDIF) {/*end*/
					bMatched = TRUE;
					bSkipped = FALSE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ENDIF\n"));
				} else if (cCond == COND_ELSE) { /*else*/
					bMatched = bSkipped?FALSE:TRUE;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("ELSE\n"));
				}
				else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("IF or ELSE IF\n"));
				}
			} else if (v1 & BIT30) { /*negative condition*/
				if (bSkipped == FALSE) {
					if (CheckPositive(pDM_Odm, pre_v1, pre_v2, v1, v2)) {
						bMatched = TRUE;
						bSkipped = TRUE;
					} else {
						bMatched = FALSE;
						bSkipped = FALSE;
					}
				} else
					bMatched = FALSE;
			}
		} else {
			if (bMatched)
				odm_ConfigRF_RadioB_8192E(pDM_Odm, v1, v2);
		}
		i = i + 2;
	}
}

u4Byte
ODM_GetVersion_MP_8192E_RadioB(void)
{
	   return 51;
}

/******************************************************************************
*                           TxPowerTrack_AP.TXT
******************************************************************************/

#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_AP_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_AP_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 16, 17, 17, 18, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_AP_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_AP_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_AP_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_AP_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_AP_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_AP_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_AP_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_AP_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_AP_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_AP_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
#endif

void
ODM_ReadAndConfig_MP_8192E_TxPowerTrack_AP(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8192E\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_AP_8192E, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TxPowerTrack_PCIE.TXT
******************************************************************************/

#if DEV_BUS_TYPE == RT_PCI_INTERFACE
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_PCIE_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_PCIE_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 16, 17, 17, 18, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_PCIE_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_PCIE_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_PCIE_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_PCIE_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_PCIE_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_PCIE_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_PCIE_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_PCIE_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_PCIE_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_PCIE_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
#endif

void
ODM_ReadAndConfig_MP_8192E_TxPowerTrack_PCIE(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if DEV_BUS_TYPE == RT_PCI_INTERFACE
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8192E\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_PCIE_8192E, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TxPowerTrack_SDIO.TXT
******************************************************************************/

#if DEV_BUS_TYPE == RT_SDIO_INTERFACE
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_SDIO_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_SDIO_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 16, 17, 17, 18, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_SDIO_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_SDIO_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_SDIO_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_SDIO_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_SDIO_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_SDIO_8192E[]    = {0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_SDIO_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_SDIO_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_SDIO_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_SDIO_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
#endif

void
ODM_ReadAndConfig_MP_8192E_TxPowerTrack_SDIO(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if DEV_BUS_TYPE == RT_SDIO_INTERFACE
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8192E\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_SDIO_8192E, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TxPowerTrack_USB.TXT
******************************************************************************/

#if DEV_BUS_TYPE == RT_USB_INTERFACE
u1Byte gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_USB_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
};
u1Byte gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_USB_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 16, 17, 17, 18, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 19, 20, 20, 20},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_USB_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16},
};
u1Byte gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_USB_8192E[][DELTA_SWINGIDX_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 21, 21},
};
u1Byte gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_USB_8192E[]    = {0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 7, 7, 7, 8, 9, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_USB_8192E[]    = {0, 0, 0, 1, 2, 2, 2, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_USB_8192E[]    = {0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 7, 7, 7, 8, 9, 9, 9, 10, 10, 11, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_USB_8192E[]    = {0, 0, 0, 1, 2, 2, 2, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_USB_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_USB_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_USB_8192E[] = {0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
u1Byte gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_USB_8192E[] = {0, 0, 0, 0, 1, 1, 1, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 7, 7, 7, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15};
#endif

void
ODM_ReadAndConfig_MP_8192E_TxPowerTrack_USB(
	IN   PDM_ODM_T  pDM_Odm
)
{
#if DEV_BUS_TYPE == RT_USB_INTERFACE
	PODM_RF_CAL_T  pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_MP_8192E\n"));


	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P, gDeltaSwingTableIdx_MP_2GA_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N, gDeltaSwingTableIdx_MP_2GA_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P, gDeltaSwingTableIdx_MP_2GB_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N, gDeltaSwingTableIdx_MP_2GB_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P, gDeltaSwingTableIdx_MP_2GCCKA_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N, gDeltaSwingTableIdx_MP_2GCCKA_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P, gDeltaSwingTableIdx_MP_2GCCKB_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N, gDeltaSwingTableIdx_MP_2GCCKB_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE);

	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_P, gDeltaSwingTableIdx_MP_5GA_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GA_N, gDeltaSwingTableIdx_MP_5GA_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_P, gDeltaSwingTableIdx_MP_5GB_P_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE*3);
	ODM_MoveMemory(pDM_Odm, pRFCalibrateInfo->DeltaSwingTableIdx_5GB_N, gDeltaSwingTableIdx_MP_5GB_N_TxPowerTrack_USB_8192E, DELTA_SWINGIDX_SIZE*3);
#endif
}

/******************************************************************************
*                           TXPWR_LMT.TXT
******************************************************************************/

const char *Array_MP_8192E_TXPWR_LMT[] = { 
	"FCC", "2.4G", "20M", "CCK", "1T", "01", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "01", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "02", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "02", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "02", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "03", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "03", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "03", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "04", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "04", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "04", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "05", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "05", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "05", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "06", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "06", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "06", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "07", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "07", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "07", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "08", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "08", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "08", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "09", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "09", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "09", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "10", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "10", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "10", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "11", "34", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "11", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "11", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "12", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "12", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "CCK", "1T", "13", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "CCK", "1T", "14", "32",
	"FCC", "2.4G", "20M", "CCK", "2T", "01", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "01", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "01", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "02", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "02", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "02", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "03", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "03", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "03", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "04", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "04", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "04", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "05", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "05", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "05", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "06", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "06", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "06", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "07", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "07", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "07", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "08", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "08", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "08", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "09", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "09", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "09", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "10", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "10", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "10", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "11", "26", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "11", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "11", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "12", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "12", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "13", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "13", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "CCK", "2T", "14", "26",
	"FCC", "2.4G", "20M", "OFDM", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "01", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "02", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "02", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "03", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "03", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "04", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "04", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "05", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "05", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "06", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "06", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "07", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "07", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "08", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "08", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "09", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "09", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "09", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "10", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "10", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "10", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "11", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "11", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "11", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "12", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "12", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "14", "63",
	"FCC", "2.4G", "20M", "OFDM", "2T", "01", "26", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "01", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "02", "26", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "02", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "03", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "03", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "04", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "04", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "05", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "05", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "06", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "06", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "07", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "07", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "08", "34", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "08", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "09", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "09", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "10", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "10", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "11", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "11", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "12", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "13", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "01", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "02", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "02", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "03", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "03", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "04", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "04", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "05", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "05", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "06", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "06", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "07", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "07", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "08", "34", 
	"ETSI", "2.4G", "20M", "HT", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "08", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "09", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "09", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "09", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "10", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "10", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "10", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "11", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "11", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "11", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "12", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "12", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "13", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "2T", "01", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "01", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "02", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "02", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "03", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "03", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "04", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "04", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "05", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "05", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "06", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "06", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "07", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "07", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "08", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "08", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "09", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "10", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "11", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "12", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "13", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "2T", "01", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "01", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "02", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "02", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "03", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "03", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "04", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "04", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "05", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "05", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "06", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "06", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "07", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "07", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "08", "34", 
	"ETSI", "2.4G", "20M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "08", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "09", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "10", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "11", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "12", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "13", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "03", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "03", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "03", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "04", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "04", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "04", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "05", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "05", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "05", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "06", "32", 
	"ETSI", "2.4G", "40M", "HT", "1T", "06", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "06", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "07", "32", 
	"ETSI", "2.4G", "40M", "HT", "1T", "07", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "07", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "08", "32", 
	"ETSI", "2.4G", "40M", "HT", "1T", "08", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "08", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "09", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "09", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "09", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "10", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "10", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "10", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "11", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "11", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "11", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "03", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "04", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "05", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "06", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "07", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "08", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "09", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "10", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "11", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "03", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "04", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "05", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "06", "30", 
	"ETSI", "2.4G", "40M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "07", "30", 
	"ETSI", "2.4G", "40M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "08", "30", 
	"ETSI", "2.4G", "40M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "14", "63"
};

void
ODM_ReadAndConfig_MP_8192E_TXPWR_LMT(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i           = 0;
	u4Byte     ArrayLen    = sizeof(Array_MP_8192E_TXPWR_LMT)/sizeof(pu1Byte);
	pu1Byte    *Array      = (pu1Byte *)Array_MP_8192E_TXPWR_LMT;

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PADAPTER		Adapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	PlatformZeroMemory(pHalData->BufOfLinesPwrLmt, MAX_LINES_HWCONFIG_TXT*MAX_BYTES_LINE_HWCONFIG_TXT);
	pHalData->nLinesReadPwrLmt = ArrayLen/7;
#endif

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8192E_TXPWR_LMT\n"));

	for (i = 0; i < ArrayLen; i += 7) {
		pu1Byte regulation = Array[i];
		pu1Byte band = Array[i+1];
		pu1Byte bandwidth = Array[i+2];
		pu1Byte rate = Array[i+3];
		pu1Byte rfPath = Array[i+4];
		pu1Byte chnl = Array[i+5];
		pu1Byte val = Array[i+6];
	
		odm_ConfigBB_TXPWR_LMT_8192E(pDM_Odm, regulation, band, bandwidth, rate, rfPath, chnl, val);
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
		rsprintf(pHalData->BufOfLinesPwrLmt[i/7], 100, "\"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\",",
			regulation, band, bandwidth, rate, rfPath, chnl, val);
#endif
	}

}

/******************************************************************************
*                           TXPWR_LMT_8192E_SAR_5mm.TXT
******************************************************************************/

const char *Array_MP_8192E_TXPWR_LMT_8192E_SAR_5mm[] = { 
	"FCC", "2.4G", "20M", "CCK", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "01", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "01", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "02", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "02", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "03", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "03", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "04", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "04", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "05", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "05", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "06", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "06", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "07", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "07", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "08", "30", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "08", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "09", "28", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "09", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "09", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "10", "28", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "10", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "10", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "11", "28", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "11", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "11", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "12", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "12", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "13", "32", 
	"MKK", "2.4G", "20M", "CCK", "1T", "13", "32",
	"FCC", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "CCK", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "CCK", "1T", "14", "32",
	"FCC", "2.4G", "20M", "CCK", "2T", "01", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "01", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "01", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "02", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "02", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "02", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "03", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "03", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "03", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "04", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "04", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "04", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "05", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "05", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "05", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "06", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "06", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "06", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "07", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "07", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "07", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "08", "24", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "08", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "08", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "09", "22", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "09", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "09", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "10", "22", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "10", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "10", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "11", "22", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "11", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "11", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "12", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "12", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "13", "22", 
	"MKK", "2.4G", "20M", "CCK", "2T", "13", "26",
	"FCC", "2.4G", "20M", "CCK", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "CCK", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "CCK", "2T", "14", "26",
	"FCC", "2.4G", "20M", "OFDM", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "01", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "02", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "02", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "03", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "03", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "04", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "04", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "05", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "05", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "06", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "06", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "07", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "07", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "08", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "08", "34",
	"FCC", "2.4G", "20M", "OFDM", "1T", "09", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "09", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "09", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "10", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "10", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "10", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "11", "30", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "11", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "11", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "12", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "12", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"FCC", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "OFDM", "1T", "14", "63",
	"FCC", "2.4G", "20M", "OFDM", "2T", "01", "26", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "01", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "02", "26", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "02", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "03", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "03", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "04", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "04", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "05", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "05", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "06", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "06", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "07", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "07", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "08", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "08", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "09", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "09", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "10", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "10", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "11", "28", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "11", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "12", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "13", "28",
	"FCC", "2.4G", "20M", "OFDM", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "OFDM", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "OFDM", "2T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "1T", "01", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "01", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "01", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "02", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "02", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "02", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "03", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "03", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "03", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "04", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "04", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "04", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "05", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "05", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "05", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "06", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "06", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "06", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "07", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "07", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "07", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "08", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "08", "32", 
	"MKK", "2.4G", "20M", "HT", "1T", "08", "34",
	"FCC", "2.4G", "20M", "HT", "1T", "09", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "09", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "09", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "10", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "10", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "10", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "11", "30", 
	"ETSI", "2.4G", "20M", "HT", "1T", "11", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "11", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "12", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "12", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "13", "28", 
	"MKK", "2.4G", "20M", "HT", "1T", "13", "28",
	"FCC", "2.4G", "20M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "2T", "01", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "01", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "02", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "02", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "03", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "03", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "04", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "04", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "05", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "05", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "06", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "06", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "07", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "07", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "08", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "08", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "09", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "10", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "11", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "12", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "13", "28",
	"FCC", "2.4G", "20M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "20M", "HT", "2T", "01", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "01", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "01", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "02", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "02", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "02", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "03", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "03", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "04", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "04", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "05", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "05", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "06", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "06", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "07", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "07", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "08", "28", 
	"ETSI", "2.4G", "20M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "08", "30",
	"FCC", "2.4G", "20M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "20M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "12", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "12", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "13", "26", 
	"MKK", "2.4G", "20M", "HT", "2T", "13", "26",
	"FCC", "2.4G", "20M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "20M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "20M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "03", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "03", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "03", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "04", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "04", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "04", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "05", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "05", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "05", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "06", "30", 
	"ETSI", "2.4G", "40M", "HT", "1T", "06", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "06", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "07", "30", 
	"ETSI", "2.4G", "40M", "HT", "1T", "07", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "07", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "08", "30", 
	"ETSI", "2.4G", "40M", "HT", "1T", "08", "32", 
	"MKK", "2.4G", "40M", "HT", "1T", "08", "30",
	"FCC", "2.4G", "40M", "HT", "1T", "09", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "09", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "09", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "10", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "10", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "10", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "11", "28", 
	"ETSI", "2.4G", "40M", "HT", "1T", "11", "28", 
	"MKK", "2.4G", "40M", "HT", "1T", "11", "28",
	"FCC", "2.4G", "40M", "HT", "1T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "1T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "1T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "1T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "03", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "04", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "05", "20", 
	"ETSI", "2.4G", "40M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "06", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "07", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "08", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "09", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "10", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "11", "22", 
	"ETSI", "2.4G", "40M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "14", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "01", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "01", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "01", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "02", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "02", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "02", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "03", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "03", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "03", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "04", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "04", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "04", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "05", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "05", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "05", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "06", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "06", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "06", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "07", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "07", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "07", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "08", "28", 
	"ETSI", "2.4G", "40M", "HT", "2T", "08", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "08", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "09", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "09", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "09", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "10", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "10", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "10", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "11", "26", 
	"ETSI", "2.4G", "40M", "HT", "2T", "11", "26", 
	"MKK", "2.4G", "40M", "HT", "2T", "11", "26",
	"FCC", "2.4G", "40M", "HT", "2T", "12", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "12", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "12", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "13", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "13", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "13", "63",
	"FCC", "2.4G", "40M", "HT", "2T", "14", "63", 
	"ETSI", "2.4G", "40M", "HT", "2T", "14", "63", 
	"MKK", "2.4G", "40M", "HT", "2T", "14", "63"
};

void
ODM_ReadAndConfig_MP_8192E_TXPWR_LMT_8192E_SAR_5mm(
	IN   PDM_ODM_T  pDM_Odm
)
{
	u4Byte     i           = 0;
	u4Byte     ArrayLen    = sizeof(Array_MP_8192E_TXPWR_LMT_8192E_SAR_5mm)/sizeof(pu1Byte);
	pu1Byte    *Array      = (pu1Byte *)Array_MP_8192E_TXPWR_LMT_8192E_SAR_5mm;

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PADAPTER		Adapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	PlatformZeroMemory(pHalData->BufOfLinesPwrLmt, MAX_LINES_HWCONFIG_TXT*MAX_BYTES_LINE_HWCONFIG_TXT);
	pHalData->nLinesReadPwrLmt = ArrayLen/7;
#endif

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("===> ODM_ReadAndConfig_MP_8192E_TXPWR_LMT_8192E_SAR_5mm\n"));

	for (i = 0; i < ArrayLen; i += 7) {
		pu1Byte regulation = Array[i];
		pu1Byte band = Array[i+1];
		pu1Byte bandwidth = Array[i+2];
		pu1Byte rate = Array[i+3];
		pu1Byte rfPath = Array[i+4];
		pu1Byte chnl = Array[i+5];
		pu1Byte val = Array[i+6];
	
		odm_ConfigBB_TXPWR_LMT_8192E(pDM_Odm, regulation, band, bandwidth, rate, rfPath, chnl, val);
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
		rsprintf(pHalData->BufOfLinesPwrLmt[i/7], 100, "\"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\",",
			regulation, band, bandwidth, rate, rfPath, chnl, val);
#endif
	}

}

#endif /* end of HWIMG_SUPPORT*/

