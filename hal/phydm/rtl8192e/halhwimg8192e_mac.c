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
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/

/*Image2HeaderVersion: R3 1.5.0*/
#include "mp_precomp.h"
#include "../phydm_precomp.h"

#define D_S_SIZE DELTA_SWINGIDX_SIZE

#if (RTL8192E_SUPPORT == 1)
static boolean
check_positive(struct dm_struct *dm,
	       const u32	condition1,
	       const u32	condition2,
	       const u32	condition3,
	       const u32	condition4
)
{
	u32	cond1 = condition1, cond2 = condition2,
		cond3 = condition3, cond4 = condition4;

	u8	cut_version_for_para =
		(dm->cut_version ==  ODM_CUT_A) ? 15 : dm->cut_version;

	u8	pkg_type_for_para =
		(dm->package_type == 0) ? 15 : dm->package_type;

	u32	driver1 = cut_version_for_para << 24 |
			(dm->support_interface & 0xF0) << 16 |
			dm->support_platform << 16 |
			pkg_type_for_para << 12 |
			(dm->support_interface & 0x0F) << 8  |
			dm->rfe_type;

	u32	driver2 = (dm->type_glna & 0xFF) <<  0 |
			(dm->type_gpa & 0xFF)  <<  8 |
			(dm->type_alna & 0xFF) << 16 |
			(dm->type_apa & 0xFF)  << 24;

	u32	driver3 = 0;

	u32	driver4 = (dm->type_glna & 0xFF00) >>  8 |
			(dm->type_gpa & 0xFF00) |
			(dm->type_alna & 0xFF00) << 8 |
			(dm->type_apa & 0xFF00)  << 16;

	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "===> %s (cond1, cond2, cond3, cond4) = (0x%X 0x%X 0x%X 0x%X)\n",
		  __func__, cond1, cond2, cond3, cond4);
	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "===> %s (driver1, driver2, driver3, driver4) = (0x%X 0x%X 0x%X 0x%X)\n",
		  __func__, driver1, driver2, driver3, driver4);

	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "	(Platform, Interface) = (0x%X, 0x%X)\n",
		  dm->support_platform, dm->support_interface);
	PHYDM_DBG(dm, ODM_COMP_INIT, "	(RFE, Package) = (0x%X, 0x%X)\n",
		  dm->rfe_type, dm->package_type);

	/*============== value Defined Check ===============*/
	/*cut version [27:24] need to do value check*/
	if (((cond1 & 0x0F000000) != 0) &&
	    ((cond1 & 0x0F000000) != (driver1 & 0x0F000000)))
		return false;

	/*pkg type [15:12] need to do value check*/
	if (((cond1 & 0x0000F000) != 0) &&
	    ((cond1 & 0x0000F000) != (driver1 & 0x0000F000)))
		return false;

	/*interface [11:8] need to do value check*/
	if (((cond1 & 0x00000F00) != 0) &&
	    ((cond1 & 0x00000F00) != (driver1 & 0x00000F00)))
		return false;
	/*=============== Bit Defined Check ================*/
	/* We don't care [31:28] */

	cond1 &= 0x000000FF;
	driver1 &= 0x000000FF;

	if (cond1 == driver1)
		return true;
	else
		return false;
}

static boolean
check_negative(struct dm_struct *dm,
	       const u32	condition1,
	       const u32	condition2
)
{
	return true;
}

/******************************************************************************
 *                           mac_reg.TXT
 ******************************************************************************/

const u32 array_mp_8192e_mac_reg[] = {
		0x011, 0x000000EB,
		0x012, 0x00000007,
		0x014, 0x00000075,
		0x303, 0x000000A7,
		0x421, 0x0000000F,
		0x428, 0x0000000A,
		0x429, 0x00000010,
		0x430, 0x00000000,
		0x431, 0x00000000,
		0x432, 0x00000000,
		0x433, 0x00000001,
		0x434, 0x00000002,
		0x435, 0x00000003,
		0x436, 0x00000005,
		0x437, 0x00000007,
		0x438, 0x00000000,
		0x439, 0x00000000,
		0x43A, 0x00000000,
		0x43B, 0x00000001,
		0x43C, 0x00000002,
		0x43D, 0x00000003,
		0x43E, 0x00000005,
		0x43F, 0x00000007,
		0x440, 0x0000005D,
		0x441, 0x00000001,
		0x442, 0x00000000,
		0x444, 0x00000010,
		0x445, 0x00000000,
		0x446, 0x00000000,
		0x447, 0x00000000,
		0x448, 0x00000000,
		0x449, 0x000000F0,
		0x44A, 0x0000000F,
		0x44B, 0x0000003E,
		0x44C, 0x00000010,
		0x44D, 0x00000000,
		0x44E, 0x00000000,
		0x44F, 0x00000000,
		0x450, 0x00000000,
		0x451, 0x000000F0,
		0x452, 0x0000000F,
		0x453, 0x00000000,
		0x456, 0x0000005E,
		0x460, 0x00000066,
		0x461, 0x00000066,
		0x4C8, 0x000000FF,
		0x4C9, 0x00000008,
		0x4CC, 0x000000FF,
		0x4CD, 0x000000FF,
		0x4CE, 0x00000001,
		0x500, 0x00000026,
		0x501, 0x000000A2,
		0x502, 0x0000002F,
		0x503, 0x00000000,
		0x504, 0x00000028,
		0x505, 0x000000A3,
		0x506, 0x0000005E,
		0x507, 0x00000000,
		0x508, 0x0000002B,
		0x509, 0x000000A4,
		0x50A, 0x0000005E,
		0x50B, 0x00000000,
		0x50C, 0x0000004F,
		0x50D, 0x000000A4,
		0x50E, 0x00000000,
		0x50F, 0x00000000,
		0x512, 0x0000001C,
		0x514, 0x0000000A,
		0x516, 0x0000000A,
		0x525, 0x0000004F,
		0x540, 0x00000012,
		0x541, 0x00000064,
		0x550, 0x00000010,
		0x551, 0x00000010,
		0x559, 0x00000002,
		0x55C, 0x00000050,
		0x55D, 0x000000FF,
		0x605, 0x00000030,
		0x608, 0x0000000E,
		0x609, 0x0000002A,
		0x620, 0x000000FF,
		0x621, 0x000000FF,
		0x622, 0x000000FF,
		0x623, 0x000000FF,
		0x624, 0x000000FF,
		0x625, 0x000000FF,
		0x626, 0x000000FF,
		0x627, 0x000000FF,
		0x638, 0x00000050,
		0x63C, 0x0000000A,
		0x63D, 0x0000000A,
		0x63E, 0x0000000E,
		0x63F, 0x0000000E,
		0x640, 0x00000040,
		0x642, 0x00000040,
		0x643, 0x00000000,
		0x652, 0x0000002B,
		0x66E, 0x00000005,
		0x700, 0x00000021,
		0x701, 0x00000043,
		0x702, 0x00000065,
		0x703, 0x00000087,
		0x708, 0x00000021,
		0x709, 0x00000043,
		0x70A, 0x00000065,
		0x70B, 0x00000087,

};

void
odm_read_and_config_mp_8192e_mac_reg(struct dm_struct *dm)
{
	u32	i = 0;
	u8	c_cond;
	boolean	is_matched = true, is_skipped = false;
	u32	array_len =
			sizeof(array_mp_8192e_mac_reg) / sizeof(u32);
	u32	*array = (u32 *)array_mp_8192e_mac_reg;

	u32	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;
	u32	a1 = 0, a2 = 0, a3 = 0, a4 = 0;

	PHYDM_DBG(dm, ODM_COMP_INIT, "===> %s\n", __func__);

	while ((i + 1) < array_len) {
		v1 = array[i];
		v2 = array[i + 1];

		if (v1 & (BIT(31) | BIT(30))) {/*positive & negative condition*/
			if (v1 & BIT(31)) {/* positive condition*/
				c_cond  =
					(u8)((v1 & (BIT(29) | BIT(28))) >> 28);
				if (c_cond == COND_ENDIF) {/*end*/
					is_matched = true;
					is_skipped = false;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ENDIF\n");
				} else if (c_cond == COND_ELSE) { /*else*/
					is_matched = is_skipped ? false : true;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ELSE\n");
				} else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					PHYDM_DBG(dm, ODM_COMP_INIT,
						  "IF or ELSE IF\n");
				}
			} else if (v1 & BIT(30)) { /*negative condition*/
				if (!is_skipped) {
					a1 = pre_v1; a2 = pre_v2;
					a3 = v1; a4 = v2;
					if (check_positive(dm,
							   a1, a2, a3, a4)) {
						is_matched = true;
						is_skipped = true;
					} else {
						is_matched = false;
						is_skipped = false;
					}
				} else {
					is_matched = false;
				}
			}
		} else {
			if (is_matched)
				odm_config_mac_8192e(dm, v1, (u8)v2);
		}
		i = i + 2;
	}
}

u32
odm_get_version_mp_8192e_mac_reg(void)
{
		return 60;
}

#endif /* end of HWIMG_SUPPORT*/

