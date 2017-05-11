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

/*
============================================================
 include files
============================================================
*/

#include "mp_precomp.h"
#include "phydm_precomp.h"

#if defined(CONFIG_PHYDM_DFS_MASTER)
VOID phydm_radar_detect_reset(PVOID pDM_VOID)
{
	PDM_ODM_T pDM_Odm = (PDM_ODM_T)pDM_VOID;

	ODM_SetBBReg(pDM_Odm, 0x924 , BIT15, 0);
	ODM_SetBBReg(pDM_Odm, 0x924 , BIT15, 1);
}

VOID phydm_radar_detect_disable(PVOID pDM_VOID)
{
	PDM_ODM_T pDM_Odm = (PDM_ODM_T)pDM_VOID;

	ODM_SetBBReg(pDM_Odm, 0x924 , BIT15, 0);
}

/* Init radar detection parameters, called after ch, bw is set */
VOID phydm_radar_detect_enable(PVOID pDM_VOID)
{
	PDM_ODM_T pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte region_domain = pDM_Odm->DFS_RegionDomain;
	u1Byte c_channel = *(pDM_Odm->pChannel);

	if (region_domain == PHYDM_DFS_DOMAIN_UNKNOWN) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD, ("PHYDM_DFS_DOMAIN_UNKNOWN\n"));
		return;
	}

	 if (pDM_Odm->SupportICType & (ODM_RTL8821 | ODM_RTL8812 | ODM_RTL8881A)) {

		ODM_SetBBReg(pDM_Odm, 0x814, 0x3fffffff, 0x04cc4d10);
		ODM_SetBBReg(pDM_Odm, 0x834, bMaskByte0, 0x06);

		if (region_domain == PHYDM_DFS_DOMAIN_ETSI) {
			ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16ecdf);
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x0152a400);
			ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x0fa21a20);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0f57204);

		} else if (region_domain == PHYDM_DFS_DOMAIN_MKK) {
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x01528400);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0d67234);

			if (c_channel >= 52 && c_channel <= 64) {
				ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16ecdf);
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x0f141a20);
			} else {
				ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16acdf);
				if (pDM_Odm->pBandWidth == ODM_BW20M)
					ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x64721a20);
				else
					ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x68721a20);
			}

		} else if (region_domain == PHYDM_DFS_DOMAIN_FCC) {
			ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16acdf);
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x0152a400);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0d67231);
			if (pDM_Odm->pBandWidth == ODM_BW20M)
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x64741a20);
			else
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x68741a20);
		} else {
			/* not supported */
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD, ("Unsupported DFS_RegionDomain:%d\n", region_domain));
		}

	} else if (pDM_Odm->SupportICType & (ODM_RTL8814A | ODM_RTL8822B)) {
	
		ODM_SetBBReg(pDM_Odm, 0x814, 0x3fffffff, 0x04cc4d10);
		ODM_SetBBReg(pDM_Odm, 0x834, bMaskByte0, 0x06);
		
		/* 8822B only, when BW = 20M, DFIR output is 40Mhz, but DFS input is 80MMHz, so it need to upgrade to 80MHz */
		if (pDM_Odm->SupportICType & ODM_RTL8822B) {
			if (pDM_Odm->pBandWidth == ODM_BW20M)
				ODM_SetBBReg(pDM_Odm, 0x1984, BIT26, 1);
			else
				ODM_SetBBReg(pDM_Odm, 0x1984, BIT26, 0);
		}

		if (region_domain == PHYDM_DFS_DOMAIN_ETSI) {
			ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16acdf);
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x095aa400);
			ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x0fa21a20);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0f57204);

		} else if (region_domain == PHYDM_DFS_DOMAIN_MKK) {
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x095aa400);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0d67234);

			if (c_channel >= 52 && c_channel <= 64) {
				ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c16ecdf);
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x0f141a20);
			} else {
				ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c166cdf);
				if (pDM_Odm->pBandWidth == ODM_BW20M)
					ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x64721a20);
				else
					ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x68721a20);
			}
		} else if (region_domain == PHYDM_DFS_DOMAIN_FCC) {
			ODM_SetBBReg(pDM_Odm, 0x918, bMaskDWord, 0x1c166cdf);
			ODM_SetBBReg(pDM_Odm, 0x924, bMaskDWord, 0x095aa400);
			ODM_SetBBReg(pDM_Odm, 0x920, bMaskDWord, 0xe0d67231);
			if (pDM_Odm->pBandWidth == ODM_BW20M)
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x64741a20);
			else
				ODM_SetBBReg(pDM_Odm, 0x91c, bMaskDWord, 0x68741a20);
		} else {
			/* not supported */
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD, ("Unsupported DFS_RegionDomain:%d\n", region_domain));
		}
	} else {
		/* not supported IC type*/
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD, ("Unsupported IC Type:%d\n", pDM_Odm->SupportICType));
	}

	phydm_radar_detect_reset(pDM_Odm);
}

BOOLEAN phydm_radar_detect(PVOID pDM_VOID)
{
	PDM_ODM_T pDM_Odm = (PDM_ODM_T)pDM_VOID;
	BOOLEAN enable_DFS = FALSE;
	BOOLEAN bypass = FALSE;
	BOOLEAN radar_detected = FALSE;
	u1Byte region_domain = pDM_Odm->DFS_RegionDomain;
	u4Byte tp_th = ((*pDM_Odm->pBandWidth == ODM_BW40M) ? 45 : 20); /* refer AP team's testing number */

	if (region_domain == PHYDM_DFS_DOMAIN_UNKNOWN) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD, ("PHYDM_DFS_DOMAIN_UNKNOWN\n"));
		return FALSE;
	}

	if ((pDM_Odm->total_tp) > tp_th) 
		bypass = TRUE;

	if (ODM_GetBBReg(pDM_Odm , 0x924, BIT15))
		enable_DFS = TRUE;

	if ((ODM_GetBBReg(pDM_Odm , 0xf98, BIT17))
		|| (!(region_domain == PHYDM_DFS_DOMAIN_ETSI) && (ODM_GetBBReg(pDM_Odm , 0xf98, BIT19))))
		radar_detected = TRUE;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_DFS, ODM_DBG_LOUD
		, ("Radar detect: enable_DFS:%d, radar_detected:%d, bypass:%d(throughput:%u, tp_th:%d)\n"
			, enable_DFS, radar_detected, bypass, pDM_Odm->total_tp, tp_th));

	if (enable_DFS && radar_detected)
		phydm_radar_detect_reset(pDM_Odm);

exit:
	return (enable_DFS && radar_detected && !bypass);
}
#endif /* defined(CONFIG_PHYDM_DFS_MASTER) */
