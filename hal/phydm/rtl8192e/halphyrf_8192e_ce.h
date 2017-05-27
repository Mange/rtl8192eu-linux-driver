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

#ifndef __HAL_PHY_RF_8192E_H__
#define __HAL_PHY_RF_8192E_H__

/*--------------------------Define Parameters-------------------------------*/
#if (DM_ODM_SUPPORT_TYPE & ODM_CE)
#define	IQK_DELAY_TIME_92E		15		//ms
#else
#define	IQK_DELAY_TIME_92E		10
#endif

#define	index_mapping_NUM_92E	15
#define AVG_THERMAL_NUM_92E	4
#define	RF_T_METER_92E			0x42

#include "../halphyrf_ce.h"

void ConfigureTxpowerTrack_8192E(
	PTXPWRTRACK_CFG	pConfig
);

VOID
GetDeltaSwingTable_8192E(
	IN	PVOID		pDM_VOID,
	OUT pu1Byte 			*TemperatureUP_A,
	OUT pu1Byte 			*TemperatureDOWN_A,
	OUT pu1Byte 			*TemperatureUP_B,
	OUT pu1Byte			*TemperatureDOWN_B
);

void DoIQK_8192E(
	PVOID		pDM_VOID,
	u1Byte 		DeltaThermalIndex,
	u1Byte		ThermalValue,
	u1Byte 		Threshold
);

VOID
ODM_TxPwrTrackSetPwr92E(
	IN	PVOID		pDM_VOID,
	PWRTRACK_METHOD 	Method,
	u1Byte 				RFPath,
	u1Byte 				ChannelMappedIndex
);

//1 7.	IQK

void
PHY_IQCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN PADAPTER	Adapter,
#endif
	IN	BOOLEAN 	bReCovery);


//
// LC calibrate
//
void
PHY_LCCalibrate_8192E(
	IN	PVOID		pDM_VOID
);

//
// AP calibrate
//
void
PHY_APCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s1Byte		delta);
void
PHY_DigitalPredistortion_8192E(IN	PADAPTER	pAdapter);


VOID
_PHY_SaveADDARegisters_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	pu4Byte		ADDABackup,
	IN	u4Byte		RegisterNum
);

VOID
_PHY_PathADDAOn_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	BOOLEAN		isPathAOn,
	IN	BOOLEAN		is2T
);

VOID
_PHY_MACSettingCalibration_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
);

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
VOID
_PHY_PathAStandBy(
	IN PDM_ODM_T		pDM_Odm
);
#endif




#endif	/* #ifndef __HAL_PHY_RF_8192E_H__*/

