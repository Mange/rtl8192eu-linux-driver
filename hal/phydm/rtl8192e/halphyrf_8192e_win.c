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

#include "mp_precomp.h"
#include "../phydm_precomp.h"


#if (RTL8192E_SUPPORT == 1)

/*---------------------------Define Local Constant---------------------------*/
// 2010/04/25 MH Define the max tx power tracking tx agc power.
#define		ODM_TXPWRTRACK_MAX_IDX_92E		6

/*---------------------------Define Local Constant---------------------------*/

//3============================================================
//3 Tx Power Tracking
//3============================================================


void setIqkMatrix_8192E(
	PDM_ODM_T	pDM_Odm,
	u1Byte 		OFDM_index,
	u1Byte 		RFPath,
	s4Byte 		IqkResult_X,
	s4Byte 		IqkResult_Y
)
{
	s4Byte			ele_A = 0, ele_D, ele_C = 0, value32;

	ele_D = (OFDMSwingTable_New[OFDM_index] & 0xFFC00000) >> 22;

	/*new element A = element D x X*/
	if ((IqkResult_X != 0) && (*(pDM_Odm->pBandType) == ODM_BAND_2_4G)) {
		if ((IqkResult_X & 0x00000200) != 0)	//consider minus
			IqkResult_X = IqkResult_X | 0xFFFFFC00;
		ele_A = ((IqkResult_X * ele_D) >> 8) & 0x000003FF;

		//new element C = element D x Y
		if ((IqkResult_Y & 0x00000200) != 0)
			IqkResult_Y = IqkResult_Y | 0xFFFFFC00;
		ele_C = ((IqkResult_Y * ele_D) >> 8) & 0x000003FF;

		/*if (RFPath == ODM_RF_PATH_A)// Remove this to Fix Path B PowerTracking */
		switch (RFPath) {
		case ODM_RF_PATH_A:
			//wirte new elements A, C, D to regC80 and regC94, element B is always 0
			value32 = (ele_D << 22) | ((ele_C & 0x3F) << 16) | ele_A;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord, value32);

			value32 = (ele_C & 0x000003C0) >> 6;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XCTxAFE, bMaskH4Bits, value32);

			value32 = ((IqkResult_X * ele_D) >> 7) & 0x01;
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT24, value32);
			break;
		case ODM_RF_PATH_B:
			//wirte new elements A, C, D to regC88 and regC9C, element B is always 0
			value32 = (ele_D << 22) | ((ele_C & 0x3F) << 16) | ele_A;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, bMaskDWord, value32);

			value32 = (ele_C & 0x000003C0) >> 6;
			ODM_SetBBReg(pDM_Odm, rOFDM0_XDTxAFE, bMaskH4Bits, value32);

			value32 = ((IqkResult_X * ele_D) >> 7) & 0x01;
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT28, value32);

			break;
		default:
			break;
		}
	} else {
		switch (RFPath) {
		case ODM_RF_PATH_A:
			ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord, OFDMSwingTable_New[OFDM_index]);
			ODM_SetBBReg(pDM_Odm, rOFDM0_XCTxAFE, bMaskH4Bits, 0x00);
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT24, 0x00);
			break;

		case ODM_RF_PATH_B:
			ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, bMaskDWord, OFDMSwingTable_New[OFDM_index]);
			ODM_SetBBReg(pDM_Odm, rOFDM0_XDTxAFE, bMaskH4Bits, 0x00);
			ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT28, 0x00);
			break;

		default:
			break;
		}
	}

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("TxPwrTracking path %c: X = 0x%x, Y = 0x%x ele_A = 0x%x ele_C = 0x%x ele_D = 0x%x 0xeb4 = 0x%x 0xebc = 0x%x\n",
				 (RFPath == ODM_RF_PATH_A ? 'A' : 'B'), (u4Byte)IqkResult_X, (u4Byte)IqkResult_Y, (u4Byte)ele_A, (u4Byte)ele_C, (u4Byte)ele_D, (u4Byte)IqkResult_X, (u4Byte)IqkResult_Y));
}

void DoIQK_8192E(
	PVOID		pDM_VOID,
	u1Byte 		DeltaThermalIndex,
	u1Byte		ThermalValue,
	u1Byte 		Threshold
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	PADAPTER 		Adapter = pDM_Odm->Adapter;
	#endif

	ODM_ResetIQKResult(pDM_Odm);

	pDM_Odm->RFCalibrateInfo.ThermalValue_IQK = ThermalValue;
	#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	PHY_IQCalibrate_8192E(pDM_Odm, FALSE);
	#else
	PHY_IQCalibrate_8192E(Adapter, FALSE);
	#endif

}

/*-----------------------------------------------------------------------------
 * Function:	odm_TxPwrTrackSetPwr88E()
 *
 * Overview:	88E change all channel tx power accordign to flag.
 *				OFDM & CCK are all different.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	04/23/2012	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/
VOID
ODM_TxPwrTrackSetPwr92E(
	PDM_ODM_T			pDM_Odm,
	PWRTRACK_METHOD 	Method,
	u1Byte 				RFPath,
	u1Byte 				ChannelMappedIndex
)
{
	PADAPTER				Adapter = pDM_Odm->Adapter;
	PHAL_DATA_TYPE			pHalData = GET_HAL_DATA(Adapter);
	u1Byte					PwrTrackingLimit_OFDM = 30; //+0dB
	u1Byte					PwrTrackingLimit_CCK = 28;  //-2dB
	u1Byte					TxRate = 0xFF;
	u1Byte					Final_OFDM_Swing_Index = 0;
	u1Byte					Final_CCK_Swing_Index = 0;
	u1Byte					i = 0;
	PODM_RF_CAL_T  	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	
	if (pDM_Odm->mp_mode == TRUE) {
	#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
		#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
			#if (MP_DRIVER == 1)
					PMPT_CONTEXT pMptCtx = &(Adapter->MptCtx);
					
					TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
			#endif
		#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
				PMPT_CONTEXT pMptCtx = &(Adapter->mppriv.MptCtx);
				
				TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
		#endif	
	#endif
	} else {
		u2Byte	rate	 = *(pDM_Odm->pForcedDataRate);
		
		if (!rate) { /*auto rate*/
			if (rate != 0xFF) {
			#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
						TxRate = Adapter->HalFunc.GetHwRateFromMRateHandler(pDM_Odm->TxRate);
			#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
						TxRate = HwRateToMRate(pDM_Odm->TxRate);
			#endif
			}
		} else { /*force rate*/
			TxRate = (u1Byte)rate;
		}
	}
		
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Power Tracking TxRate=%d\n", TxRate));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("===>ODM_TxPwrTrackSetPwr8192EA\n"));

	if (TxRate != 0xFF) { //20130429 Mimic Modify High Rate BBSwing Limit.
		//2 CCK
		if ((TxRate >= MGN_1M) && (TxRate <= MGN_11M))
			PwrTrackingLimit_CCK = 28;  //-2dB
		//2 OFDM
		else if ((TxRate >= MGN_6M) && (TxRate <= MGN_48M))
			PwrTrackingLimit_OFDM = 36; //+3dB
		else if (TxRate == MGN_54M)
			PwrTrackingLimit_OFDM = 34; //+2dB

		//2 HT
		else if ((TxRate >= MGN_MCS0) && (TxRate <= MGN_MCS2)) //QPSK/BPSK
			PwrTrackingLimit_OFDM = 38; //+4dB
		else if ((TxRate >= MGN_MCS3) && (TxRate <= MGN_MCS4)) //16QAM
			PwrTrackingLimit_OFDM = 36; //+3dB
		else if ((TxRate >= MGN_MCS5) && (TxRate <= MGN_MCS7)) //64QAM
			PwrTrackingLimit_OFDM = 34; //+2dB

		else if ((TxRate >= MGN_MCS8) && (TxRate <= MGN_MCS10)) //QPSK/BPSK
			PwrTrackingLimit_OFDM = 38; //+4dB
		else if ((TxRate >= MGN_MCS11) && (TxRate <= MGN_MCS12)) //16QAM
			PwrTrackingLimit_OFDM = 36; //+3dB
		else if ((TxRate >= MGN_MCS13) && (TxRate <= MGN_MCS15)) //64QAM
			PwrTrackingLimit_OFDM = 34; //+2dB

		else
			PwrTrackingLimit_OFDM =  pRFCalibrateInfo->DefaultOfdmIndex;   //Default OFDM index = 30
	}

	if (Method == TXAGC) {
		u1Byte	rf = 0;
		u4Byte 	pwr = 0, TxAGC = 0;
		PADAPTER Adapter = pDM_Odm->Adapter;

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("odm_TxPwrTrackSetPwr92E CH=%d\n", *(pDM_Odm->pChannel)));

		pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];   //Remnant index equal to aboslute compensate value.

		#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE ))

		#if (MP_DRIVER != 1)
		//PHY_SetTxPowerLevelByPath8192E(Adapter, pHalData->CurrentChannel, RFPath);   //Using new set power function
		//PHY_SetTxPowerLevel8192E(pDM_Odm->Adapter, *pDM_Odm->pChannel);
		pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;
		pRFCalibrateInfo->Modify_TxAGC_Flag_PathB = TRUE;
		pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;
		if (RFPath == ODM_RF_PATH_A) {
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, OFDM);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS0_MCS7);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS8_MCS15);
		} else {
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, CCK);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, OFDM);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS0_MCS7);
			PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS8_MCS15);
		}
		#else

		if (RFPath == ODM_RF_PATH_A) {
			//CCK Path A
			pwr = PHY_QueryBBReg(Adapter, rTxAGC_A_Rate18_06, 0xFF);
			pwr += pRFCalibrateInfo->PowerIndexOffset[ODM_RF_PATH_A];
			PHY_SetBBReg(Adapter, rTxAGC_A_CCK1_Mcs32, bMaskByte1, pwr);
			TxAGC = (pwr << 16) | (pwr << 8) | (pwr);
			PHY_SetBBReg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, 0x00ffffff, TxAGC);
			RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr92E: CCK Tx-rf(A) Power = 0x%x\n", TxAGC));

			//OFDM Path A
			pwr = PHY_QueryBBReg(Adapter, rTxAGC_A_Rate18_06, 0xFF);
			pwr += (pRFCalibrateInfo->BbSwingIdxOfdm[ODM_RF_PATH_A] - pRFCalibrateInfo->BbSwingIdxOfdmBase[ODM_RF_PATH_A]);
			TxAGC = ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
			PHY_SetBBReg(Adapter, rTxAGC_A_Rate18_06, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_A_Rate54_24, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_A_Mcs11_Mcs08, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_A_Mcs15_Mcs12, bMaskDWord, TxAGC);
			RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr92E: OFDM Tx-rf(A) Power = 0x%x\n", TxAGC));
		} else if (RFPath == ODM_RF_PATH_B) {
			//CCK Path B
			pwr = PHY_QueryBBReg(Adapter, rTxAGC_B_Rate18_06, 0xFF);
			pwr += pRFCalibrateInfo->PowerIndexOffset[ODM_RF_PATH_B];
			PHY_SetBBReg(Adapter, rTxAGC_B_CCK1_55_Mcs32, bMaskByte3, pwr);
			PHY_SetBBReg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, 0xff000000, pwr);
			RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr92E: CCK Tx-rf(B) Power = 0x%x\n", pwr));

			//OFDM Path B
			pwr = PHY_QueryBBReg(Adapter, rTxAGC_B_Rate18_06, 0xFF);
			pwr += (pRFCalibrateInfo->BbSwingIdxOfdm[ODM_RF_PATH_B] - pRFCalibrateInfo->BbSwingIdxOfdmBase[ODM_RF_PATH_B]);
			TxAGC = ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
			PHY_SetBBReg(Adapter, rTxAGC_B_Rate18_06, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_B_Rate54_24, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_B_Mcs03_Mcs00, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_B_Mcs07_Mcs04, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_B_Mcs11_Mcs08, bMaskDWord, TxAGC);
			PHY_SetBBReg(Adapter, rTxAGC_B_Mcs15_Mcs12, bMaskDWord, TxAGC);
			RT_DISP(FPHY, PHY_TXPWR, ("ODM_TxPwrTrackSetPwr92E: OFDM Tx-rf(B) Power = 0x%x\n", TxAGC));
		}
		#endif

		#endif
		#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
		//PHY_RF6052SetCCKTxPower(pDM_Odm->priv, *(pDM_Odm->pChannel));
		//PHY_RF6052SetOFDMTxPower(pDM_Odm->priv, *(pDM_Odm->pChannel));
		#endif

	} else if (Method == BBSWING) {
		Final_OFDM_Swing_Index = pRFCalibrateInfo->DefaultOfdmIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];
		Final_CCK_Swing_Index = pRFCalibrateInfo->DefaultCckIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];

		if (Final_OFDM_Swing_Index >= PwrTrackingLimit_OFDM)
			Final_OFDM_Swing_Index = PwrTrackingLimit_OFDM;
		else if (Final_OFDM_Swing_Index < 0)
			Final_OFDM_Swing_Index = 0;

		if (Final_CCK_Swing_Index >= CCK_TABLE_SIZE)
			Final_CCK_Swing_Index = CCK_TABLE_SIZE - 1;
		else if (pRFCalibrateInfo->BbSwingIdxCck < 0)
			Final_CCK_Swing_Index = 0;

		// Adjust BB swing by OFDM IQ matrix
		if (RFPath == ODM_RF_PATH_A) {
			setIqkMatrix_8192E(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_A,
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

			// Adjust BB swing by CCK filter coefficient
			if (!pRFCalibrateInfo->bCCKinCH14) {
				ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][0]);
				ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][1]);
				ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][2]);
				ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][3]);
				ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][4]);
				ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][5]);
				ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][6]);
				ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][7]);
			} else {
				ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][0]);
				ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][1]);
				ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][2]);
				ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][3]);
				ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][4]);
				ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][5]);
				ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][6]);
				ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][7]);
			}
		} else if (RFPath == ODM_RF_PATH_B) {
			setIqkMatrix_8192E(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_B,
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][4],
							   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][5]);
		}
	} else if (Method == MIX_MODE) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("pRFCalibrateInfo->DefaultOfdmIndex=%d,  pDM_Odm->DefaultCCKIndex=%d, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath]=%d, RF_Path = %d\n",
					 pRFCalibrateInfo->DefaultOfdmIndex, pRFCalibrateInfo->DefaultCckIndex, pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath], RFPath));

		Final_OFDM_Swing_Index = pRFCalibrateInfo->DefaultOfdmIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];

		if (RFPath == ODM_RF_PATH_A) {
			Final_CCK_Swing_Index = pRFCalibrateInfo->DefaultCckIndex + pRFCalibrateInfo->Absolute_OFDMSwingIdx[RFPath];    //CCK Follow Path-A and lower CCK index means higher power.

			if (Final_OFDM_Swing_Index > PwrTrackingLimit_OFDM) {   //BBSwing higher then Limit
				pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit_OFDM;

				setIqkMatrix_8192E(pDM_Odm, PwrTrackingLimit_OFDM, ODM_RF_PATH_A,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, OFDM);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS0_MCS7);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS8_MCS15);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d \n", PwrTrackingLimit_OFDM, pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
			} else if (Final_OFDM_Swing_Index < 0) {
				pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index ;

				setIqkMatrix_8192E(pDM_Odm, 0, ODM_RF_PATH_A,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);


				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, OFDM);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS0_MCS7);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS8_MCS15);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d \n", pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
			} else {
				setIqkMatrix_8192E(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_A,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][0],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][1]);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Compensate with BBSwing , Final_OFDM_Swing_Index = %d \n", Final_OFDM_Swing_Index));

				if (pRFCalibrateInfo->Modify_TxAGC_Flag_PathA) { //If TxAGC has changed, reset TxAGC again
					pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = 0;

					//Set TxAGC Page C{};
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, OFDM);
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS0_MCS7);
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, HT_MCS8_MCS15);

					pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = FALSE;

					ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A pDM_Odm->Modify_TxAGC_Flag = FALSE \n"));
				}
			}

			if (Final_CCK_Swing_Index > PwrTrackingLimit_CCK) {
				pRFCalibrateInfo->Remnant_CCKSwingIdx = Final_CCK_Swing_Index - PwrTrackingLimit_CCK;

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Over Limit , PwrTrackingLimit_CCK = %d , pRFCalibrateInfo->Remnant_CCKSwingIdx  = %d \n", PwrTrackingLimit_CCK, pRFCalibrateInfo->Remnant_CCKSwingIdx));

				// Adjust BB swing by CCK filter coefficient

				if (!pRFCalibrateInfo->bCCKinCH14) {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch1_Ch13_New[PwrTrackingLimit_CCK][7]);
				} else {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch14_New[PwrTrackingLimit_CCK][7]);
				}

				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, CCK);

			} else if (Final_CCK_Swing_Index < 0) { // Lowest CCK Index = 0
				pRFCalibrateInfo->Remnant_CCKSwingIdx = Final_CCK_Swing_Index;

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Under Limit , PwrTrackingLimit_CCK = %d , pRFCalibrateInfo->Remnant_CCKSwingIdx  = %d \n", 0, pRFCalibrateInfo->Remnant_CCKSwingIdx));

				if (!pRFCalibrateInfo->bCCKinCH14) {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch1_Ch13_New[0][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch1_Ch13_New[0][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch1_Ch13_New[0][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch1_Ch13_New[0][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch1_Ch13_New[0][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch1_Ch13_New[0][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch1_Ch13_New[0][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch1_Ch13_New[0][7]);
				} else {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch14_New[0][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch14_New[0][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch14_New[0][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch14_New[0][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch14_New[0][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch14_New[0][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch14_New[0][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch14_New[0][7]);
				}

				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, CCK);

			} else {
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Compensate with BBSwing , Final_CCK_Swing_Index = %d \n", Final_CCK_Swing_Index));

				if (!pRFCalibrateInfo->bCCKinCH14) {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch1_Ch13_New[Final_CCK_Swing_Index][7]);
				} else {
					ODM_Write1Byte(pDM_Odm, 0xa22, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][0]);
					ODM_Write1Byte(pDM_Odm, 0xa23, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][1]);
					ODM_Write1Byte(pDM_Odm, 0xa24, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][2]);
					ODM_Write1Byte(pDM_Odm, 0xa25, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][3]);
					ODM_Write1Byte(pDM_Odm, 0xa26, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][4]);
					ODM_Write1Byte(pDM_Odm, 0xa27, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][5]);
					ODM_Write1Byte(pDM_Odm, 0xa28, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][6]);
					ODM_Write1Byte(pDM_Odm, 0xa29, CCKSwingTable_Ch14_New[Final_CCK_Swing_Index][7]);
				}

				if (pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK) { //If TxAGC has changed, reset TxAGC again
					pRFCalibrateInfo->Remnant_CCKSwingIdx = 0;

					//Set TxAGC Page C{};
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_A, pHalData->CurrentChannel, CCK);
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, CCK);

					pRFCalibrateInfo->Modify_TxAGC_Flag_PathA_CCK = FALSE;

					ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A pDM_Odm->Modify_TxAGC_Flag_CCK = FALSE \n"));
				}

			}
		}
		if (RFPath == ODM_RF_PATH_B) {
			if (Final_OFDM_Swing_Index > PwrTrackingLimit_OFDM) {  //BBSwing higher then Limit
				pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit_OFDM;

				setIqkMatrix_8192E(pDM_Odm, PwrTrackingLimit_OFDM, ODM_RF_PATH_B,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][4],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][5]);

				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, OFDM);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS0_MCS7);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS8_MCS15);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d \n", PwrTrackingLimit_OFDM, pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
			} else if (Final_OFDM_Swing_Index < 0) {
				pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = Final_OFDM_Swing_Index ;

				setIqkMatrix_8192E(pDM_Odm, 0, ODM_RF_PATH_B,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][4],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][5]);

				pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = TRUE;

				//Set TxAGC Page C{};
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, OFDM);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS0_MCS7);
				PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS8_MCS15);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d \n", pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]));
			} else {
				setIqkMatrix_8192E(pDM_Odm, Final_OFDM_Swing_Index, ODM_RF_PATH_B,
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][4],
								   pRFCalibrateInfo->IQKMatrixRegSetting[ChannelMappedIndex].Value[0][5]);

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Compensate with BBSwing , Final_OFDM_Swing_Index = %d \n", Final_OFDM_Swing_Index));

				if (pRFCalibrateInfo->Modify_TxAGC_Flag_PathB) { //If TxAGC has changed, reset TxAGC again
					pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath] = 0;

					//Set TxAGC Page C{};
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, OFDM);
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS0_MCS7);
					PHY_SetTxPowerIndexByRateSection(Adapter, ODM_RF_PATH_B, pHalData->CurrentChannel, HT_MCS8_MCS15);

					pRFCalibrateInfo->Modify_TxAGC_Flag_PathA = FALSE;

					ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B pDM_Odm->Modify_TxAGC_Flag = FALSE \n"));
				}
			}
		}
	} else
		return;
}	// odm_TxPwrTrackSetPwr88E

VOID
GetDeltaSwingTable_8192E(
	IN 	PDM_ODM_T			pDM_Odm,
	OUT pu1Byte 			*TemperatureUP_A,
	OUT pu1Byte 			*TemperatureDOWN_A,
	OUT pu1Byte 			*TemperatureUP_B,
	OUT pu1Byte 			*TemperatureDOWN_B
)
{
	PADAPTER		Adapter 		 = pDM_Odm->Adapter;
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	HAL_DATA_TYPE	*pHalData		 = GET_HAL_DATA(Adapter);
	u1Byte			TxRate			= 0xFF;
	u1Byte			channel 		 = pHalData->CurrentChannel;

	if (pDM_Odm->mp_mode == TRUE) {
	#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
		#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
			#if (MP_DRIVER == 1)
					PMPT_CONTEXT pMptCtx = &(Adapter->MptCtx);
					
					TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
			#endif
		#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
				PMPT_CONTEXT pMptCtx = &(Adapter->mppriv.MptCtx);
				
				TxRate = MptToMgntRate(pMptCtx->MptRateIndex);
		#endif	
	#endif
	} else {
		u2Byte	rate	 = *(pDM_Odm->pForcedDataRate);
		
		if (!rate) { /*auto rate*/
			if (rate != 0xFF) {
			#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
						TxRate = Adapter->HalFunc.GetHwRateFromMRateHandler(pDM_Odm->TxRate);
			#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
						TxRate = HwRateToMRate(pDM_Odm->TxRate);
			#endif
			}
		} else { /*force rate*/
			TxRate = (u1Byte)rate;
		}
	}
		
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Power Tracking TxRate=%d\n", TxRate));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Call GetDeltaSwingTable_8192E Power Tracking TxRate=0x%X\n", TxRate));


	if (1 <= channel && channel <= 14) {
		if (IS_CCK_RATE(TxRate)) {
			*TemperatureUP_A   = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_P;
			*TemperatureDOWN_A = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKA_N;
			*TemperatureUP_B   = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_P;
			*TemperatureDOWN_B = pRFCalibrateInfo->DeltaSwingTableIdx_2GCCKB_N;
		} else {
			*TemperatureUP_A   = pRFCalibrateInfo->DeltaSwingTableIdx_2GA_P;
			*TemperatureDOWN_A = pRFCalibrateInfo->DeltaSwingTableIdx_2GA_N;
			*TemperatureUP_B   = pRFCalibrateInfo->DeltaSwingTableIdx_2GB_P;
			*TemperatureDOWN_B = pRFCalibrateInfo->DeltaSwingTableIdx_2GB_N;
		}
	} else {
		*TemperatureUP_A   = (pu1Byte)DeltaSwingTableIdx_2GA_P_8188E;
		*TemperatureDOWN_A = (pu1Byte)DeltaSwingTableIdx_2GA_N_8188E;
		*TemperatureUP_B   = (pu1Byte)DeltaSwingTableIdx_2GA_P_8188E;
		*TemperatureDOWN_B = (pu1Byte)DeltaSwingTableIdx_2GA_N_8188E;
	}

	return;
}



void ConfigureTxpowerTrack_8192E(
	PTXPWRTRACK_CFG	pConfig
)
{
	pConfig->SwingTableSize_CCK = CCK_TABLE_SIZE;
	pConfig->SwingTableSize_OFDM = OFDM_TABLE_SIZE;
	pConfig->Threshold_IQK = IQK_THRESHOLD;
	pConfig->AverageThermalNum = AVG_THERMAL_NUM_92E;
	pConfig->RfPathCount = MAX_PATH_NUM_8192E;
	pConfig->ThermalRegAddr = RF_T_METER_88E;

	pConfig->ODM_TxPwrTrackSetPwr = ODM_TxPwrTrackSetPwr92E;
	pConfig->DoIQK = DoIQK_8192E;
	pConfig->PHY_LCCalibrate = PHY_LCCalibrate_8192E;
	pConfig->GetDeltaSwingTable = GetDeltaSwingTable_8192E;
}

//1 7.	IQK
#define MAX_TOLERANCE		5
#define IQK_DELAY_TIME		1		//ms

u1Byte			//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathA_IQK_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		configPathB
)
{
	u4Byte regEAC, regE94, regE9C;
	u1Byte result = 0x00;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A IQK!\n"));

	/*8192E IQK V2.1 20150713*/
	/*1 Tx IQK*/
	//path-A IQK setting

	//	PA/PAD controlled by 0x0
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask, 0x180);

	//modify TXIQK mode table
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x20000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0x07f77);  //PA off, default: 0x7f7f

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);


//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-A IQK setting!\n"));
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x82140303);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x68160000);




	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x00462911);

	//One shot, path A LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path A LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path A LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regE94 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord);
	regE9C = ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe94 = 0x%x, 0xe9c = 0x%x\n", regE94, regE9C));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xe98, bMaskDWord)));

	if (!(regEAC & BIT28) &&
		(((regE94 & 0x03FF0000) >> 16) != 0x142) &&
		(((regE9C & 0x03FF0000) >> 16) != 0x42))
		result |= 0x01;
	else
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("pathA TX IQK is not success\n"));

	return result;


}

u1Byte			//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathA_RxIQK_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		configPathB
)
{
	u4Byte regEAC, regE94, regE9C, regEA4, u4tmp;
	u1Byte result = 0x00;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Rx IQK!\n"));

	//1 Get TXIMR setting
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Get RXIQK TXIMR!\n"));

	//modify RXIQK mode table
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-A Rx IQK modify RXIQK mode table!\n"));
	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf1173); //PA off, deafault:0xf117b

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G2, bRFRegOffsetMask, 0xf1173); //PA off, deafault:0xf117b

	//	PA/PAD control by 0x56, and set = 0x0
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask, 0x980);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x56, bRFRegOffsetMask, 0x511e0);

	//enter IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-A IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x8216031f);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x6816031f);

	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a911);

	//One shot, path A LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path A LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path A LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);


	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regE94 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord);
	regE9C = ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe94 = 0x%x, 0xe9c = 0x%x\n", regE94, regE9C));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xe90, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xe98, bMaskDWord)));

	if (!(regEAC & BIT28) &&
		(((regE94 & 0x03FF0000) >> 16) != 0x142) &&
		(((regE9C & 0x03FF0000) >> 16) != 0x42))
		result |= 0x01;
	else {						/*if Tx not OK, ignore Rx*/
		/*	PA/PAD controlled by 0x0*/
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask, 0x180);
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("pathA get TXIMR is not success\n"));
		return result;

	}

	u4tmp = 0x80007C00 | (regE94 & 0x3FF0000)  | ((regE9C & 0x3FF0000) >> 16);
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, u4tmp);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe40 = 0x%x u4tmp = 0x%x \n", ODM_GetBBReg(pDM_Odm, rTx_IQK, bMaskDWord), u4tmp));


	//1 RX IQK
	//modify RXIQK mode table
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Do RXIQK!\n"));
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-A Rx IQK modify RXIQK mode table 2!\n"));
	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf7ff2);  /*PA off : default:0xf7ffa*/

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G2, bRFRegOffsetMask, 0xf7ff2);  /*PA off : default:0xf7ffa*/


	//	PA/PAD control by 0x56, and set = 0x0
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask, 0x980);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x56, bRFRegOffsetMask, 0x510e0);
/*ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x56, bRFRegOffsetMask, 0x51000 );*/

	/*enter IQK mode*/
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-A IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x821608ff);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x281608ff);

//	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_A, bMaskDWord, 0x82160cff);
//	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_A, bMaskDWord, 0x28160cff);

	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a891);

	//One shot, path A LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path A LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf9000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path A LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regEA4 = ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_A_2, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xea4 = 0x%x, 0xeac = 0x%x\n", regEA4, regEAC));
	//monitor image power before & after IQK
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xea0(before IQK)= 0x%x, 0xea8(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xea0, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xea8, bMaskDWord)));

	//	PA/PAD controlled by 0x0
	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask, 0x180);

	if (!(regEAC & BIT27) &&	/*if Tx is OK, check whether Rx is OK*/
		(((regEA4 & 0x03FF0000) >> 16) != 0x132) &&
		(((regEAC & 0x03FF0000) >> 16) != 0x36))
		result |= 0x02;
	else
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Rx IQK is not success!!\n"));

	return result;


}

u1Byte				//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathB_IQK_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN	PADAPTER	pAdapter
#endif
)
{
	u4Byte regEAC, regEB4, regEBC;
	u1Byte	result = 0x00;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B IQK!\n"));
	/*1 Tx IQK*/
	//path-B IQK setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-B IQK setting!\n"));

	/*disable path-A PI, prevent path-A re-LOK*/
	ODM_SetBBReg(pDM_Odm, 0x820, BIT8, 0x0);

	//	PA/PAD controlled by 0x0
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask, 0x180);


	//modify TXIQK mode table
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_RCK_OS, bRFRegOffsetMask, 0x20000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G2, bRFRegOffsetMask, 0x07f77);  //PA off, default: 0x7f7f

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

	#if 0
	ODM_SetBBReg(pDM_Odm, 0xe28, 0xffffff00, 0x000000);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path A 0xdf = 0x%x\n", ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_A, 0xdf, bRFRegOffsetMask)));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B 0xdf = 0x%x\n", ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask)));
	ODM_SetBBReg(pDM_Odm, 0xe28, 0xffffff00, 0x808000);
	#endif

	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x82140303);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x68160000);

	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x00462911);

	//One shot, path B LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path B LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xfa000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

//	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Cont, bMaskDWord, 0x00000002);
//	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Cont, bMaskDWord, 0x00000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path B LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);

	/*enable path-A PI*/
	ODM_SetBBReg(pDM_Odm, 0x820, BIT8, 0x1);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regEB4 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_B, bMaskDWord);
	regEBC = ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_B, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeb4 = 0x%x, 0xebc = 0x%x\n", regEB4, regEBC));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeb0(before IQK)= 0x%x, 0xeb8(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xeb0, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xeb8, bMaskDWord)));


	if (!(regEAC & BIT31) &&
		(((regEB4 & 0x03FF0000) >> 16) != 0x142) &&
		(((regEBC & 0x03FF0000) >> 16) != 0x42))
		result |= 0x01;
	else
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B TX IQK is not success\n"));

	return result;
}



u1Byte			//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
phy_PathB_RxIQK_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		configPathB
)
{
	u4Byte regEAC, regEB4, regEBC, regECC, regEC4, u4tmp;
	u1Byte result = 0x00;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B Rx IQK!\n"));

	//1 Get TXIMR setting
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Get RXIQK TXIMR!\n"));
	//modify RXIQK mode table
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-B Rx IQK modify RXIQK mode table!\n"));
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G2, bRFRegOffsetMask, 0xf1173);  /*PA off, default: 0xf117b*/

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf1173);  /*PA off, default: 0xf117b*/

	//	PA/PAD all off
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask, 0x980);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0x56, bRFRegOffsetMask, 0x511e0);

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-B IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x18008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x38008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x8216031f);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x6816031f);

	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a911);

	//One shot, path B LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path B LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xfa000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path B LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);

	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regEB4 = ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_B, bMaskDWord);
	regEBC = ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_B, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeb4 = 0x%x, 0xebc = 0x%x\n", regEB4, regEBC));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeb0(before IQK)= 0x%x, 0xeb8(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xeb0, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xeb8, bMaskDWord)));


	if (!(regEAC & BIT31) &&
		(((regEB4 & 0x03FF0000) >> 16) != 0x142) &&
		(((regEBC & 0x03FF0000) >> 16) != 0x42))
		result |= 0x01;
	else {						/*if Tx not OK, ignore Rx*/
		//	PA/PAD controlled by 0x0
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask, 0x180);
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("pathB get TXIMR is not success\n"));
		return result;
	}

	u4tmp = 0x80007C00 | (regEB4 & 0x3FF0000)  | ((regEBC & 0x3FF0000) >> 16);
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, u4tmp);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xe40 = 0x%x u4tmp = 0x%x \n", ODM_GetBBReg(pDM_Odm, rTx_IQK, bMaskDWord), u4tmp));


	//1 RX IQK
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Do RXIQK!\n"));

	//modify RXIQK mode table
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-B Rx IQK modify RXIQK mode table 2!\n"));
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_TXPA_G2, bRFRegOffsetMask, 0xf7ff2);  /* PA off, default:0xf7ffa*/

	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_WE_LUT, bRFRegOffsetMask, 0x800a0);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_RCK_OS, bRFRegOffsetMask, 0x30000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G1, bRFRegOffsetMask, 0x0000f);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_TXPA_G2, bRFRegOffsetMask, 0xf7ff2);   /* PA off, default:0xf7ffa*/

	//	PA/PAD all off
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask, 0x980);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0x56, bRFRegOffsetMask, 0x510e0);
//	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0x56, bRFRegOffsetMask, 0x51000 );

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);

	//IQK setting
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	//path-B IQK setting
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x38008c1c);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x18008c1c);

	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x821608ff);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x281608ff);

//	ODM_SetBBReg(pDM_Odm, rTx_IQK_PI_B, bMaskDWord, 0x82160cff);
//	ODM_SetBBReg(pDM_Odm, rRx_IQK_PI_B, bMaskDWord, 0x28160cff);

	//LO calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LO calibration setting!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Rsp, bMaskDWord, 0x0046a891);

	//One shot, path B LOK & IQK
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("One shot, path B LOK & IQK!\n"));
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xfa000000);
	ODM_SetBBReg(pDM_Odm, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);

	// delay x ms
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Delay %d ms for One shot, path B LOK & IQK.\n", IQK_DELAY_TIME_92E));
	//PlatformStallExecution(IQK_DELAY_TIME_92E*1000);
	ODM_delay_ms(IQK_DELAY_TIME_92E);


	// Check failed
	regEAC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord);
	regEC4 = ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_B_2, bMaskDWord);
	regECC = ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_B_2, bMaskDWord);
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xeac = 0x%x\n", regEAC));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xec4 = 0x%x, 0xecc = 0x%x\n", regEC4, regECC));
	//monitor image power before & after IQK
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("0xec0(before IQK)= 0x%x, 0xec8(afer IQK) = 0x%x\n",
				 ODM_GetBBReg(pDM_Odm, 0xec0, bMaskDWord), ODM_GetBBReg(pDM_Odm, 0xec8, bMaskDWord)));

	//	PA/PAD controlled by 0x0
	//leave IQK mode
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0xdf, bRFRegOffsetMask, 0x180);

	if (!(regEAC & BIT30) &&/*if Tx is OK, check whether Rx is OK*/
		(((regEC4 & 0x03FF0000) >> 16) != 0x132) &&
		(((regECC & 0x03FF0000) >> 16) != 0x36))
		result |= 0x02;
	else
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B Rx IQK is not success!!\n"));

	return result;


}


VOID
_PHY_PathAFillIQKMatrix_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN  BOOLEAN    	bIQKOK,
	IN	s4Byte		result[][8],
	IN	u1Byte		final_candidate,
	IN  BOOLEAN		bTxOnly
)
{
	u4Byte	Oldval_0, X, TX0_A, reg;
	s4Byte	Y, TX0_C;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A IQ Calibration %s !\n", (bIQKOK) ? "Success" : "Failed"));

	if (final_candidate == 0xFF)
		return;

	else if (bIQKOK) {
		Oldval_0 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, bMaskDWord) >> 22) & 0x3FF;

		X = result[final_candidate][0];
		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX0_A = (X * Oldval_0) >> 8;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("X = 0x%x, TX0_A = 0x%x, Oldval_0 0x%x\n", X, TX0_A, Oldval_0));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, 0x3FF, TX0_A);

		ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT(31), ((X * Oldval_0 >> 7) & 0x1));

		Y = result[final_candidate][1];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;


		TX0_C = (Y * Oldval_0) >> 8;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Y = 0x%x, TX = 0x%x\n", Y, TX0_C));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XCTxAFE, 0xF0000000, ((TX0_C & 0x3C0) >> 6));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XATxIQImbalance, 0x003F0000, (TX0_C & 0x3F));

		ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT(29), ((Y * Oldval_0 >> 7) & 0x1));

		if (bTxOnly) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_PHY_PathAFillIQKMatrix_92E only Tx OK\n"));
			return;
		}

		reg = result[final_candidate][2];
		#if (DM_ODM_SUPPORT_TYPE==ODM_AP)
		if (RTL_ABS(reg , 0x100) >= 16)
			reg = 0x100;
		#endif
		ODM_SetBBReg(pDM_Odm, rOFDM0_XARxIQImbalance, 0x3FF, reg);

		reg = result[final_candidate][3] & 0x3F;
		ODM_SetBBReg(pDM_Odm, rOFDM0_XARxIQImbalance, 0xFC00, reg);

		reg = (result[final_candidate][3] >> 6) & 0xF;
		ODM_SetBBReg(pDM_Odm, rOFDM0_RxIQExtAnta, 0xF0000000, reg);
	}
}

VOID
_PHY_PathBFillIQKMatrix_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN  BOOLEAN   	bIQKOK,
	IN	s4Byte		result[][8],
	IN	u1Byte		final_candidate,
	IN	BOOLEAN		bTxOnly			//do Tx only
)
{
	u4Byte	Oldval_1, X, TX1_A, reg;
	s4Byte	Y, TX1_C;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B IQ Calibration %s !\n", (bIQKOK) ? "Success" : "Failed"));

	if (final_candidate == 0xFF)
		return;

	else if (bIQKOK) {
		Oldval_1 = (ODM_GetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, bMaskDWord) >> 22) & 0x3FF;

		X = result[final_candidate][4];
		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX1_A = (X * Oldval_1) >> 8;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("X = 0x%x, TX1_A = 0x%x\n", X, TX1_A));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, 0x3FF, TX1_A);

		ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT(27), ((X * Oldval_1 >> 7) & 0x1));

		Y = result[final_candidate][5];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;

		TX1_C = (Y * Oldval_1) >> 8;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Y = 0x%x, TX1_C = 0x%x\n", Y, TX1_C));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XDTxAFE, 0xF0000000, ((TX1_C & 0x3C0) >> 6));
		ODM_SetBBReg(pDM_Odm, rOFDM0_XBTxIQImbalance, 0x003F0000, (TX1_C & 0x3F));

		ODM_SetBBReg(pDM_Odm, rOFDM0_ECCAThreshold, BIT(25), ((Y * Oldval_1 >> 7) & 0x1));

		if (bTxOnly)
			return;

		reg = result[final_candidate][6];
		ODM_SetBBReg(pDM_Odm, rOFDM0_XBRxIQImbalance, 0x3FF, reg);

		reg = result[final_candidate][7] & 0x3F;
		ODM_SetBBReg(pDM_Odm, rOFDM0_XBRxIQImbalance, 0xFC00, reg);

		reg = (result[final_candidate][7] >> 6) & 0xF;
		ODM_SetBBReg(pDM_Odm, rOFDM0_AGCRSSITable, 0x0000F000, reg);
	}
}

//
// 2011/07/26 MH Add an API for testing IQK fail case.
//
/* MP Already declare in odm.c*/
#if (DM_ODM_SUPPORT_TYPE & ODM_CE)
BOOLEAN
ODM_CheckPowerStatus(
	IN	PADAPTER		Adapter)
{
	/*
		HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
		PDM_ODM_T			pDM_Odm = &pHalData->DM_OutSrc;
		RT_RF_POWER_STATE 	rtState;
		PMGNT_INFO			pMgntInfo	= &(Adapter->MgntInfo);

		// 2011/07/27 MH We are not testing ready~~!! We may fail to get correct value when init sequence.
		if (pMgntInfo->init_adpt_in_progress == TRUE)
		{
			ODM_RT_TRACE(pDM_Odm,COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return TRUE, due to initadapter"));
			return	TRUE;
		}

		//
		//	2011/07/19 MH We can not execute tx power tracking/ LLC calibrate or IQK.
		//
		Adapter->HalFunc.GetHwRegHandler(Adapter, HW_VAR_RF_STATE, (pu1Byte)(&rtState));
		if(Adapter->bDriverStopped || Adapter->bDriverIsGoingToPnpSetPowerSleep || rtState == eRfOff)
		{
			ODM_RT_TRACE(pDM_Odm,COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return FALSE, due to %d/%d/%d\n",
			Adapter->bDriverStopped, Adapter->bDriverIsGoingToPnpSetPowerSleep, rtState));
			return	FALSE;
		}
	*/
	return	TRUE;
}
#endif

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
)
{
	u4Byte	i;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif

	if (ODM_CheckPowerStatus(pAdapter) == FALSE)
		return;
	#endif

//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Save ADDA parameters.\n"));
	for (i = 0 ; i < RegisterNum ; i++)
		ADDABackup[i] = ODM_GetBBReg(pDM_Odm, ADDAReg[i], bMaskDWord);
}


VOID
_PHY_SaveMACRegisters_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	u4Byte	i;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Save MAC parameters.\n"));
	for (i = 0 ; i < (IQK_MAC_REG_NUM - 1); i++)
		MACBackup[i] = ODM_Read1Byte(pDM_Odm, MACReg[i]);
	MACBackup[i] = ODM_Read4Byte(pDM_Odm, MACReg[i]);

}


VOID
_PHY_ReloadADDARegisters_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		ADDAReg,
	IN	pu4Byte		ADDABackup,
	IN	u4Byte		RegiesterNum
)
{
	u4Byte	i;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Reload ADDA power saving parameters !\n"));
	for (i = 0 ; i < RegiesterNum; i++)
		ODM_SetBBReg(pDM_Odm, ADDAReg[i], bMaskDWord, ADDABackup[i]);
}

VOID
_PHY_ReloadMACRegisters_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	u4Byte	i;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Reload MAC parameters !\n"));
	#if 0
	ODM_SetBBReg(pDM_Odm, 0x520, bMaskByte2, 0x0);
	#else
	for (i = 0 ; i < (IQK_MAC_REG_NUM - 1); i++)
		ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)MACBackup[i]);
	ODM_Write4Byte(pDM_Odm, MACReg[i], MACBackup[i]);
	#endif
}


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
)
{
	u4Byte	pathOn;
	u4Byte	i;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("ADDA ON.\n"));

	pathOn = isPathAOn ? 0x0fc01616 : 0x0fc01616;
	if (FALSE == is2T) {
		pathOn = 0x0fc01616;
		ODM_SetBBReg(pDM_Odm, ADDAReg[0], bMaskDWord, 0x0fc01616);
	} else
		ODM_SetBBReg(pDM_Odm, ADDAReg[0], bMaskDWord, pathOn);

	for (i = 1 ; i < IQK_ADDA_REG_NUM ; i++)
		ODM_SetBBReg(pDM_Odm, ADDAReg[i], bMaskDWord, pathOn);

}

VOID
_PHY_MACSettingCalibration_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	pu4Byte		MACReg,
	IN	pu4Byte		MACBackup
)
{
	//u4Byte	i = 0;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("MAC settings for Calibration.\n"));
	/*
		ODM_Write1Byte(pDM_Odm, MACReg[i], 0x3F);

		for(i = 1 ; i < (IQK_MAC_REG_NUM - 1); i++){
			ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)(MACBackup[i]&(~BIT3)));
		}
		ODM_Write1Byte(pDM_Odm, MACReg[i], (u1Byte)(MACBackup[i]&(~BIT5)));
	*/

	//ODM_SetBBReg(pDM_Odm, 0x522, bMaskByte0, 0x7f);
	//ODM_SetBBReg(pDM_Odm, 0x550, bMaskByte0, 0x15);
	//ODM_SetBBReg(pDM_Odm, 0x551, bMaskByte0, 0x00);

	ODM_SetBBReg(pDM_Odm, 0x520, 0x00ff0000, 0xff);
//	ODM_SetBBReg(pDM_Odm, 0x550, 0x0000ffff, 0x0015);


}

VOID
_PHY_PathAStandBy_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN PADAPTER	pAdapter
#endif
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-A standby mode!\n"));

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
//	ODM_SetBBReg(pDM_Odm, 0x840, bMaskDWord, 0x00010000);
	ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)0x0, 0x0, bRFRegOffsetMask, 0x10000);

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);
}

VOID
_PHY_PathBStandBy_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN PADAPTER	pAdapter
#endif
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path-A standby mode!\n"));

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)0x1, 0x0, bRFRegOffsetMask, 0x10000);

	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);
}

VOID
_PHY_PIModeSwitch_92E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		PIMode
)
{
	u4Byte	mode;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("BB Switch to %s mode!\n", (PIMode ? "PI" : "SI")));

	mode = PIMode ? 0x01000100 : 0x01000000;
	ODM_SetBBReg(pDM_Odm, rFPGA0_XA_HSSIParameter1, bMaskDWord, mode);
	ODM_SetBBReg(pDM_Odm, rFPGA0_XB_HSSIParameter1, bMaskDWord, mode);
}

BOOLEAN
phy_SimularityCompare_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s4Byte 		result[][8],
	IN	u1Byte		 c1,
	IN	u1Byte		 c2
)
{
	u4Byte		i, j, diff, SimularityBitMap, bound = 0;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	u1Byte		final_candidate[2] = {0xFF, 0xFF};	//for path A and path B
	BOOLEAN		bResult = TRUE;
/*#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)*/
/*	BOOLEAN		is2T = IS_92C_SERIAL( pHalData->VersionID);*/
/*#else*/
	BOOLEAN		is2T = TRUE;
/*#endif*/

	s4Byte tmp1 = 0, tmp2 = 0;

	if (is2T)
		bound = 8;
	else
		bound = 4;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("===> IQK:phy_SimularityCompare_8192E c1 %d c2 %d!!!\n", c1, c2));


	SimularityBitMap = 0;

	for (i = 0; i < bound; i++) {

		if ((i == 1) || (i == 3) || (i == 5) || (i == 7)) {
			if ((result[c1][i] & 0x00000200) != 0)
				tmp1 = result[c1][i] | 0xFFFFFC00;
			else
				tmp1 = result[c1][i];

			if ((result[c2][i] & 0x00000200) != 0)
				tmp2 = result[c2][i] | 0xFFFFFC00;
			else
				tmp2 = result[c2][i];
		} else {
			tmp1 = result[c1][i];
			tmp2 = result[c2][i];
		}

		diff = (tmp1 > tmp2) ? (tmp1 - tmp2) : (tmp2 - tmp1);

		if (diff > MAX_TOLERANCE) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:differnece overflow %d index %d compare1 0x%x compare2 0x%x!!!\n",  diff, i, result[c1][i], result[c2][i]));

			if ((i == 2 || i == 6) && !SimularityBitMap) {
				if (result[c1][i] + result[c1][i + 1] == 0)
					final_candidate[(i / 4)] = c2;
				else if (result[c2][i] + result[c2][i + 1] == 0)
					final_candidate[(i / 4)] = c1;
				else
					SimularityBitMap = SimularityBitMap | (1 << i);
			} else
				SimularityBitMap = SimularityBitMap | (1 << i);
		}
	}

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:phy_SimularityCompare_8192E SimularityBitMap   %x !!!\n", SimularityBitMap));

	if (SimularityBitMap == 0) {
		for (i = 0; i < (bound / 4); i++) {
			if (final_candidate[i] != 0xFF) {
				for (j = i * 4; j < (i + 1) * 4 - 2; j++)
					result[3][j] = result[final_candidate[i]][j];
				bResult = FALSE;
			}
		}
		return bResult;
	} else {

		if (!(SimularityBitMap & 0x03)) {		/*path A TX OK*/
			for (i = 0; i < 2; i++)
				result[3][i] = result[c1][i];
		}

		if (!(SimularityBitMap & 0x0c)) {		/*path A RX OK*/
			for (i = 2; i < 4; i++)
				result[3][i] = result[c1][i];
		}

		if (!(SimularityBitMap & 0x30)) {	/*path B TX OK*/
			for (i = 4; i < 6; i++)
				result[3][i] = result[c1][i];

		}

		if (!(SimularityBitMap & 0xc0)) {	/*path B RX OK*/
			for (i = 6; i < 8; i++)
				result[3][i] = result[c1][i];
		}

		return FALSE;
	}



}



VOID
phy_IQCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s4Byte 		result[][8],
	IN	u1Byte		t,
	IN	BOOLEAN		is2T
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	u4Byte			i;
	u1Byte			PathAOK = 0, PathBOK = 0;
	u1Byte 			tmp0xc50 = (u1Byte)ODM_GetBBReg(pDM_Odm, 0xC50, bMaskByte0);
	u1Byte 			tmp0xc58 = (u1Byte)ODM_GetBBReg(pDM_Odm, 0xC58, bMaskByte0);
	u4Byte			ADDA_REG[IQK_ADDA_REG_NUM] = {
		rFPGA0_XCD_SwitchControl, 	rBlue_Tooth,
		rRx_Wait_CCA, 		rTx_CCK_RFON,
		rTx_CCK_BBON, 	rTx_OFDM_RFON,
		rTx_OFDM_BBON, 	rTx_To_Rx,
		rTx_To_Tx, 		rRx_CCK,
		rRx_OFDM, 		rRx_Wait_RIFS,
		rRx_TO_Rx, 		rStandby,
		rSleep, 			rPMPD_ANAEN
	};
	u4Byte			IQK_MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE, 		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	/*since 92C & 92D have the different define in IQK_BB_REG*/
	u4Byte	IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		rOFDM0_TRxPathEnable, 		rOFDM0_TRMuxPar,
		rFPGA0_XCD_RFInterfaceSW,	rConfig_AntA,	rConfig_AntB,
		0x92c,	0x930,
		0x938,	rCCK0_AFESetting
	};

	#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	u4Byte	retryCount = 2;
	#else
	#if MP_DRIVER
	const u4Byte	retryCount = 9;
	#else
	const u4Byte	retryCount = 2;
	#endif
	#endif

	/*Note: IQ calibration must be performed after loading*/
	/*PHY_REG.txt,and radio_a,radio_b.txt*/

	//u4Byte bbvalue;

	#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	#ifdef MP_TEST
	if (pDM_Odm->priv->pshare->rf_ft_var.mp_specific)
		retryCount = 2;
	#endif
	#endif


	if (t == 0) {
		/*bbvalue = ODM_GetBBReg(pDM_Odm, rFPGA0_RFMOD, bMaskDWord);*/
		/*RT_DISP(FINIT, INIT_IQK, ("phy_IQCalibrate_8188E()==>0x%08x\n",bbvalue));*/
		/*ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQ Calibration for %s for %d times\n", (is2T ? "2T2R" : "1T1R"), t));*/

		/*Save ADDA parameters, turn Path A ADDA on*/
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_SaveADDARegisters_92E(pAdapter, ADDA_REG, pRFCalibrateInfo->ADDA_backup, IQK_ADDA_REG_NUM);
		_PHY_SaveMACRegisters_92E(pAdapter, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);
		_PHY_SaveADDARegisters_92E(pAdapter, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup, IQK_BB_REG_NUM);
		#else
		_PHY_SaveADDARegisters_92E(pDM_Odm, ADDA_REG, pRFCalibrateInfo->ADDA_backup, IQK_ADDA_REG_NUM);
		_PHY_SaveMACRegisters_92E(pDM_Odm, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);
		_PHY_SaveADDARegisters_92E(pDM_Odm, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup, IQK_BB_REG_NUM);
		#endif
	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQ Calibration for %s for %d times\n", (is2T ? "2T2R" : "1T1R"), t));

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	_PHY_PathADDAOn_92E(pAdapter, ADDA_REG, TRUE, is2T);
	#else
	_PHY_PathADDAOn_92E(pDM_Odm, ADDA_REG, TRUE, is2T);
	#endif

	/*
		if(t==0)
		{
			pRFCalibrateInfo->bRfPiEnable = (u1Byte)ODM_GetBBReg(pDM_Odm, rFPGA0_XA_HSSIParameter1, BIT(8));
		}

		if(!pRFCalibrateInfo->bRfPiEnable){
			// Switch BB to PI mode to do IQ Calibration.
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			_PHY_PIModeSwitch_92E(pAdapter, TRUE);
	#else
			_PHY_PIModeSwitch_92E(pDM_Odm, TRUE);
	#endif
		}
	*/
	//MAC settings
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_MACSettingCalibration_92E(pAdapter, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);
	#else
	_PHY_MACSettingCalibration_92E(pDM_Odm, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);
	#endif

	//BB setting
	//ODM_SetBBReg(pDM_Odm, rFPGA0_RFMOD, BIT24, 0x00);
	ODM_SetBBReg(pDM_Odm, rCCK0_AFESetting, 0x0f000000, 0xf);
	ODM_SetBBReg(pDM_Odm, rOFDM0_TRxPathEnable, bMaskDWord, 0x03a05600);
	ODM_SetBBReg(pDM_Odm, rOFDM0_TRMuxPar, bMaskDWord, 0x000800e4);
	ODM_SetBBReg(pDM_Odm, rFPGA0_XCD_RFInterfaceSW, bMaskDWord, 0x55204200);

	if ((pDM_Odm->ExtLNA) && !(pDM_Odm->ExtPA)) { /* external LNA / external PA = 1 /0 */
		/*PAPE force to high*/
		/*just for high power with external LNA, without external PA*/
		ODM_SetBBReg(pDM_Odm, 0x930, 0xf, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x930, 0x0f000000, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x938, 0xf, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x938, 0x0f000000, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x92c, bMaskDWord, 0x00410041);
	} else if (pDM_Odm->ExtPA) { /* external PA = 1*/
		/*PAPE force to low*/
		/*just for high power with external PA, without external LNA*/
		ODM_SetBBReg(pDM_Odm, 0x930, 0xf, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x930, 0x0f000000, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x938, 0xf, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x938, 0x0f000000, 0x7);
		ODM_SetBBReg(pDM_Odm, 0x92c, bMaskDWord, 0x00000000);
	}

	// IQ calibration setting
//	ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK setting!\n"));
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);
	ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

	if (is2T) {

		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_PathBStandBy_92E(pAdapter);

		// Turn  ADDA on
		_PHY_PathADDAOn_92E(pAdapter, ADDA_REG, FALSE, is2T);
		#else
		_PHY_PathBStandBy_92E(pDM_Odm);

		// Turn ADDA on
		_PHY_PathADDAOn_92E(pDM_Odm, ADDA_REG, FALSE, is2T);
		#endif

	}

//path A TXIQK
	#if 1
	for (i = 0 ; i < retryCount ; i++) {
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		PathAOK = phy_PathA_IQK_8192E(pAdapter, is2T);
		#else
		PathAOK = phy_PathA_IQK_8192E(pDM_Odm, is2T);
		#endif
//		if(PathAOK == 0x03){
		if (PathAOK == 0x01) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Tx IQK Success!!\n"));
			result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord) & 0x3FF0000) >> 16;
			result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord) & 0x3FF0000) >> 16;
			break;
		} else
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Tx IQK Fail!!\n"));
		#if 0
		else if (i == (retryCount - 1) && PathAOK == 0x01) {	/*Tx IQK OK*/
			RT_DISP(FINIT, INIT_IQK, ("Path A IQK Only  Tx Success!!\n"));

			result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord) & 0x3FF0000) >> 16;
			result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord) & 0x3FF0000) >> 16;
		}
		#endif
	}
	#endif

//path A RXIQK
	#if 1
	for (i = 0 ; i < retryCount ; i++) {
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		PathAOK = phy_PathA_RxIQK_92E(pAdapter, is2T);
		#else
		PathAOK = phy_PathA_RxIQK_92E(pDM_Odm, is2T);
		#endif
		if (PathAOK == 0x03) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Rx IQK Success!!\n"));
//				result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
//				result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
			result[t][2] = (ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_A_2, bMaskDWord) & 0x3FF0000) >> 16;
			result[t][3] = (ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_A_2, bMaskDWord) & 0x3FF0000) >> 16;
			break;
		} else {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A Rx IQK Fail!!\n"));
		}
	}

	if(0x00 == PathAOK){		
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path A IQK failed!!\n"));
	}

	#endif

	if (is2T) {

		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_PathAStandBy_92E(pAdapter);

		// Turn ADDA on
		_PHY_PathADDAOn_92E(pAdapter, ADDA_REG, FALSE, is2T);
		#else
		_PHY_PathAStandBy_92E(pDM_Odm);

		// Turn ADDA on
		_PHY_PathADDAOn_92E(pDM_Odm, ADDA_REG, FALSE, is2T);
		#endif

		// IQ calibration setting
		/*ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK setting!\n"));*/
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x808000);
		ODM_SetBBReg(pDM_Odm, rTx_IQK, bMaskDWord, 0x01007c00);
		ODM_SetBBReg(pDM_Odm, rRx_IQK, bMaskDWord, 0x01004800);

//path B Tx IQK
		#if 1
		for (i = 0 ; i < retryCount ; i++) {
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			PathBOK = phy_PathB_IQK_8192E(pAdapter);
			#else
			PathBOK = phy_PathB_IQK_8192E(pDM_Odm);
			#endif
//		if(PathBOK == 0x03){
			if (PathBOK == 0x01) {
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B Tx IQK Success!!\n"));
				result[t][4] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_B, bMaskDWord) & 0x3FF0000) >> 16;
				result[t][5] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_B, bMaskDWord) & 0x3FF0000) >> 16;
				break;
			}
			#if 0
			else if (i == (retryCount - 1) && PathAOK == 0x01) {	/*Tx IQK OK*/
				RT_DISP(FINIT, INIT_IQK, ("Path B IQK Only  Tx Success!!\n"));

				result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_B, bMaskDWord) & 0x3FF0000) >> 16;
				result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_B, bMaskDWord) & 0x3FF0000) >> 16;
			}
			#endif
		}
		#endif

//path B RX IQK
		#if 1

		for (i = 0 ; i < retryCount ; i++) {
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			PathBOK = phy_PathB_RxIQK_92E(pAdapter, is2T);
			#else
			PathBOK = phy_PathB_RxIQK_92E(pDM_Odm, is2T);
			#endif
			if (PathBOK == 0x03) {
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B Rx IQK Success!!\n"));
//				result[t][0] = (ODM_GetBBReg(pDM_Odm, rTx_Power_Before_IQK_A, bMaskDWord)&0x3FF0000)>>16;
//				result[t][1] = (ODM_GetBBReg(pDM_Odm, rTx_Power_After_IQK_A, bMaskDWord)&0x3FF0000)>>16;
				result[t][6] = (ODM_GetBBReg(pDM_Odm, rRx_Power_Before_IQK_B_2, bMaskDWord) & 0x3FF0000) >> 16;
				result[t][7] = (ODM_GetBBReg(pDM_Odm, rRx_Power_After_IQK_B_2, bMaskDWord) & 0x3FF0000) >> 16;
				break;
		}
	}

		if (0x00 == PathBOK) {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Path B IQK failed!!\n"));
			/**/
		}
		#endif
	}

	//Back to BB mode, load original value
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:Back to BB mode, load original value!\n"));
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	if (t != 0) {


		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

		/* Reload ADDA power saving parameters*/
		_PHY_ReloadADDARegisters_92E(pAdapter, ADDA_REG, pRFCalibrateInfo->ADDA_backup, IQK_ADDA_REG_NUM);

		/* Reload MAC parameters*/
		_PHY_ReloadMACRegisters_92E(pAdapter, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);

		_PHY_ReloadADDARegisters_92E(pAdapter, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup, IQK_BB_REG_NUM);
		#else
		/* Reload ADDA power saving parameters*/
		_PHY_ReloadADDARegisters_92E(pDM_Odm, ADDA_REG, pRFCalibrateInfo->ADDA_backup, IQK_ADDA_REG_NUM);

		/* Reload MAC parameters*/
		_PHY_ReloadMACRegisters_92E(pDM_Odm, IQK_MAC_REG, pRFCalibrateInfo->IQK_MAC_backup);

		_PHY_ReloadADDARegisters_92E(pDM_Odm, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup, IQK_BB_REG_NUM);
		#endif

		/*Allen initial gain 0xc50*/
		/* Restore RX initial gain*/
		ODM_SetBBReg(pDM_Odm, 0xc50, bMaskByte0, 0x50);
		ODM_SetBBReg(pDM_Odm, 0xc50, bMaskByte0, tmp0xc50);
		if (is2T) {
			ODM_SetBBReg(pDM_Odm, 0xc58, bMaskByte0, 0x50);
			ODM_SetBBReg(pDM_Odm, 0xc58, bMaskByte0, tmp0xc58);
		}

		//load 0xe30 IQC default value
		ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x01008c00);
		ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x01008c00);


	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_IQCalibrate_8192E() <==\n"));

}


VOID
phy_LCCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		is2T
)
{
	u1Byte	tmpReg;
	u4Byte	RF_Amode = 0, RF_Bmode = 0, LC_Cal;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	//Check continuous TX and Packet TX
	tmpReg = ODM_Read1Byte(pDM_Odm, 0xd03);

	if ((tmpReg & 0x70) != 0)				/*Deal with contisuous TX case*/
		ODM_Write1Byte(pDM_Odm, 0xd03, tmpReg & 0x8F);	/*disable all continuous TX*/
	else							/* Deal with Packet TX case*/
		ODM_Write1Byte(pDM_Odm, REG_TXPAUSE, 0xFF);	/* block all queues*/

	if ((tmpReg & 0x70) != 0) {
		//1. Read original RF mode
		//Path-A
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		RF_Amode = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_A, RF_AC, bMask12Bits);

		//Path-B
		if (is2T)
			RF_Bmode = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_B, RF_AC, bMask12Bits);
		#else
		RF_Amode = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits);

		//Path-B
		if (is2T)
			RF_Bmode = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits);
		#endif

		//2. Set RF mode = standby mode
		//Path-A
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits, (RF_Amode & 0x8FFFF) | 0x10000);

		//Path-B
		if (is2T)
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits, (RF_Bmode & 0x8FFFF) | 0x10000);
	}

	//3. Read RF reg18
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	LC_Cal = PHY_QueryRFReg(pAdapter, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits);
	#else
	LC_Cal = ODM_GetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits);
	#endif

	//4. Set LC calibration begin	bit15
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_CHNLBW, bMask12Bits, LC_Cal | 0x08000);

	ODM_delay_ms(100);


	//Restore original situation
	if ((tmpReg & 0x70) != 0) {	/*Deal with contisuous TX case*/
		//Path-A
		ODM_Write1Byte(pDM_Odm, 0xd03, tmpReg);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMask12Bits, RF_Amode);

		//Path-B
		if (is2T)
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMask12Bits, RF_Bmode);
	} else {   /*Deal with Packet TX case*/
		ODM_Write1Byte(pDM_Odm, REG_TXPAUSE, 0x00);
	}
}

//Analog Pre-distortion calibration
#define		APK_BB_REG_NUM	8
#define		APK_CURVE_REG_NUM 4
#define		PATH_NUM		2

VOID
phy_APCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s1Byte 		delta,
	IN	BOOLEAN		is2T
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	u4Byte 			regD[PATH_NUM];
	u4Byte			tmpReg, index, offset,  apkbound;
	u1Byte			path, i, pathbound = PATH_NUM;
	u4Byte			BB_backup[APK_BB_REG_NUM];
	u4Byte			BB_REG[APK_BB_REG_NUM] = {
		rFPGA1_TxBlock, 	rOFDM0_TRxPathEnable,
		rFPGA0_RFMOD, 	rOFDM0_TRMuxPar,
		rFPGA0_XCD_RFInterfaceSW,	rFPGA0_XAB_RFInterfaceSW,
		rFPGA0_XA_RFInterfaceOE, 	rFPGA0_XB_RFInterfaceOE
	};
	u4Byte			BB_AP_MODE[APK_BB_REG_NUM] = {
		0x00000020, 0x00a05430, 0x02040000,
		0x000800e4, 0x00204000
	};
	u4Byte			BB_normal_AP_MODE[APK_BB_REG_NUM] = {
		0x00000020, 0x00a05430, 0x02040000,
		0x000800e4, 0x22204000
	};

	u4Byte			AFE_backup[IQK_ADDA_REG_NUM];
	u4Byte			AFE_REG[IQK_ADDA_REG_NUM] = {
		rFPGA0_XCD_SwitchControl, 	rBlue_Tooth,
		rRx_Wait_CCA, 		rTx_CCK_RFON,
		rTx_CCK_BBON, 	rTx_OFDM_RFON,
		rTx_OFDM_BBON, 	rTx_To_Rx,
		rTx_To_Tx, 		rRx_CCK,
		rRx_OFDM, 		rRx_Wait_RIFS,
		rRx_TO_Rx, 		rStandby,
		rSleep, 			rPMPD_ANAEN
	};

	u4Byte			MAC_backup[IQK_MAC_REG_NUM];
	u4Byte			MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE, 		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	u4Byte			APK_RF_init_value[PATH_NUM][APK_BB_REG_NUM] = {
		{0x0852c, 0x1852c, 0x5852c, 0x1852c, 0x5852c},
		{0x2852e, 0x0852e, 0x3852e, 0x0852e, 0x0852e}
	};

	u4Byte			APK_normal_RF_init_value[PATH_NUM][APK_BB_REG_NUM] = {
		{0x0852c, 0x0a52c, 0x3a52c, 0x5a52c, 0x5a52c},	/*path settings equal to path b settings*/
		{0x0852c, 0x0a52c, 0x5a52c, 0x5a52c, 0x5a52c}
	};

	u4Byte			APK_RF_value_0[PATH_NUM][APK_BB_REG_NUM] = {
		{0x52019, 0x52014, 0x52013, 0x5200f, 0x5208d},
		{0x5201a, 0x52019, 0x52016, 0x52033, 0x52050}
	};

	u4Byte			APK_normal_RF_value_0[PATH_NUM][APK_BB_REG_NUM] = {
		{0x52019, 0x52017, 0x52010, 0x5200d, 0x5206a},	/*path settings equal to path b settings*/
		{0x52019, 0x52017, 0x52010, 0x5200d, 0x5206a}
	};

	u4Byte			AFE_on_off[PATH_NUM] = {
		0x04db25a4, 0x0b1b25a4
	};	/*path A on path B off / path A off path B on*/

	u4Byte			APK_offset[PATH_NUM] = {
		rConfig_AntA, rConfig_AntB
	};

	u4Byte			APK_normal_offset[PATH_NUM] = {
		rConfig_Pmpd_AntA, rConfig_Pmpd_AntB
	};

	u4Byte			APK_value[PATH_NUM] = {
		0x92fc0000, 0x12fc0000
	};

	u4Byte			APK_normal_value[PATH_NUM] = {
		0x92680000, 0x12680000
	};

	s1Byte			APK_delta_mapping[APK_BB_REG_NUM][13] = {
		{ -4, -3, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{ -4, -3, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{ -6, -4, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{ -1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{ -11, -9, -7, -5, -3, -1, 0, 0, 0, 0, 0, 0, 0}
	};

	u4Byte			APK_normal_setting_value_1[13] = {
		0x01017018, 0xf7ed8f84, 0x1b1a1816, 0x2522201e, 0x322e2b28,
		0x433f3a36, 0x5b544e49, 0x7b726a62, 0xa69a8f84, 0xdfcfc0b3,
		0x12680000, 0x00880000, 0x00880000
	};

	u4Byte			APK_normal_setting_value_2[16] = {
		0x01c7021d, 0x01670183, 0x01000123, 0x00bf00e2, 0x008d00a3,
		0x0068007b, 0x004d0059, 0x003a0042, 0x002b0031, 0x001f0025,
		0x0017001b, 0x00110014, 0x000c000f, 0x0009000b, 0x00070008,
		0x00050006
	};

	u4Byte			APK_result[PATH_NUM][APK_BB_REG_NUM];	//val_1_1a, val_1_2a, val_2a, val_3a, val_4a
//	u4Byte			AP_curve[PATH_NUM][APK_CURVE_REG_NUM];

	s4Byte			BB_offset, delta_V, delta_offset;

	#if MP_DRIVER == 1
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
	#else
	PMPT_CONTEXT	pMptCtx = &(pAdapter->MptCtx);
	#endif
	pMptCtx->APK_bound[0] = 45;
	pMptCtx->APK_bound[1] = 52;

	#endif

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("==>phy_APCalibrate_8188E() delta %d\n", delta));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("AP Calibration for %s\n", (is2T ? "2T2R" : "1T1R")));
	if (!is2T)
		pathbound = 1;

	//2 FOR NORMAL CHIP SETTINGS

// Temporarily do not allow normal driver to do the following settings because these offset
// and value will cause RF internal PA to be unpredictably disabled by HW, such that RF Tx signal
// will disappear after disable/enable card many times on 88CU. RF SD and DD have not find the
// root cause, so we remove these actions temporarily. Added by tynli and SD3 Allen. 2010.05.31.
	#if MP_DRIVER != 1
	return;
	#endif
	//settings adjust for normal chip
	for (index = 0; index < PATH_NUM; index ++) {
		APK_offset[index] = APK_normal_offset[index];
		APK_value[index] = APK_normal_value[index];
		AFE_on_off[index] = 0x6fdb25a4;
	}

	for (index = 0; index < APK_BB_REG_NUM; index ++) {
		for (path = 0; path < pathbound; path++) {
			APK_RF_init_value[path][index] = APK_normal_RF_init_value[path][index];
			APK_RF_value_0[path][index] = APK_normal_RF_value_0[path][index];
		}
		BB_AP_MODE[index] = BB_normal_AP_MODE[index];
	}

	apkbound = 6;

	//save BB default value
	for (index = 0; index < APK_BB_REG_NUM ; index++) {
		if (index == 0)		/*skip*/
			continue;
		BB_backup[index] = ODM_GetBBReg(pDM_Odm, BB_REG[index], bMaskDWord);
	}

	/*save MAC default value*/
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_SaveMACRegisters_92E(pAdapter, MAC_REG, MAC_backup);

	/*save AFE default value*/
	_PHY_SaveADDARegisters_92E(pAdapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#else
	_PHY_SaveMACRegisters_92E(pDM_Odm, MAC_REG, MAC_backup);

	/*save AFE default value*/
	_PHY_SaveADDARegisters_92E(pDM_Odm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#endif

	for (path = 0; path < pathbound; path++) {


		if (path == ODM_RF_PATH_A) {
			/*path A APK*/
			/*load APK setting*/
			/*path-A*/
			offset = rPdp_AntA;
			for (index = 0; index < 11; index ++) {
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}

			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x12680000);

			offset = rConfig_AntA;
			for (; index < 13; index ++) {
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}

			//page-B1
			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);

			//path A
			offset = rPdp_AntA;
			for (index = 0; index < 16; index++) {
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_2[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}
			ODM_SetBBReg(pDM_Odm,  rFPGA0_IQK, 0xffffff00, 0x000000);
		} else if (path == ODM_RF_PATH_B) {
			/*path B APK*/
			/*load APK setting*/
			/*path-B*/
			offset = rPdp_AntB;
			for (index = 0; index < 10; index ++) {
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x12680000);
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			PHY_SetBBReg(pAdapter, rConfig_Pmpd_AntB, bMaskDWord, 0x12680000);
			#else
			PHY_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x12680000);
			#endif

			offset = rConfig_AntA;
			index = 11;
			for (; index < 13; index ++) { /*offset 0xb68, 0xb6c*/
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}

			//page-B1
			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);

			//path B
			offset = 0xb60;
			for (index = 0; index < 16; index++) {
				ODM_SetBBReg(pDM_Odm, offset, bMaskDWord, APK_normal_setting_value_2[index]);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", offset, ODM_GetBBReg(pDM_Odm, offset, bMaskDWord)));

				offset += 0x04;
			}
			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0);
		}

		//save RF default value
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		regD[path] = PHY_QueryRFReg(pAdapter, path, RF_TXBIAS_A, bMaskDWord);
		#else
		regD[path] = ODM_GetRFReg(pDM_Odm, path, RF_TXBIAS_A, bMaskDWord);
		#endif

		//Path A AFE all on, path B AFE All off or vise versa
		for (index = 0; index < IQK_ADDA_REG_NUM ; index++)
			ODM_SetBBReg(pDM_Odm, AFE_REG[index], bMaskDWord, AFE_on_off[path]);
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xe70 %x\n", ODM_GetBBReg(pDM_Odm, rRx_Wait_CCA, bMaskDWord)));

		//BB to AP mode
		if (path == 0) {
			for (index = 0; index < APK_BB_REG_NUM ; index++) {

				if (index == 0)		/*skip*/
					continue;
				else if (index < 5)
					ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_AP_MODE[index]);
				else if (BB_REG[index] == 0x870)
					ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_backup[index] | BIT10 | BIT26);
				else
					ODM_SetBBReg(pDM_Odm, BB_REG[index], BIT10, 0x0);
			}

			ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x01008c00);
			ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x01008c00);
		} else {	/*path B*/
			ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x01008c00);
			ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x01008c00);

		}

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x800 %x\n", ODM_GetBBReg(pDM_Odm, 0x800, bMaskDWord)));

		//MAC settings
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_MACSettingCalibration_92E(pAdapter, MAC_REG, MAC_backup);
		#else
		_PHY_MACSettingCalibration_92E(pDM_Odm, MAC_REG, MAC_backup);
		#endif

		if (path == ODM_RF_PATH_A) 	/*Path B to standby mode*/
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMaskDWord, 0x10000);
		else {		/*Path A to standby mode*/
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMaskDWord, 0x10000);
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE1, bMaskDWord, 0x1000f);
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE2, bMaskDWord, 0x20103);
		}

		delta_offset = ((delta + 14) / 2);
		if (delta_offset < 0)
			delta_offset = 0;
		else if (delta_offset > 12)
			delta_offset = 12;

		//AP calibration
		for (index = 0; index < APK_BB_REG_NUM; index++) {
			if (index != 1)	/*only DO PA11+PAD01001, AP RF setting*/
				continue;

			tmpReg = APK_RF_init_value[path][index];
			#if 1
			if (!pRFCalibrateInfo->bAPKThermalMeterIgnore) {
				BB_offset = (tmpReg & 0xF0000) >> 16;

				if (!(tmpReg & BIT15))   /*sign bit 0*/
					BB_offset = -BB_offset;

				delta_V = APK_delta_mapping[index][delta_offset];

				BB_offset += delta_V;

				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() APK index %d tmpReg 0x%x delta_V %d delta_offset %d\n", index, tmpReg, delta_V, delta_offset));

				if (BB_offset < 0) {
					tmpReg = tmpReg & (~BIT15);
					BB_offset = -BB_offset;
				} else
					tmpReg = tmpReg | BIT15;
				tmpReg = (tmpReg & 0xFFF0FFFF) | (BB_offset << 16);
			}
			#endif

			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, RF_IPA_A, bMaskDWord, 0x8992e);
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xc %x\n", PHY_QueryRFReg(pAdapter, path, RF_IPA_A, bMaskDWord)));
			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, RF_AC, bMaskDWord, APK_RF_value_0[path][index]);
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x0 %x\n", PHY_QueryRFReg(pAdapter, path, RF_AC, bMaskDWord)));
			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, RF_TXBIAS_A, bMaskDWord, tmpReg);
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xd %x\n", PHY_QueryRFReg(pAdapter, path, RF_TXBIAS_A, bMaskDWord)));
			#else
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xc %x\n", ODM_GetRFReg(pDM_Odm, path, RF_IPA_A, bMaskDWord)));
			ODM_SetRFReg(pDM_Odm, path, RF_AC, bMaskDWord, APK_RF_value_0[path][index]);
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x0 %x\n", ODM_GetRFReg(pDM_Odm, path, RF_AC, bMaskDWord)));
			ODM_SetRFReg(pDM_Odm, path, RF_TXBIAS_A, bMaskDWord, tmpReg);
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xd %x\n", ODM_GetRFReg(pDM_Odm, path, RF_TXBIAS_A, bMaskDWord)));
			#endif

			/* PA11+PAD01111, one shot*/
			i = 0;
			do {
				ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x800000);
				{
					ODM_SetBBReg(pDM_Odm, APK_offset[path], bMaskDWord, APK_value[0]);
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", APK_offset[path], ODM_GetBBReg(pDM_Odm, APK_offset[path], bMaskDWord)));
					ODM_delay_ms(3);
					ODM_SetBBReg(pDM_Odm, APK_offset[path], bMaskDWord, APK_value[1]);
					ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0x%x value 0x%x\n", APK_offset[path], ODM_GetBBReg(pDM_Odm, APK_offset[path], bMaskDWord)));

					ODM_delay_ms(20);
				}
				ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

				if (path == ODM_RF_PATH_A)
					tmpReg = ODM_GetBBReg(pDM_Odm, rAPK, 0x03E00000);
				else
					tmpReg = ODM_GetBBReg(pDM_Odm, rAPK, 0xF8000000);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_APCalibrate_8188E() offset 0xbd8[25:21] %x\n", tmpReg));


				i++;
			} while (tmpReg > apkbound && i < 4);

			APK_result[path][index] = tmpReg;
		}
	}

	/*reload MAC default value*/
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_ReloadMACRegisters_92E(pAdapter, MAC_REG, MAC_backup);
	#else
	_PHY_ReloadMACRegisters_92E(pDM_Odm, MAC_REG, MAC_backup);
	#endif

	/*reload BB default value*/
	for (index = 0; index < APK_BB_REG_NUM ; index++) {

		if (index == 0)		/*skip*/
			continue;
		ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_backup[index]);
	}

	//reload AFE default value
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_ReloadADDARegisters_92E(pAdapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#else
	_PHY_ReloadADDARegisters_92E(pDM_Odm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#endif

	//reload RF path default value
	for (path = 0; path < pathbound; path++) {
		ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, 0xd, bMaskDWord, regD[path]);
		if (path == ODM_RF_PATH_B) {
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE1, bMaskDWord, 0x1000f);
			ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE2, bMaskDWord, 0x20101);
		}

		//note no index == 0
		if (APK_result[path][1] > 6)
			APK_result[path][1] = 6;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("apk path %d result %d 0x%x \t", path, 1, APK_result[path][1]));
	}

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("\n"));


	for (path = 0; path < pathbound; path++) {
		ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, 0x3, bMaskDWord,
					 ((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (APK_result[path][1] << 5) | APK_result[path][1]));
		if (path == ODM_RF_PATH_A)
			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, 0x4, bMaskDWord,
						 ((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (0x00 << 5) | 0x05));
		else
			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, 0x4, bMaskDWord,
						 ((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (0x02 << 5) | 0x05));
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			ODM_SetRFReg(pDM_Odm, (ODM_RF_RADIO_PATH_E)path, RF_BS_PA_APSET_G9_G11, bMaskDWord,
						 ((0x08 << 15) | (0x08 << 10) | (0x08 << 5) | 0x08));
#endif
	}

	pRFCalibrateInfo->bAPKdone = TRUE;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("<==phy_APCalibrate_8188E()\n"));
}



#define		DP_BB_REG_NUM		7
#define		DP_RF_REG_NUM		1
#define		DP_RETRY_LIMIT		10
#define		DP_PATH_NUM		2
#define		DP_DPK_NUM			3
#define		DP_DPK_VALUE_NUM	2





VOID
PHY_IQCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN 	bReCovery
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#else  // (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif

	#if (MP_DRIVER == 1)
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->MptCtx);
	#else// (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
	#endif
	#endif//(MP_DRIVER == 1)
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);

	s4Byte			result[4][8];	//last is final result
	u1Byte			i, final_candidate, Indexforchannel;
	BOOLEAN			bPathAOK, bPathBOK;
	s4Byte			RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC, RegTmp = 0;
	BOOLEAN			is12simular, is13simular, is23simular;
	BOOLEAN 		bSingleTone = FALSE, bCarrierSuppression = FALSE;
	u4Byte			IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		rOFDM0_XARxIQImbalance, 	rOFDM0_XBRxIQImbalance,
		rOFDM0_ECCAThreshold, 	rOFDM0_AGCRSSITable,
		rOFDM0_XATxIQImbalance, 	rOFDM0_XBTxIQImbalance,
		rOFDM0_XCTxAFE, 			rOFDM0_XDTxAFE,
		rOFDM0_RxIQExtAnta
	};

	#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))
	if (ODM_CheckPowerStatus(pAdapter) == FALSE)
		return;
	#else
	prtl8192cd_priv	priv = pDM_Odm->priv;

	#ifdef MP_TEST
	if (priv->pshare->rf_ft_var.mp_specific) {
		if ((OPMODE & WIFI_MP_CTX_PACKET) || (OPMODE & WIFI_MP_CTX_ST))
			return;
	}
	#endif

	if (priv->pshare->IQK_88E_done)
		bReCovery = 1;
	priv->pshare->IQK_88E_done = 1;

	#endif

	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	if (!(pDM_Odm->SupportAbility & ODM_RF_CALIBRATION))
		return;
	#endif

	#if MP_DRIVER == 1
	bSingleTone = pMptCtx->bSingleTone;
	bCarrierSuppression = pMptCtx->bCarrierSuppression;
	#endif

	// 20120213<Kordan> Turn on when continuous Tx to pass lab testing. (required by Edlu)
	if (bSingleTone || bCarrierSuppression)
		return;

	#if DISABLE_BB_RF
	return;
	#endif

	if (pRFCalibrateInfo->bIQKInProgress)
		return;


	#if (DM_ODM_SUPPORT_TYPE & (ODM_CE|ODM_AP))
	if (bReCovery)
	#else /*for ODM_WIN*/
	if (bReCovery && 
	(!pAdapter->bInHctTest)) /*YJ,add for PowerTest,120405*/
	#endif
	{
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("PHY_IQCalibrate_92E: Return due to bReCovery!\n"));
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_ReloadADDARegisters_92E(pAdapter, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup_recover, 9);
		#else
		_PHY_ReloadADDARegisters_92E(pDM_Odm, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup_recover, 9);
		#endif
		return;
	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:Start!!!\n"));

	ODM_AcquireSpinLock(pDM_Odm, RT_IQK_SPINLOCK);
	pRFCalibrateInfo->bIQKInProgress = TRUE;
	ODM_ReleaseSpinLock(pDM_Odm, RT_IQK_SPINLOCK);

	for (i = 0; i < 8; i++) {
		result[0][i] = 0;
		result[1][i] = 0;
		result[2][i] = 0;

		if ((i == 0) || (i == 2) || (i == 4)  || (i == 6))
			result[3][i] = 0x100;
		else
			result[3][i] = 0;
	}

	final_candidate = 0xff;
	bPathAOK = FALSE;
	bPathBOK = FALSE;
	is12simular = FALSE;
	is23simular = FALSE;
	is13simular = FALSE;


	for (i = 0; i < 3; i++) {

		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

		phy_IQCalibrate_8192E(pAdapter, result, i, TRUE);
//	 		phy_IQCalibrate_8192E(pAdapter, result, i, FALSE);

		#else

//	 		phy_IQCalibrate_8192E(pDM_Odm, result, i, FALSE);
		phy_IQCalibrate_8192E(pDM_Odm, result, i, TRUE);
		#endif


		if (i == 1) {
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is12simular = phy_SimularityCompare_8192E(pAdapter, result, 0, 1);
			#else
			is12simular = phy_SimularityCompare_8192E(pDM_Odm, result, 0, 1);
			#endif

			if (is12simular) {
				final_candidate = 0;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is12simular final_candidate is %x\n", final_candidate));
				break;
			}
		}

		if (i == 2) {
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is13simular = phy_SimularityCompare_8192E(pAdapter, result, 0, 2);
			#else
			is13simular = phy_SimularityCompare_8192E(pDM_Odm, result, 0, 2);
			#endif

			if (is13simular) {
				final_candidate = 0;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is13simular final_candidate is %x\n", final_candidate));

				break;
			}
			#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			is23simular = phy_SimularityCompare_8192E(pAdapter, result, 1, 2);
			#else
			is23simular = phy_SimularityCompare_8192E(pDM_Odm, result, 1, 2);
			#endif

			if (is23simular) {
				final_candidate = 1;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is23simular final_candidate is %x\n", final_candidate));
			} else {
				/*
								for(i = 0; i < 4; i++)
									RegTmp &= result[3][i*2];

								if(RegTmp != 0)
									final_candidate = 3;
								else
									final_candidate = 0xFF;
				*/
				final_candidate = 3;

			}
		}
	}

	for (i = 0; i < 4; i++) {
		RegE94 = result[i][0];
		RegE9C = result[i][1];
		RegEA4 = result[i][2];
		RegEAC = result[i][3];
		RegEB4 = result[i][4];
		RegEBC = result[i][5];
		RegEC4 = result[i][6];
		RegECC = result[i][7];
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: RegE94=%x RegE9C=%x RegEA4=%x RegEAC=%x RegEB4=%x RegEBC=%x RegEC4=%x RegECC=%x\n ", RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC));
	}

	if (final_candidate != 0xff) {
		pRFCalibrateInfo->RegE94 = RegE94 = result[final_candidate][0];
		pRFCalibrateInfo->RegE9C = RegE9C = result[final_candidate][1];
		RegEA4 = result[final_candidate][2];
		RegEAC = result[final_candidate][3];
		pRFCalibrateInfo->RegEB4 = RegEB4 = result[final_candidate][4];
		pRFCalibrateInfo->RegEBC = RegEBC = result[final_candidate][5];
		RegEC4 = result[final_candidate][6];
		RegECC = result[final_candidate][7];
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: final_candidate is %x\n", final_candidate));
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: TX0_X=%x TX0_Y=%x RX0_X=%x RX0_Y=%x TX1_X=%x TX1_Y=%x RX1_X=%x RX1_Y=%x\n ", RegE94, RegE9C, RegEA4, RegEAC, RegEB4, RegEBC, RegEC4, RegECC));
		bPathAOK = bPathBOK = TRUE;
	} else {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: FAIL use default value\n"));

		pRFCalibrateInfo->RegE94 = pRFCalibrateInfo->RegEB4 = 0x100;	//X default value
		pRFCalibrateInfo->RegE9C = pRFCalibrateInfo->RegEBC = 0x0;		//Y default value
	}

	if ((RegE94 != 0)/*&&(RegEA4 != 0)*/) {
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_PHY_PathAFillIQKMatrix_92E(pAdapter, bPathAOK, result, final_candidate, (RegEA4 == 0));
		#else
		_PHY_PathAFillIQKMatrix_92E(pDM_Odm, bPathAOK, result, final_candidate, (RegEA4 == 0));
		#endif
	}

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
//Allen
//	if (IS_92C_SERIAL(pHalData->VersionID))
//	{
//		if((RegEB4 != 0)/*&&(RegEC4 != 0)*/)
//		{
	_PHY_PathBFillIQKMatrix_92E(pAdapter, bPathBOK, result, final_candidate, (RegEC4 == 0));
//		}
//	}
	#endif

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	Indexforchannel = ODM_GetRightChnlPlaceforIQK(pHalData->CurrentChannel);
	#else
	Indexforchannel = 0;
	#endif

//To Fix BSOD when final_candidate is 0xff
//by sherry 20120321
	if (final_candidate < 4) {
		for (i = 0; i < IQK_Matrix_REG_NUM; i++)
			pRFCalibrateInfo->IQKMatrixRegSetting[Indexforchannel].Value[0][i] = result[final_candidate][i];
		pRFCalibrateInfo->IQKMatrixRegSetting[Indexforchannel].bIQKDone = TRUE;
	}
	//RT_DISP(FINIT, INIT_IQK, ("\nIQK OK Indexforchannel %d.\n", Indexforchannel));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("\nIQK OK Indexforchannel %d.\n", Indexforchannel));
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	_PHY_SaveADDARegisters_92E(pAdapter, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup_recover, 9);
	#else
	_PHY_SaveADDARegisters_92E(pDM_Odm, IQK_BB_REG_92C, pRFCalibrateInfo->IQK_BB_backup_recover, IQK_BB_REG_NUM);
	#endif

	ODM_AcquireSpinLock(pDM_Odm, RT_IQK_SPINLOCK);
	pRFCalibrateInfo->bIQKInProgress = FALSE;
	ODM_ReleaseSpinLock(pDM_Odm, RT_IQK_SPINLOCK);

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK finished\n"));

}


VOID
PHY_LCCalibrate_8192E(
	IN	PVOID		pDM_VOID
)
{
	PDM_ODM_T		pDM_Odm = (PDM_ODM_T)pDM_VOID;
	BOOLEAN 		bSingleTone = FALSE, bCarrierSuppression = FALSE;
	u4Byte			timeout = 2000, timecount = 0;

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	PADAPTER  		pAdapter = pDM_Odm->Adapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	#if (MP_DRIVER == 1)
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->MptCtx);
	#else// (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
	#endif
	#endif//(MP_DRIVER == 1)
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);



	#if MP_DRIVER == 1
	bSingleTone = pMptCtx->bSingleTone;
	bCarrierSuppression = pMptCtx->bCarrierSuppression;
	#endif


	#if DISABLE_BB_RF
	return;
	#endif

	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	if (!(pDM_Odm->SupportAbility & ODM_RF_CALIBRATION))
		return;
	#endif
	// 20120213<Kordan> Turn on when continuous Tx to pass lab testing. (required by Edlu)
	if (bSingleTone || bCarrierSuppression)
		return;

	while (*(pDM_Odm->pbScanInProcess) && timecount < timeout) {
		ODM_delay_ms(50);
		timecount += 50;
	}

	pRFCalibrateInfo->bLCKInProgress = TRUE;

	//ODM_RT_TRACE(pDM_Odm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LCK:Start!!!interface %d currentband %x delay %d ms\n", pDM_Odm->interfaceIndex, pHalData->CurrentBandType92D, timecount));
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	if (IS_2T2R(pHalData->VersionID))
		phy_LCCalibrate_8192E(pAdapter, TRUE);
	else
	#endif
	{
		// For 88C 1T1R
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		phy_LCCalibrate_8192E(pAdapter, FALSE);
		#else
		phy_LCCalibrate_8192E(pDM_Odm, FALSE);
		#endif
	}

	pRFCalibrateInfo->bLCKInProgress = FALSE;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("LCK:Finish!!!interface %d\n", pDM_Odm->InterfaceIndex));

}

VOID
PHY_APCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	s1Byte 		delta
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	#if DISABLE_BB_RF
	return;
	#endif

	return;
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	if (!(pDM_Odm->SupportAbility & ODM_RF_CALIBRATION))
		return;
	#endif

	#if FOR_BRAZIL_PRETEST != 1
	if (pRFCalibrateInfo->bAPKdone)
	#endif
		return;

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	if (IS_92C_SERIAL(pHalData->VersionID))
		phy_APCalibrate_8192E(pAdapter, delta, TRUE);
	else
	#endif
	{
		// For 88C 1T1R
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		phy_APCalibrate_8192E(pAdapter, delta, FALSE);
		#else
		phy_APCalibrate_8192E(pDM_Odm, delta, FALSE);
		#endif
	}
}
VOID phy_SetRFPathSwitch_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		bMain,
	IN	BOOLEAN		is2T
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#elif (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif

	// MAIN: WiFi
	u4Byte reg = PHY_QueryMacReg(pAdapter, 0x64, bMaskDWord);
	reg &= 0xFFFFF000;
	reg = (bMain) ? (reg | 0x00000100) : (reg | 0x00000004);
	PHY_SetMacReg(pAdapter, 0x64, bMaskDWord, reg);

}

VOID PHY_SetRFPathSwitch_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		bMain
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	//HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#endif

	#if DISABLE_BB_RF
	return;
	#endif

	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	phy_SetRFPathSwitch_8192E(pAdapter, bMain, TRUE);
	#endif

}

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
//digital predistortion
VOID
phy_DigitalPredistortion_92E(
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN	PADAPTER	pAdapter,
#else
	IN PDM_ODM_T	pDM_Odm,
#endif
	IN	BOOLEAN		is2T
)
{
	#if (RT_PLATFORM == PLATFORM_WINDOWS)
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	u4Byte			tmpReg, tmpReg2, index,  i;
	u1Byte			path, pathbound = PATH_NUM;
	u4Byte			AFE_backup[IQK_ADDA_REG_NUM];
	u4Byte			AFE_REG[IQK_ADDA_REG_NUM] = {
		rFPGA0_XCD_SwitchControl, 	rBlue_Tooth,
		rRx_Wait_CCA, 		rTx_CCK_RFON,
		rTx_CCK_BBON, 	rTx_OFDM_RFON,
		rTx_OFDM_BBON, 	rTx_To_Rx,
		rTx_To_Tx, 		rRx_CCK,
		rRx_OFDM, 		rRx_Wait_RIFS,
		rRx_TO_Rx, 		rStandby,
		rSleep, 			rPMPD_ANAEN
	};

	u4Byte			BB_backup[DP_BB_REG_NUM];
	u4Byte			BB_REG[DP_BB_REG_NUM] = {
		rOFDM0_TRxPathEnable, rFPGA0_RFMOD,
		rOFDM0_TRMuxPar, 	rFPGA0_XCD_RFInterfaceSW,
		rFPGA0_XAB_RFInterfaceSW, rFPGA0_XA_RFInterfaceOE,
		rFPGA0_XB_RFInterfaceOE
	};
	u4Byte			BB_settings[DP_BB_REG_NUM] = {
		0x00a05430, 0x02040000, 0x000800e4, 0x22208000,
		0x0, 0x0, 0x0
	};

	u4Byte			RF_backup[DP_PATH_NUM][DP_RF_REG_NUM];
	u4Byte			RF_REG[DP_RF_REG_NUM] = {
		RF_TXBIAS_A
	};

	u4Byte			MAC_backup[IQK_MAC_REG_NUM];
	u4Byte			MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE, 		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	u4Byte			Tx_AGC[DP_DPK_NUM][DP_DPK_VALUE_NUM] = {
		{0x1e1e1e1e, 0x03901e1e},
		{0x18181818, 0x03901818},
		{0x0e0e0e0e, 0x03900e0e}
	};

	u4Byte			AFE_on_off[PATH_NUM] = {
		0x04db25a4, 0x0b1b25a4
	};	/*path A on path B off / path A off path B on*/

	u1Byte			RetryCount = 0;


	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("==>phy_DigitalPredistortion_92E()\n"));

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("phy_DigitalPredistortion_92E for %s\n", (is2T ? "2T2R" : "1T1R")));

	//save BB default value
	for (index = 0; index < DP_BB_REG_NUM; index++)
		BB_backup[index] = ODM_GetBBReg(pDM_Odm, BB_REG[index], bMaskDWord);

	//save MAC default value
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_SaveMACRegisters_92E(pAdapter, BB_REG, MAC_backup);
	#else
	_PHY_SaveMACRegisters_92E(pDM_Odm, BB_REG, MAC_backup);
	#endif

	//save RF default value
	for (path = 0; path < DP_PATH_NUM; path++) {
		for (index = 0; index < DP_RF_REG_NUM; index++)
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			RF_backup[path][index] = PHY_QueryRFReg(pAdapter, path, RF_REG[index], bMaskDWord);
		#else
			RF_backup[path][index] = ODM_GetRFReg(pAdapter, path, RF_REG[index], bMaskDWord);
		#endif
	}

	//save AFE default value
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_SaveADDARegisters_92E(pAdapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#else
	_PHY_SaveADDARegisters_92E(pDM_Odm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
	#endif

	//Path A/B AFE all on
	for (index = 0; index < IQK_ADDA_REG_NUM ; index++)
		ODM_SetBBReg(pDM_Odm, AFE_REG[index], bMaskDWord, 0x6fdb25a4);

	//BB register setting
	for (index = 0; index < DP_BB_REG_NUM; index++) {
		if (index < 4)
			ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_settings[index]);
		else if (index == 4)
			ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_backup[index] | BIT10 | BIT26);
		else
			ODM_SetBBReg(pDM_Odm, BB_REG[index], BIT10, 0x00);
	}

	//MAC register setting
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_MACSettingCalibration_92E(pAdapter, MAC_REG, MAC_backup);
	#else
	_PHY_MACSettingCalibration_92E(pDM_Odm, MAC_REG, MAC_backup);
	#endif

	/*PAGE-E IQC setting*/
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_A, bMaskDWord, 0x01008c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_A, bMaskDWord, 0x01008c00);
	ODM_SetBBReg(pDM_Odm, rTx_IQK_Tone_B, bMaskDWord, 0x01008c00);
	ODM_SetBBReg(pDM_Odm, rRx_IQK_Tone_B, bMaskDWord, 0x01008c00);

	//path_A DPK
	//Path B to standby mode
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMaskDWord, 0x10000);

	// PA gain = 11 & PAD1 => tx_agc 1f ~11
	// PA gain = 11 & PAD2 => tx_agc 10~0e
	// PA gain = 01 => tx_agc 0b~0d
	// PA gain = 00 => tx_agc 0a~00
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);
	ODM_SetBBReg(pDM_Odm, 0xbc0, bMaskDWord, 0x0005361f);
	ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

	/*do inner loopback DPK 3 times*/
	for (i = 0; i < 3; i++) {
		//PA gain = 11 & PAD2 => tx_agc = 0x0f/0x0c/0x07
		for (index = 0; index < 3; index++)
			ODM_SetBBReg(pDM_Odm, 0xe00 + index * 4, bMaskDWord, Tx_AGC[i][0]);
		ODM_SetBBReg(pDM_Odm, 0xe00 + index * 4, bMaskDWord, Tx_AGC[i][1]);
		for (index = 0; index < 4; index++)
			ODM_SetBBReg(pDM_Odm, 0xe10 + index * 4, bMaskDWord, Tx_AGC[i][0]);

		// PAGE_B for Path-A inner loopback DPK setting
		ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x02097098);
		ODM_SetBBReg(pDM_Odm, rPdp_AntA_4, bMaskDWord, 0xf76d9f84);
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x0004ab87);
		ODM_SetBBReg(pDM_Odm, rConfig_AntA, bMaskDWord, 0x00880000);

		//----send one shot signal----//
		// Path A
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x80047788);
		ODM_delay_ms(1);
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x00047788);
		ODM_delay_ms(50);
	}

	//PA gain = 11 => tx_agc = 1a
	for (index = 0; index < 3; index++)
		ODM_SetBBReg(pDM_Odm, 0xe00 + index * 4, bMaskDWord, 0x34343434);
	ODM_SetBBReg(pDM_Odm, 0xe08 + index * 4, bMaskDWord, 0x03903434);
	for (index = 0; index < 4; index++)
		ODM_SetBBReg(pDM_Odm, 0xe10 + index * 4, bMaskDWord, 0x34343434);

	//====================================
	// PAGE_B for Path-A DPK setting
	//====================================
	// open inner loopback @ b00[19]:10 od 0xb00 0x01097018
	ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x02017098);
	ODM_SetBBReg(pDM_Odm, rPdp_AntA_4, bMaskDWord, 0xf76d9f84);
	ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x0004ab87);
	ODM_SetBBReg(pDM_Odm, rConfig_AntA, bMaskDWord, 0x00880000);

	//rf_lpbk_setup
	//1.rf 00:5205a, rf 0d:0e52c
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x0c, bMaskDWord, 0x8992b);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x0d, bMaskDWord, 0x0e52c);
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, 0x00, bMaskDWord, 0x5205a);

	//----send one shot signal----//
	// Path A
	ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x800477c0);
	ODM_delay_ms(1);
	ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x000477c0);
	ODM_delay_ms(50);

	while (RetryCount < DP_RETRY_LIMIT && !pRFCalibrateInfo->bDPPathAOK) {
		//----read back measurement results----//
		ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x0c297018);
		tmpReg = ODM_GetBBReg(pDM_Odm, 0xbe0, bMaskDWord);
		ODM_delay_ms(10);
		ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x0c29701f);
		tmpReg2 = ODM_GetBBReg(pDM_Odm, 0xbe8, bMaskDWord);
		ODM_delay_ms(10);

		tmpReg = (tmpReg & bMaskHWord) >> 16;
		tmpReg2 = (tmpReg2 & bMaskHWord) >> 16;
		if (tmpReg < 0xf0 || tmpReg > 0x105 || tmpReg2 > 0xff) {
			ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x02017098);

			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x800000);
			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
			ODM_delay_ms(1);
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x800477c0);
			ODM_delay_ms(1);
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x000477c0);
			ODM_delay_ms(50);
			RetryCount++;
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path A DPK RetryCount %d 0xbe0[31:16] %x 0xbe8[31:16] %x\n", RetryCount, tmpReg, tmpReg2));
		} else {
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path A DPK Success\n"));
			pRFCalibrateInfo->bDPPathAOK = TRUE;
			break;
		}
	}
	RetryCount = 0;

	//DPP path A
	if (pRFCalibrateInfo->bDPPathAOK) {
		// DP settings
		ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x01017098);
		ODM_SetBBReg(pDM_Odm, rPdp_AntA_4, bMaskDWord, 0x776d9f84);
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntA, bMaskDWord, 0x0004ab87);
		ODM_SetBBReg(pDM_Odm, rConfig_AntA, bMaskDWord, 0x00880000);
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);

		for (i = rPdp_AntA; i <= 0xb3c; i += 4) {
			ODM_SetBBReg(pDM_Odm, i, bMaskDWord, 0x40004000);
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path A ofsset = 0x%x\n", i));
		}

		//pwsf
		ODM_SetBBReg(pDM_Odm, 0xb40, bMaskDWord, 0x40404040);
		ODM_SetBBReg(pDM_Odm, 0xb44, bMaskDWord, 0x28324040);
		ODM_SetBBReg(pDM_Odm, 0xb48, bMaskDWord, 0x10141920);

		for (i = 0xb4c; i <= 0xb5c; i += 4)
			ODM_SetBBReg(pDM_Odm, i, bMaskDWord, 0x0c0c0c0c);

		//TX_AGC boundary
		ODM_SetBBReg(pDM_Odm, 0xbc0, bMaskDWord, 0x0005361f);
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
	} else {
		ODM_SetBBReg(pDM_Odm, rPdp_AntA, bMaskDWord, 0x00000000);
		ODM_SetBBReg(pDM_Odm, rPdp_AntA_4, bMaskDWord, 0x00000000);
	}

	/*DPK path B*/
	if (is2T) {
		/*Path A to standby mode*/
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_AC, bMaskDWord, 0x10000);

		/* LUTs => tx_agc*/
		/* PA gain = 11 & PAD1, => tx_agc 1f ~11*/
		/* PA gain = 11 & PAD2, => tx_agc 10 ~0e*/
		/* PA gain = 01 => tx_agc 0b ~0d*/
		/* PA gain = 00 => tx_agc 0a ~00*/
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);
		ODM_SetBBReg(pDM_Odm, 0xbc4, bMaskDWord, 0x0005361f);
		ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

		/*do inner loopback DPK 3 times*/
		for (i = 0; i < 3; i++) {
			/*PA gain = 11 & PAD2 => tx_agc = 0x0f/0x0c/0x07*/
			for (index = 0; index < 4; index++)
				ODM_SetBBReg(pDM_Odm, 0x830 + index * 4, bMaskDWord, Tx_AGC[i][0]);
			for (index = 0; index < 2; index++)
				ODM_SetBBReg(pDM_Odm, 0x848 + index * 4, bMaskDWord, Tx_AGC[i][0]);
			for (index = 0; index < 2; index++)
				ODM_SetBBReg(pDM_Odm, 0x868 + index * 4, bMaskDWord, Tx_AGC[i][0]);

			/* PAGE_B for Path-A inner loopback DPK setting*/
			ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x02097098);
			ODM_SetBBReg(pDM_Odm, rPdp_AntB_4, bMaskDWord, 0xf76d9f84);
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x0004ab87);
			ODM_SetBBReg(pDM_Odm, rConfig_AntB, bMaskDWord, 0x00880000);

			/*----send one shot signal----*/
			/* Path B*/
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x80047788);
			ODM_delay_ms(1);
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x00047788);
			ODM_delay_ms(50);
		}

		/* PA gain = 11 => tx_agc = 1a*/
		for (index = 0; index < 4; index++)
			ODM_SetBBReg(pDM_Odm, 0x830 + index * 4, bMaskDWord, 0x34343434);
		for (index = 0; index < 2; index++)
			ODM_SetBBReg(pDM_Odm, 0x848 + index * 4, bMaskDWord, 0x34343434);
		for (index = 0; index < 2; index++)
			ODM_SetBBReg(pDM_Odm, 0x868 + index * 4, bMaskDWord, 0x34343434);

		/* PAGE_B for Path-B DPK setting*/
		ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x02017098);
		ODM_SetBBReg(pDM_Odm, rPdp_AntB_4, bMaskDWord, 0xf76d9f84);
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x0004ab87);
		ODM_SetBBReg(pDM_Odm, rConfig_AntB, bMaskDWord, 0x00880000);

		// RF lpbk switches on
		ODM_SetBBReg(pDM_Odm, 0x840, bMaskDWord, 0x0101000f);
		ODM_SetBBReg(pDM_Odm, 0x840, bMaskDWord, 0x01120103);

		//Path-B RF lpbk
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0x0c, bMaskDWord, 0x8992b);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, 0x0d, bMaskDWord, 0x0e52c);
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_B, RF_AC, bMaskDWord, 0x5205a);

		//----send one shot signal----//
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x800477c0);
		ODM_delay_ms(1);
		ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x000477c0);
		ODM_delay_ms(50);

		while (RetryCount < DP_RETRY_LIMIT && !pRFCalibrateInfo->bDPPathBOK) {
			//----read back measurement results----//
			ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x0c297018);
			tmpReg = ODM_GetBBReg(pDM_Odm, 0xbf0, bMaskDWord);
			ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x0c29701f);
			tmpReg2 = ODM_GetBBReg(pDM_Odm, 0xbf8, bMaskDWord);

			tmpReg = (tmpReg & bMaskHWord) >> 16;
			tmpReg2 = (tmpReg2 & bMaskHWord) >> 16;

			if (tmpReg < 0xf0 || tmpReg > 0x105 || tmpReg2 > 0xff) {
				ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x02017098);

				ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x800000);
				ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);
				ODM_delay_ms(1);
				ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x800477c0);
				ODM_delay_ms(1);
				ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x000477c0);
				ODM_delay_ms(50);
				RetryCount++;
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B DPK RetryCount %d 0xbf0[31:16] %x, 0xbf8[31:16] %x\n", RetryCount , tmpReg, tmpReg2));
			} else {
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B DPK Success\n"));
				pRFCalibrateInfo->bDPPathBOK = TRUE;
				break;
			}
		}

		//DPP path B
		if (pRFCalibrateInfo->bDPPathBOK) {
			// DP setting
			// LUT by SRAM
			ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x01017098);
			ODM_SetBBReg(pDM_Odm, rPdp_AntB_4, bMaskDWord, 0x776d9f84);
			ODM_SetBBReg(pDM_Odm, rConfig_Pmpd_AntB, bMaskDWord, 0x0004ab87);
			ODM_SetBBReg(pDM_Odm, rConfig_AntB, bMaskDWord, 0x00880000);

			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x400000);
			for (i = 0xb60; i <= 0xb9c; i += 4) {
				ODM_SetBBReg(pDM_Odm, i, bMaskDWord, 0x40004000);
				ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B ofsset = 0x%x\n", i));
			}

			// PWSF
			ODM_SetBBReg(pDM_Odm, 0xba0, bMaskDWord, 0x40404040);
			ODM_SetBBReg(pDM_Odm, 0xba4, bMaskDWord, 0x28324050);
			ODM_SetBBReg(pDM_Odm, 0xba8, bMaskDWord, 0x0c141920);

			for (i = 0xbac; i <= 0xbbc; i += 4)
				ODM_SetBBReg(pDM_Odm, i, bMaskDWord, 0x0c0c0c0c);

			// tx_agc boundary
			ODM_SetBBReg(pDM_Odm, 0xbc4, bMaskDWord, 0x0005361f);
			ODM_SetBBReg(pDM_Odm, rFPGA0_IQK, 0xffffff00, 0x000000);

		} else {
			ODM_SetBBReg(pDM_Odm, rPdp_AntB, bMaskDWord, 0x00000000);
			ODM_SetBBReg(pDM_Odm, rPdp_AntB_4, bMaskDWord, 0x00000000);
		}
	}

	//reload BB default value
	for (index = 0; index < DP_BB_REG_NUM; index++)
		ODM_SetBBReg(pDM_Odm, BB_REG[index], bMaskDWord, BB_backup[index]);

	//reload RF default value
	for (path = 0; path < DP_PATH_NUM; path++) {
		for (i = 0 ; i < DP_RF_REG_NUM ; i++)
			ODM_SetRFReg(pDM_Odm, path, RF_REG[i], bMaskDWord, RF_backup[path][i]);
	}
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE1, bMaskDWord, 0x1000f);	//standby mode
	ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_MODE2, bMaskDWord, 0x20101);		//RF lpbk switches off

	/*reload AFE default value*/
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_PHY_ReloadADDARegisters_92E(pAdapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);

	/*reload MAC default value*/
	_PHY_ReloadMACRegisters_92E(pAdapter, MAC_REG, MAC_backup);
	#else
	_PHY_ReloadADDARegisters_92E(pDM_Odm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);

	/*reload MAC default value*/
	_PHY_ReloadMACRegisters_92E(pDM_Odm, MAC_REG, MAC_backup);
	#endif

	pRFCalibrateInfo->bDPdone = TRUE;
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("<==phy_DigitalPredistortion_92E()\n"));
	#endif
}

VOID
PHY_DigitalPredistortion_8192E(
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN	PADAPTER	pAdapter
#else
	IN PDM_ODM_T	pDM_Odm
#endif
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	PODM_RF_CAL_T	pRFCalibrateInfo = &(pDM_Odm->RFCalibrateInfo);
	#if DISABLE_BB_RF
	return;
	#endif

	return;

	if (pRFCalibrateInfo->bDPdone)
		return;
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	if (IS_92C_SERIAL(pHalData->VersionID))
		phy_DigitalPredistortion_92E(pAdapter, TRUE);
	else
	#endif
	{
		// For 88C 1T1R
		phy_DigitalPredistortion_92E(pAdapter, FALSE);
	}
}



//return value TRUE => Main; FALSE => Aux

BOOLEAN phy_QueryRFPathSwitch_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN		is2T
)
{
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PDM_ODM_T		pDM_Odm = &pHalData->odmpriv;
	#endif
	#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;
	#endif
	#endif
	// MAIN: WiFi

	if (ODM_GetMACReg(pDM_Odm, 0x64, 0xFFF) == 0x100)
		return TRUE;
	else
		return FALSE;

}



//return value TRUE => Main; FALSE => Aux
BOOLEAN PHY_QueryRFPathSwitch_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm
#else
	IN	PADAPTER	pAdapter
#endif
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	#if DISABLE_BB_RF
	return TRUE;
	#endif
	#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	if (IS_2T2R(pHalData->VersionID))
		return phy_QueryRFPathSwitch_8192E(pAdapter, TRUE);
	else
	#endif
	{
		// For 88C 1T1R
		#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		return phy_QueryRFPathSwitch_8192E(pAdapter, FALSE);
		#else
		return phy_QueryRFPathSwitch_8192E(pDM_Odm, FALSE);
		#endif
	}
}
#endif

#else	/* #if (RTL8192E_SUPPORT == 1)*/

VOID
PHY_IQCalibrate_8192E(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	IN PDM_ODM_T		pDM_Odm,
#else
	IN	PADAPTER	pAdapter,
#endif
	IN	BOOLEAN	bReCovery
	){}
VOID
PHY_LCCalibrate_8192E(
	IN PDM_ODM_T		pDM_Odm
	){}

VOID
ODM_TxPwrTrackSetPwr92E(
	PDM_ODM_T			pDM_Odm,
	PWRTRACK_METHOD		Method,
	u1Byte				RFPath,
	u1Byte				ChannelMappedIndex
	){}

#endif	/* #if (RTL8192E_SUPPORT == 1)*/
