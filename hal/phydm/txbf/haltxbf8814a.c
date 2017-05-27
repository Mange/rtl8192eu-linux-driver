//============================================================
// Description:
//
// This file is for 8814A TXBF mechanism
//
//============================================================

#include "mp_precomp.h"
#include "../phydm_precomp.h"

#if (BEAMFORMING_SUPPORT == 1)
#if (RTL8814A_SUPPORT == 1)

VOID
HalTxbf8814A_setNDPArate(
	IN PVOID			pDM_VOID,
	IN u1Byte	BW,
	IN u1Byte	Rate
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	
	ODM_Write1Byte(pDM_Odm, REG_NDPA_OPT_CTRL_8814A, BW);
	ODM_Write1Byte(pDM_Odm, REG_NDPA_RATE_8814A, (u1Byte) Rate);

}

#define PHYDM_MEMORY_MAP_BUF_READ	0x8000
#define PHYDM_CTRL_INFO_PAGE			0x660

VOID
phydm_DataRate_8814A(
	IN	PDM_ODM_T			pDM_Odm,
	IN	u1Byte				macId,	
	OUT	pu4Byte				data,
	IN	u1Byte				dataLen
	)
{
	u1Byte	i = 0;
	u2Byte	XReadDataAddr = 0;

	ODM_Write2Byte(pDM_Odm, REG_PKTBUF_DBG_CTRL_8814A, PHYDM_CTRL_INFO_PAGE);
	XReadDataAddr = PHYDM_MEMORY_MAP_BUF_READ + macId*32; /*Ctrl Info: 32Bytes for each macid(n)*/
	
	if ((XReadDataAddr < PHYDM_MEMORY_MAP_BUF_READ) || (XReadDataAddr > 0x8FFF)) {
		ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("XReadDataAddr(0x%x) is not correct!\n", XReadDataAddr));
		return;	
	}
	
	/* Read data */
	for (i = 0; i < dataLen; i++)
		*(data+i) = ODM_Read2Byte(pDM_Odm, XReadDataAddr+i);	
	
}

VOID
HalTxbf8814A_GetTxRate(
	IN PVOID			pDM_VOID
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	PRT_BEAMFORMING_INFO	pBeamInfo = &pDM_Odm->BeamformingInfo;
	PRT_BEAMFORMEE_ENTRY	pEntry;
	u4Byte	TxRptData = 0;
	u1Byte	DataRate = 0xFF;

	pEntry = &(pBeamInfo->BeamformeeEntry[pBeamInfo->BeamformeeCurIdx]);
	
	phydm_DataRate_8814A(pDM_Odm, (u1Byte)pEntry->MacId, &TxRptData, 1);
	DataRate = (u1Byte)TxRptData;
	DataRate &= bMask7bits;   /*Bit7 indicates SGI*/

	pDM_Odm->TxBfDataRate = DataRate;

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("[%s] pDM_Odm->TxBfDataRate = 0x%x\n", __func__, pDM_Odm->TxBfDataRate));
}

VOID
HalTxbf8814A_ResetTxPath(
	IN PVOID			pDM_VOID,
	IN	u1Byte				idx
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
#if DEV_BUS_TYPE == RT_USB_INTERFACE

	PRT_BEAMFORMING_INFO	pBeamformingInfo = &pDM_Odm->BeamformingInfo;
	RT_BEAMFORMEE_ENTRY	BeamformeeEntry;
	u1Byte	Nr_index = 0;

	if (idx < BEAMFORMEE_ENTRY_NUM)
		BeamformeeEntry = pBeamformingInfo->BeamformeeEntry[idx];
	else
		return;

	if ((pDM_Odm->LastUSBHub) != (*pDM_Odm->HubUsbMode)) {
		Nr_index = TxBF_Nr(halTxbf8814A_GetNtx(pDM_Odm), BeamformeeEntry.CompSteeringNumofBFer);

		if (idx == 0) {
			switch (Nr_index) {
			case 0:
			break;

			case 1:			/*Nsts = 2	BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x9366);		/*tx2path, BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x936);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskLWord, 0x9360);
			break;

			case 2:			/*Nsts = 3	BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x93e93ee);	/*tx3path, BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93e);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93e93e0);
			break;

			default:			/*Nr>3, same as Case 3*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x93f93ff);	/*tx4path, ABCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93f);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskDWord, 0x93f93f0);
			break;
			}
		} else	{
			switch (Nr_index) {
			case 0:
				break;

			case 1:			/*Nsts = 2	BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x9366);		/*tx2path, BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x936);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskLWord, 0x9360);
			break;

			case 2:			/*Nsts = 3	BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x93e93ee);	/*tx3path, BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93e);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93e93e0);
			break;

			default:			/*Nr>3, same as Case 3*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x93f93ff);	/*tx4path, ABCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93f);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93f93f0);
			break;
			}
		}

		pDM_Odm->LastUSBHub = *pDM_Odm->HubUsbMode;
	} else
		return;
#endif
}


u1Byte
halTxbf8814A_GetNtx(
	IN PVOID			pDM_VOID
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte			Ntx = 0;

#if DEV_BUS_TYPE == RT_USB_INTERFACE
	if (pDM_Odm->SupportInterface == ODM_ITRF_USB) {
		if (*pDM_Odm->HubUsbMode == 2) {/*USB3.0*/
			if (pDM_Odm->RFType == ODM_4T4R)
				Ntx = 3;
			else if (pDM_Odm->RFType == ODM_3T3R)
				Ntx = 2;
			else
				Ntx = 1;
		} else if (*pDM_Odm->HubUsbMode == 1)	/*USB 2.0 always 2Tx*/
			Ntx = 1;
		else
			Ntx = 1;
	} else
#endif
	{
		if (pDM_Odm->RFType == ODM_4T4R)
			Ntx = 3;
		else if (pDM_Odm->RFType == ODM_3T3R)
			Ntx = 2;
		else
			Ntx = 1;
	}

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("[%s] Ntx = %d\n", __func__, Ntx));
	return Ntx;
}

u1Byte
halTxbf8814A_GetNrx(
	IN PVOID			pDM_VOID
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte			Nrx = 0;

	if (pDM_Odm->RFType == ODM_4T4R)
		Nrx = 3;
	else if (pDM_Odm->RFType == ODM_3T3R)
		Nrx = 2;
	else if (pDM_Odm->RFType == ODM_2T2R)
		Nrx = 1;
	else if (pDM_Odm->RFType == ODM_2T3R)
		Nrx = 2;
	else if (pDM_Odm->RFType == ODM_2T4R)
		Nrx = 3;
	else if (pDM_Odm->RFType == ODM_1T1R)
		Nrx = 0;
	else if (pDM_Odm->RFType == ODM_1T2R)
		Nrx = 1;
	else
		Nrx = 0;

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("[%s] Nrx = %d\n", __func__, Nrx));
	return Nrx;
}

VOID
halTxbf8814A_RfMode(
	IN PVOID			pDM_VOID,
	IN	PRT_BEAMFORMING_INFO	pBeamformingInfo,
	IN	u1Byte					idx
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte				i, Nr_index = 0;
	RT_BEAMFORMEE_ENTRY	BeamformeeEntry;

	if (idx < BEAMFORMEE_ENTRY_NUM)
		BeamformeeEntry = pBeamformingInfo->BeamformeeEntry[idx];
	else
		return;

	Nr_index = TxBF_Nr(halTxbf8814A_GetNtx(pDM_Odm), BeamformeeEntry.CompSteeringNumofBFer);

	if (pDM_Odm->RFType == ODM_1T1R)
		return;

	for (i = ODM_RF_PATH_A; i < MAX_RF_PATH; i++) {
		ODM_SetRFReg(pDM_Odm, i, RF_WeLut_Jaguar, 0x80000, 0x1);
		/*RF Mode table write enable*/
	}

	if (pBeamformingInfo->beamformee_su_cnt > 0) {
		for (i = ODM_RF_PATH_A; i < MAX_RF_PATH; i++) {
			ODM_SetRFReg(pDM_Odm, i, RF_ModeTableAddr, 0xfffff, 0x18000);
			/*Select RX mode*/
			ODM_SetRFReg(pDM_Odm, i, RF_ModeTableData0, 0xfffff, 0xBE77F);
			/*Set Table data*/
			ODM_SetRFReg(pDM_Odm, i, RF_ModeTableData1, 0xfffff, 0x226BF);
			/*Enable TXIQGEN in RX mode*/
		}
		ODM_SetRFReg(pDM_Odm, ODM_RF_PATH_A, RF_ModeTableData1, 0xfffff, 0xE26BF);
		/*Enable TXIQGEN in RX mode*/
	}

	for (i = ODM_RF_PATH_A; i < MAX_RF_PATH; i++) {
		ODM_SetRFReg(pDM_Odm, i, RF_WeLut_Jaguar, 0x80000, 0x0);
		/*RF Mode table write disable*/
	}

	if (pBeamformingInfo->beamformee_su_cnt > 0) {
#if DEV_BUS_TYPE == RT_USB_INTERFACE
		pDM_Odm->LastUSBHub = *pDM_Odm->HubUsbMode;
#endif

		/*for 8814 19ac(idx 1), 19b4(idx 0), different Tx ant setting*/
		ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, BIT28 | BIT29, 0x2);			/*enable BB TxBF ant mapping register*/

		if (idx == 0) {
			switch (Nr_index) {
			case 0:
				break;

			case 1:			/*Nsts = 2	BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x9366);		/*tx2path, BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x936);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskLWord, 0x9360);
			break;

			case 2:			/*Nsts = 3	BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x93e93ee);	/*tx3path, BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93e);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93e93e0);
			break;

			default:			/*Nr>3, same as Case 3*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF0, bMaskByte3LowNibble | bMaskL3Bytes, 0x93f93ff);	/*tx4path, ABCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93f);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskDWord, 0x93f93f0);
			break;
			}
		} else {
			switch (Nr_index) {
			case 0:
			break;

			case 1:			/*Nsts = 2	BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x9366);		/*tx2path, BC*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x936);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskLWord, 0x9360);
			break;

			case 2:			/*Nsts = 3	BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x93e93ee);	/*tx3path, BCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93e);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93e93e0);
			break;

			default:			/*Nr>3, same as Case 3*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TXBF_ANT_SET_BF1, bMaskByte3LowNibble | bMaskL3Bytes, 0x93f93ff);	/*tx4path, ABCD*/
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x93f);
			ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93f93f0);
			break;
			}
		}
	}

	if ((pBeamformingInfo->beamformee_su_cnt == 0) && (pBeamformingInfo->beamformer_su_cnt == 0)) {
		ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_1, bMaskByte3 | bMaskByte2HighNibble, 0x932);	/*set TxPath selection for 8814a BFer bug refine*/
		ODM_SetBBReg(pDM_Odm, REG_BB_TX_PATH_SEL_2, bMaskDWord, 0x93e9360);
	}
}
#if 0
VOID
halTxbf8814A_DownloadNDPA(
	IN	PADAPTER			Adapter,
	IN	u1Byte				Idx
)
{
	u1Byte			u1bTmp = 0, tmpReg422 = 0;
	u1Byte			BcnValidReg = 0, count = 0, DLBcnCount = 0;
	u2Byte			Head_Page = 0x7FE;
	BOOLEAN			bSendBeacon = FALSE;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u2Byte			TxPageBndy = LAST_ENTRY_OF_TX_PKT_BUFFER_8814A; /*default reseved 1 page for the IC type which is undefined.*/
	PRT_BEAMFORMING_INFO	pBeamInfo = GET_BEAMFORM_INFO(Adapter);
	PRT_BEAMFORMEE_ENTRY	pBeamEntry = pBeamInfo->BeamformeeEntry + Idx;

	pHalData->bFwDwRsvdPageInProgress = TRUE;
	Adapter->HalFunc.GetHalDefVarHandler(Adapter, HAL_DEF_TX_PAGE_BOUNDARY, (pu2Byte)&TxPageBndy);

	/*Set REG_CR bit 8. DMA beacon by SW.*/
	u1bTmp = PlatformEFIORead1Byte(Adapter, REG_CR_8814A + 1);
	PlatformEFIOWrite1Byte(Adapter,  REG_CR_8814A + 1, (u1bTmp | BIT0));


	/*Set FWHW_TXQ_CTRL 0x422[6]=0 to tell Hw the packet is not a real beacon frame.*/
	tmpReg422 = PlatformEFIORead1Byte(Adapter, REG_FWHW_TXQ_CTRL_8814A + 2);
	PlatformEFIOWrite1Byte(Adapter, REG_FWHW_TXQ_CTRL_8814A + 2,  tmpReg422 & (~BIT6));

	if (tmpReg422 & BIT6) {
		RT_TRACE(COMP_INIT, DBG_LOUD, ("SetBeamformDownloadNDPA_8814A(): There is an Adapter is sending beacon.\n"));
		bSendBeacon = TRUE;
	}

	/*0x204[11:0]	Beacon Head for TXDMA*/
	PlatformEFIOWrite2Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A, Head_Page);

	do {
		/*Clear beacon valid check bit.*/
		BcnValidReg = PlatformEFIORead1Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A + 1);
		PlatformEFIOWrite1Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A + 1, (BcnValidReg | BIT7));

		/*download NDPA rsvd page.*/
		if (pBeamEntry->BeamformEntryCap & BEAMFORMER_CAP_VHT_SU)
			Beamforming_SendVHTNDPAPacket(pDM_Odm, pBeamEntry->MacAddr, pBeamEntry->AID, pBeamEntry->SoundBW, BEACON_QUEUE);
		else
			Beamforming_SendHTNDPAPacket(pDM_Odm, pBeamEntry->MacAddr, pBeamEntry->SoundBW, BEACON_QUEUE);

		/*check rsvd page download OK.*/
		BcnValidReg = PlatformEFIORead1Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A + 1);
		count = 0;
		while (!(BcnValidReg & BIT7) && count < 20) {
			count++;
			delay_us(10);
			BcnValidReg = PlatformEFIORead1Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A + 2);
		}
		DLBcnCount++;
	} while (!(BcnValidReg & BIT7) && DLBcnCount < 5);

	if (!(BcnValidReg & BIT0))
		RT_DISP(FBEAM, FBEAM_ERROR, ("%s Download RSVD page failed!\n", __func__));

	/*0x204[11:0]	Beacon Head for TXDMA*/
	PlatformEFIOWrite2Byte(Adapter, REG_FIFOPAGE_CTRL_2_8814A, TxPageBndy);

	/*To make sure that if there exists an adapter which would like to send beacon.*/
	/*If exists, the origianl value of 0x422[6] will be 1, we should check this to*/
	/*prevent from setting 0x422[6] to 0 after download reserved page, or it will cause */
	/*the beacon cannot be sent by HW.*/
	/*2010.06.23. Added by tynli.*/
	if (bSendBeacon)
		PlatformEFIOWrite1Byte(Adapter, REG_FWHW_TXQ_CTRL_8814A + 2, tmpReg422);

	/*Do not enable HW DMA BCN or it will cause Pcie interface hang by timing issue. 2011.11.24. by tynli.*/
	/*Clear CR[8] or beacon packet will not be send to TxBuf anymore.*/
	u1bTmp = PlatformEFIORead1Byte(Adapter, REG_CR_8814A + 1);
	PlatformEFIOWrite1Byte(Adapter, REG_CR_8814A + 1, (u1bTmp & (~BIT0)));

	pBeamEntry->BeamformEntryState = BEAMFORMING_ENTRY_STATE_PROGRESSED;

	pHalData->bFwDwRsvdPageInProgress = FALSE;
}

VOID
halTxbf8814A_FwTxBFCmd(
	IN	PADAPTER	Adapter
)
{
	u1Byte	Idx, Period = 0;
	u1Byte	PageNum0 = 0xFF, PageNum1 = 0xFF;
	u1Byte	u1TxBFParm[3] = {0};

	PMGNT_INFO				pMgntInfo = &(Adapter->MgntInfo);
	PRT_BEAMFORMING_INFO	pBeamInfo = GET_BEAMFORM_INFO(Adapter);

	for (Idx = 0; Idx < BEAMFORMEE_ENTRY_NUM; Idx++) {
		if (pBeamInfo->BeamformeeEntry[Idx].bUsed && pBeamInfo->BeamformeeEntry[Idx].BeamformEntryState == BEAMFORMING_ENTRY_STATE_PROGRESSED) {
			if (pBeamInfo->BeamformeeEntry[Idx].bSound) {
				PageNum0 = 0xFE;
				PageNum1 = 0x07;
				Period = (u1Byte)(pBeamInfo->BeamformeeEntry[Idx].SoundPeriod);
			} else if (PageNum0 == 0xFF) {
				PageNum0 = 0xFF; /*stop sounding*/
				PageNum1 = 0x0F;
			}
		}
	}

	u1TxBFParm[0] = PageNum0;
	u1TxBFParm[1] = PageNum1;
	u1TxBFParm[2] = Period;
	FillH2CCmd(Adapter, PHYDM_H2C_TXBF, 3, u1TxBFParm);

	RT_DISP(FBEAM, FBEAM_FUN, ("@%s End, PageNum0 = 0x%x, PageNum1 = 0x%x Period = %d", __func__, PageNum0, PageNum1, Period));
}
#endif
VOID
HalTxbf8814A_Enter(
	IN PVOID			pDM_VOID,
	IN u1Byte				BFerBFeeIdx
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u1Byte					i = 0;
	u1Byte					BFerIdx = (BFerBFeeIdx & 0xF0) >> 4;
	u1Byte					BFeeIdx = (BFerBFeeIdx & 0xF);
	PRT_BEAMFORMING_INFO	pBeamformingInfo = &pDM_Odm->BeamformingInfo;
	RT_BEAMFORMEE_ENTRY	BeamformeeEntry;
	RT_BEAMFORMER_ENTRY	BeamformerEntry;
	u2Byte					STAid = 0, CSI_Param = 0;
	u1Byte					Nc_index = 0, Nr_index = 0, grouping = 0, codebookinfo = 0, coefficientsize = 0;

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("[%s] BFerIdx=%d, BFeeIdx=%d\n", __func__, BFerIdx, BFeeIdx));
	ODM_SetMACReg(pDM_Odm, REG_SND_PTCL_CTRL_8814A, bMaskByte1 | bMaskByte2, 0x0202);

	if ((pBeamformingInfo->beamformer_su_cnt > 0) && (BFerIdx < BEAMFORMER_ENTRY_NUM)) {
		BeamformerEntry = pBeamformingInfo->BeamformerEntry[BFerIdx];
		/*Sounding protocol control*/
		ODM_Write1Byte(pDM_Odm, REG_SND_PTCL_CTRL_8814A, 0xDB);

		/*MAC address/Partial AID of Beamformer*/
		if (BFerIdx == 0) {
			for (i = 0; i < 6 ; i++)
				ODM_Write1Byte(pDM_Odm, (REG_ASSOCIATED_BFMER0_INFO_8814A + i), BeamformerEntry.MacAddr[i]);
		} else {
			for (i = 0; i < 6 ; i++)
				ODM_Write1Byte(pDM_Odm, (REG_ASSOCIATED_BFMER1_INFO_8814A + i), BeamformerEntry.MacAddr[i]);
		}

		/*CSI report parameters of Beamformer*/
		Nc_index = halTxbf8814A_GetNrx(pDM_Odm);	/*for 8814A Nrx = 3(4 Ant), min=0(1 Ant)*/
		Nr_index = BeamformerEntry.NumofSoundingDim;	/*0x718[7] = 1 use Nsts, 0x718[7] = 0 use reg setting. as Bfee, we use Nsts, so Nr_index don't care*/

		grouping = 0;

		/*for ac = 1, for n = 3*/
		if (BeamformerEntry.BeamformEntryCap & BEAMFORMEE_CAP_VHT_SU)
			codebookinfo = 1;
		else if (BeamformerEntry.BeamformEntryCap & BEAMFORMEE_CAP_HT_EXPLICIT)
			codebookinfo = 3;

		coefficientsize = 3;

		CSI_Param = (u2Byte)((coefficientsize << 10) | (codebookinfo << 8) | (grouping << 6) | (Nr_index << 3) | (Nc_index));

		if (BFerIdx == 0)
			ODM_Write2Byte(pDM_Odm, REG_CSI_RPT_PARAM_BW20_8814A, CSI_Param);
		else
			ODM_Write2Byte(pDM_Odm, REG_CSI_RPT_PARAM_BW20_8814A + 2, CSI_Param);
		/*ndp_rx_standby_timer, 8814 need > 0x56, suggest from Dvaid*/
		ODM_Write1Byte(pDM_Odm, REG_SND_PTCL_CTRL_8814A + 3, 0x40);

	}

	if ((pBeamformingInfo->beamformee_su_cnt > 0) && (BFeeIdx < BEAMFORMEE_ENTRY_NUM)) {
		BeamformeeEntry = pBeamformingInfo->BeamformeeEntry[BFeeIdx];

		halTxbf8814A_RfMode(pDM_Odm, pBeamformingInfo, BFeeIdx);

		if (phydm_actingDetermine(pDM_Odm, PhyDM_ACTING_AS_IBSS))
			STAid = BeamformeeEntry.MacId;
		else
			STAid = BeamformeeEntry.P_AID;

		/*P_AID of Beamformee & enable NDPA transmission & enable NDPA interrupt*/
		if (BFeeIdx == 0) {
			ODM_Write2Byte(pDM_Odm, REG_TXBF_CTRL_8814A, STAid);
			ODM_Write1Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 3, ODM_Read1Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 3) | BIT4 | BIT6 | BIT7);
		} else
			ODM_Write2Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 2, STAid | BIT14 | BIT15 | BIT12);

		/*CSI report parameters of Beamformee*/
		if (BFeeIdx == 0) {
			/*Get BIT24 & BIT25*/
			u1Byte	tmp = ODM_Read1Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A + 3) & 0x3;

			ODM_Write1Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A + 3, tmp | 0x60);
			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A, STAid | BIT9);
		} else
			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2, STAid | 0xE200);	/*Set BIT25*/

		phydm_Beamforming_Notify(pDM_Odm);
	}

}


VOID
HalTxbf8814A_Leave(
	IN PVOID			pDM_VOID,
	IN u1Byte				Idx
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	PRT_BEAMFORMING_INFO	pBeamformingInfo = &pDM_Odm->BeamformingInfo;
	RT_BEAMFORMER_ENTRY	BeamformerEntry;
	RT_BEAMFORMEE_ENTRY	BeamformeeEntry;

	if (Idx < BEAMFORMER_ENTRY_NUM) {
		BeamformerEntry = pBeamformingInfo->BeamformerEntry[Idx];
		BeamformeeEntry = pBeamformingInfo->BeamformeeEntry[Idx];
	} else
		return;

	/*Clear P_AID of Beamformee*/
	/*Clear MAC address of Beamformer*/
	/*Clear Associated Bfmee Sel*/

	if (BeamformerEntry.BeamformEntryCap == BEAMFORMING_CAP_NONE) {
		ODM_Write1Byte(pDM_Odm, REG_SND_PTCL_CTRL_8814A, 0xD8);
		if (Idx == 0) {
			ODM_Write4Byte(pDM_Odm, REG_ASSOCIATED_BFMER0_INFO_8814A, 0);
			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMER0_INFO_8814A + 4, 0);
			ODM_Write2Byte(pDM_Odm, REG_CSI_RPT_PARAM_BW20_8814A, 0);
		} else {
			ODM_Write4Byte(pDM_Odm, REG_ASSOCIATED_BFMER1_INFO_8814A, 0);
			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMER1_INFO_8814A + 4, 0);
			ODM_Write2Byte(pDM_Odm, REG_CSI_RPT_PARAM_BW20_8814A + 2, 0);
		}
	}

	if (BeamformeeEntry.BeamformEntryCap == BEAMFORMING_CAP_NONE) {
		halTxbf8814A_RfMode(pDM_Odm, pBeamformingInfo, Idx);
		if (Idx == 0) {
			ODM_Write2Byte(pDM_Odm, REG_TXBF_CTRL_8814A, 0x0);
			ODM_Write1Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 3, ODM_Read1Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 3) | BIT4 | BIT6 | BIT7);
			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A, 0);
		} else {
			ODM_Write2Byte(pDM_Odm, REG_TXBF_CTRL_8814A + 2, 0x0 | BIT14 | BIT15 | BIT12);

			ODM_Write2Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2, ODM_Read2Byte(pDM_Odm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2) & 0x60);
		}
	}
}

VOID
HalTxbf8814A_Status(
	IN PVOID			pDM_VOID,
	IN u1Byte				Idx
)
{
	PDM_ODM_T	pDM_Odm = (PDM_ODM_T)pDM_VOID;
	u2Byte					BeamCtrlVal, tmpVal;
	u4Byte					BeamCtrlReg;
	PRT_BEAMFORMING_INFO	pBeamformingInfo = &pDM_Odm->BeamformingInfo;
	RT_BEAMFORMEE_ENTRY	BeamformEntry;

	if (Idx < BEAMFORMEE_ENTRY_NUM)
		BeamformEntry = pBeamformingInfo->BeamformeeEntry[Idx];
	else
		return;

	if (phydm_actingDetermine(pDM_Odm, PhyDM_ACTING_AS_IBSS))
		BeamCtrlVal = BeamformEntry.MacId;
	else
		BeamCtrlVal = BeamformEntry.P_AID;

	ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("@%s, BeamformEntry.BeamformEntryState = %d", __func__, BeamformEntry.BeamformEntryState));

	if (Idx == 0)
		BeamCtrlReg = REG_TXBF_CTRL_8814A;
	else {
		BeamCtrlReg = REG_TXBF_CTRL_8814A + 2;
		BeamCtrlVal |= BIT12 | BIT14 | BIT15;
	}

	if (BeamformEntry.BeamformEntryState == BEAMFORMING_ENTRY_STATE_PROGRESSED) {
		if (BeamformEntry.SoundBW == CHANNEL_WIDTH_20)
			BeamCtrlVal |= BIT9;
		else if (BeamformEntry.SoundBW == CHANNEL_WIDTH_40)
			BeamCtrlVal |= (BIT9 | BIT10);
		else if (BeamformEntry.SoundBW == CHANNEL_WIDTH_80)
			BeamCtrlVal |= (BIT9 | BIT10 | BIT11);
	} else {
		ODM_RT_TRACE(pDM_Odm, PHYDM_COMP_TXBF, ODM_DBG_LOUD, ("@%s, Don't apply Vmatrix",  __func__));
		BeamCtrlVal &= ~(BIT9 | BIT10 | BIT11);
	}

	ODM_Write2Byte(pDM_Odm, BeamCtrlReg, BeamCtrlVal);
	/*disable NDP packet use beamforming */
	tmpVal = ODM_Read2Byte(pDM_Odm, REG_TXBF_CTRL_8814A);
	ODM_Write2Byte(pDM_Odm, REG_TXBF_CTRL_8814A, tmpVal | BIT15);

}





VOID
HalTxbf8814A_FwTxBF(
	IN PVOID			pDM_VOID,
	IN	u1Byte				Idx
)
{
#if 0
	PRT_BEAMFORMING_INFO 	pBeamInfo = GET_BEAMFORM_INFO(Adapter);
	PRT_BEAMFORMEE_ENTRY	pBeamEntry = pBeamInfo->BeamformeeEntry + Idx;

	if (pBeamEntry->BeamformEntryState == BEAMFORMING_ENTRY_STATE_PROGRESSING)
		halTxbf8814A_DownloadNDPA(Adapter, Idx);

	halTxbf8814A_FwTxBFCmd(Adapter);
#endif
}

#endif	/* (RTL8814A_SUPPORT == 1)*/

#endif

