//============================================================
// Description:
//
// This file is for RTL8723A Co-exist mechanism
//
// History
// 2012/08/22 Cosa first check in.
// 2012/11/14 Cosa Revise for 8723A 2Ant out sourcing.
//
//============================================================

//============================================================
// include files
//============================================================
#include "Mp_Precomp.h"

#if WPP_SOFTWARE_TRACE
#include "HalBtc8723a2Ant.tmh"
#endif

#if(BT_30_SUPPORT == 1)
//============================================================
// Global variables, these are static variables
//============================================================
static COEX_DM_8723A_2ANT	GLCoexDm8723a2Ant;
static PCOEX_DM_8723A_2ANT 	pCoexDm=&GLCoexDm8723a2Ant;
static COEX_STA_8723A_2ANT	GLCoexSta8723a2Ant;
static PCOEX_STA_8723A_2ANT	pCoexSta=&GLCoexSta8723a2Ant;

const char *const GLBtInfoSrc8723a2Ant[]={
	"BT Info[wifi fw]",
	"BT Info[bt rsp]",
	"BT Info[bt auto report]",
};

//============================================================
// local function proto type if needed
//============================================================
//============================================================
// local function start with halbtc8723a2ant_
//============================================================
BOOLEAN
halbtc8723a2ant_IsWifiIdle(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	BOOLEAN		bWifiConnected=FALSE, bScan=FALSE, bLink=FALSE, bRoam=FALSE;

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_CONNECTED, &bWifiConnected);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_SCAN, &bScan);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_LINK, &bLink);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_ROAM, &bRoam);

	if(bWifiConnected)
		return FALSE;
	if(bScan)
		return FALSE;
	if(bLink)
		return FALSE;
	if(bRoam)
		return FALSE;

	return TRUE;
}

BOOLEAN
halbtc8723a2ant_IsWifiConnectedIdle(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	BOOLEAN		bWifiConnected=FALSE, bScan=FALSE, bLink=FALSE, bRoam=FALSE, bWifiBusy=FALSE;

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_CONNECTED, &bWifiConnected);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_SCAN, &bScan);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_LINK, &bLink);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_ROAM, &bRoam);
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_BUSY, &bWifiBusy);

	if(bScan)
		return FALSE;
	if(bLink)
		return FALSE;
	if(bRoam)
		return FALSE;
	if(bWifiConnected && !bWifiBusy)
		return TRUE;
	else 
		return FALSE;
}

u1Byte
halbtc8723a2ant_BtRssiState(
	u1Byte			levelNum,
	u1Byte			rssiThresh,
	u1Byte			rssiThresh1
	)
{
	s4Byte			btRssi=0;
	u1Byte			btRssiState=pCoexSta->preBtRssiState;

	btRssi = pCoexSta->btRssi;

	if(levelNum == 2)
	{			
		if( (pCoexSta->preBtRssiState == BTC_RSSI_STATE_LOW) ||
			(pCoexSta->preBtRssiState == BTC_RSSI_STATE_STAY_LOW))
		{
			if(btRssi >= (rssiThresh+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				btRssiState = BTC_RSSI_STATE_HIGH;
			}
			else
			{
				btRssiState = BTC_RSSI_STATE_STAY_LOW;
			}
		}
		else
		{
			if(btRssi < rssiThresh)
			{
				btRssiState = BTC_RSSI_STATE_LOW;
			}
			else
			{
				btRssiState = BTC_RSSI_STATE_STAY_HIGH;
			}
		}
	}
	else if(levelNum == 3)
	{
		if(rssiThresh > rssiThresh1)
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BT Rssi thresh error!!\n"));
			return pCoexSta->preBtRssiState;
		}
		
		if( (pCoexSta->preBtRssiState == BTC_RSSI_STATE_LOW) ||
			(pCoexSta->preBtRssiState == BTC_RSSI_STATE_STAY_LOW))
		{
			if(btRssi >= (rssiThresh+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				btRssiState = BTC_RSSI_STATE_MEDIUM;
			}
			else
			{
				btRssiState = BTC_RSSI_STATE_STAY_LOW;
			}
		}
		else if( (pCoexSta->preBtRssiState == BTC_RSSI_STATE_MEDIUM) ||
			(pCoexSta->preBtRssiState == BTC_RSSI_STATE_STAY_MEDIUM))
		{
			if(btRssi >= (rssiThresh1+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				btRssiState = BTC_RSSI_STATE_HIGH;
			}
			else if(btRssi < rssiThresh)
			{
				btRssiState = BTC_RSSI_STATE_LOW;
			}
			else
			{
				btRssiState = BTC_RSSI_STATE_STAY_MEDIUM;
			}
		}
		else
		{
			if(btRssi < rssiThresh1)
			{
				btRssiState = BTC_RSSI_STATE_MEDIUM;
			}
			else
			{
				btRssiState = BTC_RSSI_STATE_STAY_HIGH;
			}
		}
	}
		
	pCoexSta->preBtRssiState = btRssiState;

	return btRssiState;
}

u1Byte
halbtc8723a2ant_WifiRssiState(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			index,
	IN	u1Byte			levelNum,
	IN	u1Byte			rssiThresh,
	IN	u1Byte			rssiThresh1
	)
{
	s4Byte			wifiRssi=0;
	u1Byte			wifiRssiState=pCoexSta->preWifiRssiState[index];

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_S4_WIFI_RSSI, &wifiRssi);
	
	if(levelNum == 2)
	{
		if( (pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_LOW) ||
			(pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_STAY_LOW))
		{
			if(wifiRssi >= (rssiThresh+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				wifiRssiState = BTC_RSSI_STATE_HIGH;
			}
			else
			{
				wifiRssiState = BTC_RSSI_STATE_STAY_LOW;
			}
		}
		else
		{
			if(wifiRssi < rssiThresh)
			{
				wifiRssiState = BTC_RSSI_STATE_LOW;
			}
			else
			{
				wifiRssiState = BTC_RSSI_STATE_STAY_HIGH;
			}
		}
	}
	else if(levelNum == 3)
	{
		if(rssiThresh > rssiThresh1)
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], wifi RSSI thresh error!!\n"));
			return pCoexSta->preWifiRssiState[index];
		}
		
		if( (pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_LOW) ||
			(pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_STAY_LOW))
		{
			if(wifiRssi >= (rssiThresh+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				wifiRssiState = BTC_RSSI_STATE_MEDIUM;
			}
			else
			{
				wifiRssiState = BTC_RSSI_STATE_STAY_LOW;
			}
		}
		else if( (pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_MEDIUM) ||
			(pCoexSta->preWifiRssiState[index] == BTC_RSSI_STATE_STAY_MEDIUM))
		{
			if(wifiRssi >= (rssiThresh1+BTC_RSSI_COEX_THRESH_TOL_8723A_2ANT))
			{
				wifiRssiState = BTC_RSSI_STATE_HIGH;
			}
			else if(wifiRssi < rssiThresh)
			{
				wifiRssiState = BTC_RSSI_STATE_LOW;
			}
			else
			{
				wifiRssiState = BTC_RSSI_STATE_STAY_MEDIUM;
			}
		}
		else
		{
			if(wifiRssi < rssiThresh1)
			{
				wifiRssiState = BTC_RSSI_STATE_MEDIUM;
			}
			else
			{
				wifiRssiState = BTC_RSSI_STATE_STAY_HIGH;
			}
		}
	}
		
	pCoexSta->preWifiRssiState[index] = wifiRssiState;

	return wifiRssiState;
}

VOID
halbtc8723a2ant_IndicateWifiChnlBwInfo(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	)
{
	u1Byte			H2C_Parameter[3] ={0};
	u4Byte			wifiBw;
	u1Byte			wifiCentralChnl;
	
	// only 2.4G we need to inform bt the chnl mask
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U1_WIFI_CENTRAL_CHNL, &wifiCentralChnl);
	if( (BTC_MEDIA_CONNECT == type) &&
		(wifiCentralChnl <= 14) )
	{
		H2C_Parameter[0] = 0x1;
		H2C_Parameter[1] = wifiCentralChnl;
		pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
		if(BTC_WIFI_BW_HT40 == wifiBw)
			H2C_Parameter[2] = 0x30;
		else
			H2C_Parameter[2] = 0x20;
	}
		
	pCoexDm->wifiChnlInfo[0] = H2C_Parameter[0];
	pCoexDm->wifiChnlInfo[1] = H2C_Parameter[1];
	pCoexDm->wifiChnlInfo[2] = H2C_Parameter[2];
	
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], FW write 0x19=0x%x\n", 
		H2C_Parameter[0]<<16|H2C_Parameter[1]<<8|H2C_Parameter[2]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x19, 3, H2C_Parameter);
}

VOID
halbtc8723a2ant_QueryBtInfo(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte			H2C_Parameter[1] ={0};

	pCoexSta->bC2hBtInfoReqSent = TRUE;

	H2C_Parameter[0] |= BIT0;	// trigger

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Query Bt Info, FW write 0x38=0x%x\n", 
		H2C_Parameter[0]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x38, 1, H2C_Parameter);
}
u1Byte
halbtc8723a2ant_ActionAlgorithm(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	PBTC_STACK_INFO		pStackInfo=&pBtCoexist->stackInfo;
	BOOLEAN				bBtHsOn=FALSE, bBtBusy=FALSE, bLimitedDig=FALSE;
	u1Byte				algorithm=BT_8723A_2ANT_COEX_ALGO_UNDEFINED;
	u1Byte				numOfDiffProfile=0;

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_HS_OPERATION, &bBtHsOn);

	//======================
	// here we get BT status first
	//======================
	pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_IDLE;
	
	if((pStackInfo->bScoExist) ||(bBtHsOn) ||(pStackInfo->bHidExist))
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO or HID or HS exists, set BT non-idle !!!\n"));
		pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_NON_IDLE;
	}
	else
	{
		// A2dp profile
		if( (pBtCoexist->stackInfo.numOfLink == 1) &&
			(pStackInfo->bA2dpExist) )
		{		
			if( (pCoexSta->lowPriorityTx+	pCoexSta->lowPriorityRx) < 100)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], A2DP, low priority tx+rx < 100, set BT connected-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE;
			}
			else
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], A2DP, low priority tx+rx >= 100, set BT non-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_NON_IDLE;
			}
		}
		// Pan profile
		if( (pBtCoexist->stackInfo.numOfLink == 1) &&
			(pStackInfo->bPanExist) )
		{		
			if((pCoexSta->lowPriorityTx+	pCoexSta->lowPriorityRx) < 600)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN, low priority tx+rx < 600, set BT connected-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE;
			}
			else
			{
				if(pCoexSta->lowPriorityTx)
				{
					if((pCoexSta->lowPriorityRx /pCoexSta->lowPriorityTx)>9 )
					{
						RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN, low priority rx/tx > 9, set BT connected-idle!!!\n"));
						pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE;
					}
				}
			}
			if(BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE != pCoexDm->btStatus)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN, set BT non-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_NON_IDLE;
			}
		}
		// Pan+A2dp profile
		if( (pBtCoexist->stackInfo.numOfLink == 2) &&
			(pStackInfo->bA2dpExist) &&
			(pStackInfo->bPanExist) )
		{
			if((pCoexSta->lowPriorityTx+	pCoexSta->lowPriorityRx) < 600)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN+A2DP, low priority tx+rx < 600, set BT connected-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE;
			}
			else
			{
				if(pCoexSta->lowPriorityTx)
				{
					if((pCoexSta->lowPriorityRx /pCoexSta->lowPriorityTx)>9 )
					{
						RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN+A2DP, low priority rx/tx > 9, set BT connected-idle!!!\n"));
						pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE;
					}
				}
			}
			if(BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE != pCoexDm->btStatus)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN+A2DP, set BT non-idle!!!\n"));
				pCoexDm->btStatus = BT_8723A_2ANT_BT_STATUS_NON_IDLE;
			}
		}
	}
	if(BT_8723A_2ANT_BT_STATUS_IDLE != pCoexDm->btStatus)
	{
		bBtBusy = TRUE;
		bLimitedDig = TRUE;
	}
	else
	{
		bBtBusy = FALSE;
		bLimitedDig = FALSE;
	}
	pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_BL_BT_TRAFFIC_BUSY, &bBtBusy);
	pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_BL_BT_LIMITED_DIG, &bLimitedDig);
	//======================

	if(!pStackInfo->bBtLinkExist)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], No profile exists!!!\n"));
		return algorithm;
	}

	if(pStackInfo->bScoExist)
		numOfDiffProfile++;
	if(pStackInfo->bHidExist)
		numOfDiffProfile++;
	if(pStackInfo->bPanExist)
		numOfDiffProfile++;
	if(pStackInfo->bA2dpExist)
		numOfDiffProfile++;
	
	if(numOfDiffProfile == 1)
	{
		if(pStackInfo->bScoExist)
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO only\n"));
			algorithm = BT_8723A_2ANT_COEX_ALGO_SCO;
		}
		else
		{
			if(pStackInfo->bHidExist)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID only\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_HID;
			}
			else if(pStackInfo->bA2dpExist)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], A2DP only\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_A2DP;
			}
			else if(pStackInfo->bPanExist)
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN(HS) only\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANHS;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PAN(EDR) only\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR;
				}
			}
		}
	}
	else if(numOfDiffProfile == 2)
	{
		if(pStackInfo->bScoExist)
		{
			if(pStackInfo->bHidExist)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + HID\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_HID;
			}
			else if(pStackInfo->bA2dpExist)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + A2DP ==> SCO\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_SCO;
			}
			else if(pStackInfo->bPanExist)
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_SCO;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + PAN(EDR)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_HID;
				}
			}
		}
		else
		{
			if( pStackInfo->bHidExist &&
				pStackInfo->bA2dpExist )
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID + A2DP\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_HID_A2DP;
			}
			else if( pStackInfo->bHidExist &&
				pStackInfo->bPanExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_HID_A2DP;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID + PAN(EDR)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_HID;
				}
			}
			else if( pStackInfo->bPanExist &&
				pStackInfo->bA2dpExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], A2DP + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_A2DP;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], A2DP + PAN(EDR)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_A2DP;
				}
			}
		}
	}
	else if(numOfDiffProfile == 3)
	{
		if(pStackInfo->bScoExist)
		{
			if( pStackInfo->bHidExist &&
				pStackInfo->bA2dpExist )
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + HID + A2DP ==> HID\n"));
				algorithm = BT_8723A_2ANT_COEX_ALGO_HID;
			}
			else if( pStackInfo->bHidExist &&
				pStackInfo->bPanExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + HID + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_HID_A2DP;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + HID + PAN(EDR)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_HID;
				}
			}
			else if( pStackInfo->bPanExist &&
				pStackInfo->bA2dpExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + A2DP + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_SCO;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + A2DP + PAN(EDR) ==> HID\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_HID;
				}
			}
		}
		else
		{
			if( pStackInfo->bHidExist &&
				pStackInfo->bPanExist &&
				pStackInfo->bA2dpExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID + A2DP + PAN(HS)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_HID_A2DP;
				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], HID + A2DP + PAN(EDR)\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_HID_A2DP_PANEDR;
				}
			}
		}
	}
	else if(numOfDiffProfile >= 3)
	{
		if(pStackInfo->bScoExist)
		{
			if( pStackInfo->bHidExist &&
				pStackInfo->bPanExist &&
				pStackInfo->bA2dpExist )
			{
				if(bBtHsOn)
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Error!!! SCO + HID + A2DP + PAN(HS)\n"));

				}
				else
				{
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCO + HID + A2DP + PAN(EDR)==>PAN(EDR)+HID\n"));
					algorithm = BT_8723A_2ANT_COEX_ALGO_PANEDR_HID;
				}
			}
		}
	}

	return algorithm;
}

BOOLEAN
halbtc8723a2ant_NeedToDecBtPwr(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	BOOLEAN		bRet=FALSE;
	BOOLEAN		bBtHsOn=FALSE, bWifiConnected=FALSE;
	s4Byte		btHsRssi=0;
	u1Byte		btRssiState=BTC_RSSI_STATE_HIGH;

	btRssiState = halbtc8723a2ant_BtRssiState(2, 42, 0);

	if(!pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_HS_OPERATION, &bBtHsOn))
		return FALSE;
	if(!pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_CONNECTED, &bWifiConnected))
		return FALSE;
	if(!pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_S4_HS_RSSI, &btHsRssi))
		return FALSE;
	if(BTC_RSSI_LOW(btRssiState))
		return FALSE;

	if(bWifiConnected)
	{
		if(bBtHsOn)
		{
			if(btHsRssi > 37)
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Need to decrease bt power for HS mode!!\n"));
				bRet = TRUE;
			}
		}
		else
		{
			RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Need to decrease bt power for Wifi is connected!!\n"));
			bRet = TRUE;
		}
	}
	
	return bRet;
}

VOID
halbtc8723a2ant_SetFwDacSwingLevel(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			dacSwingLvl
	)
{
	u1Byte			H2C_Parameter[1] ={0};

	// There are several type of dacswing
	// 0x18/ 0x10/ 0xc/ 0x8/ 0x4/ 0x6
	H2C_Parameter[0] = dacSwingLvl;

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Set Dac Swing Level=0x%x\n", dacSwingLvl));
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], FW write 0x29=0x%x\n", H2C_Parameter[0]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x29, 1, H2C_Parameter);
}

VOID
halbtc8723a2ant_SetFwDecBtPwr(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bDecBtPwr
	)
{
	u1Byte			H2C_Parameter[1] ={0};
	
	H2C_Parameter[0] = 0;

	if(bDecBtPwr)
	{
		H2C_Parameter[0] |= BIT1;
	}

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], decrease Bt Power : %s, FW write 0x21=0x%x\n", 
		(bDecBtPwr? "Yes!!":"No!!"), H2C_Parameter[0]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x21, 1, H2C_Parameter);	
}

VOID
halbtc8723a2ant_DecBtPwr(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bDecBtPwr
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s Dec BT power = %s\n",  
		(bForceExec? "force to":""), ((bDecBtPwr)? "ON":"OFF")));
	pCoexDm->bCurDecBtPwr = bDecBtPwr;

	if(!bForceExec)
	{
		if(pCoexDm->bPreDecBtPwr == pCoexDm->bCurDecBtPwr) 
			return;
	}
	halbtc8723a2ant_SetFwDecBtPwr(pBtCoexist, pCoexDm->bCurDecBtPwr);

	pCoexDm->bPreDecBtPwr = pCoexDm->bCurDecBtPwr;
}

VOID
halbtc8723a2ant_FwDacSwingLvl(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	u1Byte			fwDacSwingLvl
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s set FW Dac Swing level = %d\n",  
		(bForceExec? "force to":""), fwDacSwingLvl));
	pCoexDm->curFwDacSwingLvl = fwDacSwingLvl;

	if(!bForceExec)
	{
		if(pCoexDm->preFwDacSwingLvl == pCoexDm->curFwDacSwingLvl) 
			return;
	}

	halbtc8723a2ant_SetFwDacSwingLevel(pBtCoexist, pCoexDm->curFwDacSwingLvl);

	pCoexDm->preFwDacSwingLvl = pCoexDm->curFwDacSwingLvl;
}

VOID
halbtc8723a2ant_SetSwRfRxLpfCorner(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bRxRfShrinkOn
	)
{
	if(bRxRfShrinkOn)
	{
		//Shrink RF Rx LPF corner
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Shrink RF Rx LPF corner!!\n"));
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x1e, 0xfffff, 0xf0ff7);
	}
	else
	{
		//Resume RF Rx LPF corner
		// After initialized, we can use pCoexDm->btRf0x1eBackup
		if(pBtCoexist->bInitilized)
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Resume RF Rx LPF corner!!\n"));
			pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x1e, 0xfffff, pCoexDm->btRf0x1eBackup);
		}
	}
}

VOID
halbtc8723a2ant_RfShrink(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bRxRfShrinkOn
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn Rx RF Shrink = %s\n",  
		(bForceExec? "force to":""), ((bRxRfShrinkOn)? "ON":"OFF")));
	pCoexDm->bCurRfRxLpfShrink = bRxRfShrinkOn;

	if(!bForceExec)
	{
		if(pCoexDm->bPreRfRxLpfShrink == pCoexDm->bCurRfRxLpfShrink) 
			return;
	}
	halbtc8723a2ant_SetSwRfRxLpfCorner(pBtCoexist, pCoexDm->bCurRfRxLpfShrink);

	pCoexDm->bPreRfRxLpfShrink = pCoexDm->bCurRfRxLpfShrink;
}

VOID
halbtc8723a2ant_SetSwPenaltyTxRateAdaptive(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bLowPenaltyRa
	)
{
	u1Byte	tmpU1;

	tmpU1 = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x4fd);
	tmpU1 |= BIT0;
	if(bLowPenaltyRa)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Tx rate adaptive, set low penalty!!\n"));
		tmpU1 &= ~BIT2;
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Tx rate adaptive, set normal!!\n"));
		tmpU1 |= BIT2;
	}

	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x4fd, tmpU1);
}

VOID
halbtc8723a2ant_LowPenaltyRa(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bLowPenaltyRa
	)
{
	return;
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn LowPenaltyRA = %s\n",  
		(bForceExec? "force to":""), ((bLowPenaltyRa)? "ON":"OFF")));
	pCoexDm->bCurLowPenaltyRa = bLowPenaltyRa;

	if(!bForceExec)
	{
		if(pCoexDm->bPreLowPenaltyRa == pCoexDm->bCurLowPenaltyRa) 
			return;
	}
	halbtc8723a2ant_SetSwPenaltyTxRateAdaptive(pBtCoexist, pCoexDm->bCurLowPenaltyRa);

	pCoexDm->bPreLowPenaltyRa = pCoexDm->bCurLowPenaltyRa;
}

VOID
halbtc8723a2ant_SetSwFullTimeDacSwing(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bSwDacSwingOn,
	IN	u4Byte			swDacSwingLvl
	)
{
	if(bSwDacSwingOn)
	{
		pBtCoexist->fBtcSetBbReg(pBtCoexist, 0x880, 0xff000000, swDacSwingLvl);
	}
	else
	{
		pBtCoexist->fBtcSetBbReg(pBtCoexist, 0x880, 0xff000000, 0xc0);
	}
}


VOID
halbtc8723a2ant_DacSwing(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bDacSwingOn,
	IN	u4Byte			dacSwingLvl
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn DacSwing=%s, dacSwingLvl=0x%x\n",  
		(bForceExec? "force to":""), ((bDacSwingOn)? "ON":"OFF"), dacSwingLvl));
	pCoexDm->bCurDacSwingOn = bDacSwingOn;
	pCoexDm->curDacSwingLvl = dacSwingLvl;

	if(!bForceExec)
	{
		if( (pCoexDm->bPreDacSwingOn == pCoexDm->bCurDacSwingOn) &&
			(pCoexDm->preDacSwingLvl == pCoexDm->curDacSwingLvl) )
			return;
	}
	delay_ms(30);
	halbtc8723a2ant_SetSwFullTimeDacSwing(pBtCoexist, bDacSwingOn, dacSwingLvl);

	pCoexDm->bPreDacSwingOn = pCoexDm->bCurDacSwingOn;
	pCoexDm->preDacSwingLvl = pCoexDm->curDacSwingLvl;
}

VOID
halbtc8723a2ant_SetAdcBackOff(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bAdcBackOff
	)
{
	if(bAdcBackOff)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BB BackOff Level On!\n"));
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc04,0x3a07611);
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BB BackOff Level Off!\n"));		
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc04,0x3a05611);
	}
}

VOID
halbtc8723a2ant_AdcBackOff(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bAdcBackOff
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn AdcBackOff = %s\n",  
		(bForceExec? "force to":""), ((bAdcBackOff)? "ON":"OFF")));
	pCoexDm->bCurAdcBackOff = bAdcBackOff;

	if(!bForceExec)
	{
		if(pCoexDm->bPreAdcBackOff == pCoexDm->bCurAdcBackOff) 
			return;
	}
	halbtc8723a2ant_SetAdcBackOff(pBtCoexist, pCoexDm->bCurAdcBackOff);

	pCoexDm->bPreAdcBackOff = pCoexDm->bCurAdcBackOff;
}

VOID
halbtc8723a2ant_SetAgcTable(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bAgcTableEn
	)
{
	u1Byte		rssiAdjustVal=0;

	if(bAgcTableEn)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Agc Table On!\n"));
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x4e1c0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x4d1d0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x4c1e0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x4b1f0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x4a200001);
		
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0xdc000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0x90000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0x51000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0x12000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x1a, 0xfffff, 0x00355);
		
		rssiAdjustVal = 6;
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Agc Table Off!\n"));
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x641c0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x631d0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x621e0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x611f0001);
		pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0xc78,0x60200001);

		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0x32000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0x71000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0xb0000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x12, 0xfffff, 0xfc000);
		pBtCoexist->fBtcSetRfReg(pBtCoexist, BTC_RF_A, 0x1a, 0xfffff, 0x30355);
	}

	// set rssiAdjustVal for wifi module.
	pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_U1_RSSI_ADJ_VAL_FOR_AGC_TABLE_ON, &rssiAdjustVal);
}


VOID
halbtc8723a2ant_AgcTable(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bAgcTableEn
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s %s Agc Table\n",  
		(bForceExec? "force to":""), ((bAgcTableEn)? "Enable":"Disable")));
	pCoexDm->bCurAgcTableEn = bAgcTableEn;

	if(!bForceExec)
	{
		if(pCoexDm->bPreAgcTableEn == pCoexDm->bCurAgcTableEn) 
			return;
	}
	halbtc8723a2ant_SetAgcTable(pBtCoexist, bAgcTableEn);

	pCoexDm->bPreAgcTableEn = pCoexDm->bCurAgcTableEn;
}

VOID
halbtc8723a2ant_SetCoexTable(
	IN	PBTC_COEXIST	pBtCoexist,
	IN	u4Byte		val0x6c0,
	IN	u4Byte		val0x6c8,
	IN	u1Byte		val0x6cc
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], set coex table, set 0x6c0=0x%x\n", val0x6c0));
	pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0x6c0, val0x6c0);

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], set coex table, set 0x6c8=0x%x\n", val0x6c8));
	pBtCoexist->fBtcWrite4Byte(pBtCoexist, 0x6c8, val0x6c8);

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], set coex table, set 0x6cc=0x%x\n", val0x6cc));
	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x6cc, val0x6cc);
}

VOID
halbtc8723a2ant_CoexTable(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	u4Byte			val0x6c0,
	IN	u4Byte			val0x6c8,
	IN	u1Byte			val0x6cc
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s write Coex Table 0x6c0=0x%x, 0x6c8=0x%x, 0x6cc=0x%x\n", 
		(bForceExec? "force to":""), val0x6c0, val0x6c8, val0x6cc));
	pCoexDm->curVal0x6c0 = val0x6c0;
	pCoexDm->curVal0x6c8 = val0x6c8;
	pCoexDm->curVal0x6cc = val0x6cc;

	if(!bForceExec)
	{	
		if( (pCoexDm->preVal0x6c0 == pCoexDm->curVal0x6c0) &&
			(pCoexDm->preVal0x6c8 == pCoexDm->curVal0x6c8) &&
			(pCoexDm->preVal0x6cc == pCoexDm->curVal0x6cc) )
			return;
	}
	halbtc8723a2ant_SetCoexTable(pBtCoexist, val0x6c0, val0x6c8, val0x6cc);

	pCoexDm->preVal0x6c0 = pCoexDm->curVal0x6c0;
	pCoexDm->preVal0x6c8 = pCoexDm->curVal0x6c8;
	pCoexDm->preVal0x6cc = pCoexDm->curVal0x6cc;
}

VOID
halbtc8723a2ant_SetFwIgnoreWlanAct(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bEnable
	)
{
	u1Byte			H2C_Parameter[1] ={0};
		
	if(bEnable)
	{
		H2C_Parameter[0] |= BIT0;		// function enable
	}
	
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], set FW for BT Ignore Wlan_Act, FW write 0x25=0x%x\n", 
		H2C_Parameter[0]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x25, 1, H2C_Parameter);	
}

VOID
halbtc8723a2ant_IgnoreWlanAct(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bEnable
	)
{
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn Ignore WlanAct %s\n", 
		(bForceExec? "force to":""), (bEnable? "ON":"OFF")));
	pCoexDm->bCurIgnoreWlanAct = bEnable;

	if(!bForceExec)
	{
		if(pCoexDm->bPreIgnoreWlanAct == pCoexDm->bCurIgnoreWlanAct)
			return;
	}
	halbtc8723a2ant_SetFwIgnoreWlanAct(pBtCoexist, bEnable);

	pCoexDm->bPreIgnoreWlanAct = pCoexDm->bCurIgnoreWlanAct;
}

VOID
halbtc8723a2ant_SetFwPstdma(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			byte1,
	IN	u1Byte			byte2,
	IN	u1Byte			byte3,
	IN	u1Byte			byte4,
	IN	u1Byte			byte5
	)
{
	u1Byte			H2C_Parameter[5] ={0};

	H2C_Parameter[0] = byte1;	
	H2C_Parameter[1] = byte2;	
	H2C_Parameter[2] = byte3;
	H2C_Parameter[3] = byte4;
	H2C_Parameter[4] = byte5;

	pCoexDm->psTdmaPara[0] = byte1;
	pCoexDm->psTdmaPara[1] = byte2;
	pCoexDm->psTdmaPara[2] = byte3;
	pCoexDm->psTdmaPara[3] = byte4;
	pCoexDm->psTdmaPara[4] = byte5;
	
	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], FW write 0x3a(5bytes)=0x%x%08x\n", 
		H2C_Parameter[0], 
		H2C_Parameter[1]<<24|H2C_Parameter[2]<<16|H2C_Parameter[3]<<8|H2C_Parameter[4]));

	pBtCoexist->fBtcFillH2c(pBtCoexist, 0x3a, 5, H2C_Parameter);
}

VOID
halbtc8723a2ant_PsTdma(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bForceExec,
	IN	BOOLEAN			bTurnOn,
	IN	u1Byte			type
	)
{
	u4Byte	btTxRxCnt=0;

	btTxRxCnt = pCoexSta->highPriorityTx+pCoexSta->highPriorityRx+
				pCoexSta->lowPriorityTx+pCoexSta->lowPriorityRx;

	if(btTxRxCnt > 3000)
	{		
		pCoexDm->bCurPsTdmaOn = TRUE;
		pCoexDm->curPsTdma = 8;
		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], turn ON PS TDMA, type=%d for BT tx/rx counters=%d(>3000)\n", 
			pCoexDm->curPsTdma, btTxRxCnt));
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], %s turn %s PS TDMA, type=%d\n", 
			(bForceExec? "force to":""), (bTurnOn? "ON":"OFF"), type));		
		pCoexDm->bCurPsTdmaOn = bTurnOn;
		pCoexDm->curPsTdma = type;
	}

	if(!bForceExec)
	{
		if( (pCoexDm->bPrePsTdmaOn == pCoexDm->bCurPsTdmaOn) &&
			(pCoexDm->prePsTdma == pCoexDm->curPsTdma) )
			return;
	}	
	if(pCoexDm->bCurPsTdmaOn)
	{
		switch(pCoexDm->curPsTdma)
		{
			case 1:
			default:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x1a, 0x1a, 0xe1, 0x98);
				break;
			case 2:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x12, 0x12, 0xe1, 0x98);
				break;
			case 3:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0xa, 0xa, 0xe1, 0x98);
				break;
			case 4:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xa3, 0x5, 0x5, 0xe1, 0x80);
				break;
			case 5:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x1a, 0x1a, 0x60, 0x98);
				break;
			case 6:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x12, 0x12, 0x60, 0x98);
				break;
			case 7:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0xa, 0xa, 0x60, 0x98);
				break;
			case 8: 
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xa3, 0x5, 0x5, 0x60, 0x80);
				break;
			case 9: 
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x1a, 0x1a, 0xe1, 0x98);
				break;
			case 10:	
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x12, 0x12, 0xe1, 0x98);
				break;
			case 11:	
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0xa, 0xa, 0xe1, 0x98);
				break;
			case 12:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x5, 0x5, 0xe1, 0x98);
				break;
			case 13:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x1a, 0x1a, 0x60, 0x98);
				break;
			case 14:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x12, 0x12, 0x60, 0x98);
				break;
			case 15:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0xa, 0xa, 0x60, 0x98);
				break;
			case 16:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x5, 0x5, 0x60, 0x98);
				break;
			case 17:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xa3, 0x2f, 0x2f, 0x60, 0x80);
				break;
			case 18:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x5, 0x5, 0xe1, 0x98);
				break;			
			case 19:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x25, 0x25, 0xe1, 0x98);
				break;
			case 20:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0xe3, 0x25, 0x25, 0x60, 0x98);
				break;
		}
	}
	else
	{
		// disable PS tdma
		switch(pCoexDm->curPsTdma)
		{
			case 0:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0x0, 0x0, 0x0, 0x8, 0x0);
				break;
			case 1:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0x0, 0x0, 0x0, 0x0, 0x0);
				break;
			default:
				halbtc8723a2ant_SetFwPstdma(pBtCoexist, 0x0, 0x0, 0x0, 0x8, 0x0);
				break;
		}
	}

	// update pre state
	pCoexDm->bPrePsTdmaOn = pCoexDm->bCurPsTdmaOn;
	pCoexDm->prePsTdma = pCoexDm->curPsTdma;
}


VOID
halbtc8723a2ant_CoexAllOff(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	// fw all off
	halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
	halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
	halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

	// sw all off
	halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);

	// hw all off
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);
}

VOID
halbtc8723a2ant_InitCoexDm(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	// force to reset coex mechanism
	halbtc8723a2ant_CoexTable(pBtCoexist, FORCE_EXEC, 0x55555555, 0xffff, 0x3);
	halbtc8723a2ant_PsTdma(pBtCoexist, FORCE_EXEC, FALSE, 0);
	halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, FORCE_EXEC, 0x20);
	halbtc8723a2ant_DecBtPwr(pBtCoexist, FORCE_EXEC, FALSE);
	halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, FORCE_EXEC, FALSE);

	halbtc8723a2ant_AgcTable(pBtCoexist, FORCE_EXEC, FALSE);
	halbtc8723a2ant_AdcBackOff(pBtCoexist, FORCE_EXEC, FALSE);
	halbtc8723a2ant_LowPenaltyRa(pBtCoexist, FORCE_EXEC, FALSE);
	halbtc8723a2ant_RfShrink(pBtCoexist, FORCE_EXEC, FALSE);
	halbtc8723a2ant_DacSwing(pBtCoexist, FORCE_EXEC, FALSE, 0xc0);
}

VOID
halbtc8723a2ant_BtInquiryPage(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	BOOLEAN	bLowPwrDisable=TRUE;
	
	pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_ACT_DISABLE_LOW_POWER, &bLowPwrDisable);

	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);
	halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
}

VOID
halbtc8723a2ant_BtEnableAction(
	IN 	PBTC_COEXIST		pBtCoexist
	)
{
	BOOLEAN		bWifiConnected=FALSE;
	
	// Here we need to resend some wifi info to BT
	// because bt is reset and loss of the info.						
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_CONNECTED, &bWifiConnected);
	if(bWifiConnected)
	{
		halbtc8723a2ant_IndicateWifiChnlBwInfo(pBtCoexist, BTC_MEDIA_CONNECT);
	}
	else
	{
		halbtc8723a2ant_IndicateWifiChnlBwInfo(pBtCoexist, BTC_MEDIA_DISCONNECT);
	}

	halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, FORCE_EXEC, FALSE);
}

VOID
halbtc8723a2ant_MonitorBtCtr(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u4Byte 			regHPTxRx, regLPTxRx, u4Tmp;
	u4Byte			regHPTx=0, regHPRx=0, regLPTx=0, regLPRx=0;
	u1Byte			u1Tmp;
	
	regHPTxRx = 0x770;
	regLPTxRx = 0x774;

	u4Tmp = pBtCoexist->fBtcRead4Byte(pBtCoexist, regHPTxRx);
	regHPTx = u4Tmp & bMaskLWord;
	regHPRx = (u4Tmp & bMaskHWord)>>16;

	u4Tmp = pBtCoexist->fBtcRead4Byte(pBtCoexist, regLPTxRx);
	regLPTx = u4Tmp & bMaskLWord;
	regLPRx = (u4Tmp & bMaskHWord)>>16;
		
	pCoexSta->highPriorityTx = regHPTx;
	pCoexSta->highPriorityRx = regHPRx;
	pCoexSta->lowPriorityTx = regLPTx;
	pCoexSta->lowPriorityRx = regLPRx;

	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], High Priority Tx/Rx (reg 0x%x)=0x%x(%d)/0x%x(%d)\n", 
		regHPTxRx, regHPTx, regHPTx, regHPRx, regHPRx));
	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Low Priority Tx/Rx (reg 0x%x)=0x%x(%d)/0x%x(%d)\n", 
		regLPTxRx, regLPTx, regLPTx, regLPRx, regLPRx));

	// reset counter
	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x76e, 0xc);
}

VOID
halbtc8723a2ant_MonitorBtEnableDisable(
	IN 	PBTC_COEXIST		pBtCoexist
	)
{
	static BOOLEAN	bPreBtDisabled=FALSE;
	static u4Byte	btDisableCnt=0;
	BOOLEAN			bBtActive=TRUE, bBtDisabled=FALSE;

	// This function check if bt is disabled

	if(	pCoexSta->highPriorityTx == 0 &&
		pCoexSta->highPriorityRx == 0 &&
		pCoexSta->lowPriorityTx == 0 &&
		pCoexSta->lowPriorityRx == 0)
	{
		bBtActive = FALSE;
	}
	if(	pCoexSta->highPriorityTx == 0xffff &&
		pCoexSta->highPriorityRx == 0xffff &&
		pCoexSta->lowPriorityTx == 0xffff &&
		pCoexSta->lowPriorityRx == 0xffff)
	{
		bBtActive = FALSE;
	}
	if(bBtActive)
	{
		btDisableCnt = 0;
		bBtDisabled = FALSE;
		pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_BL_BT_DISABLE, &bBtDisabled);
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BT is enabled !!\n"));
	}
	else
	{
		btDisableCnt++;
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], bt all counters=0, %d times!!\n", 
				btDisableCnt));
		if(btDisableCnt >= 2)
		{
			bBtDisabled = TRUE;
			pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_BL_BT_DISABLE, &bBtDisabled);
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BT is disabled !!\n"));
		}
	}
	if(bPreBtDisabled != bBtDisabled)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], BT is from %s to %s!!\n", 
			(bPreBtDisabled ? "disabled":"enabled"), 
			(bBtDisabled ? "disabled":"enabled")));
		bPreBtDisabled = bBtDisabled;
		if(!bBtDisabled)
		{
			halbtc8723a2ant_BtEnableAction(pBtCoexist);
		}
	}
}

BOOLEAN
halbtc8723a2ant_IsCommonAction(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	PBTC_STACK_INFO		pStackInfo=&pBtCoexist->stackInfo;
	BOOLEAN			bCommon=FALSE, bWifiConnected=FALSE;
	BOOLEAN			bLowPwrDisable=FALSE;

	if(!pStackInfo->bBtLinkExist)
	{
		bLowPwrDisable = FALSE;
		pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_ACT_DISABLE_LOW_POWER, &bLowPwrDisable);
	}
	else
	{
		bLowPwrDisable = TRUE;
		pBtCoexist->fBtcSet(pBtCoexist, BTC_SET_ACT_DISABLE_LOW_POWER, &bLowPwrDisable);
	}

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_CONNECTED, &bWifiConnected);

	if(halbtc8723a2ant_IsWifiIdle(pBtCoexist) && 
		BT_8723A_2ANT_BT_STATUS_IDLE == pCoexDm->btStatus)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi idle + Bt idle!!\n"));
			
		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);

		bCommon = TRUE;
	}
	else if(!halbtc8723a2ant_IsWifiIdle(pBtCoexist) && 
			(BT_8723A_2ANT_BT_STATUS_IDLE == pCoexDm->btStatus) )
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi non-idle + BT idle!!\n"));

		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);

		bCommon = TRUE;
	}
	else if(halbtc8723a2ant_IsWifiIdle(pBtCoexist) && 
		(BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE == pCoexDm->btStatus) )
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi idle + Bt connected idle!!\n"));
		
		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);

		bCommon = TRUE;
	}
	else if(!halbtc8723a2ant_IsWifiIdle(pBtCoexist) && 
		(BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE == pCoexDm->btStatus) )
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi non-idle + Bt connected idle!!\n"));

		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);

		bCommon = TRUE;
	}
	else if(halbtc8723a2ant_IsWifiIdle(pBtCoexist) && 
			(BT_8723A_2ANT_BT_STATUS_NON_IDLE == pCoexDm->btStatus) )
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi idle + BT non-idle!!\n"));
		
		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		
		bCommon = TRUE;
	}
	else if(halbtc8723a2ant_IsWifiConnectedIdle(pBtCoexist) && 
			(BT_8723A_2ANT_BT_STATUS_NON_IDLE == pCoexDm->btStatus) )
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi connected-idle + BT non-idle!!\n"));

		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);

		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		
		bCommon = TRUE;
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Wifi non-idle + BT non-idle!!\n"));
		halbtc8723a2ant_LowPenaltyRa(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_RfShrink(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_FwDacSwingLvl(pBtCoexist, NORMAL_EXEC, 0x20);
		
		bCommon = FALSE;
	}
	
	return bCommon;
}
VOID
halbtc8723a2ant_TdmaDurationAdjust(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN			bScoHid,
	IN	BOOLEAN			bTxPause,
	IN	u1Byte			maxInterval
	)
{
	static s4Byte		up,dn,m,n,WaitCount;
	s4Byte			result;   //0: no change, +1: increase WiFi duration, -1: decrease WiFi duration
	u1Byte			retryCount=0;

	RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TdmaDurationAdjust()\n"));

	if(pCoexDm->bResetTdmaAdjust)
	{
		pCoexDm->bResetTdmaAdjust = FALSE;
		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], first run TdmaDurationAdjust()!!\n"));
		{
			if(bScoHid)
			{
				if(bTxPause)
				{
					if(maxInterval == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 13);
						pCoexDm->psTdmaDuAdjType = 13;	
					}
					else if(maxInterval == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;	
					}
					else if(maxInterval == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;	
					}
					else
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
				}
				else
				{
					if(maxInterval == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 9);
						pCoexDm->psTdmaDuAdjType = 9;	
					}
					else if(maxInterval == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;	
					}
					else if(maxInterval == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
				}
			}
			else
			{
				if(bTxPause)
				{
					if(maxInterval == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 5);
						pCoexDm->psTdmaDuAdjType = 5;	
					}
					else if(maxInterval == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;	
					}
					else if(maxInterval == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
				}
				else
				{
					if(maxInterval == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 1);
						pCoexDm->psTdmaDuAdjType = 1;	
					}
					else if(maxInterval == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;	
					}
					else if(maxInterval == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
				}
			}
		}
		//============
		up = 0;
		dn = 0;
		m = 1;
		n= 3;
		result = 0;
		WaitCount = 0;
	}
	else
	{
		//accquire the BT TRx retry count from BT_Info byte2
		retryCount = pCoexSta->btRetryCnt;
		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], retryCount = %d\n", retryCount));
		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], up=%d, dn=%d, m=%d, n=%d, WaitCount=%d\n", 
			up, dn, m, n, WaitCount));
		result = 0;
		WaitCount++; 
		  
		if(retryCount == 0)  // no retry in the last 2-second duration
		{
			up++;
			dn--;

			if (dn <= 0)
				dn = 0;				 

			if(up >= n)	// if �s�� n ��2�� retry count��0, �h�ռeWiFi duration
			{
				WaitCount = 0; 
				n = 3;
				up = 0;
				dn = 0;
				result = 1; 
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Increase wifi duration!!\n"));
			}
		}
		else if (retryCount <= 3)	// <=3 retry in the last 2-second duration
		{
			up--; 
			dn++;

			if (up <= 0)
				up = 0;

			if (dn == 2)	// if �s�� 2 ��2�� retry count< 3, �h�կ�WiFi duration
			{
				if (WaitCount <= 2)
					m++; // �קK�@���b���level���Ӧ^
				else
					m = 1;

				if ( m >= 20) //m �̤j�� = 20 ' �̤j120�� recheck�O�_�վ� WiFi duration.
					m = 20;

				n = 3*m;
				up = 0;
				dn = 0;
				WaitCount = 0;
				result = -1; 
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Decrease wifi duration for retryCounter<3!!\n"));
			}
		}
		else  //retry count > 3, �u�n1�� retry count > 3, �h�կ�WiFi duration
		{
			if (WaitCount == 1)
				m++; // �קK�@���b���level���Ӧ^
			else
				m = 1;

			if ( m >= 20) //m �̤j�� = 20 ' �̤j120�� recheck�O�_�վ� WiFi duration.
				m = 20;

			n = 3*m;
			up = 0;
			dn = 0;
			WaitCount = 0; 
			result = -1;
			RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], Decrease wifi duration for retryCounter>3!!\n"));
		}

		RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], max Interval = %d\n", maxInterval));
		if(maxInterval == 1)
		{
			if(bTxPause)
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 1\n"));

				if(pCoexDm->curPsTdma == 1)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 5);
					pCoexDm->psTdmaDuAdjType = 5;
				}
				else if(pCoexDm->curPsTdma == 2)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
					pCoexDm->psTdmaDuAdjType = 6;
				}
				else if(pCoexDm->curPsTdma == 3)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
					pCoexDm->psTdmaDuAdjType = 7;
				}
				else if(pCoexDm->curPsTdma == 4)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
					pCoexDm->psTdmaDuAdjType = 8;
				}
				if(pCoexDm->curPsTdma == 9)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 13);
					pCoexDm->psTdmaDuAdjType = 13;
				}
				else if(pCoexDm->curPsTdma == 10)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
					pCoexDm->psTdmaDuAdjType = 14;
				}
				else if(pCoexDm->curPsTdma == 11)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
					pCoexDm->psTdmaDuAdjType = 15;
				}
				else if(pCoexDm->curPsTdma == 12)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
					pCoexDm->psTdmaDuAdjType = 16;
				}
				
				if(result == -1)
				{					
					if(pCoexDm->curPsTdma == 5)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
						pCoexDm->psTdmaDuAdjType = 8;
					}
					else if(pCoexDm->curPsTdma == 13)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
						pCoexDm->psTdmaDuAdjType = 16;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 8)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 5);
						pCoexDm->psTdmaDuAdjType = 5;
					}
					else if(pCoexDm->curPsTdma == 16)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 13);
						pCoexDm->psTdmaDuAdjType = 13;
					}
				}
			}
			else
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 0\n"));
				if(pCoexDm->curPsTdma == 5)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 1);
					pCoexDm->psTdmaDuAdjType = 1;
				}
				else if(pCoexDm->curPsTdma == 6)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
					pCoexDm->psTdmaDuAdjType = 2;
				}
				else if(pCoexDm->curPsTdma == 7)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
					pCoexDm->psTdmaDuAdjType = 3;
				}
				else if(pCoexDm->curPsTdma == 8)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
					pCoexDm->psTdmaDuAdjType = 4;
				}
				if(pCoexDm->curPsTdma == 13)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 9);
					pCoexDm->psTdmaDuAdjType = 9;
				}
				else if(pCoexDm->curPsTdma == 14)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
					pCoexDm->psTdmaDuAdjType = 10;
				}
				else if(pCoexDm->curPsTdma == 15)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
					pCoexDm->psTdmaDuAdjType = 11;
				}
				else if(pCoexDm->curPsTdma == 16)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
					pCoexDm->psTdmaDuAdjType = 12;
				}
				
				if(result == -1)
				{
					if(pCoexDm->curPsTdma == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
						pCoexDm->psTdmaDuAdjType = 4;
					}
					else if(pCoexDm->curPsTdma == 9)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
						pCoexDm->psTdmaDuAdjType = 12;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 4)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 1);
						pCoexDm->psTdmaDuAdjType = 1;
					}
					else if(pCoexDm->curPsTdma == 12)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 9);
						pCoexDm->psTdmaDuAdjType = 9;
					}
				}
			}
		}
		else if(maxInterval == 2)
		{
			if(bTxPause)
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 1\n"));
				if(pCoexDm->curPsTdma == 1)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
					pCoexDm->psTdmaDuAdjType = 6;
				}
				else if(pCoexDm->curPsTdma == 2)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
					pCoexDm->psTdmaDuAdjType = 6;
				}
				else if(pCoexDm->curPsTdma == 3)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
					pCoexDm->psTdmaDuAdjType = 7;
				}
				else if(pCoexDm->curPsTdma == 4)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
					pCoexDm->psTdmaDuAdjType = 8;
				}
				if(pCoexDm->curPsTdma == 9)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
					pCoexDm->psTdmaDuAdjType = 14;
				}
				else if(pCoexDm->curPsTdma == 10)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
					pCoexDm->psTdmaDuAdjType = 14;
				}
				else if(pCoexDm->curPsTdma == 11)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
					pCoexDm->psTdmaDuAdjType = 15;
				}
				else if(pCoexDm->curPsTdma == 12)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
					pCoexDm->psTdmaDuAdjType = 16;
				}
				if(result == -1)
				{
					if(pCoexDm->curPsTdma == 5) 
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
						pCoexDm->psTdmaDuAdjType = 8;
					}
					else if(pCoexDm->curPsTdma == 13)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
						pCoexDm->psTdmaDuAdjType = 16;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 8)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
						pCoexDm->psTdmaDuAdjType = 6;
					}					
					else if(pCoexDm->curPsTdma == 16)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
						pCoexDm->psTdmaDuAdjType = 14;
					}
				}
			}
			else
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 0\n"));
				if(pCoexDm->curPsTdma == 5)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
					pCoexDm->psTdmaDuAdjType = 2;
				}
				else if(pCoexDm->curPsTdma == 6)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
					pCoexDm->psTdmaDuAdjType = 2;
				}
				else if(pCoexDm->curPsTdma == 7)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
					pCoexDm->psTdmaDuAdjType = 3;
				}
				else if(pCoexDm->curPsTdma == 8)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
					pCoexDm->psTdmaDuAdjType = 4;
				}
				if(pCoexDm->curPsTdma == 13)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
					pCoexDm->psTdmaDuAdjType = 10;
				}
				else if(pCoexDm->curPsTdma == 14)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
					pCoexDm->psTdmaDuAdjType = 10;
				}
				else if(pCoexDm->curPsTdma == 15)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
					pCoexDm->psTdmaDuAdjType = 11;
				}
				else if(pCoexDm->curPsTdma == 16)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
					pCoexDm->psTdmaDuAdjType = 12;
				}
				if(result == -1)
				{
					if(pCoexDm->curPsTdma == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
						pCoexDm->psTdmaDuAdjType = 4;
					}
					else if(pCoexDm->curPsTdma == 9)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
						pCoexDm->psTdmaDuAdjType = 12;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 4)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
						pCoexDm->psTdmaDuAdjType = 2;
					}
					else if(pCoexDm->curPsTdma == 12)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
						pCoexDm->psTdmaDuAdjType = 10;
					}
				}
			}
		}
		else if(maxInterval == 3)
		{
			if(bTxPause)
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 1\n"));
				if(pCoexDm->curPsTdma == 1)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
					pCoexDm->psTdmaDuAdjType = 7;
				}
				else if(pCoexDm->curPsTdma == 2)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
					pCoexDm->psTdmaDuAdjType = 7;
				}
				else if(pCoexDm->curPsTdma == 3)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
					pCoexDm->psTdmaDuAdjType = 7;
				}
				else if(pCoexDm->curPsTdma == 4)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
					pCoexDm->psTdmaDuAdjType = 8;
				}
				if(pCoexDm->curPsTdma == 9)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
					pCoexDm->psTdmaDuAdjType = 15;
				}
				else if(pCoexDm->curPsTdma == 10)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
					pCoexDm->psTdmaDuAdjType = 15;
				}
				else if(pCoexDm->curPsTdma == 11)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
					pCoexDm->psTdmaDuAdjType = 15;
				}
				else if(pCoexDm->curPsTdma == 12)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
					pCoexDm->psTdmaDuAdjType = 16;
				}
				if(result == -1)
				{
					if(pCoexDm->curPsTdma == 5) 
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
						pCoexDm->psTdmaDuAdjType = 8;
					}
					else if(pCoexDm->curPsTdma == 13)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
						pCoexDm->psTdmaDuAdjType = 16;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 8)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 7)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}
					else if(pCoexDm->curPsTdma == 6)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 7);
						pCoexDm->psTdmaDuAdjType = 7;
					}					
					else if(pCoexDm->curPsTdma == 16)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 15)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
					else if(pCoexDm->curPsTdma == 14)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
						pCoexDm->psTdmaDuAdjType = 15;
					}
				}
			}
			else
			{
				RT_TRACE(COMP_COEX, DBG_TRACE, ("[BTCoex], TxPause = 0\n"));
				if(pCoexDm->curPsTdma == 5)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
					pCoexDm->psTdmaDuAdjType = 3;
				}
				else if(pCoexDm->curPsTdma == 6)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
					pCoexDm->psTdmaDuAdjType = 3;
				}
				else if(pCoexDm->curPsTdma == 7)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
					pCoexDm->psTdmaDuAdjType = 3;
				}
				else if(pCoexDm->curPsTdma == 8)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
					pCoexDm->psTdmaDuAdjType = 4;
				}
				if(pCoexDm->curPsTdma == 13)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
					pCoexDm->psTdmaDuAdjType = 11;
				}
				else if(pCoexDm->curPsTdma == 14)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
					pCoexDm->psTdmaDuAdjType = 11;
				}
				else if(pCoexDm->curPsTdma == 15)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
					pCoexDm->psTdmaDuAdjType = 11;
				}
				else if(pCoexDm->curPsTdma == 16)
				{
					halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
					pCoexDm->psTdmaDuAdjType = 12;
				}
				if(result == -1)
				{
					if(pCoexDm->curPsTdma == 1)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
						pCoexDm->psTdmaDuAdjType = 4;
					}
					else if(pCoexDm->curPsTdma == 9)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
						pCoexDm->psTdmaDuAdjType = 12;
					}
				} 
				else if (result == 1)
				{
					if(pCoexDm->curPsTdma == 4)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 3)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 2)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 3);
						pCoexDm->psTdmaDuAdjType = 3;
					}
					else if(pCoexDm->curPsTdma == 12)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 11)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
					else if(pCoexDm->curPsTdma == 10)
					{
						halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
						pCoexDm->psTdmaDuAdjType = 11;
					}
				}
			}
		}
	}

	// if current PsTdma not match with the recorded one (when scan, dhcp...), 
	// then we have to adjust it back to the previous record one.
	if(pCoexDm->curPsTdma != pCoexDm->psTdmaDuAdjType)
	{
		BOOLEAN	bScan=FALSE, bLink=FALSE, bRoam=FALSE;
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], PsTdma type dismatch!!!, curPsTdma=%d, recordPsTdma=%d\n", 
			pCoexDm->curPsTdma, pCoexDm->psTdmaDuAdjType));

		pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_SCAN, &bScan);
		pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_LINK, &bLink);
		pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_WIFI_ROAM, &bRoam);
		
		if( !bScan && !bLink && !bRoam)
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, pCoexDm->psTdmaDuAdjType);
		}
		else
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], roaming/link/scan is under progress, will adjust next time!!!\n"));
		}
	}
}

// SCO only or SCO+PAN(HS)
VOID
halbtc8723a2ant_ActionSco(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte	wifiRssiState, wifiRssiState1;
	u4Byte	wifiBw;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);
	
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
		}

		// sw mechanism		
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 11);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 15);
		}
		
		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}		
	}
}


VOID
halbtc8723a2ant_ActionHid(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte	wifiRssiState, wifiRssiState1;
	u4Byte	wifiBw;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 9);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 13);
		}

		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 9);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 13);
		}

		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}		
	}
}

//A2DP only / PAN(EDR) only/ A2DP+PAN(HS)
VOID
halbtc8723a2ant_ActionA2dp(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1, btInfoExt;
	u4Byte		wifiBw;

	btInfoExt = pCoexSta->btInfoExt;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);
	
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp rate, 1:basic /0:edr
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, FALSE, 3);
			}
			else
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, FALSE, 1);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp rate, 1:basic /0:edr
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, TRUE, 3);
			}
			else
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, TRUE, 1);
			}
		}

		// sw mechanism		
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp rate, 1:basic /0:edr
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, FALSE, 3);
			}
			else
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, FALSE, 1);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp rate, 1:basic /0:edr
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, TRUE, 3);
			}
			else
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, FALSE, TRUE, 1);
			}
		}
		
		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}		
	}
}

VOID
halbtc8723a2ant_ActionPanEdr(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1, btInfoExt;
	u4Byte		wifiBw;
	
	btInfoExt = pCoexSta->btInfoExt;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);
			
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
		}

		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
		}

		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}


//PAN(HS) only
VOID
halbtc8723a2ant_ActionPanHs(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState;
	u4Byte		wifiBw;

	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
		}
		else
		{
			halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
		}
		halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);

		// sw mechanism		
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		}
		else
		{
			halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, FALSE, 0);
		}

		// sw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}

//PAN(EDR)+A2DP
VOID
halbtc8723a2ant_ActionPanEdrA2dp(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1, btInfoExt;
	u4Byte		wifiBw;

	btInfoExt = pCoexSta->btInfoExt;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
			}
		}

		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 4);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 2);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 8);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 6);
			}
		}

		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}

VOID
halbtc8723a2ant_ActionPanEdrHid(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1;
	u4Byte		wifiBw;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);

		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10); 
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14); 
		}

		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
		}
		else
		{
			halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
		}
		
		// sw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}

// HID+A2DP+PAN(EDR)
VOID
halbtc8723a2ant_ActionHidA2dpPanEdr(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1, btInfoExt;
	u4Byte		wifiBw;

	btInfoExt = pCoexSta->btInfoExt;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);
			
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
			}
		}
		
		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 12);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 10);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 16);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_PsTdma(pBtCoexist, NORMAL_EXEC, TRUE, 14);
			}
		}

		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}

VOID
halbtc8723a2ant_ActionHidA2dp(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	u1Byte		wifiRssiState, wifiRssiState1, btInfoExt;
	u4Byte		wifiBw;

	btInfoExt = pCoexSta->btInfoExt;

	if(halbtc8723a2ant_NeedToDecBtPwr(pBtCoexist))
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, TRUE);
	else	
		halbtc8723a2ant_DecBtPwr(pBtCoexist, NORMAL_EXEC, FALSE);
	halbtc8723a2ant_CoexTable(pBtCoexist, NORMAL_EXEC, 0x55555555, 0xffff, 0x3);

	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_U4_WIFI_BW, &wifiBw);
	if(BTC_WIFI_BW_HT40 == wifiBw)
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 37, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, FALSE, 3);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, FALSE, 1);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, TRUE, 3);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, TRUE, 1);
			}
		}
		
		// sw mechanism
		halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
		halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
		halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
	}
	else
	{
		wifiRssiState = halbtc8723a2ant_WifiRssiState(pBtCoexist, 0, 2, 27, 0);
		wifiRssiState1 = halbtc8723a2ant_WifiRssiState(pBtCoexist, 1, 2, 47, 0);
		
		// fw mechanism
		if( (wifiRssiState == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState == BTC_RSSI_STATE_STAY_HIGH) )
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, FALSE, 3);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, FALSE, 1);
			}
		}
		else
		{
			if(btInfoExt&BIT0)	//a2dp basic rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, TRUE, 3);
			}
			else				//a2dp edr rate
			{
				halbtc8723a2ant_TdmaDurationAdjust(pBtCoexist, TRUE, TRUE, 1);
			}
		}

		// sw mechanism
		if( (wifiRssiState1 == BTC_RSSI_STATE_HIGH) ||
			(wifiRssiState1 == BTC_RSSI_STATE_STAY_HIGH) )
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, TRUE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
		else
		{
			halbtc8723a2ant_AgcTable(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_AdcBackOff(pBtCoexist, NORMAL_EXEC, FALSE);
			halbtc8723a2ant_DacSwing(pBtCoexist, NORMAL_EXEC, FALSE, 0xc0);
		}
	}
}

VOID
halbtc8723a2ant_RunCoexistMechanism(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	PBTC_STACK_INFO		pStackInfo=&pBtCoexist->stackInfo;
	u1Byte				btInfoOriginal=0, btRetryCnt=0;
	u1Byte				algorithm=0;

	if(pBtCoexist->bManualControl)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Manual control!!!\n"));
		return;
	}

	if(pStackInfo->bProfileNotified)
	{
		if(pCoexSta->bHoldForStackOperation)
		{
			// if bt inquiry/page/pair, do not execute.
			return;
		}
		
		algorithm = halbtc8723a2ant_ActionAlgorithm(pBtCoexist);
		if(pCoexSta->bHoldPeriodCnt && (BT_8723A_2ANT_COEX_ALGO_PANHS!=algorithm))
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex],Hold BT inquiry/page scan setting (cnt = %d)!!\n", 
				pCoexSta->bHoldPeriodCnt));
			if(pCoexSta->bHoldPeriodCnt >= 6)
			{
				pCoexSta->bHoldPeriodCnt = 0;
				// next time the coexist parameters should be reset again.
			}
			else
				pCoexSta->bHoldPeriodCnt++;
			return;
		}

		pCoexDm->curAlgorithm = algorithm;
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Algorithm = %d \n", pCoexDm->curAlgorithm));
		if(halbtc8723a2ant_IsCommonAction(pBtCoexist))
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant common.\n"));
			pCoexDm->bResetTdmaAdjust = TRUE;
		}
		else
		{
			if(pCoexDm->curAlgorithm != pCoexDm->preAlgorithm)
			{
				RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], preAlgorithm=%d, curAlgorithm=%d\n", 
					pCoexDm->preAlgorithm, pCoexDm->curAlgorithm));
				pCoexDm->bResetTdmaAdjust = TRUE;
			}
			switch(pCoexDm->curAlgorithm)
			{
				case BT_8723A_2ANT_COEX_ALGO_SCO:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = SCO.\n"));
					halbtc8723a2ant_ActionSco(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_HID:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = HID.\n"));
					halbtc8723a2ant_ActionHid(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_A2DP:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = A2DP.\n"));
					halbtc8723a2ant_ActionA2dp(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_PANEDR:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = PAN(EDR).\n"));
					halbtc8723a2ant_ActionPanEdr(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_PANHS:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = HS mode.\n"));
					halbtc8723a2ant_ActionPanHs(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_PANEDR_A2DP:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = PAN+A2DP.\n"));
					halbtc8723a2ant_ActionPanEdrA2dp(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_PANEDR_HID:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = PAN(EDR)+HID.\n"));
					halbtc8723a2ant_ActionPanEdrHid(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_HID_A2DP_PANEDR:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = HID+A2DP+PAN.\n"));
					halbtc8723a2ant_ActionHidA2dpPanEdr(pBtCoexist);
					break;
				case BT_8723A_2ANT_COEX_ALGO_HID_A2DP:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = HID+A2DP.\n"));
					halbtc8723a2ant_ActionHidA2dp(pBtCoexist);
					break;
				default:
					RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Action 2-Ant, algorithm = coexist All Off!!\n"));
					halbtc8723a2ant_CoexAllOff(pBtCoexist);
					break;
			}
			pCoexDm->preAlgorithm = pCoexDm->curAlgorithm;
		}
	}
}

//============================================================
// work around function start with wa_halbtc8723a2ant_
//============================================================
VOID
wa_halbtc8723a2ant_MonitorC2h(
	IN	PBTC_COEXIST			pBtCoexist
	)
{
	u1Byte	tmp1b=0x0;
	u4Byte	curC2hTotalCnt=0x0;
	static u4Byte	preC2hTotalCnt=0x0, sameCntPollingTime=0x0;

	curC2hTotalCnt+=pCoexSta->btInfoC2hCnt[BT_INFO_SRC_8723A_2ANT_BT_RSP];

	if(curC2hTotalCnt == preC2hTotalCnt)
	{
		sameCntPollingTime++;
	}
	else
	{
		preC2hTotalCnt = curC2hTotalCnt;
		sameCntPollingTime = 0;
	}

	if(sameCntPollingTime >= 2)
	{
		tmp1b = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x1af);
		if(tmp1b != 0x0)
		{
			pCoexSta->c2hHangDetectCnt++;
			pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x1af, 0x0);
		}
	}
}

//============================================================
// extern function start with EXhalbtc8723a2ant_
//============================================================
VOID
EXhalbtc8723a2ant_PowerOnSetting(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
}

VOID
EXhalbtc8723a2ant_InitHwConfig(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN				bWifiOnly
	)
{
	u4Byte	u4Tmp=0;
	u1Byte	u1Tmp=0;

	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], 2Ant Init HW Config!!\n"));

	// backup rf 0x1e value
	pCoexDm->btRf0x1eBackup = 
		pBtCoexist->fBtcGetRfReg(pBtCoexist, BTC_RF_A, 0x1e, 0xfffff);

	// Enable counter statistics
	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x76e, 0x4);
	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x778, 0x3);
	pBtCoexist->fBtcWrite1Byte(pBtCoexist, 0x40, 0x20);
}

VOID
EXhalbtc8723a2ant_InitCoexDm(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Coex Mechanism Init!!\n"));
	
	halbtc8723a2ant_InitCoexDm(pBtCoexist);
}

VOID
EXhalbtc8723a2ant_DisplayCoexInfo(
	IN	PBTC_COEXIST		pBtCoexist
	)
{
	PBTC_BOARD_INFO		pBoardInfo=&pBtCoexist->boardInfo;
	PBTC_STACK_INFO		pStackInfo=&pBtCoexist->stackInfo;
	pu1Byte				cliBuf=pBtCoexist->cliBuf;
	u1Byte				u1Tmp[4], i, btInfoExt, psTdmaCase=0;
	u4Byte				u4Tmp[4];

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n ============[BT Coexist info]============");
	CL_PRINTF(cliBuf);

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d ", "Ant PG number/ Ant mechanism:", \
		pBoardInfo->pgAntNum, pBoardInfo->btdmAntNum);
	CL_PRINTF(cliBuf);	
	
	if(pBtCoexist->bManualControl)
	{
		CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "[Action Manual control]!!");
		CL_PRINTF(cliBuf);
	}
	
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %s / %d", "BT stack/ hci ext ver", \
		((pStackInfo->bProfileNotified)? "Yes":"No"), pStackInfo->hciVersion);
	CL_PRINTF(cliBuf);

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %02x %02x %02x ", "Wifi channel informed to BT", \
		pCoexDm->wifiChnlInfo[0], pCoexDm->wifiChnlInfo[1],
		pCoexDm->wifiChnlInfo[2]);
	CL_PRINTF(cliBuf);
	
	// wifi status
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "============[Wifi Status]============");
	CL_PRINTF(cliBuf);
	pBtCoexist->fBtcDispDbgMsg(pBtCoexist, BTC_DBG_DISP_WIFI_STATUS);

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "============[BT Status]============");
	CL_PRINTF(cliBuf);

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = [%s/ %d/ %d] ", "BT [status/ rssi/ retryCnt]", \
		((pCoexSta->bC2hBtInquiryPage)?("inquiry/page scan"):((BT_8723A_2ANT_BT_STATUS_IDLE == pCoexDm->btStatus)? "idle":(  (BT_8723A_2ANT_BT_STATUS_CONNECTED_IDLE == pCoexDm->btStatus)? "connected-idle":"busy"))),
		pCoexSta->btRssi, pCoexSta->btRetryCnt);
	CL_PRINTF(cliBuf);
	
	if(pStackInfo->bProfileNotified)
	{			
		CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d / %d / %d / %d", "SCO/HID/PAN/A2DP", \
			pStackInfo->bScoExist, pStackInfo->bHidExist, pStackInfo->bPanExist, pStackInfo->bA2dpExist);
		CL_PRINTF(cliBuf);	

		pBtCoexist->fBtcDispDbgMsg(pBtCoexist, BTC_DBG_DISP_BT_LINK_INFO);
	}

	btInfoExt = pCoexSta->btInfoExt;
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %s", "BT Info A2DP rate", \
		(btInfoExt&BIT0)? "Basic rate":"EDR rate");
	CL_PRINTF(cliBuf);	

	for(i=0; i<BT_INFO_SRC_8723A_2ANT_MAX; i++)
	{
		if(pCoexSta->btInfoC2hCnt[i])
		{				
			CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %02x %02x %02x %02x %02x %02x %02x(%d)", GLBtInfoSrc8723a2Ant[i], \
				pCoexSta->btInfoC2h[i][0], pCoexSta->btInfoC2h[i][1],
				pCoexSta->btInfoC2h[i][2], pCoexSta->btInfoC2h[i][3],
				pCoexSta->btInfoC2h[i][4], pCoexSta->btInfoC2h[i][5],
				pCoexSta->btInfoC2h[i][6], pCoexSta->btInfoC2hCnt[i]);
			CL_PRINTF(cliBuf);
		}
	}

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d", "write 0x1af=0x0 num", \
		pCoexSta->c2hHangDetectCnt);
	CL_PRINTF(cliBuf);
	
	// Sw mechanism	
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "============[Sw mechanism]============");
	CL_PRINTF(cliBuf);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d/ %d", "SM1[ShRf/ LpRA/ LimDig]", \
		pCoexDm->bCurRfRxLpfShrink, pCoexDm->bCurLowPenaltyRa, pCoexDm->bLimitedDig);
	CL_PRINTF(cliBuf);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d/ %d(0x%x) ", "SM2[AgcT/ AdcB/ SwDacSwing(lvl)]", \
		pCoexDm->bCurAgcTableEn, pCoexDm->bCurAdcBackOff, pCoexDm->bCurDacSwingOn, pCoexDm->curDacSwingLvl);
	CL_PRINTF(cliBuf);

	// Fw mechanism		
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "============[Fw mechanism]============");
	CL_PRINTF(cliBuf);	
	
	if(!pBtCoexist->bManualControl)
	{
		psTdmaCase = pCoexDm->curPsTdma;
		CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %02x %02x %02x %02x %02x case-%d", "PS TDMA", \
			pCoexDm->psTdmaPara[0], pCoexDm->psTdmaPara[1],
			pCoexDm->psTdmaPara[2], pCoexDm->psTdmaPara[3],
			pCoexDm->psTdmaPara[4], psTdmaCase);
		CL_PRINTF(cliBuf);
	
		CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d ", "DecBtPwr/ IgnWlanAct", \
			pCoexDm->bCurDecBtPwr, pCoexDm->bCurIgnoreWlanAct);
		CL_PRINTF(cliBuf);
	}

	// Hw setting		
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s", "============[Hw setting]============");
	CL_PRINTF(cliBuf);	

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x", "RF-A, 0x1e initVal", \
		pCoexDm->btRf0x1eBackup);
	CL_PRINTF(cliBuf);

	u1Tmp[0] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x778);
	u1Tmp[1] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x783);
	u1Tmp[2] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x796);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x/ 0x%x/ 0x%x", "0x778/ 0x783/ 0x796", \
		u1Tmp[0], u1Tmp[1], u1Tmp[2]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x880);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x", "0x880", \
		u4Tmp[0]);
	CL_PRINTF(cliBuf);

	u1Tmp[0] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x40);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x", "0x40", \
		u1Tmp[0]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x550);
	u1Tmp[0] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x522);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x/ 0x%x", "0x550(bcn ctrl)/0x522", \
		u4Tmp[0], u1Tmp[0]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x484);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x", "0x484(rate adaptive)", \
		u4Tmp[0]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0xc50);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x", "0xc50(dig)", \
		u4Tmp[0]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0xda0);
	u4Tmp[1] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0xda4);
	u4Tmp[2] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0xda8);
	u4Tmp[3] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0xdac);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x/ 0x%x/ 0x%x/ 0x%x", "0xda0/0xda4/0xda8/0xdac(FA cnt)", \
		u4Tmp[0], u4Tmp[1], u4Tmp[2], u4Tmp[3]);
	CL_PRINTF(cliBuf);

	u4Tmp[0] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x6c0);
	u4Tmp[1] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x6c4);
	u4Tmp[2] = pBtCoexist->fBtcRead4Byte(pBtCoexist, 0x6c8);
	u1Tmp[0] = pBtCoexist->fBtcRead1Byte(pBtCoexist, 0x6cc);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = 0x%x/ 0x%x/ 0x%x/ 0x%x", "0x6c0/0x6c4/0x6c8/0x6cc(coexTable)", \
		u4Tmp[0], u4Tmp[1], u4Tmp[2], u1Tmp[0]);
	CL_PRINTF(cliBuf);

	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d", "0x770 (hp rx[31:16]/tx[15:0])", \
		pCoexSta->highPriorityRx, pCoexSta->highPriorityTx);
	CL_PRINTF(cliBuf);
	CL_SPRINTF(cliBuf, BT_TMP_BUF_SIZE, "\r\n %-35s = %d/ %d", "0x774(lp rx[31:16]/tx[15:0])", \
		pCoexSta->lowPriorityRx, pCoexSta->lowPriorityTx);
	CL_PRINTF(cliBuf);

	pBtCoexist->fBtcDispDbgMsg(pBtCoexist, BTC_DBG_DISP_COEX_STATISTICS);
}


VOID
EXhalbtc8723a2ant_IpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	)
{
	if(BTC_IPS_ENTER == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], IPS ENTER notify\n"));
		halbtc8723a2ant_CoexAllOff(pBtCoexist);
	}
	else if(BTC_IPS_LEAVE == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], IPS LEAVE notify\n"));
		//halbtc8723a2ant_InitCoexDm(pBtCoexist);
	}
}

VOID
EXhalbtc8723a2ant_LpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	)
{
	if(BTC_LPS_ENABLE == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], LPS ENABLE notify\n"));
	}
	else if(BTC_LPS_DISABLE == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], LPS DISABLE notify\n"));
	}
}

VOID
EXhalbtc8723a2ant_ScanNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	)
{
	if(BTC_SCAN_START == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCAN START notify\n"));
	}
	else if(BTC_SCAN_FINISH == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], SCAN FINISH notify\n"));
	}
}

VOID
EXhalbtc8723a2ant_ConnectNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	)
{
	if(BTC_ASSOCIATE_START == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], CONNECT START notify\n"));
	}
	else if(BTC_ASSOCIATE_FINISH == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], CONNECT FINISH notify\n"));
	}
}

VOID
EXhalbtc8723a2ant_MediaStatusNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	)
{
	if(BTC_MEDIA_CONNECT == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], MEDIA connect notify\n"));
	}
	else
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], MEDIA disconnect notify\n"));
	}

	halbtc8723a2ant_IndicateWifiChnlBwInfo(pBtCoexist, type);
}

VOID
EXhalbtc8723a2ant_SpecialPacketNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	)
{
	if(type == BTC_PACKET_DHCP)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], DHCP Packet notify\n"));
	}
}

VOID
EXhalbtc8723a2ant_BtInfoNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	pu1Byte			tmpBuf,
	IN	u1Byte			length
	)
{
	u1Byte			btInfo=0;
	u1Byte			i, rspSource=0;
	BOOLEAN			bBtBusy=FALSE, bLimitedDig=FALSE;
	BOOLEAN			bWifiConnected=FALSE, bBtHsOn=FALSE;

	pCoexSta->bC2hBtInfoReqSent = FALSE;
	
	rspSource = BT_INFO_SRC_8723A_2ANT_BT_RSP;
	pCoexSta->btInfoC2hCnt[rspSource]++;

	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Bt info[%d], length=%d, hex data=[", rspSource, length));
	for(i=0; i<length; i++)
	{
		pCoexSta->btInfoC2h[rspSource][i] = tmpBuf[i];
		if(i == 0)
			btInfo = tmpBuf[i];
		if(i == length-1)
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("0x%02x]\n", tmpBuf[i]));
		}
		else
		{
			RT_TRACE(COMP_COEX, DBG_LOUD, ("0x%02x, ", tmpBuf[i]));
		}
	}

	if(BT_INFO_SRC_8723A_2ANT_WIFI_FW != rspSource)
	{
		pCoexSta->btRetryCnt =
			pCoexSta->btInfoC2h[rspSource][1];

		pCoexSta->btRssi =
			pCoexSta->btInfoC2h[rspSource][2]*2+10;

		pCoexSta->btInfoExt = 
			pCoexSta->btInfoC2h[rspSource][3];
	}
		
	pBtCoexist->fBtcGet(pBtCoexist, BTC_GET_BL_HS_OPERATION, &bBtHsOn);
	// check BIT2 first ==> check if bt is under inquiry or page scan
	if(btInfo & BT_INFO_8723A_2ANT_B_INQ_PAGE)
	{
		pCoexSta->bC2hBtInquiryPage = TRUE;
	}
	else
	{
		pCoexSta->bC2hBtInquiryPage = FALSE;
	}
}

VOID
EXhalbtc8723a2ant_StackOperationNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	)
{
	if(BTC_STACK_OP_INQ_PAGE_PAIR_START == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], StackOP Inquiry/page/pair start notify\n"));
		pCoexSta->bHoldForStackOperation = TRUE;
		pCoexSta->bHoldPeriodCnt = 1;
		halbtc8723a2ant_BtInquiryPage(pBtCoexist);
	}
	else if(BTC_STACK_OP_INQ_PAGE_PAIR_FINISH == type)
	{
		RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], StackOP Inquiry/page/pair finish notify\n"));
		pCoexSta->bHoldForStackOperation = FALSE;
	}
}

VOID
EXhalbtc8723a2ant_HaltNotify(
	IN	PBTC_COEXIST			pBtCoexist
	)
{
	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], Halt notify\n"));

	halbtc8723a2ant_IgnoreWlanAct(pBtCoexist, FORCE_EXEC, TRUE);
	EXhalbtc8723a2ant_MediaStatusNotify(pBtCoexist, BTC_MEDIA_DISCONNECT);
}

VOID
EXhalbtc8723a2ant_Periodical(
	IN	PBTC_COEXIST			pBtCoexist
	)
{
	RT_TRACE(COMP_COEX, DBG_LOUD, ("[BTCoex], 2Ant Periodical!!\n"));

	// work around for c2h hang
	wa_halbtc8723a2ant_MonitorC2h(pBtCoexist);
	
	halbtc8723a2ant_QueryBtInfo(pBtCoexist);
	halbtc8723a2ant_MonitorBtCtr(pBtCoexist);
	halbtc8723a2ant_MonitorBtEnableDisable(pBtCoexist);

	halbtc8723a2ant_RunCoexistMechanism(pBtCoexist);
}


#endif

