//===========================================
// The following is for 8723A 1Ant BT Co-exist definition
//===========================================
#define	BT_INFO_8723A_1ANT_B_FTP						BIT7
#define	BT_INFO_8723A_1ANT_B_A2DP					BIT6
#define	BT_INFO_8723A_1ANT_B_HID						BIT5
#define	BT_INFO_8723A_1ANT_B_SCO_BUSY				BIT4
#define	BT_INFO_8723A_1ANT_B_ACL_BUSY				BIT3
#define	BT_INFO_8723A_1ANT_B_INQ_PAGE				BIT2
#define	BT_INFO_8723A_1ANT_B_SCO_ESCO				BIT1
#define	BT_INFO_8723A_1ANT_B_CONNECTION				BIT0

typedef enum _BT_STATE_8723A_1ANT{
	BT_STATE_8723A_1ANT_DISABLED				= 0,
	BT_STATE_8723A_1ANT_NO_CONNECTION		= 1,
	BT_STATE_8723A_1ANT_CONNECT_IDLE		= 2,
	BT_STATE_8723A_1ANT_INQ_OR_PAG			= 3,
	BT_STATE_8723A_1ANT_ACL_ONLY_BUSY		= 4,
	BT_STATE_8723A_1ANT_SCO_ONLY_BUSY		= 5,
	BT_STATE_8723A_1ANT_ACL_SCO_BUSY			= 6,
	BT_STATE_8723A_1ANT_HID_BUSY				= 7,
	BT_STATE_8723A_1ANT_HID_SCO_BUSY			= 8,
	BT_STATE_8723A_1ANT_MAX
}BT_STATE_8723A_1ANT, *PBT_STATE_8723A_1ANT;

#define		BTC_RSSI_COEX_THRESH_TOL_8723A_1ANT		2

typedef enum _BT_INFO_SRC_8723A_1ANT{
	BT_INFO_SRC_8723A_1ANT_WIFI_FW			= 0x0,
	BT_INFO_SRC_8723A_1ANT_BT_RSP				= 0x1,
	BT_INFO_SRC_8723A_1ANT_BT_ACTIVE_SEND		= 0x2,
	BT_INFO_SRC_8723A_1ANT_MAX
}BT_INFO_SRC_8723A_1ANT,*PBT_INFO_SRC_8723A_1ANT;

typedef enum _BT_8723A_1ANT_BT_STATUS{
	BT_8723A_1ANT_BT_STATUS_IDLE				= 0x0,
	BT_8723A_1ANT_BT_STATUS_CONNECTED_IDLE	= 0x1,
	BT_8723A_1ANT_BT_STATUS_NON_IDLE			= 0x2,
	BT_8723A_1ANT_BT_STATUS_MAX
}BT_8723A_1ANT_BT_STATUS,*PBT_8723A_1ANT_BT_STATUS;

typedef enum _BT_8723A_1ANT_COEX_ALGO{
	BT_8723A_1ANT_COEX_ALGO_UNDEFINED			= 0x0,
	BT_8723A_1ANT_COEX_ALGO_SCO				= 0x1,
	BT_8723A_1ANT_COEX_ALGO_HID				= 0x2,
	BT_8723A_1ANT_COEX_ALGO_A2DP				= 0x3,
	BT_8723A_1ANT_COEX_ALGO_PANEDR			= 0x4,
	BT_8723A_1ANT_COEX_ALGO_PANHS			= 0x5,
	BT_8723A_1ANT_COEX_ALGO_PANEDR_A2DP		= 0x6,
	BT_8723A_1ANT_COEX_ALGO_PANEDR_HID		= 0x7,
	BT_8723A_1ANT_COEX_ALGO_HID_A2DP_PANEDR	= 0x8,
	BT_8723A_1ANT_COEX_ALGO_HID_A2DP			= 0x9,
	BT_8723A_1ANT_COEX_ALGO_MAX
}BT_8723A_1ANT_COEX_ALGO,*PBT_8723A_1ANT_COEX_ALGO;

typedef struct _COEX_DM_8723A_1ANT{
	// fw mechanism
	BOOLEAN		bCurIgnoreWlanAct;
	BOOLEAN		bPreIgnoreWlanAct;
	u1Byte		prePsTdma;
	u1Byte		curPsTdma;
	u1Byte		psTdmaPara[5];
	u1Byte		psTdmaDuAdjType;
	u4Byte		psTdmaMonitorCnt;
	u4Byte		psTdmaGlobalCnt;
	BOOLEAN		bResetTdmaAdjust;
	BOOLEAN		bPrePsTdmaOn;
	BOOLEAN		bCurPsTdmaOn;

	// sw mechanism
	BOOLEAN		bPreRfRxLpfShrink;
	BOOLEAN		bCurRfRxLpfShrink;
	u4Byte		btRf0x1eBackup;
	BOOLEAN 	bPreLowPenaltyRa;
	BOOLEAN		bCurLowPenaltyRa;
	u4Byte		preVal0x6c0;
	u4Byte		curVal0x6c0;
	u4Byte		preVal0x6c8;
	u4Byte		curVal0x6c8;
	u1Byte		preVal0x6cc;
	u1Byte		curVal0x6cc;
	BOOLEAN		bLimitedDig;

	// algorithm related
	u1Byte		preAlgorithm;
	u1Byte		curAlgorithm;
	u1Byte		btStatus;
	u1Byte		wifiChnlInfo[3];
} COEX_DM_8723A_1ANT, *PCOEX_DM_8723A_1ANT;

typedef struct _COEX_STA_8723A_1ANT{
	u4Byte					highPriorityTx;
	u4Byte					highPriorityRx;
	u4Byte					lowPriorityTx;
	u4Byte					lowPriorityRx;
	u1Byte					btRssi;
	u1Byte					preBtRssiState;
	u1Byte					preBtRssiState1;
	u1Byte					preWifiRssiState[4];
	BOOLEAN					bC2hBtInfoReqSent;
	u1Byte					btInfoC2h[BT_INFO_SRC_8723A_1ANT_MAX][10];
	u4Byte					btInfoC2hCnt[BT_INFO_SRC_8723A_1ANT_MAX];
	BOOLEAN					bC2hBtInquiryPage;
	u1Byte					btRetryCnt;
	u1Byte					btInfoExt;
	//BOOLEAN					bHoldForStackOperation;
	//u1Byte					bHoldPeriodCnt;
	// this is for c2h hang work-around
	u4Byte					c2hHangDetectCnt;
}COEX_STA_8723A_1ANT, *PCOEX_STA_8723A_1ANT;

//===========================================
// The following is interface which will notify coex module.
//===========================================
VOID
EXhalbtc8723a1ant_InitHwConfig(
	IN	PBTC_COEXIST		pBtCoexist
	);
VOID
EXhalbtc8723a1ant_InitCoexDm(
	IN	PBTC_COEXIST		pBtCoexist
	);
VOID
EXhalbtc8723a1ant_IpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	);
VOID
EXhalbtc8723a1ant_LpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	);
VOID
EXhalbtc8723a1ant_ScanNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	);
VOID
EXhalbtc8723a1ant_ConnectNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
	);
VOID
EXhalbtc8723a1ant_MediaStatusNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	);
VOID
EXhalbtc8723a1ant_SpecialPacketNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
	);
VOID
EXhalbtc8723a1ant_BtInfoNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	pu1Byte			tmpBuf,
	IN	u1Byte			length
	);
VOID
EXhalbtc8723a1ant_HaltNotify(
	IN	PBTC_COEXIST			pBtCoexist
	);
VOID
EXhalbtc8723a1ant_Periodical(
	IN	PBTC_COEXIST			pBtCoexist
	);
VOID
EXhalbtc8723a1ant_DisplayCoexInfo(
	IN	PBTC_COEXIST		pBtCoexist
	);

