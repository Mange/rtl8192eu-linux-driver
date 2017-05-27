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


#ifndef	__ODM_DBG_H__
#define __ODM_DBG_H__

#define DEBUG_VERSION	"1.0"  /*2015.01.13 Dino*/
//-----------------------------------------------------------------------------
//	Define the debug levels
//
//	1.	DBG_TRACE and DBG_LOUD are used for normal cases.
//	So that, they can help SW engineer to develope or trace states changed 
//	and also help HW enginner to trace every operation to and from HW, 
//	e.g IO, Tx, Rx. 
//
//	2.	DBG_WARNNING and DBG_SERIOUS are used for unusual or error cases, 
//	which help us to debug SW or HW.
//
//-----------------------------------------------------------------------------
//
//	Never used in a call to ODM_RT_TRACE()!
//
#define ODM_DBG_OFF					1

//
//	Fatal bug. 
//	For example, Tx/Rx/IO locked up, OS hangs, memory access violation, 
//	resource allocation failed, unexpected HW behavior, HW BUG and so on.
//
#define ODM_DBG_SERIOUS				2

//
//	Abnormal, rare, or unexpeted cases.
//	For example, IRP/Packet/OID canceled, device suprisely unremoved and so on.
//
#define ODM_DBG_WARNING				3

//
//	Normal case with useful information about current SW or HW state. 
//	For example, Tx/Rx descriptor to fill, Tx/Rx descriptor completed status, 
//	SW protocol state change, dynamic mechanism state change and so on.
//
#define ODM_DBG_LOUD					4

//
//	Normal case with detail execution flow or information.
//
#define ODM_DBG_TRACE					5

/*FW DBG MSG*/
#define	RATE_DECISION	BIT0
#define	INIT_RA_TABLE	BIT1
#define	RATE_UP			BIT2
#define	RATE_DOWN		BIT3
#define	TRY_DONE		BIT4
#define	F_RATE_AP_RPT	BIT7

//-----------------------------------------------------------------------------
// Define the tracing components
//
//-----------------------------------------------------------------------------
//BB Functions
#define	ODM_COMP_DIG					BIT0	
#define	ODM_COMP_RA_MASK				BIT1	
#define	ODM_COMP_DYNAMIC_TXPWR		BIT2
#define	ODM_COMP_FA_CNT				BIT3
#define	ODM_COMP_RSSI_MONITOR		BIT4
#define	ODM_COMP_CCK_PD				BIT5
#define	ODM_COMP_ANT_DIV				BIT6
#define	ODM_COMP_PWR_SAVE			BIT7
#define	ODM_COMP_PWR_TRAIN			BIT8
#define	ODM_COMP_RATE_ADAPTIVE		BIT9
#define	ODM_COMP_PATH_DIV			BIT10
#define	ODM_COMP_PSD					BIT11
#define	ODM_COMP_DYNAMIC_PRICCA		BIT12
#define	ODM_COMP_RXHP					BIT13
#define	ODM_COMP_MP					BIT14
#define	ODM_COMP_CFO_TRACKING		BIT15
#define	ODM_COMP_ACS					BIT16
#define	PHYDM_COMP_ADAPTIVITY		BIT17
#define	PHYDM_COMP_RA_DBG			BIT18
#define	PHYDM_COMP_TXBF				BIT19
//MAC Functions
#define	ODM_COMP_EDCA_TURBO			BIT20
#define	ODM_COMP_EARLY_MODE			BIT21
#define	ODM_FW_DEBUG_TRACE			BIT22
//RF Functions
#define	ODM_COMP_TX_PWR_TRACK		BIT24
#define	ODM_COMP_RX_GAIN_TRACK		BIT25
#define	ODM_COMP_CALIBRATION			BIT26
//Common Functions
#define	ODM_PHY_CONFIG				BIT28
#define	BEAMFORMING_DEBUG				BIT29
#define	ODM_COMP_COMMON				BIT30
#define	ODM_COMP_INIT					BIT31
#define	ODM_COMP_NOISY_DETECT		BIT32
#define	ODM_COMP_DFS					BIT33

/*------------------------Export Marco Definition---------------------------*/
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	#define RT_PRINTK				DbgPrint
#elif (DM_ODM_SUPPORT_TYPE == ODM_CE)
	#define DbgPrint	printk
	#define RT_PRINTK(fmt, args...)	DbgPrint( "%s(): " fmt, __FUNCTION__, ## args);
	#define	RT_DISP(dbgtype, dbgflag, printstr)
#else
	#define DbgPrint	panic_printk
	#define RT_PRINTK(fmt, args...)	DbgPrint( "%s(): " fmt, __FUNCTION__, ## args);
#endif

#ifndef ASSERT
	#define ASSERT(expr)
#endif

#if DBG
#define ODM_RT_TRACE(pDM_Odm, comp, level, fmt)									\
	do {	\
		if(((comp) & pDM_Odm->DebugComponents) && (level <= pDM_Odm->DebugLevel || level == ODM_DBG_SERIOUS))	\
		{																			\
			if(pDM_Odm->SupportICType == ODM_RTL8192C)								\
				DbgPrint("[ODM-92C] ");												\
			else if(pDM_Odm->SupportICType == ODM_RTL8192D)							\
				DbgPrint("[ODM-92D] ");												\
			else if(pDM_Odm->SupportICType == ODM_RTL8723A)							\
				DbgPrint("[ODM-8723A] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8188E)							\
				DbgPrint("[ODM-8188E] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8192E) 						\
				DbgPrint("[ODM-8192E] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8812)							\
				DbgPrint("[ODM-8812] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8821)							\
				DbgPrint("[ODM-8821] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8814A)							\
				DbgPrint("[ODM-8814] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8703B)							\
				DbgPrint("[ODM-8703B] ");											\
			else if(pDM_Odm->SupportICType == ODM_RTL8822B)							\
				DbgPrint("[ODM-8822] ");											\
			else if (pDM_Odm->SupportICType == ODM_RTL8188F)							\
				DbgPrint("[ODM-8188F] ");											\
			RT_PRINTK fmt;															\
		}	\
	} while (0)
	
#define ODM_RT_TRACE_F(pDM_Odm, comp, level, fmt)									\
		if(((comp) & pDM_Odm->DebugComponents) && (level <= pDM_Odm->DebugLevel))	\
		{																			\
			RT_PRINTK fmt;															\
		}

#define ODM_RT_ASSERT(pDM_Odm, expr, fmt)											\
		if(!(expr)) {																	\
			DbgPrint( "Assertion failed! %s at ......\n", #expr);								\
			DbgPrint( "      ......%s,%s,line=%d\n",__FILE__,__FUNCTION__,__LINE__);			\
			RT_PRINTK fmt;															\
			ASSERT(FALSE);															\
		}
#define ODM_dbg_enter() { DbgPrint("==> %s\n", __FUNCTION__); }
#define ODM_dbg_exit() { DbgPrint("<== %s\n", __FUNCTION__); }
#define ODM_dbg_trace(str) { DbgPrint("%s:%s\n", __FUNCTION__, str); }

#define ODM_PRINT_ADDR(pDM_Odm, comp, level, title_str, ptr)							\
			if(((comp) & pDM_Odm->DebugComponents) && (level <= pDM_Odm->DebugLevel))	\
			{																		\
				int __i;																\
				pu1Byte	__ptr = (pu1Byte)ptr;											\
				DbgPrint("[ODM] ");													\
				DbgPrint(title_str);													\
				DbgPrint(" ");														\
				for( __i=0; __i<6; __i++ )												\
					DbgPrint("%02X%s", __ptr[__i], (__i==5)?"":"-");						\
				DbgPrint("\n");														\
			}
#else
#define ODM_RT_TRACE(pDM_Odm, comp, level, fmt)
#define ODM_RT_TRACE_F(pDM_Odm, comp, level, fmt)
#define ODM_RT_ASSERT(pDM_Odm, expr, fmt)
#define ODM_dbg_enter()
#define ODM_dbg_exit()
#define ODM_dbg_trace(str)
#define ODM_PRINT_ADDR(pDM_Odm, comp, level, title_str, ptr)
#endif


VOID 
PHYDM_InitDebugSetting(IN		PDM_ODM_T		pDM_Odm);

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
VOID phydm_BB_RxHang_Info(IN PDM_ODM_T pDM_Odm);
#endif

#define	BB_TMP_BUF_SIZE		100
VOID phydm_BB_Debug_Info(IN PDM_ODM_T pDM_Odm);
VOID phydm_BasicDbgMessage(	IN		PVOID			pDM_VOID);

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
#define	PHYDM_DBGPRINT		0
#define	PHYDM_SSCANF(x, y, z)	DCMD_Scanf(x, y, z)
#if (PHYDM_DBGPRINT == 1)
#define	PHYDM_SNPRINTF(msg)	\
		do {\
			rsprintf msg;\
			DbgPrint(output);\
		} while (0)
#else
#define	PHYDM_SNPRINTF(msg)	\
		do {\
			rsprintf msg;\
			DCMD_Printf(output);\
		} while (0)
#endif
#else
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
#define	PHYDM_DBGPRINT		0
#else
#define	PHYDM_DBGPRINT		1
#endif
#define	MAX_ARGC				20
#define	MAX_ARGV				16
#define	DCMD_DECIMAL			"%d"
#define	DCMD_CHAR				"%c"
#define	DCMD_HEX				"%x"

#define	PHYDM_SSCANF(x, y, z)	sscanf(x, y, z)
#if (PHYDM_DBGPRINT == 1)
#define	PHYDM_SNPRINTF(msg)\
		do {\
			snprintf msg;\
			DbgPrint(output);\
		} while (0)
#else
#define	PHYDM_SNPRINTF(msg)\
		do {\
			if(out_len > used)\
				used+=snprintf msg;\
		} while (0)
#endif
#endif


VOID phydm_BasicProfile(
	IN		PVOID			pDM_VOID,
	IN		u4Byte			*_used,
	OUT		char				*output,
	IN		u4Byte			*_out_len
	);
#if(DM_ODM_SUPPORT_TYPE & (ODM_CE|ODM_AP))
s4Byte
phydm_cmd(
	IN PDM_ODM_T	pDM_Odm,
	IN char		*input,
	IN u4Byte	in_len,
	IN u1Byte	flag,
	OUT char	*output,
	IN u4Byte	out_len
);
#endif
VOID
phydm_cmd_parser(
	IN PDM_ODM_T	pDM_Odm,
	IN char		input[][16],
	IN u4Byte	input_num,
	IN u1Byte	flag,
	OUT char	*output,
	IN u4Byte	out_len
);

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
void phydm_sbd_check(
	IN	PDM_ODM_T					pDM_Odm
	);

void phydm_sbd_callback(
	PRT_TIMER		pTimer
	);

void phydm_sbd_workitem_callback(
    IN PVOID            pContext
	);
#endif

VOID
phydm_fw_trace_en_h2c(
	IN	PVOID	pDM_VOID,
	IN	BOOLEAN		enable,
	IN	u4Byte		monitor_mode,
	IN	u4Byte		macid
);

VOID
phydm_fw_trace_handler(
	IN		PVOID	pDM_VOID,
	IN		pu1Byte	CmdBuf,
	IN		u1Byte	CmdLen
);

VOID
phydm_fw_trace_handler_code(
	IN	PVOID	pDM_VOID,
	IN	pu1Byte	Buffer,
	IN	u1Byte	CmdLen
);

VOID
phydm_fw_trace_handler_8051(
	IN	PVOID	pDM_VOID,
	IN	pu1Byte	CmdBuf,
	IN	u1Byte	CmdLen
);

#endif	// __ODM_DBG_H__

