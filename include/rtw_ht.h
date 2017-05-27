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
#ifndef _RTW_HT_H_
#define _RTW_HT_H_


struct ht_priv
{
	u8	ht_option;
	u8	ampdu_enable;//for enable Tx A-MPDU
	u8	tx_amsdu_enable;//for enable Tx A-MSDU
	u8	bss_coexist;//for 20/40 Bss coexist

	//u8	baddbareq_issued[16];
	u32	tx_amsdu_maxlen; // 1: 8k, 0:4k ; default:8k, for tx
	u32	rx_ampdu_maxlen; //for rx reordering ctrl win_sz, updated when join_callback.
	
	u8 	rx_ampdu_min_spacing;
	
	u8	ch_offset;//PRIME_CHNL_OFFSET
	u8	sgi_20m;
	u8	sgi_40m;

	//for processing Tx A-MPDU
	u8	agg_enable_bitmap;
	//u8	ADDBA_retry_count;
	u8	candidate_tid_bitmap;

	u8	ldpc_cap;
	u8	stbc_cap;
	u8	beamform_cap;
	u8	smps_cap; /*spatial multiplexing power save mode. 0:static SMPS, 1:dynamic SMPS, 3:SMPS disabled, 2:reserved*/

	struct rtw_ieee80211_ht_cap ht_cap;
	
};

typedef enum AGGRE_SIZE{
	HT_AGG_SIZE_8K = 0,
	HT_AGG_SIZE_16K = 1,
	HT_AGG_SIZE_32K = 2,
	HT_AGG_SIZE_64K = 3,
	VHT_AGG_SIZE_128K = 4,
	VHT_AGG_SIZE_256K = 5,
	VHT_AGG_SIZE_512K = 6,
	VHT_AGG_SIZE_1024K = 7,
}AGGRE_SIZE_E, *PAGGRE_SIZE_E;

typedef enum _RT_HT_INF0_CAP{
	RT_HT_CAP_USE_TURBO_AGGR = 0x01,
	RT_HT_CAP_USE_LONG_PREAMBLE = 0x02,
	RT_HT_CAP_USE_AMPDU = 0x04,
	RT_HT_CAP_USE_WOW = 0x8,	
	RT_HT_CAP_USE_SOFTAP = 0x10,	
	RT_HT_CAP_USE_92SE = 0x20,
	RT_HT_CAP_USE_88C_92C = 0x40,
	RT_HT_CAP_USE_AP_CLIENT_MODE = 0x80,	// AP team request to reserve this bit, by Emily
}RT_HT_INF0_CAPBILITY, *PRT_HT_INF0_CAPBILITY;

typedef enum _RT_HT_INF1_CAP{
	RT_HT_CAP_USE_VIDEO_CLIENT = 0x01,
	RT_HT_CAP_USE_JAGUAR_BCUT = 0x02,
	RT_HT_CAP_USE_JAGUAR_CCUT = 0x04,
}RT_HT_INF1_CAPBILITY, *PRT_HT_INF1_CAPBILITY;

#define	LDPC_HT_ENABLE_RX			BIT0
#define	LDPC_HT_ENABLE_TX			BIT1
#define	LDPC_HT_TEST_TX_ENABLE		BIT2
#define	LDPC_HT_CAP_TX				BIT3

#define	STBC_HT_ENABLE_RX			BIT0
#define	STBC_HT_ENABLE_TX			BIT1
#define	STBC_HT_TEST_TX_ENABLE		BIT2
#define	STBC_HT_CAP_TX				BIT3

#define	BEAMFORMING_HT_BEAMFORMER_ENABLE	BIT0	// Declare our NIC supports beamformer
#define	BEAMFORMING_HT_BEAMFORMEE_ENABLE	BIT1	// Declare our NIC supports beamformee
#define	BEAMFORMING_HT_BEAMFORMER_TEST		BIT2	// Transmiting Beamforming no matter the target supports it or not
#define	BEAMFORMING_HT_BEAMFORMER_STEER_NUM		(BIT4|BIT5)
#define	BEAMFORMING_HT_BEAMFORMEE_CHNL_EST_CAP	(BIT6|BIT7)

//------------------------------------------------------------
// The HT Control field
//------------------------------------------------------------
#define SET_HT_CTRL_CSI_STEERING(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart))+2, 6, 2, _val)
#define SET_HT_CTRL_NDP_ANNOUNCEMENT(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart))+3, 0, 1, _val)
#define GET_HT_CTRL_NDP_ANNOUNCEMENT(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+3, 0, 1)

// 20/40 BSS Coexist
#define SET_EXT_CAPABILITY_ELE_BSS_COEXIST(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 0, 1, _val)
#define GET_EXT_CAPABILITY_ELE_BSS_COEXIST(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 0, 1)

/* HT Capabilities Info field */
#define HT_CAP_ELE_CAP_INFO(_pEleStart)					((u8 *)(_pEleStart))
#define GET_HT_CAP_ELE_LDPC_CAP(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 0, 1)
#define GET_HT_CAP_ELE_CHL_WIDTH(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 1, 1)
#define GET_HT_CAP_ELE_SM_PS(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 2, 2)
#define GET_HT_CAP_ELE_GREENFIELD(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 4, 1)
#define GET_HT_CAP_ELE_SHORT_GI20M(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 5, 1)
#define GET_HT_CAP_ELE_SHORT_GI40M(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 6, 1)
#define GET_HT_CAP_ELE_TX_STBC(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 7, 1)
#define GET_HT_CAP_ELE_RX_STBC(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 0, 2)
#define GET_HT_CAP_ELE_DELAYED_BA(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 2, 1)
#define GET_HT_CAP_ELE_MAX_AMSDU_LENGTH(_pEleStart)		LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 3, 1)
#define GET_HT_CAP_ELE_DSSS_CCK_40M(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 4, 1)
#define GET_HT_CAP_ELE_FORTY_INTOLERANT(_pEleStart)		LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 6, 1)
#define GET_HT_CAP_ELE_LSIG_TXOP_PROTECT(_pEleStart)	LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+1, 7, 1)

#define SET_HT_CAP_ELE_LDPC_CAP(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 0, 1, _val)
#define SET_HT_CAP_ELE_CHL_WIDTH(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 1, 1, _val)
#define SET_HT_CAP_ELE_SM_PS(_pEleStart, _val)				SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 2, 2, _val)
#define SET_HT_CAP_ELE_GREENFIELD(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 4, 1, _val)
#define SET_HT_CAP_ELE_SHORT_GI20M(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 5, 1, _val)
#define SET_HT_CAP_ELE_SHORT_GI40M(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 6, 1, _val)
#define SET_HT_CAP_ELE_TX_STBC(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 7, 1, _val)
#define SET_HT_CAP_ELE_RX_STBC(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 0, 2, _val)
#define SET_HT_CAP_ELE_DELAYED_BA(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 2, 1, _val)
#define SET_HT_CAP_ELE_MAX_AMSDU_LENGTH(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 3, 1, _val)
#define SET_HT_CAP_ELE_DSSS_CCK_40M(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 4, 1, _val)
#define SET_HT_CAP_ELE_FORTY_INTOLERANT(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 6, 1, _val)
#define SET_HT_CAP_ELE_LSIG_TXOP_PROTECT(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 7, 1, _val)

/* A-MPDU Parameters field */
#define HT_CAP_ELE_AMPDU_PARA(_pEleStart)				(((u8 *)(_pEleStart))+2)
#define GET_HT_CAP_ELE_MAX_AMPDU_LEN_EXP(_pEleStart)	LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+2, 0, 2)
#define GET_HT_CAP_ELE_MIN_MPDU_S_SPACE(_pEleStart)		LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+2, 2, 3)

#define HT_AMPDU_PARA_FMT "%02x " \
	"MAX AMPDU len:%u bytes, MIN MPDU Start Spacing:%u"

#define HT_AMPDU_PARA_ARG(x) \
	*((u8*)(x)) \
	, (1 << (13+GET_HT_CAP_ELE_MAX_AMPDU_LEN_EXP(((u8*)x)-2)))-1 \
	, GET_HT_CAP_ELE_MIN_MPDU_S_SPACE(((u8*)x)-2)

/* Supported MCS Set field */
#define HT_CAP_ELE_SUP_MCS_SET(_pEleStart)				(((u8 *)(_pEleStart))+3)
#define HT_CAP_ELE_RX_MCS_MAP(_pEleStart)				HT_CAP_ELE_SUP_MCS_SET(_pEleStart)
#define GET_HT_CAP_ELE_RX_HIGHEST_DATA_RATE(_pEleStart)	LE_BITS_TO_2BYTE(((u8 *)(_pEleStart))+13, 0, 10)
#define GET_HT_CAP_ELE_TX_MCS_DEF(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+15, 0, 1)
#define GET_HT_CAP_ELE_TRX_MCS_NEQ(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+15, 1, 1)
#define GET_HT_CAP_ELE_TX_MAX_SS(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+15, 2, 2)
#define GET_HT_CAP_ELE_TX_UEQM(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart))+15, 4, 1)

#define HT_SUP_MCS_SET_FMT "%02x %02x %02x %02x %02x%02x%02x%02x%02x%02x" \
	/* "\n%02x%02x%02x%02x%02x%02x" */\
	" %uMbps %s%s%s"
#define HT_SUP_MCS_SET_ARG(x) ((u8 *)(x))[0], ((u8 *)(x))[1], ((u8 *)(x))[2], ((u8 *)(x))[3], ((u8 *)(x))[4], ((u8 *)(x))[5], \
	((u8 *)(x))[6], ((u8 *)(x))[7], ((u8 *)(x))[8], ((u8 *)(x))[9] \
	/*,((u8 *)(x))[10],((u8 *)(x))[11], ((u8 *)(x))[12],((u8 *)(x))[13],((u8 *)(x))[14],((u8 *)(x))[15] */\
	, GET_HT_CAP_ELE_RX_HIGHEST_DATA_RATE(((u8 *)x)-3) \
	, GET_HT_CAP_ELE_TX_MCS_DEF(((u8 *)x)-3) ? "TX_MCS_DEF " : "" \
	, GET_HT_CAP_ELE_TRX_MCS_NEQ(((u8 *)x)-3) ? "TRX_MCS_NEQ " : "" \
	, GET_HT_CAP_ELE_TX_UEQM(((u8 *)x)-3) ? "TX_UEQM " : ""

//TXBF Capabilities
#define SET_HT_CAP_TXBF_RECEIVE_NDP_CAP(_pEleStart, _val)				SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 3, 1, ((u8)_val))
#define SET_HT_CAP_TXBF_TRANSMIT_NDP_CAP(_pEleStart, _val)				SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 4, 1, ((u8)_val))
#define SET_HT_CAP_TXBF_EXPLICIT_COMP_STEERING_CAP(_pEleStart, _val)	SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 10, 1, ((u8)_val))
#define SET_HT_CAP_TXBF_EXPLICIT_COMP_FEEDBACK_CAP(_pEleStart, _val)	SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 15, 2, ((u8)_val))
#define SET_HT_CAP_TXBF_COMP_STEERING_NUM_ANTENNAS(_pEleStart, _val)	SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 23, 2, ((u8)_val))
#define SET_HT_CAP_TXBF_CHNL_ESTIMATION_NUM_ANTENNAS(_pEleStart, _val)	SET_BITS_TO_LE_4BYTE(((u8 *)(_pEleStart))+21, 27, 2, ((u8)_val))


#define GET_HT_CAP_TXBF_EXPLICIT_COMP_STEERING_CAP(_pEleStart)			LE_BITS_TO_4BYTE(((u8 *)(_pEleStart))+21, 10, 1)
#define GET_HT_CAP_TXBF_EXPLICIT_COMP_FEEDBACK_CAP(_pEleStart)			LE_BITS_TO_4BYTE(((u8 *)(_pEleStart))+21, 15, 2)
#define GET_HT_CAP_TXBF_COMP_STEERING_NUM_ANTENNAS(_pEleStart)		LE_BITS_TO_4BYTE(((u8 *)(_pEleStart))+21, 23, 2)
#define GET_HT_CAP_TXBF_CHNL_ESTIMATION_NUM_ANTENNAS(_pEleStart)		LE_BITS_TO_4BYTE(((u8 *)(_pEleStart))+21, 27, 2)

/* HT Operation element */

#define GET_HT_OP_ELE_PRI_CHL(_pEleStart)					LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)), 0, 8)
#define SET_HT_OP_ELE_PRI_CHL(_pEleStart, _val)				SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)), 0, 8, _val)

/* HT Operation Info field */
#define HT_OP_ELE_OP_INFO(_pEleStart)						(((u8 *)(_pEleStart)) + 1)
#define GET_HT_OP_ELE_2ND_CHL_OFFSET(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 1, 0, 2)
#define GET_HT_OP_ELE_STA_CHL_WIDTH(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 1, 2, 1)
#define GET_HT_OP_ELE_RIFS_MODE(_pEleStart)					LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 1, 3, 1)
#define GET_HT_OP_ELE_HT_PROTECT(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 2, 0, 2)
#define GET_HT_OP_ELE_NON_GREEN_PRESENT(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 2, 2, 1)
#define GET_HT_OP_ELE_OBSS_NON_HT_PRESENT(_pEleStart)		LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 2, 4, 1)
#define GET_HT_OP_ELE_DUAL_BEACON(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 4, 6, 1)
#define GET_HT_OP_ELE_DUAL_CTS(_pEleStart)					LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 4, 7, 1)
#define GET_HT_OP_ELE_STBC_BEACON(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 5, 0, 1)
#define GET_HT_OP_ELE_LSIG_TXOP_PROTECT(_pEleStart)			LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 5, 1, 1)
#define GET_HT_OP_ELE_PCO_ACTIVE(_pEleStart)				LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 5, 2, 1)
#define GET_HT_OP_ELE_PCO_PHASE(_pEleStart)					LE_BITS_TO_1BYTE(((u8 *)(_pEleStart)) + 5, 3, 1)

#define SET_HT_OP_ELE_2ND_CHL_OFFSET(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 0, 2, _val)
#define SET_HT_OP_ELE_STA_CHL_WIDTH(_pEleStart, _val)		SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 2, 1, _val)
#define SET_HT_OP_ELE_RIFS_MODE(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 1, 3, 1, _val)
#define SET_HT_OP_ELE_HT_PROTECT(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 2, 0, 2, _val)
#define SET_HT_OP_ELE_NON_GREEN_PRESENT(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 2, 2, 1, _val)
#define SET_HT_OP_ELE_OBSS_NON_HT_PRESENT(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 2, 4, 1, _val)
#define SET_HT_OP_ELE_DUAL_BEACON(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 4, 6, 1, _val)
#define SET_HT_OP_ELE_DUAL_CTS(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 4, 7, 1, _val)
#define SET_HT_OP_ELE_STBC_BEACON(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 5, 0, 1, _val)
#define SET_HT_OP_ELE_LSIG_TXOP_PROTECT(_pEleStart, _val)	SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 5, 1, 1, _val)
#define SET_HT_OP_ELE_PCO_ACTIVE(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 5, 2, 1, _val)
#define SET_HT_OP_ELE_PCO_PHASE(_pEleStart, _val)			SET_BITS_TO_LE_1BYTE(((u8 *)(_pEleStart)) + 5, 3, 1, _val)

#endif	//_RTL871X_HT_H_

