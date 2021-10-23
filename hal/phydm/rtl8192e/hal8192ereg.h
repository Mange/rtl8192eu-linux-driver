/******************************************************************************
 *
 * Copyright(c) 2016 - 2017 Realtek Corporation.
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
 *****************************************************************************/
/*****************************************************************************
 *	Copyright(c) 2009,  RealTEK Technology Inc. All Right Reserved.
 *
 * Module:	__INC_HAL8192EREG_H
 *
 *
 * Note:	1. Define Mac register address and corresponding bit mask map
 *
 *
 * Export:	Constants, macro, functions(API), global variables(None).
 *
 * Abbrev:
 *
 * History:
 *		data		Who		Remark
 *
 *****************************************************************************/
#ifndef __INC_HAL8192EREG_H
#define __INC_HAL8192EREG_H

/* ************************************************************
 *
 * ************************************************************ */

/* -----------------------------------------------------
 *
 *	0x0000h ~ 0x00FFh	System Configuration
 *
 * ----------------------------------------------------- */
#define REG_SYS_ISO_CTRL_8192E 0x0000 /* 2 Byte */
#define REG_SYS_FUNC_EN_8192E 0x0002 /* 2 Byte */
#define REG_SYS_PW_CTRL_8192E 0x0004 /* 4 Byte */
#define REG_SYS_CLKR_8192E 0x0008 /* 2 Byte */
#define REG_SYS_EEPROM_CTRL_8192E 0x000A /* 2 Byte */
#define REG_EE_VPD_8192E 0x000C /* 2 Byte */
#define REG_SYS_SWR_CTRL1_8192E 0x0010 /* 1 Byte */
#define REG_SYS_SWR_CTRL2_8192E 0x0014 /* 1 Byte */
#define REG_SYS_SWR_CTRL3_8192E 0x0018 /* 4 Byte */
#define REG_RSV_CTRL_8192E 0x001C /* 3 Byte */
#define REG_RF_CTRL_8192E 0x001F /* 1 Byte */
#define REG_AFE_CTRL1_8192E 0x0024 /* 4 Byte */
#define REG_AFE_CTRL2_8192E 0x0028 /* 4 Byte */
#define REG_AFE_CTRL3_8192E 0x002c /* 4 Byte */
#define REG_EFUSE_CTRL_8192E 0x0030
#define REG_LDO_EFUSE_CTRL_8192E 0x0034
#define REG_PWR_DATA_8192E 0x0038
#define REG_CAL_TIMER_8192E 0x003C
#define REG_ACLK_MON_8192E 0x003E
#define REG_GPIO_MUXCFG_8192E 0x0040
#define REG_GPIO_IO_SEL_8192E 0x0042
#define REG_MAC_PINMUX_CFG_8192E 0x0043 /* ?????? */
#define REG_GPIO_PIN_CTRL_8192E 0x0044
#define REG_GPIO_INTM_8192E 0x0048
#define BIT_REG_LED_CFG_8192E 0x004C
#define REG_LEDCFG2_8192E 0x004E /* ?????? */
#define REG_FSIMR_8192E 0x0050
#define REG_FSISR_8192E 0x0054
#define REG_HSIMR_8192E 0x0058
#define REG_HSISR_8192E 0x005c
#define REG_PAD_CTRL1_8192E 0x0064
#define REG_WL_BT_PWR_CTRL_8192E 0x0068
#define REG_SDM_DEBUG_8192E 0x006C
#define REG_SDIO_CTRL_8192E 0x0070
#define REG_HCI_OPT_CTRL_8192E 0x0074
#define REG_AFE_CTRL4_8192E 0x0078
#define REG_8051FW_CTRL_8192E 0x0080
#define REG_WLLPS_CTRL_8192E 0x0090
#define REG_HIMR0_8192E 0x00B0
#define REG_HISR0_8192E 0x00B4
#define REG_HIMR1_8192E 0x00B8
#define REG_HISR1_8192E 0x00BC
#define REG_PMC_DBG_CTRL2_8192E 0x00CC
#define REG_EFUSE_BURN_GNT_8192E 0x00CF
#define REG_SYS_CFG1_8192E 0x00F0
#define REG_SYS_CFG2_8192E 0x00FC

/* -----------------------------------------------------
 *
 *	0x0100h ~ 0x01FFh	MACTOP General Configuration
 *
 * ----------------------------------------------------- */
#define REG_CR_8192E 0x0100
#define REG_PBP_8192E 0x0104 /* ?????? */
#define REG_PKT_BUFF_ACCESS_CTRL_8192E 0x0106 /* ?????? */
#define REG_TRXDMA_CTRL_8192E 0x010C
#define REG_TRXFF_BNDY_8192E 0x0114
#define REG_RXFF_PTR_8192E 0x011C
#define REG_CPWM_8192E 0x012C
#define REG_FWIMR_8192E 0x0130
#define REG_FWISR_8192E 0x0134
#define REG_FTIMR_8192E 0x0138
#define REG_PKTBUF_DBG_CTRL_8192E 0x0140
#define REG_RXPKTBUF_CTRL_8192E 0x0142 /* ?????? */
#define REG_PKTBUF_DBG_DATA_L_8192E 0x0144
#define REG_PKTBUF_DBG_DATA_H_8192E 0x0148

#define REG_TC0_CTRL_8192E 0x0150
#define REG_TC1_CTRL_8192E 0x0154
#define REG_TC2_CTRL_8192E 0x0158
#define REG_TC3_CTRL_8192E 0x015C
#define REG_TC4_CTRL_8192E 0x0160
#define REG_TCUNIT_BASE_8192E 0x0164
#define REG_RSVD3_8192E 0x0168 /* ????? */

#define REG_C2HEVT_MSG_NORMAL_8192E 0x01A0 /* ?????? */
#define REG_C2HEVT_CMD_SEQ_88XX 0x01A1 /* ?????? */
#define reg_c2h_evt_cmd_content_88xx 0x01A2 /* ?????? */
#define REG_C2HEVT_CMD_LEN_88XX 0x01AE /* ?????? */
#define REG_C2HEVT_CLEAR_8192E 0x01AF /* ?????? */
#define REG_MCUTST_1_8192E 0x01C0
#define REG_MCUTST_2_8192E 0x01C4
#define REG_MCUTST_WOWLAN_8192E 0x01C7 /* ?????? */
#define REG_FMETHR_8192E 0x01C8
#define REG_HMETFR_8192E 0x01CC
#define REG_HMEBOX_0_8192E 0x01D0
#define REG_HMEBOX_1_8192E 0x01D4
#define REG_HMEBOX_2_8192E 0x01D8
#define REG_HMEBOX_3_8192E 0x01DC
#define REG_LLT_INIT_8192E 0x01E0
#define REG_HMEBOX_EXT0_8192E 0x01F0 /* ?????? */
#define REG_HMEBOX_EXT1_8192E 0x01F4 /* ?????? */
#define REG_HMEBOX_EXT2_8192E 0x01F8 /* ?????? */
#define REG_HMEBOX_EXT3_8192E 0x01FC /* ?????? */

/* -----------------------------------------------------
 *
 *	0x0200h ~ 0x027Fh	TXDMA Configuration
 *
 * ----------------------------------------------------- */
#define REG_RQPN_8192E 0x0200
#define REG_FIFOPAGE_8192E 0x0204
#define REG_DWBCN0_CTRL_8192E 0x0208
#define REG_TXDMA_OFFSET_CHK_8192E 0x020C
#define REG_TXDMA_STATUS_8192E 0x0210
#define REG_RQPN_NPQ_8192E 0x0214
#define REG_AUTO_LLT_8192E 0x0224
#define REG_DWBCN1_CTRL_8192E 0x0228

/* -----------------------------------------------------
 *
 *	0x0280h ~ 0x02FFh	RXDMA Configuration
 *
 * ----------------------------------------------------- */
#define REG_RXDMA_AGG_PG_TH_8192E 0x0280
#define REG_RXPKT_NUM_8192E 0x0284 /* The number of packets in RXPKTBUF. */
#define REG_RXDMA_CONTROL_8192E 0x0286 /* ?????? Control the RX DMA. */
#define REG_RXDMA_STATUS_8192E 0x0288
#define REG_RXDMA_PRO_8192E 0x0290 /* ?????? */
#define REG_EARLY_MODE_CONTROL_8192E 0x02BC /* ?????? */
#define REG_RSVD5_8192E 0x02F0 /* ?????? */

/* -----------------------------------------------------
 *
 *	0x0300h ~ 0x03FFh	PCIe
 *
 * ----------------------------------------------------- */
#define REG_PCIE_CTRL_REG_8192E 0x0300
#define REG_INT_MIG_8192E 0x0304 /* Interrupt Migration */
#define REG_BCNQ_TXBD_DESA_8192E 0x0308 /* TX Beacon Descriptor Address */
#define REG_MGQ_TXBD_DESA_8192E 0x0310 /* TX Manage Queue Descriptor Address */
#define REG_VOQ_TXBD_DESA_8192E 0x0318 /* TX VO Queue Descriptor Address */
#define REG_VIQ_TXBD_DESA_8192E 0x0320 /* TX VI Queue Descriptor Address */
#define REG_BEQ_TXBD_DESA_8192E 0x0328 /* TX BE Queue Descriptor Address */
#define REG_BKQ_TXBD_DESA_8192E 0x0330 /* TX BK Queue Descriptor Address */
#define REG_RXQ_RXBD_DESA_8192E 0x0338 /* RX Queue	Descriptor Address */
#define REG_HI0Q_TXBD_DESA_8192E 0x0340
#define REG_HI1Q_TXBD_DESA_8192E 0x0348
#define REG_HI2Q_TXBD_DESA_8192E 0x0350
#define REG_HI3Q_TXBD_DESA_8192E 0x0358
#define REG_HI4Q_TXBD_DESA_8192E 0x0360
#define REG_HI5Q_TXBD_DESA_8192E 0x0368
#define REG_HI6Q_TXBD_DESA_8192E 0x0370
#define REG_HI7Q_TXBD_DESA_8192E 0x0378
#define REG_MGQ_TXBD_NUM_8192E 0x0380
#define REG_RX_RXBD_NUM_8192E 0x0382
#define REG_VOQ_TXBD_NUM_8192E 0x0384
#define REG_VIQ_TXBD_NUM_8192E 0x0386
#define REG_BEQ_TXBD_NUM_8192E 0x0388
#define REG_BKQ_TXBD_NUM_8192E 0x038A
#define REG_HI0Q_TXBD_NUM_8192E 0x038C
#define REG_HI1Q_TXBD_NUM_8192E 0x038E
#define REG_HI2Q_TXBD_NUM_8192E 0x0390
#define REG_HI3Q_TXBD_NUM_8192E 0x0392
#define REG_HI4Q_TXBD_NUM_8192E 0x0394
#define REG_HI5Q_TXBD_NUM_8192E 0x0396
#define REG_HI6Q_TXBD_NUM_8192E 0x0398
#define REG_HI7Q_TXBD_NUM_8192E 0x039A
#define REG_TSFTIMER_HCI_8192E 0x039C

/* Read Write Point */
#define REG_VOQ_TXBD_IDX_8192E 0x03A0
#define REG_VIQ_TXBD_IDX_8192E 0x03A4
#define REG_BEQ_TXBD_IDX_8192E 0x03A8
#define REG_BKQ_TXBD_IDX_8192E 0x03AC
#define REG_MGQ_TXBD_IDX_8192E 0x03B0
#define REG_RXQ_TXBD_IDX_8192E 0x03B4
#define REG_HI0Q_TXBD_IDX_8192E 0x03B8
#define REG_HI1Q_TXBD_IDX_8192E 0x03BC
#define REG_HI2Q_TXBD_IDX_8192E 0x03C0
#define REG_HI3Q_TXBD_IDX_8192E 0x03C4
#define REG_HI4Q_TXBD_IDX_8192E 0x03C8
#define REG_HI5Q_TXBD_IDX_8192E 0x03CC
#define REG_HI6Q_TXBD_IDX_8192E 0x03D0
#define REG_HI7Q_TXBD_IDX_8192E 0x03D4

#define REG_PCIE_HCPWM_8192EE 0x03D8 /* ?????? */
#define REG_PCIE_HRPWM_8192EE 0x03DC /* PCIe RPWM */ /* ?????? */
#define REG_DBI_WDATA_V1_8192E 0x03E8
#define REG_DBI_RDATA_V1_8192E 0x03EC
#define REG_DBI_FLAG_V1_8192E 0x03F0
#define REG_MDIO_V1_8192E 0x3F4
#define REG_PCIE_MIX_CFG_8192E 0x3F8

/* -----------------------------------------------------
 *
 *	0x0400h ~ 0x047Fh	Protocol Configuration
 *
 * ----------------------------------------------------- */
#define REG_TXPKT_EMPTY_8192E 0x041A
#define REG_FWHW_TXQ_CTRL_8192E 0x0420
#define REG_HWSEQ_CTRL_8192E 0x0423
#define REG_BCNQ_BDNY_8192E 0x0424
#define REG_MGQ_BDNY_8192E 0x0425
#define REG_LIFETIME_EN_8192E 0x0426
#define REG_FW_FREE_TAIL_8192E 0x0427
#define REG_SPEC_SIFS_8192E 0x0428
#define REG_RETRY_LIMIT_8192E 0x042A
#define REG_TXBF_CTRL_8192E 0x042C
#define REG_DARFRC_8192E 0x0430
#define REG_RARFRC_8192E 0x0438
#define REG_RRSR_8192E 0x0440
#define REG_ARFR0_8192E 0x0444
#define REG_ARFR1_8192E 0x044C
#define REG_CCK_CHECK_8192E 0x0454
#define REG_AMPDU_MAX_TIME_8192E 0x0456
#define REG_BCNQ1_BDNY_8192E 0x0457
#define REG_AMPDU_MAX_LENGTH_8192E 0x0458
#define REG_WMAC_LBK_BUF_HD_8192E 0x045D
#define REG_NDPA_OPT_CTRL_8192E 0x045F
#define REG_FAST_EDCA_CTRL_8192E 0x0460
#define REG_RD_RESP_PKT_TH_8192E 0x0463
#define REG_DATA_SC_8192E 0x0483
#define REG_TXRPT_START_OFFSET 0x04AC
#define REG_POWER_STAGE1_8192E 0x04B4
#define REG_SW_AMPDU_BURST_MODE_CTRL_8192E 0x04BC
#define REG_PKT_LIFE_TIME_8192E 0x04C0
#define REG_PKT_BE_BK_LIFE_TIME_8192E 0x04C2 /* ?????? */

#define REG_STBC_SETTING_8192E 0x04C4
#define REG_PROT_MODE_CTRL_8192E 0x04C8
#define REG_MAX_AGGR_NUM_8192E 0x04CA
#define REG_RTS_MAX_AGGR_NUM_8192E 0x04CB
#define REG_BAR_MODE_CTRL_8192E 0x04CC
#define REG_RA_TRY_RATE_AGG_LMT_8192E 0x04CF
#define REG_MACID_SLEEP2_8192E 0x04D0
#define REG_MACID_SLEEP_8192E 0x04D4
#define REG_HW_SEQ0_8192E 0x04D8
#define REG_HW_SEQ1_8192E 0x04DA
#define REG_HW_SEQ2_8192E 0x04DC
#define REG_HW_SEQ3_8192E 0x04DE
#define REG_TXPKTBUF_WMAC_LBK_BF_HD_8192E 0x045D /* ?????? */

/* -----------------------------------------------------
 *
 *	0x0500h ~ 0x05FFh	EDCA Configuration
 *
 * -----------------------------------------------------
 * gogogo */
#define REG_EDCA_VO_PARAM_8192E 0x0500
#define REG_EDCA_VI_PARAM_8192E 0x0504
#define REG_EDCA_BE_PARAM_8192E 0x0508
#define REG_EDCA_BK_PARAM_8192E 0x050C
#define REG_BCNTCFG_8192E 0x0510
#define REG_PIFS_8192E 0x0512
#define REG_RDG_PIFS_8192E 0x0513
#define REG_SIFS_CTX_8192E 0x0514
#define REG_SIFS_TRX_8192E 0x0516
#define REG_AGGR_BREAK_TIME_8192E 0x051A
#define REG_SLOT_8192E 0x051B
#define REG_TX_PTCL_CTRL_8192E 0x0520
#define REG_TXPAUSE_8192E 0x0522
#define REG_DIS_TXREQ_CLR_8192E 0x0523
#define REG_RD_CTRL_8192E 0x0524
/*
 * Format for offset 540h-542h:
 *	[3:0]:   TBTT prohibit setup in unit of 32us. The time for HW getting beacon content before TBTT.
 *	[7:4]:   Reserved.
 *	[19:8]:  TBTT prohibit hold in unit of 32us. The time for HW holding to send the beacon packet.
 *	[23:20]: Reserved
 * Description:
 *	              |
 * |<--Setup--|--Hold------------>|
 *	--------------|----------------------
 * |
 * TBTT
 * Note: We cannot update beacon content to HW or send any AC packets during the time between Setup and Hold.
 * Described by Designer Tim and Bruce, 2011-01-14.
 *   */
#define REG_TBTT_PROHIBIT_8192E 0x0540
#define REG_RD_NAV_NXT_8192E 0x0544
#define REG_NAV_PROT_LEN_8192E 0x0546
#define REG_BCN_CTRL_8192E 0x0550
#define REG_BCN_CTRL_1_8192E 0x0551
#define REG_MBID_NUM_8192E 0x0552
#define REG_DUAL_TSF_RST_8192E 0x0553
#define REG_MBSSID_BCN_SPACE_8192E 0x0554
#define REG_DRVERLYINT_8192E 0x0558
#define REG_BCNDMATIM_8192E 0x0559
#define REG_ATIMWND_8192E 0x055A
#define REG_USTIME_TSF_8192E 0x055C
#define REG_BCN_MAX_ERR_8192E 0x055D
#define REG_RXTSF_OFFSET_CCK_8192E 0x055E
#define REG_RXTSF_OFFSET_OFDM_8192E 0x055F
#define REG_TSFTR_8192E 0x0560
#define REG_CTWND_8192E 0x0572
#define REG_SECONDARY_CCA_CTRL_8192E 0x0577 /* ?????? */
#define REG_PSTIMER_8192E 0x0580
#define REG_TIMER0_8192E 0x0584
#define REG_TIMER1_8192E 0x0588
#define REG_ACMHWCTRL_8192E 0x05C0
#define REG_SCH_TXCMD_8192E 0x05F8

/* -----------------------------------------------------
 *
 *	0x0600h ~ 0x07FFh	WMAC Configuration
 *
 * -----------------------------------------------------
 * gogogo */
#define REG_MAC_CR_8192E 0x0600
#define REG_TCR_8192E 0x0604
#define REG_RCR_8192E 0x0608
#define REG_RX_PKT_LIMIT_8192E 0x060C
#define REG_RX_DLK_TIME_8192E 0x060D
#define REG_RX_DRVINFO_SZ_8192E 0x060F

#define REG_MACID_8192E 0x0610
#define REG_BSSID_8192E 0x0618
#define REG_MAR_8192E 0x0620
#define REG_MBIDCAMCFG_8192E 0x0628

#define REG_USTIME_EDCA_8192E 0x0638
#define REG_MAC_SPEC_SIFS_8192E 0x063A
#define REG_RESP_SIFP_CCK_8192E 0x063C
#define REG_RESP_SIFS_OFDM_8192E 0x063E
#define REG_ACKTO_8192E 0x0640
#define REG_CTS2TO_8192E 0x0641
#define REG_EIFS_8192E 0x0642

#define REG_NAV_UPPER_8192E 0x0652 /* ?????? */
#define REG_TRXPTCL_CTL_8192E 0x0668

/* security */
#define REG_CAMCMD_8192E 0x0670
#define REG_CAMWRITE_8192E 0x0674
#define REG_CAMREAD_8192E 0x0678
#define REG_CAMDBG_8192E 0x067C
#define REG_SECCFG_8192E 0x0680

/* Power */
#define REG_WOW_CTRL_8192E 0x0690
#define REG_PS_RX_INFO_8192E 0x0692
#define REG_UAPSD_TID_8192E 0x0693
#define REG_WKFMCAM_NUM_8192E 0x0698
#define REG_RXFLTMAP0_8192E 0x06A0
#define REG_RXFLTMAP1_8192E 0x06A2
#define REG_RXFLTMAP2_8192E 0x06A4
#define REG_BCN_PSR_RPT_8192E 0x06A8
#define REG_BT_COEX_TABLE0_8192E 0x06C0
#define REG_BT_COEX_TABLE1_8192E 0x06C4
#define REG_BT_COEX_TABLE2_8192E 0x06C8
#define REG_BT_COEX_TABLE3_8192E 0x06CC
#define REG_ASSOCIATED_BFMER0_INFO_8192E 0x06E4
#define REG_ASSOCIATED_BFMER1_INFO_8192E 0x06EC
#define REG_CSI_RPT_PARAM_BW20_8192E 0x06F4
#define REG_CSI_RPT_PARAM_BW40_8192E 0x06F8
#define REG_CSI_RPT_PARAM_BW80_8192E 0x06FC

/* Hardware Port 2 */
#define REG_MACID1_8192E 0x0700
#define REG_BSSID1_8192E 0x0708
#define REG_ASSOCIATED_BFMEE_SEL_8192E 0x0714
#define REG_SND_PTCL_CTRL_8192E 0x0718

/* BT */
#define REG_BT_STATISTICS_CTRL_8192E 0x076E /* bit0~bit3 for REG_BT_STATISTICS_CTRL , bit4~bit15 for REG_BT_COEX_ENH_INTF_CTRL */
#define REG_BT_STATISTICS_OTH_CTRL_8192E 0x0778
#define REG_TDMA_TIME_AND_RPT_SAM_SET_8192E 0x0790

/* -----------------------------------------------------
 *
 *	Redifine 8192C register definition for compatibility
 *
 * ----------------------------------------------------- */

/* TODO: use these definition when using REG_xxx naming rule.
 * NOTE: DO NOT Remove these definition. Use later. */
#define EFUSE_CTRL_8192E REG_EFUSE_CTRL_8192E /* E-Fuse Control. */
#define EFUSE_TEST_8192E REG_LDO_EFUSE_CTRL_8192E /* E-Fuse Test. */
#define MSR_8192E (REG_CR_8192E + 2) /* Media status register */
#define ISR_8192E REG_HISR0_8192E
#define TSFR_8192E REG_TSFTR_8192E /* Timing Sync Function Timer Register. */

#define PBP_8192E REG_PBP_8192E

/* Redifine MACID register, to compatible prior ICs. */
#define IDR0_8192E REG_MACID_8192E /* MAC ID Register, Offset 0x0050-0x0053 */
#define IDR4_8192E (REG_MACID_8192E + 4) /* MAC ID Register, Offset 0x0054-0x0055 */

/*
 * 9. security Control Registers	(Offset: )
 *   */
#define RWCAM_8192E REG_CAMCMD_8192E /* 8190 data Sheet is called CAMcmd */
#define WCAMI_8192E REG_CAMWRITE_8192E /* Software write CAM input content */
#define RCAMO_8192E REG_CAMREAD_8192E /* Software read/write CAM config */
#define CAMDBG_8192E REG_CAMDBG_8192E
#define SECR_8192E REG_SECCFG_8192E /* security Configuration Register */

/* RSSI Dump Message */
#define REG_A_RSSI_DUMP_92E 0xcb0
#define REG_B_RSSI_DUMP_92E 0xcb1
#define REG_S1_RXEVM_DUMP_92E 0xcb2
#define REG_S2_RXEVM_DUMP_92E 0xcb3
#define REG_A_RX_SNR_DUMP_92E 0xcb4
#define REG_B_RX_SNR_DUMP_92E 0xcb5
#define REG_A_CFO_SHORT_DUMP_92E 0xcb6
#define REG_B_CFO_SHORT_DUMP_92E 0xcb8
#define REG_A_CFO_LONG_DUMP_92E 0xcba
#define REG_B_CFO_LONG_DUMP_92E 0xcbc

/* ----------------------------------------------------------------------------
 * 8195 IMR/ISR bits						(offset 0xB0,  8bits)
 * ---------------------------------------------------------------------------- */
#define IMR_DISABLED_8192E 0
/* IMR DW0(0x00B0-00B3) Bit 0-31 */
#define IMR_TIMER2_8192E BIT(31) /* Timeout interrupt 2 */
#define IMR_TIMER1_8192E BIT(30) /* Timeout interrupt 1 */
#define IMR_PSTIMEOUT_8192E BIT(29) /* Power Save Time Out Interrupt */
#define IMR_GTINT4_8192E BIT(28) /* When GTIMER4 expires, this bit is set to 1 */
#define IMR_GTINT3_8192E BIT(27) /* When GTIMER3 expires, this bit is set to 1 */
#define IMR_TXBCN0ERR_8192E BIT(26) /* Transmit Beacon0 Error */
#define IMR_TXBCN0OK_8192E BIT(25) /* Transmit Beacon0 OK */
#define IMR_TSF_BIT32_TOGGLE_8192E BIT(24) /* TSF Timer BIT32 toggle indication interrupt */
#define IMR_BCNDMAINT0_8192E BIT(20) /* Beacon DMA Interrupt 0 */
#define IMR_BCNDERR0_8192E BIT(16) /* Beacon Queue DMA OK0 */
#define IMR_HSISR_IND_ON_INT_8192E BIT(15) /* HSISR Indicator (HSIMR & HSISR is true, this bit is set to 1) */
#define IMR_BCNDMAINT_E_8192E BIT(14) /* Beacon DMA Interrupt Extension for Win7 */
#define IMR_ATIMEND_8192E BIT(12) /* CTWidnow End or ATIM Window End */
#define IMR_C2HCMD_8192E BIT(10) /* CPU to Host Command INT status, Write 1 clear */
#define IMR_CPWM2_8192E BIT(9) /* CPU power mode exchange INT status, Write 1 clear */
#define IMR_CPWM_8192E BIT(8) /* CPU power mode exchange INT status, Write 1 clear */
#define IMR_HIGHDOK_8192E BIT(7) /* High Queue DMA OK */
#define IMR_MGNTDOK_8192E BIT(6) /* Management Queue DMA OK */
#define IMR_BKDOK_8192E BIT(5) /* AC_BK DMA OK */
#define IMR_BEDOK_8192E BIT(4) /* AC_BE DMA OK */
#define IMR_VIDOK_8192E BIT(3) /* AC_VI DMA OK */
#define IMR_VODOK_8192E BIT(2) /* AC_VO DMA OK */
#define IMR_RDU_8192E BIT(1) /* Rx Descriptor Unavailable */
#define IMR_ROK_8192E BIT(0) /* Receive DMA OK */

/* IMR DW1(0x00B4-00B7) Bit 0-31 */
#define IMR_BCNDMAINT7_8192E BIT(27) /* Beacon DMA Interrupt 7 */
#define IMR_BCNDMAINT6_8192E BIT(26) /* Beacon DMA Interrupt 6 */
#define IMR_BCNDMAINT5_8192E BIT(25) /* Beacon DMA Interrupt 5 */
#define IMR_BCNDMAINT4_8192E BIT(24) /* Beacon DMA Interrupt 4 */
#define IMR_BCNDMAINT3_8192E BIT(23) /* Beacon DMA Interrupt 3 */
#define IMR_BCNDMAINT2_8192E BIT(22) /* Beacon DMA Interrupt 2 */
#define IMR_BCNDMAINT1_8192E BIT(21) /* Beacon DMA Interrupt 1 */
#define IMR_BCNDOK7_8192E BIT(20) /* Beacon Queue DMA OK Interrup 7 */
#define IMR_BCNDOK6_8192E BIT(19) /* Beacon Queue DMA OK Interrup 6 */
#define IMR_BCNDOK5_8192E BIT(18) /* Beacon Queue DMA OK Interrup 5 */
#define IMR_BCNDOK4_8192E BIT(17) /* Beacon Queue DMA OK Interrup 4 */
#define IMR_BCNDOK3_8192E BIT(16) /* Beacon Queue DMA OK Interrup 3 */
#define IMR_BCNDOK2_8192E BIT(15) /* Beacon Queue DMA OK Interrup 2 */
#define IMR_BCNDOK1_8192E BIT(14) /* Beacon Queue DMA OK Interrup 1 */
#define IMR_ATIMEND_E_8192E BIT(13) /* ATIM Window End Extension for Win7 */
#define IMR_TXERR_8192E BIT(11) /* Tx Error Flag Interrupt status, write 1 clear. */
#define IMR_RXERR_8192E BIT(10) /* Rx Error Flag INT status, Write 1 clear */
#define IMR_TXFOVW_8192E BIT(9) /* Transmit FIFO Overflow */
#define IMR_RXFOVW_8192E BIT(8) /* Receive FIFO Overflow */

#define IMR_MCUERR_8192E BIT(28) /* Beacon DMA Interrupt 7 */

/*===================================================================
=====================================================================
Here the register defines are for 92C. When the define is as same with 92C,
we will use the 92C's define for the consistency
So the following defines for 92C is not entire!!!!!!
=====================================================================
=====================================================================*/
/*
Based on Datasheet V33---090401
Register Summary
Current IOREG MAP
0x0000h ~ 0x00FFh   System Configuration (256 Bytes)
0x0100h ~ 0x01FFh   MACTOP General Configuration (256 Bytes)
0x0200h ~ 0x027Fh   TXDMA Configuration (128 Bytes)
0x0280h ~ 0x02FFh   RXDMA Configuration (128 Bytes)
0x0300h ~ 0x03FFh   PCIE EMAC Reserved Region (256 Bytes)
0x0400h ~ 0x04FFh   Protocol Configuration (256 Bytes)
0x0500h ~ 0x05FFh   EDCA Configuration (256 Bytes)
0x0600h ~ 0x07FFh   WMAC Configuration (512 Bytes)
0x2000h ~ 0x3FFFh   8051 FW Download Region (8196 Bytes)
*/
/* ----------------------------------------------------------------------------
 *		 8195 (TXPAUSE) transmission pause	(Offset 0x522, 8 bits)
 * ----------------------------------------------------------------------------
 *
#define		StopBecon			BIT(6)
#define		StopHigh				BIT(5)
#define		StopMgt				BIT(4)
#define		StopVO				BIT(3)
#define		StopVI				BIT(2)
#define		StopBE				BIT(1)
#define		StopBK				BIT(0)
*/

/* 2 ACMHWCTRL 0x05C0 */
#define acm_hw_hw_en_8192e BIT(0)
#define acm_hw_voq_en_8192e BIT(1)
#define acm_hw_viq_en_8192e BIT(2)
#define acm_hw_beq_en_8192e BIT(3)
#define acm_hw_voq_status_8192e BIT(5)
#define acm_hw_viq_status_8192e BIT(6)
#define acm_hw_beq_status_8192e BIT(7)

#endif /*  #ifndef __INC_HAL8192EREG_H */
