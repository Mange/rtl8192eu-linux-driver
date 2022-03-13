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
/* ************************************************************
 * Description:
 *
 * This file is for 8814A TXBF mechanism
 *
 * ************************************************************ */

#include "mp_precomp.h"
#include "../phydm_precomp.h"

#ifdef PHYDM_BEAMFORMING_SUPPORT
#if (RTL8814A_SUPPORT == 1)

boolean
phydm_beamforming_set_iqgen_8814A(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u8 i = 0;
	u16 counter = 0;
	u32 rf_mode[4];

	for (i = RF_PATH_A; i < MAX_RF_PATH; i++)
		odm_set_rf_reg(dm, i, RF_WE_LUT, 0x80000, 0x1); /*RF mode table write enable*/

	while (1) {
		counter++;
		for (i = RF_PATH_A; i < MAX_RF_PATH; i++)
			odm_set_rf_reg(dm, i, RF_RCK_OS, 0xfffff, 0x18000); /*Select Rx mode*/

		ODM_delay_us(2);

		for (i = RF_PATH_A; i < MAX_RF_PATH; i++)
			rf_mode[i] = odm_get_rf_reg(dm, i, RF_RCK_OS, 0xfffff);

		if (rf_mode[0] == 0x18000 && rf_mode[1] == 0x18000 && rf_mode[2] == 0x18000 && rf_mode[3] == 0x18000)
			break;
		else if (counter == 100) {
			PHYDM_DBG(dm, DBG_TXBF, "iqgen setting fail:8814A\n");
			return false;
		}
	}

	for (i = RF_PATH_A; i < MAX_RF_PATH; i++) {
		odm_set_rf_reg(dm, i, RF_TXPA_G1, 0xfffff, 0xBE77F); /*Set Table data*/
		odm_set_rf_reg(dm, i, RF_TXPA_G2, 0xfffff, 0x226BF); /*@Enable TXIQGEN in Rx mode*/
	}
	odm_set_rf_reg(dm, RF_PATH_A, RF_TXPA_G2, 0xfffff, 0xE26BF); /*@Enable TXIQGEN in Rx mode*/

	for (i = RF_PATH_A; i < MAX_RF_PATH; i++)
		odm_set_rf_reg(dm, i, RF_WE_LUT, 0x80000, 0x0); /*RF mode table write disable*/

	return true;
}

void hal_txbf_8814a_set_ndpa_rate(void *dm_void, u8 BW, u8 rate)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;

	odm_write_1byte(dm, REG_NDPA_OPT_CTRL_8814A, BW);
	odm_write_1byte(dm, REG_NDPA_RATE_8814A, (u8)rate);
}

void hal_txbf_8814a_get_tx_rate(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct _RT_BEAMFORMING_INFO *beam_info = &dm->beamforming_info;
	struct _RT_BEAMFORMEE_ENTRY *entry;
	struct ra_table *ra_tab = &dm->dm_ra_table;
	struct cmn_sta_info *sta = NULL;
	u8 data_rate = 0xFF;
	u8 macid = 0;

	entry = &(beam_info->beamformee_entry[beam_info->beamformee_cur_idx]);
	macid = (u8)entry->mac_id;

	sta = dm->phydm_sta_info[macid];

	if (is_sta_active(sta)) {
		data_rate = (sta->ra_info.curr_tx_rate) & 0x7f; /*@Bit7 indicates SGI*/
		beam_info->tx_bf_data_rate = data_rate;
	}

	PHYDM_DBG(dm, DBG_TXBF, "[%s] dm->tx_bf_data_rate = 0x%x\n", __func__,
		  beam_info->tx_bf_data_rate);
}

void hal_txbf_8814a_reset_tx_path(void *dm_void, u8 idx)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
#if DEV_BUS_TYPE == RT_USB_INTERFACE
	struct _RT_BEAMFORMING_INFO *beamforming_info = &dm->beamforming_info;
	struct _RT_BEAMFORMEE_ENTRY beamformee_entry;
	u8 nr_index = 0, tx_ss = 0;

	if (idx < BEAMFORMEE_ENTRY_NUM)
		beamformee_entry = beamforming_info->beamformee_entry[idx];
	else
		return;

	if (beamforming_info->last_usb_hub != (*dm->hub_usb_mode)) {
		nr_index = tx_bf_nr(hal_txbf_8814a_get_ntx(dm), beamformee_entry.comp_steering_num_of_bfer);

		if (*dm->hub_usb_mode == 2) {
			if (dm->rf_type == RF_4T4R)
				tx_ss = 0xf;
			else if (dm->rf_type == RF_3T3R)
				tx_ss = 0xe;
			else
				tx_ss = 0x6;
		} else if (*dm->hub_usb_mode == 1) /*USB 2.0 always 2Tx*/
			tx_ss = 0x6;
		else
			tx_ss = 0x6;

		if (tx_ss == 0xf) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x93f);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKDWORD, 0x93f93f0);
		} else if (tx_ss == 0xe) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x93e);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_2_8814A, MASKDWORD, 0x93e93e0);
		} else if (tx_ss == 0x6) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x936);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_2_8814A, MASKLWORD, 0x9360);
		}

		if (idx == 0) {
			switch (nr_index) {
			case 0:
				break;

			case 1: /*Nsts = 2	BC*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x9366); /*tx2path, BC*/
				break;

			case 2: /*Nsts = 3	BCD*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93e93ee); /*tx3path, BCD*/
				break;

			default: /*nr>3, same as Case 3*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93f93ff); /*tx4path, ABCD*/
				break;
			}
		} else {
			switch (nr_index) {
			case 0:
				break;

			case 1: /*Nsts = 2	BC*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x9366); /*tx2path, BC*/
				break;

			case 2: /*Nsts = 3	BCD*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93e93ee); /*tx3path, BCD*/
				break;

			default: /*nr>3, same as Case 3*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93f93ff); /*tx4path, ABCD*/
				break;
			}
		}

		beamforming_info->last_usb_hub = *dm->hub_usb_mode;
	} else
		return;
#endif
}

u8 hal_txbf_8814a_get_ntx(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u8 ntx = 0, tx_ss = 3;

#if DEV_BUS_TYPE == RT_USB_INTERFACE
	tx_ss = *dm->hub_usb_mode;
#endif
	if (tx_ss == 3 || tx_ss == 2) {
		if (dm->rf_type == RF_4T4R)
			ntx = 3;
		else if (dm->rf_type == RF_3T3R)
			ntx = 2;
		else
			ntx = 1;
	} else if (tx_ss == 1) /*USB 2.0 always 2Tx*/
		ntx = 1;
	else
		ntx = 1;

	PHYDM_DBG(dm, DBG_TXBF, "[%s] ntx = %d\n", __func__, ntx);
	return ntx;
}

u8 hal_txbf_8814a_get_nrx(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u8 nrx = 0;

	if (dm->rf_type == RF_4T4R)
		nrx = 3;
	else if (dm->rf_type == RF_3T3R)
		nrx = 2;
	else if (dm->rf_type == RF_2T2R)
		nrx = 1;
	else if (dm->rf_type == RF_2T3R)
		nrx = 2;
	else if (dm->rf_type == RF_2T4R)
		nrx = 3;
	else if (dm->rf_type == RF_1T1R)
		nrx = 0;
	else if (dm->rf_type == RF_1T2R)
		nrx = 1;
	else
		nrx = 0;

	PHYDM_DBG(dm, DBG_TXBF, "[%s] nrx = %d\n", __func__, nrx);
	return nrx;
}

void hal_txbf_8814a_rf_mode(void *dm_void,
			    struct _RT_BEAMFORMING_INFO *beamforming_info,
			    u8 idx)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u8 nr_index = 0;
	u8 tx_ss = 3; /*@default use 3 Tx*/
	struct _RT_BEAMFORMEE_ENTRY beamformee_entry;

	if (idx < BEAMFORMEE_ENTRY_NUM)
		beamformee_entry = beamforming_info->beamformee_entry[idx];
	else
		return;

	nr_index = tx_bf_nr(hal_txbf_8814a_get_ntx(dm), beamformee_entry.comp_steering_num_of_bfer);

	if (dm->rf_type == RF_1T1R)
		return;

	if (beamforming_info->beamformee_su_cnt > 0) {
#if DEV_BUS_TYPE == RT_USB_INTERFACE
		beamforming_info->last_usb_hub = *dm->hub_usb_mode;
		tx_ss = *dm->hub_usb_mode;
#endif
		if (tx_ss == 3 || tx_ss == 2) {
			if (dm->rf_type == RF_4T4R)
				tx_ss = 0xf;
			else if (dm->rf_type == RF_3T3R)
				tx_ss = 0xe;
			else
				tx_ss = 0x6;
		} else if (tx_ss == 1) /*USB 2.0 always 2Tx*/
			tx_ss = 0x6;
		else
			tx_ss = 0x6;

		if (tx_ss == 0xf) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x93f);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKDWORD, 0x93f93f0);
		} else if (tx_ss == 0xe) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x93e);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_2_8814A, MASKDWORD, 0x93e93e0);
		} else if (tx_ss == 0x6) {
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x936);
			odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_2_8814A, MASKLWORD, 0x9360);
		}

		/*@for 8814 19ac(idx 1), 19b4(idx 0), different Tx ant setting*/
		odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, BIT(28) | BIT29, 0x2); /*@enable BB TxBF ant mapping register*/
		odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, BIT30, 0x1); /*@if Nsts > Nc don't apply V matrix*/

		if (idx == 0) {
			switch (nr_index) {
			case 0:
				break;

			case 1: /*Nsts = 2	BC*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x9366); /*tx2path, BC*/
				break;

			case 2: /*Nsts = 3	BCD*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93e93ee); /*tx3path, BCD*/
				break;

			default: /*nr>3, same as Case 3*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF0_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93f93ff); /*tx4path, ABCD*/

				break;
			}
		} else {
			switch (nr_index) {
			case 0:
				break;

			case 1: /*Nsts = 2	BC*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x9366); /*tx2path, BC*/
				break;

			case 2: /*Nsts = 3	BCD*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93e93ee); /*tx3path, BCD*/
				break;

			default: /*nr>3, same as Case 3*/
				odm_set_bb_reg(dm, REG_BB_TXBF_ANT_SET_BF1_8814A, MASKBYTE3LOWNIBBLE | MASKL3BYTES, 0x93f93ff); /*tx4path, ABCD*/
				break;
			}
		}
	}

	if (beamforming_info->beamformee_su_cnt == 0 && beamforming_info->beamformer_su_cnt == 0) {
		odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_1_8814A, MASKBYTE3 | MASKBYTE2HIGHNIBBLE, 0x932); /*set tx_path selection for 8814a BFer bug refine*/
		odm_set_bb_reg(dm, REG_BB_TX_PATH_SEL_2_8814A, MASKDWORD, 0x93e9360);
	}
}
void hal_txbf_8814a_enter(void *dm_void, u8 bfer_bfee_idx)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u8 i = 0;
	u8 bfer_idx = (bfer_bfee_idx & 0xF0) >> 4;
	u8 bfee_idx = (bfer_bfee_idx & 0xF);
	struct _RT_BEAMFORMING_INFO *beamforming_info = &dm->beamforming_info;
	struct _RT_BEAMFORMEE_ENTRY beamformee_entry;
	struct _RT_BEAMFORMER_ENTRY beamformer_entry;
	u16 sta_id = 0, csi_param = 0;
	u8 nc_index = 0, nr_index = 0, grouping = 0, codebookinfo = 0, coefficientsize = 0;

	PHYDM_DBG(dm, DBG_TXBF, "[%s] bfer_idx=%d, bfee_idx=%d\n", __func__,
		  bfer_idx, bfee_idx);
	odm_set_mac_reg(dm, REG_SND_PTCL_CTRL_8814A, MASKBYTE1 | MASKBYTE2, 0x0202);

	if (beamforming_info->beamformer_su_cnt > 0 && bfer_idx < BEAMFORMER_ENTRY_NUM) {
		beamformer_entry = beamforming_info->beamformer_entry[bfer_idx];
		/*Sounding protocol control*/
		odm_write_1byte(dm, REG_SND_PTCL_CTRL_8814A, 0xDB);

		/*@MAC address/Partial AID of Beamformer*/
		if (bfer_idx == 0) {
			for (i = 0; i < 6; i++)
				odm_write_1byte(dm, (REG_ASSOCIATED_BFMER0_INFO_8814A + i), beamformer_entry.mac_addr[i]);
		} else {
			for (i = 0; i < 6; i++)
				odm_write_1byte(dm, (REG_ASSOCIATED_BFMER1_INFO_8814A + i), beamformer_entry.mac_addr[i]);
		}

		/*@CSI report parameters of Beamformer*/
		nc_index = hal_txbf_8814a_get_nrx(dm); /*@for 8814A nrx = 3(4 ant), min=0(1 ant)*/
		nr_index = beamformer_entry.num_of_sounding_dim; /*@0x718[7] = 1 use Nsts, 0x718[7] = 0 use reg setting. as Bfee, we use Nsts, so nr_index don't care*/

		grouping = 0;

		/*@for ac = 1, for n = 3*/
		if (beamformer_entry.beamform_entry_cap & BEAMFORMEE_CAP_VHT_SU)
			codebookinfo = 1;
		else if (beamformer_entry.beamform_entry_cap & BEAMFORMEE_CAP_HT_EXPLICIT)
			codebookinfo = 3;

		coefficientsize = 3;

		csi_param = (u16)((coefficientsize << 10) | (codebookinfo << 8) | (grouping << 6) | (nr_index << 3) | (nc_index));

		if (bfer_idx == 0)
			odm_write_2byte(dm, REG_CSI_RPT_PARAM_BW20_8814A, csi_param);
		else
			odm_write_2byte(dm, REG_CSI_RPT_PARAM_BW20_8814A + 2, csi_param);
		/*ndp_rx_standby_timer, 8814 need > 0x56, suggest from Dvaid*/
		odm_write_1byte(dm, REG_SND_PTCL_CTRL_8814A + 3, 0x40);
	}

	if (beamforming_info->beamformee_su_cnt > 0 && bfee_idx < BEAMFORMEE_ENTRY_NUM) {
		beamformee_entry = beamforming_info->beamformee_entry[bfee_idx];

		hal_txbf_8814a_rf_mode(dm, beamforming_info, bfee_idx);

		if (phydm_acting_determine(dm, phydm_acting_as_ibss))
			sta_id = beamformee_entry.mac_id;
		else
			sta_id = beamformee_entry.p_aid;

		/*P_AID of Beamformee & enable NDPA transmission & enable NDPA interrupt*/
		if (bfee_idx == 0) {
			odm_write_2byte(dm, REG_TXBF_CTRL_8814A, sta_id);
			odm_write_1byte(dm, REG_TXBF_CTRL_8814A + 3, odm_read_1byte(dm, REG_TXBF_CTRL_8814A + 3) | BIT(4) | BIT(6) | BIT(7));
		} else
			odm_write_2byte(dm, REG_TXBF_CTRL_8814A + 2, sta_id | BIT(14) | BIT(15) | BIT(12));

		/*@CSI report parameters of Beamformee*/
		if (bfee_idx == 0) {
			/*@Get BIT24 & BIT25*/
			u8 tmp = odm_read_1byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A + 3) & 0x3;

			odm_write_1byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A + 3, tmp | 0x60);
			odm_write_2byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A, sta_id | BIT(9));
		} else
			odm_write_2byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2, sta_id | 0xE200); /*Set BIT25*/

		phydm_beamforming_notify(dm);
	}
}

void hal_txbf_8814a_leave(void *dm_void, u8 idx)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct _RT_BEAMFORMING_INFO *beamforming_info = &dm->beamforming_info;
	struct _RT_BEAMFORMER_ENTRY beamformer_entry;
	struct _RT_BEAMFORMEE_ENTRY beamformee_entry;

	if (idx < BEAMFORMER_ENTRY_NUM) {
		beamformer_entry = beamforming_info->beamformer_entry[idx];
		beamformee_entry = beamforming_info->beamformee_entry[idx];
	} else
		return;

	/*@Clear P_AID of Beamformee*/
	/*@Clear MAC address of Beamformer*/
	/*@Clear Associated Bfmee Sel*/

	if (beamformer_entry.beamform_entry_cap == BEAMFORMING_CAP_NONE) {
		odm_write_1byte(dm, REG_SND_PTCL_CTRL_8814A, 0xD8);
		if (idx == 0) {
			odm_write_4byte(dm, REG_ASSOCIATED_BFMER0_INFO_8814A, 0);
			odm_write_2byte(dm, REG_ASSOCIATED_BFMER0_INFO_8814A + 4, 0);
			odm_write_2byte(dm, REG_CSI_RPT_PARAM_BW20_8814A, 0);
		} else {
			odm_write_4byte(dm, REG_ASSOCIATED_BFMER1_INFO_8814A, 0);
			odm_write_2byte(dm, REG_ASSOCIATED_BFMER1_INFO_8814A + 4, 0);
			odm_write_2byte(dm, REG_CSI_RPT_PARAM_BW20_8814A + 2, 0);
		}
	}

	if (beamformee_entry.beamform_entry_cap == BEAMFORMING_CAP_NONE) {
		hal_txbf_8814a_rf_mode(dm, beamforming_info, idx);
		if (idx == 0) {
			odm_write_2byte(dm, REG_TXBF_CTRL_8814A, 0x0);
			odm_write_1byte(dm, REG_TXBF_CTRL_8814A + 3, odm_read_1byte(dm, REG_TXBF_CTRL_8814A + 3) | BIT(4) | BIT(6) | BIT(7));
			odm_write_2byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A, 0);
		} else {
			odm_write_2byte(dm, REG_TXBF_CTRL_8814A + 2, 0x0 | BIT(14) | BIT(15) | BIT(12));

			odm_write_2byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2, odm_read_2byte(dm, REG_ASSOCIATED_BFMEE_SEL_8814A + 2) & 0x60);
		}
	}
}

void hal_txbf_8814a_status(void *dm_void, u8 idx)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u16 beam_ctrl_val, tmp_val;
	u32 beam_ctrl_reg;
	struct _RT_BEAMFORMING_INFO *beamforming_info = &dm->beamforming_info;
	struct _RT_BEAMFORMEE_ENTRY beamform_entry;

	if (idx < BEAMFORMEE_ENTRY_NUM)
		beamform_entry = beamforming_info->beamformee_entry[idx];
	else
		return;

	if (phydm_acting_determine(dm, phydm_acting_as_ibss))
		beam_ctrl_val = beamform_entry.mac_id;
	else
		beam_ctrl_val = beamform_entry.p_aid;

	PHYDM_DBG(dm, DBG_TXBF, "@%s, beamform_entry.beamform_entry_state = %d",
		  __func__, beamform_entry.beamform_entry_state);

	if (idx == 0)
		beam_ctrl_reg = REG_TXBF_CTRL_8814A;
	else {
		beam_ctrl_reg = REG_TXBF_CTRL_8814A + 2;
		beam_ctrl_val |= BIT(12) | BIT(14) | BIT(15);
	}

	if (beamform_entry.beamform_entry_state == BEAMFORMING_ENTRY_STATE_PROGRESSED && beamforming_info->apply_v_matrix == true) {
		if (beamform_entry.sound_bw == CHANNEL_WIDTH_20)
			beam_ctrl_val |= BIT(9);
		else if (beamform_entry.sound_bw == CHANNEL_WIDTH_40)
			beam_ctrl_val |= (BIT(9) | BIT(10));
		else if (beamform_entry.sound_bw == CHANNEL_WIDTH_80)
			beam_ctrl_val |= (BIT(9) | BIT(10) | BIT(11));
	} else {
		PHYDM_DBG(dm, DBG_TXBF, "@%s, Don't apply Vmatrix", __func__);
		beam_ctrl_val &= ~(BIT(9) | BIT(10) | BIT(11));
	}

	odm_write_2byte(dm, beam_ctrl_reg, beam_ctrl_val);
	/*@disable NDP packet use beamforming */
	tmp_val = odm_read_2byte(dm, REG_TXBF_CTRL_8814A);
	odm_write_2byte(dm, REG_TXBF_CTRL_8814A, tmp_val | BIT(15));
}

void hal_txbf_8814a_fw_txbf(void *dm_void, u8 idx)
{
}

#endif /* @(RTL8814A_SUPPORT == 1)*/

#endif
