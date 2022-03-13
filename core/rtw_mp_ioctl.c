/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
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
#define _RTW_MP_IOCTL_C_

#include <drv_types.h>
#include <rtw_mp_ioctl.h>
#include "../hal/phydm/phydm_precomp.h"

/* ****************  oid_rtl_seg_81_85   section start **************** */
NDIS_STATUS oid_rt_wireless_mode_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->information_buf_len < sizeof(u8))
		return NDIS_STATUS_INVALID_LENGTH;

	if (poid_par_priv->type_of_oid == SET_OID)
		Adapter->registrypriv.wireless_mode = *(u8 *)poid_par_priv->information_buf;
	else if (poid_par_priv->type_of_oid == QUERY_OID) {
		*(u8 *)poid_par_priv->information_buf = Adapter->registrypriv.wireless_mode;
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	} else
		status = NDIS_STATUS_NOT_ACCEPTED;


	return status;
}
/* ****************  oid_rtl_seg_81_87_80   section start **************** */
NDIS_STATUS oid_rt_pro_write_bb_reg_hdl(struct oid_par_priv *poid_par_priv)
{
	struct bb_reg_param *pbbreg;
	u16 offset;
	u32 value;
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(struct bb_reg_param))
		return NDIS_STATUS_INVALID_LENGTH;

	pbbreg = (struct bb_reg_param *)(poid_par_priv->information_buf);

	offset = (u16)(pbbreg->offset) & 0xFFF; /* 0ffset :0x800~0xfff */
	if (offset < BB_REG_BASE_ADDR)
		offset |= BB_REG_BASE_ADDR;

	value = pbbreg->value;


	_irqlevel_changed_(&oldirql, LOWER);
	write_bbreg(Adapter, offset, 0xFFFFFFFF, value);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_read_bb_reg_hdl(struct oid_par_priv *poid_par_priv)
{
	struct bb_reg_param *pbbreg;
	u16 offset;
	u32 value;
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(struct bb_reg_param))
		return NDIS_STATUS_INVALID_LENGTH;

	pbbreg = (struct bb_reg_param *)(poid_par_priv->information_buf);

	offset = (u16)(pbbreg->offset) & 0xFFF; /* 0ffset :0x800~0xfff */
	if (offset < BB_REG_BASE_ADDR)
		offset |= BB_REG_BASE_ADDR;

	_irqlevel_changed_(&oldirql, LOWER);
	value = read_bbreg(Adapter, offset, 0xFFFFFFFF);
	_irqlevel_changed_(&oldirql, RAISE);

	pbbreg->value = value;
	*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_write_rf_reg_hdl(struct oid_par_priv *poid_par_priv)
{
	struct rf_reg_param *pbbreg;
	u8 path;
	u8 offset;
	u32 value;
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(struct rf_reg_param))
		return NDIS_STATUS_INVALID_LENGTH;

	pbbreg = (struct rf_reg_param *)(poid_par_priv->information_buf);

	if (pbbreg->path >= MAX_RF_PATH_NUMS)
		return NDIS_STATUS_NOT_ACCEPTED;
	if (pbbreg->offset > 0xFF)
		return NDIS_STATUS_NOT_ACCEPTED;
	if (pbbreg->value > 0xFFFFF)
		return NDIS_STATUS_NOT_ACCEPTED;

	path = (u8)pbbreg->path;
	offset = (u8)pbbreg->offset;
	value = pbbreg->value;


	_irqlevel_changed_(&oldirql, LOWER);
	write_rfreg(Adapter, path, offset, value);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_read_rf_reg_hdl(struct oid_par_priv *poid_par_priv)
{
	struct rf_reg_param *pbbreg;
	u8 path;
	u8 offset;
	u32 value;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(struct rf_reg_param))
		return NDIS_STATUS_INVALID_LENGTH;

	pbbreg = (struct rf_reg_param *)(poid_par_priv->information_buf);

	if (pbbreg->path >= MAX_RF_PATH_NUMS)
		return NDIS_STATUS_NOT_ACCEPTED;
	if (pbbreg->offset > 0xFF)
		return NDIS_STATUS_NOT_ACCEPTED;

	path = (u8)pbbreg->path;
	offset = (u8)pbbreg->offset;

	_irqlevel_changed_(&oldirql, LOWER);
	value = read_rfreg(Adapter, path, offset);
	_irqlevel_changed_(&oldirql, RAISE);

	pbbreg->value = value;

	*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;



	return status;
}
/* ****************  oid_rtl_seg_81_87_00   section end****************
 * ------------------------------------------------------------------------------ */

/* ****************  oid_rtl_seg_81_80_00   section start ****************
 * ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_data_rate_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		ratevalue;/* 4 */
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len != sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	ratevalue = *((u32 *)poid_par_priv->information_buf); /* 4 */
	if (ratevalue >= MPT_RATE_LAST)
		return NDIS_STATUS_INVALID_DATA;

	Adapter->mppriv.rateidx = ratevalue;

	_irqlevel_changed_(&oldirql, LOWER);
	SetDataRate(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_start_test_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		mode;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (Adapter->registrypriv.mp_mode == 0)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);

	/* IQCalibrateBcut(Adapter); */

	mode = *((u32 *)poid_par_priv->information_buf);
	Adapter->mppriv.mode = mode;/* 1 for loopback */

	if (mp_start_test(Adapter) == _FAIL) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		goto exit;
	}

exit:
	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_stop_test_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);
	mp_stop_test(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_channel_direct_call_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		Channel;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->information_buf_len != sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	if (poid_par_priv->type_of_oid == QUERY_OID) {
		*((u32 *)poid_par_priv->information_buf) = Adapter->mppriv.channel;
		return NDIS_STATUS_SUCCESS;
	}

	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	Channel = *((u32 *)poid_par_priv->information_buf);
	if (Channel > 14)
		return NDIS_STATUS_NOT_ACCEPTED;
	Adapter->mppriv.channel = Channel;

	_irqlevel_changed_(&oldirql, LOWER);
	SetChannel(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_set_bandwidth_hdl(struct oid_par_priv *poid_par_priv)
{
	u16		bandwidth;
	u16		channel_offset;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	padapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	bandwidth = *((u32 *)poid_par_priv->information_buf); /* 4 */
	channel_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

	if (bandwidth != CHANNEL_WIDTH_40)
		bandwidth = CHANNEL_WIDTH_20;
	padapter->mppriv.bandwidth = (u8)bandwidth;
	padapter->mppriv.prime_channel_offset = (u8)channel_offset;

	_irqlevel_changed_(&oldirql, LOWER);
	SetBandwidth(padapter);
	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_antenna_bb_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		antenna;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->information_buf_len != sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	if (poid_par_priv->type_of_oid == SET_OID) {
		antenna = *(u32 *)poid_par_priv->information_buf;

		Adapter->mppriv.antenna_tx = (u16)((antenna & 0xFFFF0000) >> 16);
		Adapter->mppriv.antenna_rx = (u16)(antenna & 0x0000FFFF);

		_irqlevel_changed_(&oldirql, LOWER);
		SetAntenna(Adapter);
		_irqlevel_changed_(&oldirql, RAISE);
	} else {
		antenna = (Adapter->mppriv.antenna_tx << 16) | Adapter->mppriv.antenna_rx;
		*(u32 *)poid_par_priv->information_buf = antenna;
	}


	return status;
}

NDIS_STATUS oid_rt_pro_set_tx_power_control_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		tx_pwr_idx;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len != sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	tx_pwr_idx = *((u32 *)poid_par_priv->information_buf);
	if (tx_pwr_idx > MAX_TX_PWR_INDEX_N_MODE)
		return NDIS_STATUS_NOT_ACCEPTED;

	Adapter->mppriv.txpoweridx = (u8)tx_pwr_idx;


	_irqlevel_changed_(&oldirql, LOWER);
	SetTxPower(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}

/* ------------------------------------------------------------------------------
 * ****************  oid_rtl_seg_81_80_20   section start ****************
 * ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_query_tx_packet_sent_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}

	if (poid_par_priv->information_buf_len == sizeof(ULONG)) {
		*(ULONG *)poid_par_priv->information_buf =  Adapter->mppriv.tx_pktcount;
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	} else
		status = NDIS_STATUS_INVALID_LENGTH;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_query_rx_packet_received_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}
	if (poid_par_priv->information_buf_len == sizeof(ULONG)) {
		*(ULONG *)poid_par_priv->information_buf =  Adapter->mppriv.rx_pktcount;
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	} else
		status = NDIS_STATUS_INVALID_LENGTH;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_query_rx_packet_crc32_error_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}
	if (poid_par_priv->information_buf_len == sizeof(ULONG)) {
		*(ULONG *)poid_par_priv->information_buf =  Adapter->mppriv.rx_crcerrpktcount;
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	} else
		status = NDIS_STATUS_INVALID_LENGTH;


	return status;
}
/* ------------------------------------------------------------------------------ */

NDIS_STATUS oid_rt_pro_reset_tx_packet_sent_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != SET_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}

	Adapter->mppriv.tx_pktcount = 0;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_reset_rx_packet_received_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != SET_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}

	if (poid_par_priv->information_buf_len == sizeof(ULONG)) {
		Adapter->mppriv.rx_pktcount = 0;
		Adapter->mppriv.rx_crcerrpktcount = 0;
	} else
		status = NDIS_STATUS_INVALID_LENGTH;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_reset_phy_rx_packet_count_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != SET_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}

	_irqlevel_changed_(&oldirql, LOWER);
	ResetPhyRxPktCount(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_get_phy_rx_packet_received_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len != sizeof(ULONG))
		return NDIS_STATUS_INVALID_LENGTH;

	_irqlevel_changed_(&oldirql, LOWER);
	*(ULONG *)poid_par_priv->information_buf = GetPhyRxPktReceived(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);

	*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_get_phy_rx_packet_crc32_error_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;


	if (poid_par_priv->information_buf_len != sizeof(ULONG))
		return NDIS_STATUS_INVALID_LENGTH;

	_irqlevel_changed_(&oldirql, LOWER);
	*(ULONG *)poid_par_priv->information_buf = GetPhyRxPktCRC32Error(Adapter);
	_irqlevel_changed_(&oldirql, RAISE);

	*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;



	return status;
}
/* ****************  oid_rtl_seg_81_80_20   section end **************** */
NDIS_STATUS oid_rt_pro_set_continuous_tx_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		bStartTest;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	bStartTest = *((u32 *)poid_par_priv->information_buf);

	_irqlevel_changed_(&oldirql, LOWER);
	SetContinuousTx(Adapter, (u8)bStartTest);
	if (bStartTest) {
		struct mp_priv *pmp_priv = &Adapter->mppriv;
		if (pmp_priv->tx.stop == 0) {
			pmp_priv->tx.stop = 1;
			RTW_INFO("%s: pkt tx is running...\n", __func__);
			msleep(5);
		}
		pmp_priv->tx.stop = 0;
		pmp_priv->tx.count = 1;
		SetPacketTx(Adapter);
	}
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}

NDIS_STATUS oid_rt_pro_set_single_carrier_tx_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		bStartTest;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	bStartTest = *((u32 *)poid_par_priv->information_buf);

	_irqlevel_changed_(&oldirql, LOWER);
	SetSingleCarrierTx(Adapter, (u8)bStartTest);
	if (bStartTest) {
		struct mp_priv *pmp_priv = &Adapter->mppriv;
		if (pmp_priv->tx.stop == 0) {
			pmp_priv->tx.stop = 1;
			RTW_INFO("%s: pkt tx is running...\n", __func__);
			msleep(5);
		}
		pmp_priv->tx.stop = 0;
		pmp_priv->tx.count = 1;
		SetPacketTx(Adapter);
	}
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}

NDIS_STATUS oid_rt_pro_set_carrier_suppression_tx_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		bStartTest;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	bStartTest = *((u32 *)poid_par_priv->information_buf);

	_irqlevel_changed_(&oldirql, LOWER);
	SetCarrierSuppressionTx(Adapter, (u8)bStartTest);
	if (bStartTest) {
		struct mp_priv *pmp_priv = &Adapter->mppriv;
		if (pmp_priv->tx.stop == 0) {
			pmp_priv->tx.stop = 1;
			RTW_INFO("%s: pkt tx is running...\n", __func__);
			msleep(5);
		}
		pmp_priv->tx.stop = 0;
		pmp_priv->tx.count = 1;
		SetPacketTx(Adapter);
	}
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}

NDIS_STATUS oid_rt_pro_set_single_tone_tx_hdl(struct oid_par_priv *poid_par_priv)
{
	u32		bStartTest;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	bStartTest = *((u32 *)poid_par_priv->information_buf);

	_irqlevel_changed_(&oldirql, LOWER);
	SetSingleToneTx(Adapter, (u8)bStartTest);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}

NDIS_STATUS oid_rt_pro_set_modulation_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

NDIS_STATUS oid_rt_pro_trigger_gpio_hdl(struct oid_par_priv *poid_par_priv)
{
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);

	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;

	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);
	rtw_hal_set_hwreg(Adapter, HW_VAR_TRIGGER_GPIO_0, 0);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ****************  oid_rtl_seg_81_80_00   section end ****************
 * ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro8711_join_bss_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_read_register_hdl(struct oid_par_priv *poid_par_priv)
{

	pRW_Reg	RegRWStruct;
	u32		offset, width;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	RegRWStruct = (pRW_Reg)poid_par_priv->information_buf;
	offset = RegRWStruct->offset;
	width = RegRWStruct->width;

	if (offset > 0xFFF)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);

	switch (width) {
	case 1:
		RegRWStruct->value = rtw_read8(Adapter, offset);
		break;
	case 2:
		RegRWStruct->value = rtw_read16(Adapter, offset);
		break;
	default:
		width = 4;
		RegRWStruct->value = rtw_read32(Adapter, offset);
		break;
	}

	_irqlevel_changed_(&oldirql, RAISE);

	*poid_par_priv->bytes_rw = width;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_write_register_hdl(struct oid_par_priv *poid_par_priv)
{

	pRW_Reg	RegRWStruct;
	u32		offset, width, value;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	padapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	RegRWStruct = (pRW_Reg)poid_par_priv->information_buf;
	offset = RegRWStruct->offset;
	width = RegRWStruct->width;
	value = RegRWStruct->value;

	if (offset > 0xFFF)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);

	switch (RegRWStruct->width) {
	case 1:
		if (value > 0xFF) {
			status = NDIS_STATUS_NOT_ACCEPTED;
			break;
		}
		rtw_write8(padapter, offset, (u8)value);
		break;
	case 2:
		if (value > 0xFFFF) {
			status = NDIS_STATUS_NOT_ACCEPTED;
			break;
		}
		rtw_write16(padapter, offset, (u16)value);
		break;
	case 4:
		rtw_write32(padapter, offset, value);
		break;
	default:
		status = NDIS_STATUS_NOT_ACCEPTED;
		break;
	}

	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_burst_read_register_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_burst_write_register_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_write_txcmd_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_read16_eeprom_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_write16_eeprom_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro8711_wi_poll_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro8711_pkt_loss_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_rd_attrib_mem_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_wr_attrib_mem_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS  oid_rt_pro_set_rf_intfs_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_poll_rx_status_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_cfg_debug_message_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_data_rate_ex_hdl(struct oid_par_priv *poid_par_priv)
{
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);

	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, LOWER);

	if (rtw_setdatarate_cmd(Adapter, poid_par_priv->information_buf) != _SUCCESS)
		status = NDIS_STATUS_NOT_ACCEPTED;

	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ----------------------------------------------------------------------------- */
NDIS_STATUS oid_rt_get_thermal_meter_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	u8 thermal = 0;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	_irqlevel_changed_(&oldirql, LOWER);
	GetThermalMeter(Adapter, &thermal);
	_irqlevel_changed_(&oldirql, RAISE);

	*(u32 *)poid_par_priv->information_buf = (u32)thermal;
	*poid_par_priv->bytes_rw = sizeof(u32);


	return status;
}
/* ----------------------------------------------------------------------------- */
NDIS_STATUS oid_rt_pro_read_tssi_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_power_tracking_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	/*	if (poid_par_priv->type_of_oid != SET_OID)
	 *		return NDIS_STATUS_NOT_ACCEPTED; */

	if (poid_par_priv->information_buf_len < sizeof(u8))
		return NDIS_STATUS_INVALID_LENGTH;

	_irqlevel_changed_(&oldirql, LOWER);
	if (poid_par_priv->type_of_oid == SET_OID) {
		u8 enable;

		enable = *(u8 *)poid_par_priv->information_buf;

		SetPowerTracking(Adapter, enable);
	} else
		GetPowerTracking(Adapter, (u8 *)poid_par_priv->information_buf);
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ----------------------------------------------------------------------------- */
NDIS_STATUS oid_rt_pro_set_basic_rate_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_qry_pwrstate_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_set_pwrstate_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_h2c_set_rate_table_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_h2c_get_rate_table_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

/* ****************  oid_rtl_seg_87_12_00   section start **************** */
NDIS_STATUS oid_rt_pro_encryption_ctrl_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_add_sta_info_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_dele_sta_info_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

NDIS_STATUS oid_rt_pro_query_dr_variable_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_rx_packet_type_hdl(struct oid_par_priv *poid_par_priv)
{
	return NDIS_STATUS_SUCCESS;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_read_efuse_hdl(struct oid_par_priv *poid_par_priv)
{

	PEFUSE_ACCESS_STRUCT pefuse;
	u8 *data;
	u16 addr = 0, cnts = 0, max_available_size = 0;
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(EFUSE_ACCESS_STRUCT))
		return NDIS_STATUS_INVALID_LENGTH;

	pefuse = (PEFUSE_ACCESS_STRUCT)poid_par_priv->information_buf;
	addr = pefuse->start_addr;
	cnts = pefuse->cnts;
	data = pefuse->data;


	EFUSE_GetEfuseDefinition(Adapter, EFUSE_WIFI, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, (PVOID)&max_available_size, _FALSE);

	if ((addr + cnts) > max_available_size) {
		return NDIS_STATUS_NOT_ACCEPTED;
	}

	_irqlevel_changed_(&oldirql, LOWER);
	if (rtw_efuse_access(Adapter, _FALSE, addr, cnts, data) == _FAIL) {
		status = NDIS_STATUS_FAILURE;
	} else
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_write_efuse_hdl(struct oid_par_priv *poid_par_priv)
{

	PEFUSE_ACCESS_STRUCT pefuse;
	u8 *data;
	u16 addr = 0, cnts = 0, max_available_size = 0;
	NDIS_STATUS status = NDIS_STATUS_SUCCESS;
	PADAPTER Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	pefuse = (PEFUSE_ACCESS_STRUCT)poid_par_priv->information_buf;
	addr = pefuse->start_addr;
	cnts = pefuse->cnts;
	data = pefuse->data;


	EFUSE_GetEfuseDefinition(Adapter, EFUSE_WIFI, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, (PVOID)&max_available_size, _FALSE);

	if ((addr + cnts) > max_available_size) {
		return NDIS_STATUS_NOT_ACCEPTED;
	}

	_irqlevel_changed_(&oldirql, LOWER);
	if (rtw_efuse_access(Adapter, _TRUE, addr, cnts, data) == _FAIL)
		status = NDIS_STATUS_FAILURE;
	_irqlevel_changed_(&oldirql, RAISE);


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_rw_efuse_pgpkt_hdl(struct oid_par_priv *poid_par_priv)
{

	PPGPKT_STRUCT	ppgpkt;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);



	*poid_par_priv->bytes_rw = 0;

	if (poid_par_priv->information_buf_len < sizeof(PGPKT_STRUCT))
		return NDIS_STATUS_INVALID_LENGTH;

	ppgpkt = (PPGPKT_STRUCT)poid_par_priv->information_buf;

	_irqlevel_changed_(&oldirql, LOWER);

	if (poid_par_priv->type_of_oid == QUERY_OID) {

		Efuse_PowerSwitch(Adapter, _FALSE, _TRUE);
		if (Efuse_PgPacketRead(Adapter, ppgpkt->offset, ppgpkt->data, _FALSE) == _TRUE)
			*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
		else
			status = NDIS_STATUS_FAILURE;
		Efuse_PowerSwitch(Adapter, _FALSE, _FALSE);
	} else {

		Efuse_PowerSwitch(Adapter, _TRUE, _TRUE);
		if (Efuse_PgPacketWrite(Adapter, ppgpkt->offset, ppgpkt->word_en, ppgpkt->data, _FALSE) == _TRUE)
			*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
		else
			status = NDIS_STATUS_FAILURE;
		Efuse_PowerSwitch(Adapter, _TRUE, _FALSE);
	}

	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_get_efuse_current_size_hdl(struct oid_par_priv *poid_par_priv)
{

	u16 size;
	u8 ret;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	_irqlevel_changed_(&oldirql, LOWER);
	ret = efuse_GetCurrentSize(Adapter, &size);
	_irqlevel_changed_(&oldirql, RAISE);
	if (ret == _SUCCESS) {
		*(u32 *)poid_par_priv->information_buf = size;
		*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;
	} else
		status = NDIS_STATUS_FAILURE;


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_get_efuse_max_size_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);


	if (poid_par_priv->type_of_oid != QUERY_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(u32))
		return NDIS_STATUS_INVALID_LENGTH;

	*(u32 *)poid_par_priv->information_buf = efuse_GetMaxSize(Adapter);
	*poid_par_priv->bytes_rw = poid_par_priv->information_buf_len;



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_efuse_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status;



	if (poid_par_priv->type_of_oid == QUERY_OID)
		status = oid_rt_pro_read_efuse_hdl(poid_par_priv);
	else
		status = oid_rt_pro_write_efuse_hdl(poid_par_priv);



	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_pro_efuse_map_hdl(struct oid_par_priv *poid_par_priv)
{
	u8		*data;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PADAPTER	Adapter = (PADAPTER)(poid_par_priv->adapter_context);
	u16	mapLen = 0;



	EFUSE_GetEfuseDefinition(Adapter, EFUSE_WIFI, TYPE_EFUSE_MAP_LEN, (PVOID)&mapLen, _FALSE);

	*poid_par_priv->bytes_rw = 0;

	if (poid_par_priv->information_buf_len < mapLen)
		return NDIS_STATUS_INVALID_LENGTH;

	data = (u8 *)poid_par_priv->information_buf;

	_irqlevel_changed_(&oldirql, LOWER);

	if (poid_par_priv->type_of_oid == QUERY_OID) {

		if (rtw_efuse_map_read(Adapter, 0, mapLen, data) == _SUCCESS)
			*poid_par_priv->bytes_rw = mapLen;
		else {
			status = NDIS_STATUS_FAILURE;
		}
	} else {
		/* SET_OID */

		if (rtw_efuse_map_write(Adapter, 0, mapLen, data) == _SUCCESS)
			*poid_par_priv->bytes_rw = mapLen;
		else {
			status = NDIS_STATUS_FAILURE;
		}
	}

	_irqlevel_changed_(&oldirql, RAISE);



	return status;
}

NDIS_STATUS oid_rt_set_crystal_cap_hdl(struct oid_par_priv *poid_par_priv)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	return status;
}

NDIS_STATUS oid_rt_set_rx_packet_type_hdl(struct oid_par_priv *poid_par_priv)
{

	u8		rx_pkt_type;
	/*	u32		rcr_val32; */
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	/*	PADAPTER	padapter = (PADAPTER)(poid_par_priv->adapter_context); */



	if (poid_par_priv->type_of_oid != SET_OID)
		return NDIS_STATUS_NOT_ACCEPTED;

	if (poid_par_priv->information_buf_len < sizeof(u8))
		return NDIS_STATUS_INVALID_LENGTH;

	rx_pkt_type = *((u8 *)poid_par_priv->information_buf); /* 4 */

	return status;
}

NDIS_STATUS oid_rt_pro_set_tx_agc_offset_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

NDIS_STATUS oid_rt_pro_set_pkt_test_mode_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}

unsigned int mp_ioctl_xmit_packet_hdl(struct oid_par_priv *poid_par_priv)
{
	PMP_XMIT_PARM pparm;
	PADAPTER padapter;
	struct mp_priv *pmp_priv;
	struct pkt_attrib *pattrib;


	pparm = (PMP_XMIT_PARM)poid_par_priv->information_buf;
	padapter = (PADAPTER)poid_par_priv->adapter_context;
	pmp_priv = &padapter->mppriv;

	if (poid_par_priv->type_of_oid == QUERY_OID) {
		pparm->enable = !pmp_priv->tx.stop;
		pparm->count = pmp_priv->tx.sended;
	} else {
		if (pparm->enable == 0)
			pmp_priv->tx.stop = 1;
		else if (pmp_priv->tx.stop == 1) {
			pmp_priv->tx.stop = 0;
			pmp_priv->tx.count = pparm->count;
			pmp_priv->tx.payload = pparm->payload_type;
			pattrib = &pmp_priv->tx.attrib;
			pattrib->pktlen = pparm->length;
			memcpy(pattrib->dst, pparm->da, ETH_ALEN);
			SetPacketTx(padapter);
		} else
			return NDIS_STATUS_FAILURE;
	}

	return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS oid_rt_set_power_down_hdl(struct oid_par_priv *poid_par_priv)
{

	u8		bpwrup;
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
#if defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
	PADAPTER	padapter = (PADAPTER)(poid_par_priv->adapter_context);
#endif

	if (poid_par_priv->type_of_oid != SET_OID) {
		status = NDIS_STATUS_NOT_ACCEPTED;
		return status;
	}


	_irqlevel_changed_(&oldirql, LOWER);

	bpwrup = *(u8 *)poid_par_priv->information_buf;
	/* CALL  the power_down function */
#if defined(CONFIG_RTL8712) /* Linux MP insmod unknown symbol */
	dev_power_down(padapter, bpwrup);
#endif
	_irqlevel_changed_(&oldirql, RAISE);

	/* DEBUG_ERR(("\n <=== Query OID_RT_PRO_READ_REGISTER. */
	/*	Add:0x%08x Width:%d Value:0x%08x\n",RegRWStruct->offset,RegRWStruct->width,RegRWStruct->value)); */


	return status;
}
/* ------------------------------------------------------------------------------ */
NDIS_STATUS oid_rt_get_power_mode_hdl(struct oid_par_priv *poid_par_priv)
{
	return 0;
}
