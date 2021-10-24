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
#ifndef __IEEE80211_H
#define __IEEE80211_H

#define ETH_TYPE_LEN		2
#define PAYLOAD_TYPE_LEN	1

#define NET80211_TU_TO_US	1024		/* unit:us */
#define DEFAULT_BCN_INTERVAL 100 /* 100 ms */

#ifdef CONFIG_AP_MODE

#define RTL_IOCTL_HOSTAPD (SIOCIWFIRSTPRIV + 28)

/* RTL871X_IOCTL_HOSTAPD ioctl() cmd: */
enum {
	RTL871X_HOSTAPD_FLUSH = 1,
	RTL871X_HOSTAPD_ADD_STA = 2,
	RTL871X_HOSTAPD_REMOVE_STA = 3,
	RTL871X_HOSTAPD_GET_INFO_STA = 4,
	/* REMOVED: PRISM2_HOSTAPD_RESET_TXEXC_STA = 5, */
	RTL871X_HOSTAPD_GET_WPAIE_STA = 5,
	RTL871X_SET_ENCRYPTION = 6,
	RTL871X_GET_ENCRYPTION = 7,
	RTL871X_HOSTAPD_SET_FLAGS_STA = 8,
	RTL871X_HOSTAPD_GET_RID = 9,
	RTL871X_HOSTAPD_SET_RID = 10,
	RTL871X_HOSTAPD_SET_ASSOC_AP_ADDR = 11,
	RTL871X_HOSTAPD_SET_GENERIC_ELEMENT = 12,
	RTL871X_HOSTAPD_MLME = 13,
	RTL871X_HOSTAPD_SCAN_REQ = 14,
	RTL871X_HOSTAPD_STA_CLEAR_STATS = 15,
	RTL871X_HOSTAPD_SET_BEACON = 16,
	RTL871X_HOSTAPD_SET_WPS_BEACON = 17,
	RTL871X_HOSTAPD_SET_WPS_PROBE_RESP = 18,
	RTL871X_HOSTAPD_SET_WPS_ASSOC_RESP = 19,
	RTL871X_HOSTAPD_SET_HIDDEN_SSID = 20,
	RTL871X_HOSTAPD_SET_MACADDR_ACL = 21,
	RTL871X_HOSTAPD_ACL_ADD_STA = 22,
	RTL871X_HOSTAPD_ACL_REMOVE_STA = 23,
};

/* STA flags */
#define WLAN_STA_AUTH BIT(0)
#define WLAN_STA_ASSOC BIT(1)
#define WLAN_STA_PS BIT(2)
#define WLAN_STA_TIM BIT(3)
#define WLAN_STA_PERM BIT(4)
#define WLAN_STA_AUTHORIZED BIT(5)
#define WLAN_STA_PENDING_POLL BIT(6) /* pending activity poll not ACKed */
#define WLAN_STA_SHORT_PREAMBLE BIT(7)
#define WLAN_STA_PREAUTH BIT(8)
#define WLAN_STA_WME BIT(9)
#define WLAN_STA_MFP BIT(10)
#define WLAN_STA_HT BIT(11)
#define WLAN_STA_WPS BIT(12)
#define WLAN_STA_MAYBE_WPS BIT(13)
#define WLAN_STA_VHT BIT(14)
#define WLAN_STA_NONERP BIT(31)

#endif

#define IEEE_CMD_SET_WPA_PARAM			1
#define IEEE_CMD_SET_WPA_IE				2
#define IEEE_CMD_SET_ENCRYPTION			3
#define IEEE_CMD_MLME						4

#define IEEE_PARAM_WPA_ENABLED				1
#define IEEE_PARAM_TKIP_COUNTERMEASURES		2
#define IEEE_PARAM_DROP_UNENCRYPTED			3
#define IEEE_PARAM_PRIVACY_INVOKED			4
#define IEEE_PARAM_AUTH_ALGS					5
#define IEEE_PARAM_IEEE_802_1X				6
#define IEEE_PARAM_WPAX_SELECT				7


#define IEEE_MLME_STA_DEAUTH				1
#define IEEE_MLME_STA_DISASSOC			2

#define IEEE_CRYPT_ERR_UNKNOWN_ALG			2
#define IEEE_CRYPT_ERR_UNKNOWN_ADDR			3
#define IEEE_CRYPT_ERR_CRYPT_INIT_FAILED		4
#define IEEE_CRYPT_ERR_KEY_SET_FAILED			5
#define IEEE_CRYPT_ERR_TX_KEY_SET_FAILED		6
#define IEEE_CRYPT_ERR_CARD_CONF_FAILED		7


#define	IEEE_CRYPT_ALG_NAME_LEN			16

#define WPA_CIPHER_NONE	BIT(0)
#define WPA_CIPHER_WEP40	BIT(1)
#define WPA_CIPHER_WEP104 BIT(2)
#define WPA_CIPHER_TKIP	BIT(3)
#define WPA_CIPHER_CCMP	BIT(4)



#define WPA_SELECTOR_LEN 4
extern u8 RTW_WPA_OUI_TYPE[] ;
extern u8 WPA_AUTH_KEY_MGMT_NONE[];
extern u8 WPA_AUTH_KEY_MGMT_UNSPEC_802_1X[];
extern u8 WPA_AUTH_KEY_MGMT_PSK_OVER_802_1X[];
extern u8 WPA_CIPHER_SUITE_NONE[];
extern u8 WPA_CIPHER_SUITE_WEP40[];
extern u8 WPA_CIPHER_SUITE_TKIP[];
extern u8 WPA_CIPHER_SUITE_WRAP[];
extern u8 WPA_CIPHER_SUITE_CCMP[];
extern u8 WPA_CIPHER_SUITE_WEP104[];


#define RSN_HEADER_LEN 4
#define RSN_SELECTOR_LEN 4

extern u16 RSN_VERSION_BSD;
extern u8 RSN_CIPHER_SUITE_NONE[];
extern u8 RSN_CIPHER_SUITE_WEP40[];
extern u8 RSN_CIPHER_SUITE_TKIP[];
extern u8 RSN_CIPHER_SUITE_WRAP[];
extern u8 RSN_CIPHER_SUITE_CCMP[];
extern u8 RSN_CIPHER_SUITE_WEP104[];

/* AKM suite type */
extern u8 WLAN_AKM_8021X[];
extern u8 WLAN_AKM_PSK[];
extern u8 WLAN_AKM_FT_8021X[];
extern u8 WLAN_AKM_FT_PSK[];
extern u8 WLAN_AKM_8021X_SHA256[];
extern u8 WLAN_AKM_PSK_SHA256[];
extern u8 WLAN_AKM_TDLS[];
extern u8 WLAN_AKM_SAE[];
extern u8 WLAN_AKM_FT_OVER_SAE[];
extern u8 WLAN_AKM_8021X_SUITE_B[];
extern u8 WLAN_AKM_8021X_SUITE_B_192[];
extern u8 WLAN_AKM_FILS_SHA256[];
extern u8 WLAN_AKM_FILS_SHA384[];
extern u8 WLAN_AKM_FT_FILS_SHA256[];
extern u8 WLAN_AKM_FT_FILS_SHA384[];

#define WLAN_AKM_TYPE_8021X BIT(0)
#define WLAN_AKM_TYPE_PSK BIT(1)
#define WLAN_AKM_TYPE_FT_8021X BIT(2)
#define WLAN_AKM_TYPE_FT_PSK BIT(3)
#define WLAN_AKM_TYPE_8021X_SHA256 BIT(4)
#define WLAN_AKM_TYPE_PSK_SHA256 BIT(5)
#define WLAN_AKM_TYPE_TDLS BIT(6)
#define WLAN_AKM_TYPE_SAE BIT(7)
#define WLAN_AKM_TYPE_FT_OVER_SAE BIT(8)
#define WLAN_AKM_TYPE_8021X_SUITE_B BIT(9)
#define WLAN_AKM_TYPE_8021X_SUITE_B_192 BIT(10)
#define WLAN_AKM_TYPE_FILS_SHA256 BIT(11)
#define WLAN_AKM_TYPE_FILS_SHA384 BIT(12)
#define WLAN_AKM_TYPE_FT_FILS_SHA256 BIT(13)
#define WLAN_AKM_TYPE_FT_FILS_SHA384 BIT(14)

/* IEEE 802.11i */

#define WPA_REPLAY_COUNTER_LEN 8
#define WPA_NONCE_LEN 32
#define WPA_KEY_RSC_LEN 8
#define WPA_GMK_LEN 32
#define WPA_GTK_MAX_LEN 32

/* IEEE 802.11, 8.5.2 EAPOL-Key frames */
#define WPA_KEY_INFO_KEY_TYPE BIT(3) /* 1 = Pairwise, 0 = Group key */

/* bit4..5 is used in WPA, but is reserved in IEEE 802.11i/RSN */

#define WPA_KEY_INFO_ACK BIT(7)
#define WPA_KEY_INFO_MIC BIT(8)

struct ieee802_1x_hdr {
	u8 version;
	u8 type;
	u16 length;
	/* followed by length octets of data */
};

struct wpa_eapol_key {
	u8 type;
	/* Note: key_info, key_length, and key_data_length are unaligned */
	u8 key_info[2]; /* big endian */
	u8 key_length[2]; /* big endian */
	u8 replay_counter[WPA_REPLAY_COUNTER_LEN];
	u8 key_nonce[WPA_NONCE_LEN];
	u8 key_iv[16];
	u8 key_rsc[WPA_KEY_RSC_LEN];
	u8 key_id[8]; /* Reserved in IEEE 802.11i/RSN */
	u8 key_mic[16];
	u8 key_data_length[2]; /* big endian */
	/* followed by key_data_length bytes of key_data */
};

enum RATEID_IDX {
	RATEID_IDX_BGN_40M_2SS = 0,
	RATEID_IDX_BGN_40M_1SS = 1,
	RATEID_IDX_BGN_20M_2SS_BN = 2,
	RATEID_IDX_BGN_20M_1SS_BN = 3,
	RATEID_IDX_GN_N2SS = 4,
	RATEID_IDX_GN_N1SS = 5,
	RATEID_IDX_BG = 6,
	RATEID_IDX_G = 7,
	RATEID_IDX_B = 8,
	RATEID_IDX_VHT_2SS = 9,
	RATEID_IDX_VHT_1SS = 10,
	RATEID_IDX_MIX1 = 11,
	RATEID_IDX_MIX2 = 12,
	RATEID_IDX_VHT_3SS = 13,
	RATEID_IDX_BGN_3SS = 14,
};

enum RATR_TABLE_MODE {
	RATR_INX_WIRELESS_NGB = 0,	/* BGN 40 Mhz 2SS 1SS */
	RATR_INX_WIRELESS_NG = 1,		/* GN or N */
	RATR_INX_WIRELESS_NB = 2,		/* BGN 20 Mhz 2SS 1SS  or BN */
	RATR_INX_WIRELESS_N = 3,
	RATR_INX_WIRELESS_GB = 4,
	RATR_INX_WIRELESS_G = 5,
	RATR_INX_WIRELESS_B = 6,
	RATR_INX_WIRELESS_MC = 7,
	RATR_INX_WIRELESS_AC_N = 8,
};


enum NETWORK_TYPE {
	WIRELESS_INVALID = 0,
	/* Sub-Element */
	WIRELESS_11B = BIT(0), /* tx: cck only , rx: cck only, hw: cck */
	WIRELESS_11G = BIT(1), /* tx: ofdm only, rx: ofdm & cck, hw: cck & ofdm */
	WIRELESS_11A = BIT(2), /* tx: ofdm only, rx: ofdm only, hw: ofdm only */
	WIRELESS_11_24N = BIT(3), /* tx: MCS only, rx: MCS & cck, hw: MCS & cck */
	WIRELESS_11_5N = BIT(4), /* tx: MCS only, rx: MCS & ofdm, hw: ofdm only */
	WIRELESS_AUTO = BIT(5),
	WIRELESS_11AC = BIT(6),

	/* Combination */
	/* Type for current wireless mode */
	WIRELESS_11BG = (WIRELESS_11B | WIRELESS_11G), /* tx: cck & ofdm, rx: cck & ofdm & MCS, hw: cck & ofdm */
	WIRELESS_11G_24N = (WIRELESS_11G | WIRELESS_11_24N), /* tx: ofdm & MCS, rx: ofdm & cck & MCS, hw: cck & ofdm */
	WIRELESS_11A_5N = (WIRELESS_11A | WIRELESS_11_5N), /* tx: ofdm & MCS, rx: ofdm & MCS, hw: ofdm only */
	WIRELESS_11B_24N = (WIRELESS_11B | WIRELESS_11_24N), /* tx: ofdm & cck & MCS, rx: ofdm & cck & MCS, hw: ofdm & cck */
	WIRELESS_11BG_24N = (WIRELESS_11B | WIRELESS_11G | WIRELESS_11_24N), /* tx: ofdm & cck & MCS, rx: ofdm & cck & MCS, hw: ofdm & cck */
	WIRELESS_11_24AC = (WIRELESS_11B | WIRELESS_11G | WIRELESS_11AC),
	WIRELESS_11_5AC = (WIRELESS_11A | WIRELESS_11AC),


	/* Type for registry default wireless mode */
	WIRELESS_11AGN = (WIRELESS_11A | WIRELESS_11G | WIRELESS_11_24N | WIRELESS_11_5N), /* tx: ofdm & MCS, rx: ofdm & MCS, hw: ofdm only */
	WIRELESS_11ABGN = (WIRELESS_11A | WIRELESS_11B | WIRELESS_11G | WIRELESS_11_24N | WIRELESS_11_5N),
	WIRELESS_MODE_24G = (WIRELESS_11B | WIRELESS_11G | WIRELESS_11_24N),
	WIRELESS_MODE_5G = (WIRELESS_11A | WIRELESS_11_5N | WIRELESS_11AC),
	WIRELESS_MODE_MAX = (WIRELESS_11A | WIRELESS_11B | WIRELESS_11G | WIRELESS_11_24N | WIRELESS_11_5N | WIRELESS_11AC),
};

#define SUPPORTED_24G_NETTYPE_MSK WIRELESS_MODE_24G
#define SUPPORTED_5G_NETTYPE_MSK WIRELESS_MODE_5G

#define IsLegacyOnly(NetType)  ((NetType) == ((NetType) & (WIRELESS_11BG | WIRELESS_11A)))

#define IsSupported24G(NetType) ((NetType) & SUPPORTED_24G_NETTYPE_MSK ? _TRUE : _FALSE)
#define is_supported_5g(NetType) ((NetType) & SUPPORTED_5G_NETTYPE_MSK ? _TRUE : _FALSE)

#define IsEnableHWOFDM(NetType) ((NetType) & (WIRELESS_11G | WIRELESS_11_24N | SUPPORTED_5G_NETTYPE_MSK) ? _TRUE : _FALSE)
#define IsSupportedRxOFDM(NetType) IsEnableHWOFDM(NetType)
#define IsSupportedTxCCK(NetType) ((NetType) & (WIRELESS_11B) ? _TRUE : _FALSE)
#define IsSupportedTxOFDM(NetType) ((NetType) & (WIRELESS_11G | WIRELESS_11A) ? _TRUE : _FALSE)
#define is_supported_ht(NetType) ((NetType) & (WIRELESS_11_24N | WIRELESS_11_5N) ? _TRUE : _FALSE)

#define is_supported_vht(NetType) ((NetType) & (WIRELESS_11AC) ? _TRUE : _FALSE)





struct ieee_param {
	u32 cmd;
	u8 sta_addr[ETH_ALEN];
	union {
		struct {
			u8 name;
			u32 value;
		} wpa_param;
		struct {
			u32 len;
			u8 reserved[32];
			u8 data[0];
		} wpa_ie;
		struct {
			int command;
			int reason_code;
		} mlme;
		struct {
			u8 alg[IEEE_CRYPT_ALG_NAME_LEN];
			u8 set_tx;
			u32 err;
			u8 idx;
			u8 seq[8]; /* sequence counter (set: RX, get: TX) */
			u16 key_len;
			u8 key[0];
		} crypt;
#ifdef CONFIG_AP_MODE
		struct {
			u16 aid;
			u16 capability;
			int flags;
			u8 tx_supp_rates[16];
			struct ieee80211_ht_cap ht_cap;
		} add_sta;
		struct {
			u8	reserved[2];/* for set max_num_sta */
			u8	buf[0];
		} bcn_ie;
#endif

	} u;
};

#ifdef CONFIG_AP_MODE
struct ieee_param_ex {
	u32 cmd;
	u8 sta_addr[ETH_ALEN];
	u8 data[0];
};

struct sta_data {
	u16 aid;
	u16 capability;
	int flags;
	u32 sta_set;
	u8 tx_supp_rates[16];
	u32 tx_supp_rates_len;
	struct ieee80211_ht_cap ht_cap;
	u64	rx_pkts;
	u64	rx_bytes;
	u64	rx_drops;
	u64	tx_pkts;
	u64	tx_bytes;
	u64	tx_drops;
};
#endif


#if WIRELESS_EXT < 17
	#define IW_QUAL_QUAL_INVALID   0x10
	#define IW_QUAL_LEVEL_INVALID  0x20
	#define IW_QUAL_NOISE_INVALID  0x40
	#define IW_QUAL_QUAL_UPDATED   0x1
	#define IW_QUAL_LEVEL_UPDATED  0x2
	#define IW_QUAL_NOISE_UPDATED  0x4
#endif

struct eapol {
	u8 snap[6];
	u16 ethertype;
	u8 version;
	u8 type;
	u16 length;
} __attribute__((packed));

#define MIN_FRAG_THRESHOLD     256U
#define	MAX_FRAG_THRESHOLD     2346U

#define RTW_ERP_INFO_NON_ERP_PRESENT BIT(0)
#define RTW_ERP_INFO_USE_PROTECTION BIT(1)
#define RTW_ERP_INFO_BARKER_PREAMBLE_MODE BIT(2)

/* QoS,QOS */
#define NORMAL_ACK			0

#ifndef ETH_P_PAE
	#define ETH_P_PAE 0x888E /* Port Access Entity (IEEE 802.1X) */
#endif /* ETH_P_PAE */

#define ETH_P_ECONET	0x0018

#ifndef ETH_P_80211_RAW
	#define ETH_P_80211_RAW (ETH_P_ECONET + 1)
#endif

/* IEEE 802.11 defines */

#define P80211_OUI_LEN 3


struct ieee80211_snap_hdr {

	u8    dsap;   /* always 0xAA */
	u8    ssap;   /* always 0xAA */
	u8    ctrl;   /* always 0x03 */
	u8    oui[P80211_OUI_LEN];    /* organizational universal id */

} __attribute__((packed));

#define SNAP_SIZE sizeof(struct ieee80211_snap_hdr)

#define WLAN_REASON_SA_QUERY_TIMEOUT 65532
#define WLAN_REASON_ACTIVE_ROAM 65533
#define WLAN_REASON_JOIN_WRONG_CHANNEL       65534
#define WLAN_REASON_EXPIRATION_CHK 65535

#define WLAN_REASON_IS_PRIVATE(reason) ( \
	reason == WLAN_REASON_EXPIRATION_CHK \
	|| reason == WLAN_REASON_JOIN_WRONG_CHANNEL \
	|| reason == WLAN_REASON_ACTIVE_ROAM \
	|| reason == WLAN_REASON_SA_QUERY_TIMEOUT \
	)

#define IEEE80211_CCK_RATE_LEN		4
#define IEEE80211_NUM_OFDM_RATESLEN	8


#define IEEE80211_CCK_RATE_1MB		        0x02
#define IEEE80211_CCK_RATE_2MB		        0x04
#define IEEE80211_CCK_RATE_5MB		        0x0B
#define IEEE80211_CCK_RATE_11MB		        0x16
#define IEEE80211_OFDM_RATE_LEN		8
#define IEEE80211_OFDM_RATE_6MB		        0x0C
#define IEEE80211_OFDM_RATE_9MB		        0x12
#define IEEE80211_OFDM_RATE_12MB		0x18
#define IEEE80211_OFDM_RATE_18MB		0x24
#define IEEE80211_OFDM_RATE_24MB		0x30
#define IEEE80211_OFDM_RATE_36MB		0x48
#define IEEE80211_OFDM_RATE_48MB		0x60
#define IEEE80211_OFDM_RATE_54MB		0x6C
#define IEEE80211_BASIC_RATE_MASK		0x80

enum MGN_RATE {
	MGN_1M		= 0x02,
	MGN_2M		= 0x04,
	MGN_5_5M	= 0x0B,
	MGN_6M		= 0x0C,
	MGN_9M		= 0x12,
	MGN_11M	= 0x16,
	MGN_12M	= 0x18,
	MGN_18M	= 0x24,
	MGN_24M	= 0x30,
	MGN_36M	= 0x48,
	MGN_48M	= 0x60,
	MGN_54M	= 0x6C,
	MGN_MCS32	= 0x7F,
	MGN_MCS0,
	MGN_MCS1,
	MGN_MCS2,
	MGN_MCS3,
	MGN_MCS4,
	MGN_MCS5,
	MGN_MCS6,
	MGN_MCS7,
	MGN_MCS8,
	MGN_MCS9,
	MGN_MCS10,
	MGN_MCS11,
	MGN_MCS12,
	MGN_MCS13,
	MGN_MCS14,
	MGN_MCS15,
	MGN_MCS16,
	MGN_MCS17,
	MGN_MCS18,
	MGN_MCS19,
	MGN_MCS20,
	MGN_MCS21,
	MGN_MCS22,
	MGN_MCS23,
	MGN_MCS24,
	MGN_MCS25,
	MGN_MCS26,
	MGN_MCS27,
	MGN_MCS28,
	MGN_MCS29,
	MGN_MCS30,
	MGN_MCS31,
	MGN_VHT1SS_MCS0,
	MGN_VHT1SS_MCS1,
	MGN_VHT1SS_MCS2,
	MGN_VHT1SS_MCS3,
	MGN_VHT1SS_MCS4,
	MGN_VHT1SS_MCS5,
	MGN_VHT1SS_MCS6,
	MGN_VHT1SS_MCS7,
	MGN_VHT1SS_MCS8,
	MGN_VHT1SS_MCS9,
	MGN_VHT2SS_MCS0,
	MGN_VHT2SS_MCS1,
	MGN_VHT2SS_MCS2,
	MGN_VHT2SS_MCS3,
	MGN_VHT2SS_MCS4,
	MGN_VHT2SS_MCS5,
	MGN_VHT2SS_MCS6,
	MGN_VHT2SS_MCS7,
	MGN_VHT2SS_MCS8,
	MGN_VHT2SS_MCS9,
	MGN_VHT3SS_MCS0,
	MGN_VHT3SS_MCS1,
	MGN_VHT3SS_MCS2,
	MGN_VHT3SS_MCS3,
	MGN_VHT3SS_MCS4,
	MGN_VHT3SS_MCS5,
	MGN_VHT3SS_MCS6,
	MGN_VHT3SS_MCS7,
	MGN_VHT3SS_MCS8,
	MGN_VHT3SS_MCS9,
	MGN_VHT4SS_MCS0,
	MGN_VHT4SS_MCS1,
	MGN_VHT4SS_MCS2,
	MGN_VHT4SS_MCS3,
	MGN_VHT4SS_MCS4,
	MGN_VHT4SS_MCS5,
	MGN_VHT4SS_MCS6,
	MGN_VHT4SS_MCS7,
	MGN_VHT4SS_MCS8,
	MGN_VHT4SS_MCS9,
	MGN_UNKNOWN
};

#define IS_HT_RATE(_rate)	((_rate) >= MGN_MCS0 && (_rate) <= MGN_MCS31)
#define IS_VHT_RATE(_rate)	((_rate) >= MGN_VHT1SS_MCS0 && (_rate) <= MGN_VHT4SS_MCS9)
#define IS_CCK_RATE(_rate)	((_rate) == MGN_1M || (_rate) == MGN_2M || (_rate) == MGN_5_5M || (_rate) == MGN_11M)
#define IS_OFDM_RATE(_rate)	((_rate) >= MGN_6M && (_rate) <= MGN_54M  && (_rate) != MGN_11M)

#define IS_HT1SS_RATE(_rate) ((_rate) >= MGN_MCS0 && (_rate) <= MGN_MCS7)
#define IS_HT2SS_RATE(_rate) ((_rate) >= MGN_MCS8 && (_rate) <= MGN_MCS15)
#define IS_HT3SS_RATE(_rate) ((_rate) >= MGN_MCS16 && (_rate) <= MGN_MCS23)
#define IS_HT4SS_RATE(_rate) ((_rate) >= MGN_MCS24 && (_rate) <= MGN_MCS31)

#define IS_VHT1SS_RATE(_rate) ((_rate) >= MGN_VHT1SS_MCS0 && (_rate) <= MGN_VHT1SS_MCS9)
#define IS_VHT2SS_RATE(_rate) ((_rate) >= MGN_VHT2SS_MCS0 && (_rate) <= MGN_VHT2SS_MCS9)
#define IS_VHT3SS_RATE(_rate) ((_rate) >= MGN_VHT3SS_MCS0 && (_rate) <= MGN_VHT3SS_MCS9)
#define IS_VHT4SS_RATE(_rate) ((_rate) >= MGN_VHT4SS_MCS0 && (_rate) <= MGN_VHT4SS_MCS9)

#define IS_1T_RATE(_rate)	(IS_CCK_RATE((_rate)) || IS_OFDM_RATE((_rate)) || IS_HT1SS_RATE((_rate)) || IS_VHT1SS_RATE((_rate)))
#define IS_2T_RATE(_rate)	(IS_HT2SS_RATE((_rate)) || IS_VHT2SS_RATE((_rate)))
#define IS_3T_RATE(_rate)	(IS_HT3SS_RATE((_rate)) || IS_VHT3SS_RATE((_rate)))
#define IS_4T_RATE(_rate)	(IS_HT4SS_RATE((_rate)) || IS_VHT4SS_RATE((_rate)))

#define MGN_RATE_STR(_rate) \
	(_rate == MGN_1M) ? "CCK_1M" : \
	(_rate == MGN_2M) ? "CCK_2M" : \
	(_rate == MGN_5_5M) ? "CCK_5.5M" : \
	(_rate == MGN_11M) ? "CCK_11M" : \
	(_rate == MGN_6M) ? "OFDM_6M" : \
	(_rate == MGN_9M) ? "OFDM_9M" : \
	(_rate == MGN_12M) ? "OFDM_12M" : \
	(_rate == MGN_18M) ? "OFDM_18M" : \
	(_rate == MGN_24M) ? "OFDM_24M" : \
	(_rate == MGN_36M) ? "OFDM_36M" : \
	(_rate == MGN_48M) ? "OFDM_48M" : \
	(_rate == MGN_54M) ? "OFDM_54M" : \
	(_rate == MGN_MCS32) ? "MCS32" : \
	(_rate == MGN_MCS0) ? "MCS0" : \
	(_rate == MGN_MCS1) ? "MCS1" : \
	(_rate == MGN_MCS2) ? "MCS2" : \
	(_rate == MGN_MCS3) ? "MCS3" : \
	(_rate == MGN_MCS4) ? "MCS4" : \
	(_rate == MGN_MCS5) ? "MCS5" : \
	(_rate == MGN_MCS6) ? "MCS6" : \
	(_rate == MGN_MCS7) ? "MCS7" : \
	(_rate == MGN_MCS8) ? "MCS8" : \
	(_rate == MGN_MCS9) ? "MCS9" : \
	(_rate == MGN_MCS10) ? "MCS10" : \
	(_rate == MGN_MCS11) ? "MCS11" : \
	(_rate == MGN_MCS12) ? "MCS12" : \
	(_rate == MGN_MCS13) ? "MCS13" : \
	(_rate == MGN_MCS14) ? "MCS14" : \
	(_rate == MGN_MCS15) ? "MCS15" : \
	(_rate == MGN_MCS16) ? "MCS16" : \
	(_rate == MGN_MCS17) ? "MCS17" : \
	(_rate == MGN_MCS18) ? "MCS18" : \
	(_rate == MGN_MCS19) ? "MCS19" : \
	(_rate == MGN_MCS20) ? "MCS20" : \
	(_rate == MGN_MCS21) ? "MCS21" : \
	(_rate == MGN_MCS22) ? "MCS22" : \
	(_rate == MGN_MCS23) ? "MCS23" : \
	(_rate == MGN_MCS24) ? "MCS24" : \
	(_rate == MGN_MCS25) ? "MCS25" : \
	(_rate == MGN_MCS26) ? "MCS26" : \
	(_rate == MGN_MCS27) ? "MCS27" : \
	(_rate == MGN_MCS28) ? "MCS28" : \
	(_rate == MGN_MCS29) ? "MCS29" : \
	(_rate == MGN_MCS30) ? "MCS30" : \
	(_rate == MGN_MCS31) ? "MCS31" : \
	(_rate == MGN_VHT1SS_MCS0) ? "VHT1SMCS0" : \
	(_rate == MGN_VHT1SS_MCS1) ? "VHT1SMCS1" : \
	(_rate == MGN_VHT1SS_MCS2) ? "VHT1SMCS2" : \
	(_rate == MGN_VHT1SS_MCS3) ? "VHT1SMCS3" : \
	(_rate == MGN_VHT1SS_MCS4) ? "VHT1SMCS4" : \
	(_rate == MGN_VHT1SS_MCS5) ? "VHT1SMCS5" : \
	(_rate == MGN_VHT1SS_MCS6) ? "VHT1SMCS6" : \
	(_rate == MGN_VHT1SS_MCS7) ? "VHT1SMCS7" : \
	(_rate == MGN_VHT1SS_MCS8) ? "VHT1SMCS8" : \
	(_rate == MGN_VHT1SS_MCS9) ? "VHT1SMCS9" : \
	(_rate == MGN_VHT2SS_MCS0) ? "VHT2SMCS0" : \
	(_rate == MGN_VHT2SS_MCS1) ? "VHT2SMCS1" : \
	(_rate == MGN_VHT2SS_MCS2) ? "VHT2SMCS2" : \
	(_rate == MGN_VHT2SS_MCS3) ? "VHT2SMCS3" : \
	(_rate == MGN_VHT2SS_MCS4) ? "VHT2SMCS4" : \
	(_rate == MGN_VHT2SS_MCS5) ? "VHT2SMCS5" : \
	(_rate == MGN_VHT2SS_MCS6) ? "VHT2SMCS6" : \
	(_rate == MGN_VHT2SS_MCS7) ? "VHT2SMCS7" : \
	(_rate == MGN_VHT2SS_MCS8) ? "VHT2SMCS8" : \
	(_rate == MGN_VHT2SS_MCS9) ? "VHT2SMCS9" : \
	(_rate == MGN_VHT3SS_MCS0) ? "VHT3SMCS0" : \
	(_rate == MGN_VHT3SS_MCS1) ? "VHT3SMCS1" : \
	(_rate == MGN_VHT3SS_MCS2) ? "VHT3SMCS2" : \
	(_rate == MGN_VHT3SS_MCS3) ? "VHT3SMCS3" : \
	(_rate == MGN_VHT3SS_MCS4) ? "VHT3SMCS4" : \
	(_rate == MGN_VHT3SS_MCS5) ? "VHT3SMCS5" : \
	(_rate == MGN_VHT3SS_MCS6) ? "VHT3SMCS6" : \
	(_rate == MGN_VHT3SS_MCS7) ? "VHT3SMCS7" : \
	(_rate == MGN_VHT3SS_MCS8) ? "VHT3SMCS8" : \
	(_rate == MGN_VHT3SS_MCS9) ? "VHT3SMCS9" : \
	(_rate == MGN_VHT4SS_MCS0) ? "VHT4SMCS0" : \
	(_rate == MGN_VHT4SS_MCS1) ? "VHT4SMCS1" : \
	(_rate == MGN_VHT4SS_MCS2) ? "VHT4SMCS2" : \
	(_rate == MGN_VHT4SS_MCS3) ? "VHT4SMCS3" : \
	(_rate == MGN_VHT4SS_MCS4) ? "VHT4SMCS4" : \
	(_rate == MGN_VHT4SS_MCS5) ? "VHT4SMCS5" : \
	(_rate == MGN_VHT4SS_MCS6) ? "VHT4SMCS6" : \
	(_rate == MGN_VHT4SS_MCS7) ? "VHT4SMCS7" : \
	(_rate == MGN_VHT4SS_MCS8) ? "VHT4SMCS8" : \
	(_rate == MGN_VHT4SS_MCS9) ? "VHT4SMCS9" : "UNKNOWN"

typedef enum _RATE_SECTION {
	CCK = 0,
	OFDM = 1,
	HT_MCS0_MCS7 = 2,
	HT_MCS8_MCS15 = 3,
	HT_MCS16_MCS23 = 4,
	HT_MCS24_MCS31 = 5,
	HT_1SS = HT_MCS0_MCS7,
	HT_2SS = HT_MCS8_MCS15,
	HT_3SS = HT_MCS16_MCS23,
	HT_4SS = HT_MCS24_MCS31,
	VHT_1SSMCS0_1SSMCS9 = 6,
	VHT_2SSMCS0_2SSMCS9 = 7,
	VHT_3SSMCS0_3SSMCS9 = 8,
	VHT_4SSMCS0_4SSMCS9 = 9,
	VHT_1SS = VHT_1SSMCS0_1SSMCS9,
	VHT_2SS = VHT_2SSMCS0_2SSMCS9,
	VHT_3SS = VHT_3SSMCS0_3SSMCS9,
	VHT_4SS = VHT_4SSMCS0_4SSMCS9,
	RATE_SECTION_NUM,
} RATE_SECTION;

const char *rate_section_str(u8 section);

#define IS_CCK_RATE_SECTION(section) ((section) == CCK)
#define IS_OFDM_RATE_SECTION(section) ((section) == OFDM)
#define IS_HT_RATE_SECTION(section) ((section) >= HT_1SS && (section) <= HT_4SS)
#define IS_VHT_RATE_SECTION(section) ((section) >= VHT_1SS && (section) <= VHT_4SS)

extern u8 mgn_rates_cck[];
extern u8 mgn_rates_ofdm[];
extern u8 mgn_rates_mcs0_7[];
extern u8 mgn_rates_mcs8_15[];
extern u8 mgn_rates_mcs16_23[];
extern u8 mgn_rates_mcs24_31[];
extern u8 mgn_rates_vht1ss[];
extern u8 mgn_rates_vht2ss[];
extern u8 mgn_rates_vht3ss[];
extern u8 mgn_rates_vht4ss[];

struct rate_section_ent {
	u8 tx_num; /* value of RF_TX_NUM */
	u8 rate_num;
	u8 *rates;
};

extern struct rate_section_ent rates_by_sections[];

#define rate_section_to_tx_num(section) (rates_by_sections[(section)].tx_num)
#define rate_section_rate_num(section) (rates_by_sections[(section)].rate_num)

/* IEEE 802.11 requires that STA supports concurrent reception of at least
 * three fragmented frames. This define can be increased to support more
 * concurrent frames, but it should be noted that each entry can consume about
 * 2 kB of RAM and increasing cache size will slow down frame reassembly. */

#define WEP_KEYS 4

#define BIP_MAX_KEYID 5
#define BIP_AAD_SIZE  20

struct ieee80211_info_element {
	u8 id;
	u8 len;
	u8 data[0];
} __attribute__((packed));

/*
 * These are the data types that can make up management packets
 *
	u16 auth_algorithm;
	u16 auth_sequence;
	u16 beacon_interval;
	u16 capability;
	u8 current_ap[ETH_ALEN];
	u16 listen_interval;
	struct {
		u16 association_id:14, reserved:2;
	} __attribute__ ((packed));
	u32 time_stamp[2];
	u16 reason;
	u16 status;
*/

/* MAX_RATES_LENGTH needs to be 12.  The spec says 8, and many APs
 * only use 8, and then use extended rates for the remaining supported
 * rates.  Other APs, however, stick all of their supported rates on the
 * main rates information element... */
#define MAX_RATES_LENGTH                  ((u8)12)
#define MAX_RATES_EX_LENGTH               ((u8)16)
#define MAX_NETWORK_COUNT                  128
#define IEEE80211_SOFTMAC_SCAN_TIME	  400
/* (HZ / 2) */
#define IEEE80211_SOFTMAC_ASSOC_RETRY_TIME (HZ * 2)

#define CRC_LENGTH                 4U

#define MAX_WPA_IE_LEN (256)
#define MAX_WPS_IE_LEN (512)
#define MAX_P2P_IE_LEN (256)
#define MAX_WFD_IE_LEN (128)

#define NETWORK_EMPTY_ESSID (1<<0)
#define NETWORK_HAS_OFDM    (1<<1)
#define NETWORK_HAS_CCK     (1<<2)

#define IEEE80211_DTIM_MBCAST 4
#define IEEE80211_DTIM_UCAST 2
#define IEEE80211_DTIM_VALID 1
#define IEEE80211_DTIM_INVALID 0

#define IEEE80211_PS_DISABLED 0
#define IEEE80211_PS_UNICAST IEEE80211_DTIM_UCAST
#define IEEE80211_PS_MBCAST IEEE80211_DTIM_MBCAST
#define IW_ESSID_MAX_SIZE 32
/*
join_res:
-1: authentication fail
-2: association fail
> 0: TID
*/

#define DEFAULT_MAX_SCAN_AGE (15 * HZ)
#define DEFAULT_FTS 2346
#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(x) ((u8 *)(x))[0], ((u8 *)(x))[1], ((u8 *)(x))[2], ((u8 *)(x))[3], ((u8 *)(x))[4], ((u8 *)(x))[5]
#define MAC_SFMT "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx"
#define MAC_SARG(x) ((u8*)(x)),((u8*)(x)) + 1,((u8*)(x)) + 2,((u8*)(x)) + 3,((u8*)(x)) + 4,((u8*)(x)) + 5
#define IP_FMT "%d.%d.%d.%d"
#define IP_ARG(x) ((u8 *)(x))[0], ((u8 *)(x))[1], ((u8 *)(x))[2], ((u8 *)(x))[3]
#define PORT_FMT "%u"
#define PORT_ARG(x) ntohs(*((u16 *)(x)))

extern __inline int is_multicast_mac_addr(const u8 *addr)
{
	return (addr[0] != 0xff) && (0x01 & addr[0]);
}

extern __inline int is_broadcast_mac_addr(const u8 *addr)
{
	return ((addr[0] == 0xff) && (addr[1] == 0xff) && (addr[2] == 0xff) &&   \
		(addr[3] == 0xff) && (addr[4] == 0xff) && (addr[5] == 0xff));
}

extern __inline int is_zero_mac_addr(const u8 *addr)
{
	return ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) &&   \
		(addr[3] == 0x00) && (addr[4] == 0x00) && (addr[5] == 0x00));
}

#define CFG_IEEE80211_RESERVE_FCS (1<<0)
#define CFG_IEEE80211_COMPUTE_FCS (1<<1)

#define TID_NUM	16

#define IEEE_A            (1<<0)
#define IEEE_B            (1<<1)
#define IEEE_G            (1<<2)
#define IEEE_MODE_MASK    (IEEE_A | IEEE_B | IEEE_G)

#define CATEGORY_IS_GROUP_PRIVACY(cat) \
	(cat == WLAN_CATEGORY_MESH_ACTION || cat == WLAN_CATEGORY_MULTIHOP_ACTION)

#define CATEGORY_IS_NON_ROBUST(cat) \
	(cat == WLAN_CATEGORY_PUBLIC \
	|| cat == WLAN_CATEGORY_HT \
	|| cat == WLAN_CATEGORY_WNM_UNPROTECTED \
	|| cat == WLAN_CATEGORY_SELF_PROTECTED \
	|| cat == WLAN_CATEGORY_VHT \
	|| cat == WLAN_CATEGORY_VENDOR_SPECIFIC)

#define CATEGORY_IS_ROBUST(cat) !CATEGORY_IS_NON_ROBUST(cat)

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
/*Commit 3cb57df37bf3c87c7bbd2bd6f94d9d48c1c8e2ae v4.13-rc1 */
enum ieee80211_pub_actioncode_extend {
	WLAN_PUB_ACTION_20_40_BSS_COEX = 0,
	WLAN_PUB_ACTION_DSE_ENABLEMENT = 1,
	WLAN_PUB_ACTION_DSE_DEENABLEMENT = 2,
	WLAN_PUB_ACTION_DSE_REG_LOC_ANN = 3,
	WLAN_PUB_ACTION_DSE_MSMT_REQ = 5,
	WLAN_PUB_ACTION_DSE_MSMT_RESP = 6,
	WLAN_PUB_ACTION_MSMT_PILOT = 7,
	WLAN_PUB_ACTION_DSE_PC = 8,
	WLAN_PUB_ACTION_VENDOR_SPECIFIC = 9,
	WLAN_PUB_ACTION_GAS_INITIAL_REQ = 10,
	WLAN_PUB_ACTION_GAS_INITIAL_RESP = 11,
	WLAN_PUB_ACTION_GAS_COMEBACK_REQ = 12,
	WLAN_PUB_ACTION_GAS_COMEBACK_RESP = 13,
	WLAN_PUB_ACTION_LOC_TRACK_NOTI = 15,
	WLAN_PUB_ACTION_QAB_REQUEST_FRAME = 16,
	WLAN_PUB_ACTION_QAB_RESPONSE_FRAME = 17,
	WLAN_PUB_ACTION_QMF_POLICY = 18,
	WLAN_PUB_ACTION_QMF_POLICY_CHANGE = 19,
	WLAN_PUB_ACTION_QLOAD_REQUEST = 20,
	WLAN_PUB_ACTION_QLOAD_REPORT = 21,
	WLAN_PUB_ACTION_HCCA_TXOP_ADVERT = 22,
	WLAN_PUB_ACTION_HCCA_TXOP_RESPONSE = 23,
	WLAN_PUB_ACTION_PUBLIC_KEY = 24,
	WLAN_PUB_ACTION_CHANNEL_AVAIL_QUERY = 25,
	WLAN_PUB_ACTION_CHANNEL_SCHEDULE_MGMT = 26,
	WLAN_PUB_ACTION_CONTACT_VERI_SIGNAL = 27,
	WLAN_PUB_ACTION_GDD_ENABLEMENT_REQ = 28,
	WLAN_PUB_ACTION_GDD_ENABLEMENT_RESP = 29,
	WLAN_PUB_ACTION_NETWORK_CHANNEL_CONTROL = 30,
	WLAN_PUB_ACTION_WHITE_SPACE_MAP_ANN = 31,
	WLAN_PUB_ACTION_FTM_REQUEST = 32,
	WLAN_PUB_ACTION_FTM = 33,
	WLAN_PUB_ACTION_FILS_DISCOVERY = 34,
};
#endif
#define ACT_PUBLIC_MAX (WLAN_PUB_ACTION_LOC_TRACK_NOTI + 1)

#ifdef CONFIG_TDLS
#define WLAN_TDLS_DISCOVERY_RESPONSE = 14 /* it's used in public action frame */

#define	TUNNELED_PROBE_REQ	15
#define	TUNNELED_PROBE_RSP	16
#endif /* CONFIG_TDLS */

/*20/40 BSS Coexistence element */
#define RTW_WLAN_20_40_BSS_COEX_40MHZ_INTOL         BIT(1)
#define RTW_WLAN_20_40_BSS_COEX_20MHZ_WIDTH_REQ     BIT(2)
#define RTW_WLAN_20_40_BSS_COEX_OBSS_EXEMPT_REQ     BIT(3)
#define RTW_WLAN_20_40_BSS_COEX_OBSS_EXEMPT_GRNT    BIT(4)

#ifndef WLAN_CATEGORY_FAST_BBS_TRANSITION
/*Commit 1806239dec0dacde373f0b53f076319f6c6d95cb v5.14-rc1 */
#define WLAN_CATEGORY_FAST_BBS_TRANSITION 6
#endif
/*IEEE 802.11r action code*/
#ifdef CONFIG_RTW_80211R
enum rtw_ieee80211_ft_actioncode {
	RTW_WLAN_ACTION_FT_RESV,
	RTW_WLAN_ACTION_FT_REQ,
	RTW_WLAN_ACTION_FT_RSP,
	RTW_WLAN_ACTION_FT_CONF,
	RTW_WLAN_ACTION_FT_ACK,
	RTW_WLAN_ACTION_FT_MAX,
};
#endif

#ifdef CONFIG_RTW_WNM
enum rtw_ieee80211_wnm_actioncode {
	RTW_WLAN_ACTION_WNM_BTM_QUERY = 6,
	RTW_WLAN_ACTION_WNM_BTM_REQ = 7,
	RTW_WLAN_ACTION_WNM_BTM_RSP = 8,
};
#endif

#define WME_OUI_SUBTYPE_INFORMATION_ELEMENT 0
#define WME_OUI_SUBTYPE_PARAMETER_ELEMENT 1
#define WME_OUI_SUBTYPE_TSPEC_ELEMENT 2

#define OUI_BROADCOM 0x00904c /* Broadcom (Epigram) */

#define VENDOR_HT_CAPAB_OUI_TYPE 0x33 /* 00-90-4c:0x33 */

#define IEEE80211_PREQ_IS_GATE_FLAG (1 << 0)

/* for kernel 3.14 , both value are changed to IEEE80211_CHAN_NO_IR*/
#define RTW_IEEE80211_CHAN_NO_IBSS          (1 << 2)
#define RTW_IEEE80211_CHAN_PASSIVE_SCAN     (1 << 1)

/* Represent channel details, subset of ieee80211_channel */
struct rtw_ieee80211_channel {
	/* enum ieee80211_band band; */
	/* u16 center_freq; */
	u16 hw_value;
	u32 flags;
	/* int max_antenna_gain; */
	/* int max_power; */
	/* int max_reg_power; */
	/* bool beacon_found; */
	/* u32 orig_flags; */
	/* int orig_mag; */
	/* int orig_mpwr; */
};

#define CHAN_FMT \
	/*"band:%d, "*/ \
	/*"center_freq:%u, "*/ \
	"hw_value:%u, " \
	"flags:0x%08x" \
	/*"max_antenna_gain:%d\n"*/ \
	/*"max_power:%d\n"*/ \
	/*"max_reg_power:%d\n"*/ \
	/*"beacon_found:%u\n"*/ \
	/*"orig_flags:0x%08x\n"*/ \
	/*"orig_mag:%d\n"*/ \
	/*"orig_mpwr:%d\n"*/

#define CHAN_ARG(channel) \
	/*(channel)->band*/ \
	/*, (channel)->center_freq*/ \
	(channel)->hw_value \
	, (channel)->flags \
	/*, (channel)->max_antenna_gain*/ \
	/*, (channel)->max_power*/ \
	/*, (channel)->max_reg_power*/ \
	/*, (channel)->beacon_found*/ \
	/*, (channel)->orig_flags*/ \
	/*, (channel)->orig_mag*/ \
	/*, (channel)->orig_mpwr*/ \

/* Parsed Information Elements */
struct rtw_ieee802_11_elems {
	u8 *ssid;
	u8 ssid_len;
	u8 *supp_rates;
	u8 supp_rates_len;
	u8 *fh_params;
	u8 fh_params_len;
	u8 *ds_params;
	u8 ds_params_len;
	u8 *cf_params;
	u8 cf_params_len;
	u8 *tim;
	u8 tim_len;
	u8 *ibss_params;
	u8 ibss_params_len;
	u8 *challenge;
	u8 challenge_len;
	u8 *erp_info;
	u8 erp_info_len;
	u8 *ext_supp_rates;
	u8 ext_supp_rates_len;
	u8 *wpa_ie;
	u8 wpa_ie_len;
	u8 *rsn_ie;
	u8 rsn_ie_len;
	u8 *wme;
	u8 wme_len;
	u8 *wme_tspec;
	u8 wme_tspec_len;
	u8 *wps_ie;
	u8 wps_ie_len;
	u8 *power_cap;
	u8 power_cap_len;
	u8 *supp_channels;
	u8 supp_channels_len;
	u8 *mdie;
	u8 mdie_len;
	u8 *ftie;
	u8 ftie_len;
	u8 *timeout_int;
	u8 timeout_int_len;
	u8 *ht_capabilities;
	u8 ht_capabilities_len;
	u8 *ht_operation;
	u8 ht_operation_len;
	u8 *vendor_ht_cap;
	u8 vendor_ht_cap_len;
	u8 *vht_capabilities;
	u8 vht_capabilities_len;
	u8 *vht_operation;
	u8 vht_operation_len;
	u8 *vht_op_mode_notify;
	u8 vht_op_mode_notify_len;
	u8 *rm_en_cap;
	u8 rm_en_cap_len;
#ifdef CONFIG_RTW_MESH
	u8 *preq;
	u8 preq_len;
	u8 *prep;
	u8 prep_len;
	u8 *perr;
	u8 perr_len;
	u8 *rann;
	u8 rann_len;
#endif
};

enum ParseRes { ParseOK = 0, ParseUnknown = 1, ParseFailed = -1 };

enum ParseRes rtw_ieee802_11_parse_elems(u8 *start, uint len,
				struct rtw_ieee802_11_elems *elems,
				int show_errors);

u8 *rtw_set_fixed_ie(unsigned char *pbuf, unsigned int len, unsigned char *source, unsigned int *frlen);
u8 *rtw_set_ie(u8 *pbuf, sint index, uint len, const u8 *source, uint *frlen);

enum secondary_ch_offset {
	SCN = 0, /* no secondary channel */
	SCA = 1, /* secondary channel above */
	SCB = 3,  /* secondary channel below */
};
u8 secondary_ch_offset_to_hal_ch_offset(u8 ch_offset);
u8 hal_ch_offset_to_secondary_ch_offset(u8 ch_offset);
u8 *rtw_set_ie_ch_switch(u8 *buf, u32 *buf_len, u8 ch_switch_mode, u8 new_ch, u8 ch_switch_cnt);
u8 *rtw_set_ie_secondary_ch_offset(u8 *buf, u32 *buf_len, u8 secondary_ch_offset);
u8 *rtw_set_ie_mesh_ch_switch_parm(u8 *buf, u32 *buf_len, u8 ttl, u8 flags, u16 reason, u16 precedence);

u8 *rtw_get_ie(const u8 *pbuf, sint index, sint *len, sint limit);
int rtw_remove_ie_g_rate(u8 *ie, uint *ie_len, uint offset, u8 eid);
u8 *rtw_get_ie_ex(const u8 *in_ie, uint in_len, u8 eid, const u8 *oui, u8 oui_len, u8 *ie, uint *ielen);
int rtw_ies_remove_ie(u8 *ies, uint *ies_len, uint offset, u8 eid, u8 *oui, u8 oui_len);

void rtw_set_supported_rate(u8 *SupportedRates, uint mode) ;

#define GET_RSN_CAP_MFP_OPTION(cap)	LE_BITS_TO_2BYTE(((u8 *)(cap)), 6, 2)

#define MFP_NO			0
#define MFP_INVALID		1
#define MFP_OPTIONAL	2
#define MFP_REQUIRED	3

struct rsne_info {
	u8 *gcs;
	u16 pcs_cnt;
	u8 *pcs_list;
	u16 akm_cnt;
	u8 *akm_list;
	u8 *cap;
	u16 pmkid_cnt;
	u8 *pmkid_list;
	u8 *gmcs;

	u8 err;
};
int rtw_rsne_info_parse(const u8 *ie, uint ie_len, struct rsne_info *info);

unsigned char *rtw_get_wpa_ie(unsigned char *pie, int *wpa_ie_len, int limit);
unsigned char *rtw_get_wpa2_ie(unsigned char *pie, int *rsn_ie_len, int limit);
int rtw_get_wpa_cipher_suite(u8 *s);
int rtw_get_wpa2_cipher_suite(u8 *s);
int rtw_get_wapi_ie(u8 *in_ie, uint in_len, u8 *wapi_ie, u16 *wapi_len);
int rtw_parse_wpa_ie(u8 *wpa_ie, int wpa_ie_len, int *group_cipher, int *pairwise_cipher, u32 *akm);
int rtw_parse_wpa2_ie(u8 *wpa_ie, int wpa_ie_len, int *group_cipher, int *pairwise_cipher, u32 *akm, u8 *mfp_opt);

int rtw_get_sec_ie(u8 *in_ie, uint in_len, u8 *rsn_ie, u16 *rsn_len, u8 *wpa_ie, u16 *wpa_len);

u8 rtw_is_wps_ie(u8 *ie_ptr, uint *wps_ielen);
u8 *rtw_get_wps_ie_from_scan_queue(u8 *in_ie, uint in_len, u8 *wps_ie, uint *wps_ielen, enum bss_type frame_type);
u8 *rtw_get_wps_ie(const u8 *in_ie, uint in_len, u8 *wps_ie, uint *wps_ielen);
u8 *rtw_get_wps_attr(u8 *wps_ie, uint wps_ielen, u16 target_attr_id , u8 *buf_attr, u32 *len_attr);
u8 *rtw_get_wps_attr_content(u8 *wps_ie, uint wps_ielen, u16 target_attr_id , u8 *buf_content, uint *len_content);

/**
 * for_each_ie - iterate over continuous IEs
 * @ie:
 * @buf:
 * @buf_len:
 */
#define for_each_ie(ie, buf, buf_len) \
	for (ie = (void *)buf; (((u8 *)ie) - ((u8 *)buf) + 1) < buf_len; ie = (void *)(((u8 *)ie) + *(((u8 *)ie)+1) + 2))

void dump_ies(void *sel, const u8 *buf, u32 buf_len);

#ifdef CONFIG_80211N_HT
#define HT_SC_OFFSET_MAX 4
extern const char *const _ht_sc_offset_str[];
#define ht_sc_offset_str(sc) (((sc) >= HT_SC_OFFSET_MAX) ? _ht_sc_offset_str[2] : _ht_sc_offset_str[(sc)])

void dump_ht_cap_ie_content(void *sel, const u8 *buf, u32 buf_len);
#endif

void dump_wps_ie(void *sel, const u8 *ie, u32 ie_len);

void rtw_ies_get_chbw(u8 *ies, int ies_len, u8 *ch, u8 *bw, u8 *offset, u8 ht, u8 vht);

void rtw_bss_get_chbw(WLAN_BSSID_EX *bss, u8 *ch, u8 *bw, u8 *offset, u8 ht, u8 vht);

bool rtw_is_chbw_grouped(u8 ch_a, u8 bw_a, u8 offset_a
	, u8 ch_b, u8 bw_b, u8 offset_b);
void rtw_sync_chbw(u8 *req_ch, u8 *req_bw, u8 *req_offset
	, u8 *g_ch, u8 *g_bw, u8 *g_offset);

u32 rtw_get_p2p_merged_ies_len(u8 *in_ie, u32 in_len);
int rtw_p2p_merge_ies(u8 *in_ie, u32 in_len, u8 *merge_ie);
void dump_p2p_ie(void *sel, const u8 *ie, u32 ie_len);
u8 *rtw_get_p2p_ie(const u8 *in_ie, int in_len, u8 *p2p_ie, uint *p2p_ielen);
u8 *rtw_get_p2p_attr(u8 *p2p_ie, uint p2p_ielen, u8 target_attr_id, u8 *buf_attr, u32 *len_attr);
u8 *rtw_get_p2p_attr_content(u8 *p2p_ie, uint p2p_ielen, u8 target_attr_id, u8 *buf_content, uint *len_content);
u32 rtw_set_p2p_attr_content(u8 *pbuf, u8 attr_id, u16 attr_len, u8 *pdata_attr);
uint rtw_del_p2p_ie(u8 *ies, uint ies_len_ori, const char *msg);
uint rtw_del_p2p_attr(u8 *ie, uint ielen_ori, u8 attr_id);
u8 *rtw_bss_ex_get_p2p_ie(WLAN_BSSID_EX *bss_ex, u8 *p2p_ie, uint *p2p_ielen);
void rtw_bss_ex_del_p2p_ie(WLAN_BSSID_EX *bss_ex);
void rtw_bss_ex_del_p2p_attr(WLAN_BSSID_EX *bss_ex, u8 attr_id);

void dump_wfd_ie(void *sel, const u8 *ie, u32 ie_len);
u8 *rtw_get_wfd_ie(const u8 *in_ie, int in_len, u8 *wfd_ie, uint *wfd_ielen);
u8 *rtw_get_wfd_attr(u8 *wfd_ie, uint wfd_ielen, u8 target_attr_id, u8 *buf_attr, u32 *len_attr);
u8 *rtw_get_wfd_attr_content(u8 *wfd_ie, uint wfd_ielen, u8 target_attr_id, u8 *buf_content, uint *len_content);
uint rtw_del_wfd_ie(u8 *ies, uint ies_len_ori, const char *msg);
uint rtw_del_wfd_attr(u8 *ie, uint ielen_ori, u8 attr_id);
u8 *rtw_bss_ex_get_wfd_ie(WLAN_BSSID_EX *bss_ex, u8 *wfd_ie, uint *wfd_ielen);
void rtw_bss_ex_del_wfd_ie(WLAN_BSSID_EX *bss_ex);
void rtw_bss_ex_del_wfd_attr(WLAN_BSSID_EX *bss_ex, u8 attr_id);

uint	rtw_get_rateset_len(u8	*rateset);

struct registry_priv;
int rtw_generate_ie(struct registry_priv *pregistrypriv);

int rtw_get_bit_value_from_ieee_value(u8 val);

uint	rtw_is_cckrates_included(u8 *rate);

uint	rtw_is_cckratesonly_included(u8 *rate);
uint rtw_get_cckrate_size(u8 *rate,u32 rate_length);
int rtw_check_network_type(unsigned char *rate, int ratelen, int channel);

u8 rtw_check_invalid_mac_address(u8 *mac_addr, u8 check_local_bit);
void rtw_macaddr_cfg(u8 *out, const u8 *hw_mac_addr);

u16 rtw_mcs_rate(u8 rf_type, u8 bw_40MHz, u8 short_GI, unsigned char *MCS_rate);
u8	rtw_ht_mcsset_to_nss(u8 *supp_mcs_set);
u32	rtw_ht_mcs_set_to_bitmap(u8 *mcs_set, u8 nss);

int rtw_action_frame_parse(const u8 *frame, u32 frame_len, u8 *category, u8 *action);

u8 key_2char2num(u8 hch, u8 lch);
u8 str_2char2num(u8 hch, u8 lch);
void macstr2num(u8 *dst, u8 *src);
u8 convert_ip_addr(u8 hch, u8 mch, u8 lch);
int wifirate2_ratetbl_inx(unsigned char rate);


#endif /* IEEE80211_H */
