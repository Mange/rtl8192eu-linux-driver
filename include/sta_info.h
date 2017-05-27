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
#ifndef __STA_INFO_H_
#define __STA_INFO_H_


#define IBSS_START_MAC_ID	2
#define NUM_STA MACID_NUM_SW_LIMIT
#define NUM_ACL 16

#ifdef CONFIG_TDLS
#define MAX_ALLOWED_TDLS_STA_NUM	4
#endif

enum sta_info_update_type {
	STA_INFO_UPDATE_NONE = 0, 
	STA_INFO_UPDATE_BW = BIT(0), 
	STA_INFO_UPDATE_RATE = BIT(1),	
	STA_INFO_UPDATE_PROTECTION_MODE = BIT(2),
	STA_INFO_UPDATE_CAP = BIT(3),
	STA_INFO_UPDATE_HT_CAP = BIT(4),
	STA_INFO_UPDATE_VHT_CAP = BIT(5),
	STA_INFO_UPDATE_ALL = STA_INFO_UPDATE_BW
							|STA_INFO_UPDATE_RATE
							|STA_INFO_UPDATE_PROTECTION_MODE
							|STA_INFO_UPDATE_CAP
							|STA_INFO_UPDATE_HT_CAP
							|STA_INFO_UPDATE_VHT_CAP,
	STA_INFO_UPDATE_MAX
};

//if mode ==0, then the sta is allowed once the addr is hit.
//if mode ==1, then the sta is rejected once the addr is non-hit.
struct rtw_wlan_acl_node {
        _list		        list;
        u8       addr[ETH_ALEN];
        u8       valid;
};

//mode=0, disable
//mode=1, accept unless in deny list
//mode=2, deny unless in accept list
struct wlan_acl_pool {
	int mode;
	int num;
	struct rtw_wlan_acl_node aclnode[NUM_ACL];
	_queue	acl_node_q;
};

typedef struct _RSSI_STA{
	s32	UndecoratedSmoothedPWDB;
	s32	UndecoratedSmoothedCCK;
	s32	UndecoratedSmoothedOFDM;
	u32	OFDM_pkt;
	u64	PacketMap;
	u8	ValidBit;
}RSSI_STA, *PRSSI_STA;

struct	stainfo_stats	{

	u64 rx_mgnt_pkts;
		u64 rx_beacon_pkts;
		u64 rx_probereq_pkts;
		u64 rx_probersp_pkts;
		u64 rx_probersp_bm_pkts;
		u64 rx_probersp_uo_pkts;
	u64 rx_ctrl_pkts;
	u64 rx_data_pkts;
	u64 rx_data_qos_pkts[TID_NUM];
	u64	last_rx_mgnt_pkts;
		u64 last_rx_beacon_pkts;
		u64 last_rx_probereq_pkts;
		u64 last_rx_probersp_pkts;
		u64 last_rx_probersp_bm_pkts;
		u64 last_rx_probersp_uo_pkts;
	u64	last_rx_ctrl_pkts;
	u64	last_rx_data_pkts;
	u64 last_rx_data_qos_pkts[TID_NUM];
#ifdef CONFIG_TDLS
	u64 rx_tdls_disc_rsp_pkts;
	u64 last_rx_tdls_disc_rsp_pkts;
#endif
	u64	rx_bytes;
	u64	rx_drops;

	u64	tx_pkts;
	u64	tx_bytes;
	u64  tx_drops;
};

#ifndef DBG_SESSION_TRACKER
#define DBG_SESSION_TRACKER 0
#endif

/* session tracker status */
#define ST_STATUS_NONE		0
#define ST_STATUS_CHECK		BIT0
#define ST_STATUS_ESTABLISH	BIT1
#define ST_STATUS_EXPIRE	BIT2

#define ST_EXPIRE_MS (10 * 1000)

struct session_tracker {
	_list list; /* session_tracker_queue */
	u32 local_naddr;
	u16 local_port;
	u32 remote_naddr;
	u16 remote_port;
	u32 set_time;
	u8 status;
};

/* session tracker cmd */
#define ST_CMD_ADD 0
#define ST_CMD_DEL 1
#define ST_CMD_CHK 2

struct st_cmd_parm {
	u8 cmd;
	struct sta_info *sta;
	u32 local_naddr; /* TODO: IPV6 */
	u16 local_port;
	u32 remote_naddr; /* TODO: IPV6 */
	u16 remote_port;
};

typedef bool (*st_match_rule)(_adapter *adapter, u8 *local_naddr, u8 *local_port, u8 *remote_naddr, u8 *remote_port);

struct st_register {
	u8 s_proto;
	st_match_rule rule;
};

#define SESSION_TRACKER_REG_ID_WFD 0
#define SESSION_TRACKER_REG_ID_NUM 1

struct st_ctl_t {
	struct st_register reg[SESSION_TRACKER_REG_ID_NUM];
	_queue tracker_q;
};

void rtw_st_ctl_init(struct st_ctl_t *st_ctl);
void rtw_st_ctl_deinit(struct st_ctl_t *st_ctl);
void rtw_st_ctl_register(struct st_ctl_t *st_ctl, u8 st_reg_id, struct st_register *reg);
void rtw_st_ctl_unregister(struct st_ctl_t *st_ctl, u8 st_reg_id);
bool rtw_st_ctl_chk_reg_s_proto(struct st_ctl_t *st_ctl, u8 s_proto);
bool rtw_st_ctl_chk_reg_rule(struct st_ctl_t *st_ctl, _adapter *adapter, u8 *local_naddr, u8 *local_port, u8 *remote_naddr, u8 *remote_port);
void dump_st_ctl(void *sel, struct st_ctl_t *st_ctl);

#ifdef CONFIG_TDLS
struct TDLS_PeerKey {
	u8 kck[16]; /* TPK-KCK */
	u8 tk[16]; /* TPK-TK; only CCMP will be used */
} ;
#endif //CONFIG_TDLS

#ifdef DBG_RX_DFRAME_RAW_DATA
struct sta_recv_dframe_info {

	u8 sta_data_rate;
	u8 sta_sgi;
	u8 sta_bw_mode;
	s8 sta_mimo_signal_strength[4];
	s8 sta_RxPwr[4];
	u8 sta_ofdm_snr[4];
};
#endif

struct sta_info {

	_lock	lock;
	_list	list; //free_sta_queue
	_list	hash_list; //sta_hash
	//_list asoc_list; //20061114
	//_list sleep_list;//sleep_q
	//_list wakeup_list;//wakeup_q
	_adapter *padapter;
	
	struct sta_xmit_priv sta_xmitpriv;
	struct sta_recv_priv sta_recvpriv;
	
#ifdef DBG_RX_DFRAME_RAW_DATA
	struct sta_recv_dframe_info  sta_dframe_info;
#endif
	_queue sleep_q;
	unsigned int sleepq_len;
	
	uint state;
	uint aid;
	uint mac_id;
	uint qos_option;
	u8	hwaddr[ETH_ALEN];
	u16 hwseq;
	u8	ra_rpt_linked;

	uint	ieee8021x_blocked;	//0: allowed, 1:blocked 
	uint	dot118021XPrivacy; //aes, tkip...
	union Keytype	dot11tkiptxmickey;
	union Keytype	dot11tkiprxmickey;
	union Keytype	dot118021x_UncstKey;	
	union pn48		dot11txpn;			// PN48 used for Unicast xmit
#ifdef CONFIG_GTK_OL
	u8 kek[RTW_KEK_LEN];
	u8 kck[RTW_KCK_LEN];
	u8 replay_ctr[RTW_REPLAY_CTR_LEN];
#endif //CONFIG_GTK_OL
#ifdef CONFIG_IEEE80211W
	union pn48		dot11wtxpn;			// PN48 used for Unicast mgmt xmit.
	_timer dot11w_expire_timer;
#endif //CONFIG_IEEE80211W
	union pn48		dot11rxpn;			// PN48 used for Unicast recv.


	u8	bssrateset[16];
	u32	bssratelen;
	s32  rssi;
	s32	signal_quality;
	
	u8	cts2self;
	u8	rtsen;

	u8	raid;
	u8 	init_rate;
	u64	ra_mask;
	u8	wireless_mode;	// NETWORK_TYPE
	u8	bw_mode;

	u8	ldpc;
	u8	stbc;

#ifdef CONFIG_BEAMFORMING
	u16 txbf_paid;
	u16 txbf_gid;
#endif
		
	struct stainfo_stats sta_stats;

#ifdef CONFIG_TDLS
	u32	tdls_sta_state;
	u8	SNonce[32];
	u8	ANonce[32];
	u32	TDLS_PeerKey_Lifetime;
	u16	TPK_count;
	_timer	TPK_timer;
	struct TDLS_PeerKey	tpk;
#ifdef CONFIG_TDLS_CH_SW	
	u16	ch_switch_time;
	u16	ch_switch_timeout;
	//u8	option;
	_timer	ch_sw_timer;
	_timer	delay_timer;
	_timer	stay_on_base_chnl_timer;
	_timer	ch_sw_monitor_timer;
#endif	
	_timer handshake_timer;
	u8 alive_count;
	_timer	pti_timer;
	u8	TDLS_RSNIE[20];	/* Save peer's RSNIE, used for sending TDLS_SETUP_RSP */
#endif /* CONFIG_TDLS */

	//for A-MPDU TX, ADDBA timeout check	
	_timer addba_retry_timer;
	
	//for A-MPDU Rx reordering buffer control 
	struct recv_reorder_ctrl recvreorder_ctrl[TID_NUM];
	ATOMIC_T continual_no_rx_packet[TID_NUM];
	//for A-MPDU Tx
	//unsigned char		ampdu_txen_bitmap;
	u16	BA_starting_seqctrl[16];
	

#ifdef CONFIG_80211N_HT
	struct ht_priv	htpriv;
#endif

#ifdef CONFIG_80211AC_VHT
	struct vht_priv	vhtpriv;
#endif

	//Notes:	
	//STA_Mode:
	//curr_network(mlme_priv/security_priv/qos/ht) + sta_info: (STA & AP) CAP/INFO	
	//scan_q: AP CAP/INFO

	//AP_Mode:
	//curr_network(mlme_priv/security_priv/qos/ht) : AP CAP/INFO
	//sta_info: (AP & STA) CAP/INFO

	unsigned int expire_to;

#ifdef CONFIG_AP_MODE

	_list asoc_list;
	_list auth_list;

	unsigned int auth_seq;
	unsigned int authalg;
	unsigned char chg_txt[128];

	u16 capability;	
	int flags;	

	int dot8021xalg;//0:disable, 1:psk, 2:802.1x
	int wpa_psk;//0:disable, bit(0): WPA, bit(1):WPA2
	int wpa_group_cipher;
	int wpa2_group_cipher;
	int wpa_pairwise_cipher;
	int wpa2_pairwise_cipher;	

	u8 bpairwise_key_installed;

#ifdef CONFIG_NATIVEAP_MLME
	u8 wpa_ie[32];

	u8 nonerp_set;
	u8 no_short_slot_time_set;
	u8 no_short_preamble_set;
	u8 no_ht_gf_set;
	u8 no_ht_set;
	u8 ht_20mhz_set;
	u8 ht_40mhz_intolerant;
#endif	// CONFIG_NATIVEAP_MLME

#ifdef CONFIG_ATMEL_RC_PATCH
	u8 flag_atmel_rc;
#endif

	u8 qos_info;

	u8 max_sp_len;
	u8 uapsd_bk;//BIT(0): Delivery enabled, BIT(1): Trigger enabled
	u8 uapsd_be;
	u8 uapsd_vi;
	u8 uapsd_vo;	

	u8 has_legacy_ac;
	unsigned int sleepq_ac_len;

#ifdef CONFIG_P2P
	//p2p priv data
	u8 is_p2p_device;
	u8 p2p_status_code;

	//p2p client info
	u8 dev_addr[ETH_ALEN];
	//u8 iface_addr[ETH_ALEN];//= hwaddr[ETH_ALEN]
	u8 dev_cap;
	u16 config_methods;
	u8 primary_dev_type[8];
	u8 num_of_secdev_type;
	u8 secdev_types_list[32];// 32/8 == 4;
	u16 dev_name_len;
	u8 dev_name[32];	
#endif //CONFIG_P2P

#ifdef CONFIG_WFD
	u8 op_wfd_mode;
#endif

#ifdef CONFIG_TX_MCAST2UNI
	u8 under_exist_checking;
#endif	// CONFIG_TX_MCAST2UNI
	
	u8 keep_alive_trycnt;

#ifdef CONFIG_AUTO_AP_MODE
	u8 isrc; //this device is rc
	u16 pid; // pairing id
#endif

#endif	// CONFIG_AP_MODE	

#ifdef CONFIG_IOCTL_CFG80211
	u8 *passoc_req;
	u32 assoc_req_len;
#endif

	//for DM
	RSSI_STA	 rssi_stat;
	
	//ODM_STA_INFO_T
	// ================ODM Relative Info=======================
	// Please be care, dont declare too much structure here. It will cost memory * STA support num.
	//
	//
	// 2011/10/20 MH Add for ODM STA info.	
	//
	// Driver Write
	u8		bValid;				// record the sta status link or not?
	//u8		WirelessMode;		// 
	u8		IOTPeer;			// Enum value.	HT_IOT_PEER_E
	// ODM Write
	//1 PHY_STATUS_INFO
	u8		RSSI_Path[4];		// 
	u8		RSSI_Ave;
	u8		RXEVM[4];
	u8		RXSNR[4];

	u8		rssi_level;			//for Refresh RA mask
	// ODM Write
	//1 TX_INFO (may changed by IC)
	//TX_INFO_T		pTxInfo;				// Define in IC folder. Move lower layer.
	//
	// ================ODM Relative Info=======================
	//

	/* To store the sequence number of received management frame */
	u16 RxMgmtFrameSeqNum;

	struct st_ctl_t st_ctl;
};

#define sta_rx_pkts(sta) \
	(sta->sta_stats.rx_mgnt_pkts \
	+ sta->sta_stats.rx_ctrl_pkts \
	+ sta->sta_stats.rx_data_pkts)

#define sta_last_rx_pkts(sta) \
	(sta->sta_stats.last_rx_mgnt_pkts \
	+ sta->sta_stats.last_rx_ctrl_pkts \
	+ sta->sta_stats.last_rx_data_pkts)

#define sta_rx_data_pkts(sta) \
	(sta->sta_stats.rx_data_pkts)

#define sta_rx_data_qos_pkts(sta, i) \
	(sta->sta_stats.rx_data_qos_pkts[i])

#define sta_last_rx_data_pkts(sta) \
	(sta->sta_stats.last_rx_data_pkts)

#define sta_last_rx_data_qos_pkts(sta, i) \
	(sta->sta_stats.last_rx_data_qos_pkts[i])

#define sta_rx_mgnt_pkts(sta) \
	(sta->sta_stats.rx_mgnt_pkts)

#define sta_last_rx_mgnt_pkts(sta) \
	(sta->sta_stats.last_rx_mgnt_pkts)

#define sta_rx_beacon_pkts(sta) \
	(sta->sta_stats.rx_beacon_pkts)

#define sta_last_rx_beacon_pkts(sta) \
	(sta->sta_stats.last_rx_beacon_pkts)

#define sta_rx_probereq_pkts(sta) \
	(sta->sta_stats.rx_probereq_pkts)

#define sta_last_rx_probereq_pkts(sta) \
	(sta->sta_stats.last_rx_probereq_pkts)

#define sta_rx_probersp_pkts(sta) \
	(sta->sta_stats.rx_probersp_pkts)

#define sta_last_rx_probersp_pkts(sta) \
	(sta->sta_stats.last_rx_probersp_pkts)

#define sta_rx_probersp_bm_pkts(sta) \
	(sta->sta_stats.rx_probersp_bm_pkts)

#define sta_last_rx_probersp_bm_pkts(sta) \
	(sta->sta_stats.last_rx_probersp_bm_pkts)

#define sta_rx_probersp_uo_pkts(sta) \
	(sta->sta_stats.rx_probersp_uo_pkts)

#define sta_last_rx_probersp_uo_pkts(sta) \
	(sta->sta_stats.last_rx_probersp_uo_pkts)

#define sta_update_last_rx_pkts(sta) \
	do { \
		sta->sta_stats.last_rx_mgnt_pkts = sta->sta_stats.rx_mgnt_pkts; \
		sta->sta_stats.last_rx_beacon_pkts = sta->sta_stats.rx_beacon_pkts; \
		sta->sta_stats.last_rx_probereq_pkts = sta->sta_stats.rx_probereq_pkts; \
		sta->sta_stats.last_rx_probersp_pkts = sta->sta_stats.rx_probersp_pkts; \
		sta->sta_stats.last_rx_probersp_bm_pkts = sta->sta_stats.rx_probersp_bm_pkts; \
		sta->sta_stats.last_rx_probersp_uo_pkts = sta->sta_stats.rx_probersp_uo_pkts; \
		sta->sta_stats.last_rx_ctrl_pkts = sta->sta_stats.rx_ctrl_pkts; \
		sta->sta_stats.last_rx_data_pkts = sta->sta_stats.rx_data_pkts; \
	} while(0)

#define STA_RX_PKTS_ARG(sta) \
	sta->sta_stats.rx_mgnt_pkts \
	, sta->sta_stats.rx_ctrl_pkts \
	, sta->sta_stats.rx_data_pkts

#define STA_LAST_RX_PKTS_ARG(sta) \
	sta->sta_stats.last_rx_mgnt_pkts \
	, sta->sta_stats.last_rx_ctrl_pkts \
	, sta->sta_stats.last_rx_data_pkts

#define STA_RX_PKTS_DIFF_ARG(sta) \
	sta->sta_stats.rx_mgnt_pkts - sta->sta_stats.last_rx_mgnt_pkts \
	, sta->sta_stats.rx_ctrl_pkts - sta->sta_stats.last_rx_ctrl_pkts \
	, sta->sta_stats.rx_data_pkts -sta->sta_stats.last_rx_data_pkts

#define STA_PKTS_FMT "(m:%llu, c:%llu, d:%llu)"

#ifdef CONFIG_WFD
#define STA_OP_WFD_MODE(sta) (sta)->op_wfd_mode
#define STA_SET_OP_WFD_MODE(sta, mode) (sta)->op_wfd_mode = (mode)
#else
#define STA_OP_WFD_MODE(sta) 0
#define STA_SET_OP_WFD_MODE(sta, mode) do {} while (0)
#endif

struct	sta_priv {
	
	u8 *pallocated_stainfo_buf;
	u8 *pstainfo_buf;
	_queue	free_sta_queue;
	
	_lock sta_hash_lock;
	_list   sta_hash[NUM_STA];
	int asoc_sta_count;
	_queue sleep_q;
	_queue wakeup_q;
	
	_adapter *padapter;

	u32 adhoc_expire_to;

#ifdef CONFIG_AP_MODE
	_list asoc_list;
	_list auth_list;
	_lock asoc_list_lock;
	_lock auth_list_lock;
	u8 asoc_list_cnt;
	u8 auth_list_cnt;

	unsigned int auth_to;  //sec, time to expire in authenticating.
	unsigned int assoc_to; //sec, time to expire before associating.
	unsigned int expire_to; //sec , time to expire after associated.
	
	/* pointers to STA info; based on allocated AID or NULL if AID free
	 * AID is in the range 1-2007, so sta_aid[0] corresponders to AID 1
	 * and so on
	 */
	struct sta_info *sta_aid[NUM_STA];

	u16 sta_dz_bitmap;//only support 15 stations, staion aid bitmap for sleeping sta.
	u16 tim_bitmap;//only support 15 stations, aid=0~15 mapping bit0~bit15	

	u16 max_num_sta;

	struct wlan_acl_pool acl_list;
#endif

#ifdef CONFIG_ATMEL_RC_PATCH
	u8 atmel_rc_pattern [6];
#endif

};


__inline static u32 wifi_mac_hash(u8 *mac)
{
        u32 x;

        x = mac[0];
        x = (x << 2) ^ mac[1];
        x = (x << 2) ^ mac[2];
        x = (x << 2) ^ mac[3];
        x = (x << 2) ^ mac[4];
        x = (x << 2) ^ mac[5];

        x ^= x >> 8;
        x  = x & (NUM_STA - 1);
		
        return x;
}


extern u32	_rtw_init_sta_priv(struct sta_priv *pstapriv);
extern u32	_rtw_free_sta_priv(struct sta_priv *pstapriv);

#define stainfo_offset_valid(offset) (offset < NUM_STA && offset >= 0)
int rtw_stainfo_offset(struct sta_priv *stapriv, struct sta_info *sta);
struct sta_info *rtw_get_stainfo_by_offset(struct sta_priv *stapriv, int offset);

extern struct sta_info *rtw_alloc_stainfo(struct	sta_priv *pstapriv, u8 *hwaddr);
extern u32	rtw_free_stainfo(_adapter *padapter , struct sta_info *psta);
extern void rtw_free_all_stainfo(_adapter *padapter);
extern struct sta_info *rtw_get_stainfo(struct sta_priv *pstapriv, u8 *hwaddr);
extern u32 rtw_init_bcmc_stainfo(_adapter* padapter);
extern struct sta_info* rtw_get_bcmc_stainfo(_adapter* padapter);
extern u8 rtw_access_ctrl(_adapter *padapter, u8 *mac_addr);

#endif //_STA_INFO_H_

