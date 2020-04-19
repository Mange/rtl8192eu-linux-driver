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
#define _HAL_COM_PHYCFG_C_

#include <drv_types.h>
#include <hal_data.h>

#define PG_TXPWR_1PATH_BYTE_NUM_2G 18
#define PG_TXPWR_BASE_BYTE_NUM_2G 11

#define PG_TXPWR_1PATH_BYTE_NUM_5G 24
#define PG_TXPWR_BASE_BYTE_NUM_5G 14

#define PG_TXPWR_MSB_DIFF_S4BIT(_pg_v) (((_pg_v) & 0xf0) >> 4)
#define PG_TXPWR_LSB_DIFF_S4BIT(_pg_v) ((_pg_v) & 0x0f)
#define PG_TXPWR_MSB_DIFF_TO_S8BIT(_pg_v) ((PG_TXPWR_MSB_DIFF_S4BIT(_pg_v) & BIT3) ? (PG_TXPWR_MSB_DIFF_S4BIT(_pg_v) | 0xF0) : PG_TXPWR_MSB_DIFF_S4BIT(_pg_v))
#define PG_TXPWR_LSB_DIFF_TO_S8BIT(_pg_v) ((PG_TXPWR_LSB_DIFF_S4BIT(_pg_v) & BIT3) ? (PG_TXPWR_LSB_DIFF_S4BIT(_pg_v) | 0xF0) : PG_TXPWR_LSB_DIFF_S4BIT(_pg_v))
#define IS_PG_TXPWR_BASE_INVALID(hal_spec, _base) ((_base) > hal_spec->txgi_max)
#define IS_PG_TXPWR_DIFF_INVALID(_diff) ((_diff) > 7 || (_diff) < -8)
#define PG_TXPWR_INVALID_BASE 255
#define PG_TXPWR_INVALID_DIFF 8

#if !IS_PG_TXPWR_DIFF_INVALID(PG_TXPWR_INVALID_DIFF)
#error "PG_TXPWR_DIFF definition has problem"
#endif

#define PG_TXPWR_SRC_PG_DATA	0
#define PG_TXPWR_SRC_IC_DEF		1
#define PG_TXPWR_SRC_DEF		2
#define PG_TXPWR_SRC_NUM		3

const char *const _pg_txpwr_src_str[] = {
	"PG_DATA",
	"IC_DEF",
	"DEF",
	"UNKNOWN"
};

#define pg_txpwr_src_str(src) (((src) >= PG_TXPWR_SRC_NUM) ? _pg_txpwr_src_str[PG_TXPWR_SRC_NUM] : _pg_txpwr_src_str[(src)])

#ifndef DBG_PG_TXPWR_READ
#define DBG_PG_TXPWR_READ 0
#endif

#if DBG_PG_TXPWR_READ
static void dump_pg_txpwr_info_2g(void *sel, TxPowerInfo24G *txpwr_info, u8 rfpath_num, u8 max_tx_cnt)
{
	int path, group, tx_idx;

	RTW_PRINT_SEL(sel, "2.4G\n");
	RTW_PRINT_SEL(sel, "CCK-1T base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (group = 0; group < MAX_CHNL_GROUP_24G; group++)
		_RTW_PRINT_SEL(sel, "G%02d ", group);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (group = 0; group < MAX_CHNL_GROUP_24G; group++)
			_RTW_PRINT_SEL(sel, "%3u ", txpwr_info->IndexCCK_Base[path][group]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "CCK diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dT ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->CCK_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40-1S base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (group = 0; group < MAX_CHNL_GROUP_24G - 1; group++)
		_RTW_PRINT_SEL(sel, "G%02d ", group);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (group = 0; group < MAX_CHNL_GROUP_24G - 1; group++)
			_RTW_PRINT_SEL(sel, "%3u ", txpwr_info->IndexBW40_Base[path][group]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "OFDM diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dT ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->OFDM_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW20 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW20_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW40_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");
}

static void dump_pg_txpwr_info_5g(void *sel, TxPowerInfo5G *txpwr_info, u8 rfpath_num, u8 max_tx_cnt)
{
	int path, group, tx_idx;

	RTW_PRINT_SEL(sel, "5G\n");
	RTW_PRINT_SEL(sel, "BW40-1S base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (group = 0; group < MAX_CHNL_GROUP_5G; group++)
		_RTW_PRINT_SEL(sel, "G%02d ", group);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (group = 0; group < MAX_CHNL_GROUP_5G; group++)
			_RTW_PRINT_SEL(sel, "%3u ", txpwr_info->IndexBW40_Base[path][group]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "OFDM diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dT ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->OFDM_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW20 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW20_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW40_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW80 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW80_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW160 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++)
		_RTW_PRINT_SEL(sel, "%dS ", path + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", txpwr_info->BW160_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");
}
#endif /* DBG_PG_TXPWR_READ */

const struct map_t pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 168,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE, 0xEE, 0xEE,
			0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x04, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x04, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
			0xEE, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24,
			0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x04, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x04, 0xEE,
			0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE)
	);

#ifdef CONFIG_RTL8188E
static const struct map_t rtl8188e_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24)
	);
#endif

#ifdef CONFIG_RTL8188F
static const struct map_t rtl8188f_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x27, 0x27, 0x27, 0x27, 0x27, 0x24)
	);
#endif

#ifdef CONFIG_RTL8188GTV
static const struct map_t rtl8188gtv_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x27, 0x27, 0x27, 0x27, 0x27, 0x24)
	);
#endif

#ifdef CONFIG_RTL8723B
static const struct map_t rtl8723b_pg_txpwr_def_info =
	MAP_ENT(0xB8, 2, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0xE0)
		, MAPSEG_ARRAY_ENT(0x3A, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0xE0)
	);
#endif

#ifdef CONFIG_RTL8703B
static const struct map_t rtl8703b_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02)
	);
#endif

#ifdef CONFIG_RTL8723D
static const struct map_t rtl8723d_pg_txpwr_def_info =
	MAP_ENT(0xB8, 2, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02)
		, MAPSEG_ARRAY_ENT(0x3A, 12,
			0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x21, 0x21, 0x21, 0x21, 0x21, 0x02)
	);
#endif

#ifdef CONFIG_RTL8192E
static const struct map_t rtl8192e_pg_txpwr_def_info =
	MAP_ENT(0xB8, 2, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 14,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE)
		, MAPSEG_ARRAY_ENT(0x3A, 14,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE)
	);
#endif

#ifdef CONFIG_RTL8821A
static const struct map_t rtl8821a_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 39,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00)
	);
#endif

#ifdef CONFIG_RTL8821C
static const struct map_t rtl8821c_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 54,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
			0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEC, 0xFF, 0xFF, 0xFF, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02)
	);
#endif

#ifdef CONFIG_RTL8710B
static const struct map_t rtl8710b_pg_txpwr_def_info =
	MAP_ENT(0xC8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x20, 12,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x20)
	);
#endif

#ifdef CONFIG_RTL8812A
static const struct map_t rtl8812a_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 82,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xFF, 0xFF,
			0xFF, 0xFF, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x02, 0xEE, 0xFF, 0xFF, 0xEE, 0xFF, 0x00, 0xEE, 0xFF, 0xFF, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xFF, 0xFF, 0xFF, 0xFF, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x02, 0xEE, 0xFF, 0xFF, 0xEE, 0xFF,
			0x00, 0xEE)
	);
#endif

#ifdef CONFIG_RTL8822B
static const struct map_t rtl8822b_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 82,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xFF, 0xFF,
			0xFF, 0xFF, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x02, 0xEE, 0xFF, 0xFF, 0xEE, 0xFF, 0xEC, 0xEC, 0xFF, 0xFF, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xFF, 0xFF, 0xFF, 0xFF, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x02, 0xEE, 0xFF, 0xFF, 0xEE, 0xFF,
			0xEC, 0xEC)
	);
#endif

#ifdef CONFIG_RTL8814A
static const struct map_t rtl8814a_pg_txpwr_def_info =
	MAP_ENT(0xB8, 1, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 168,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xEE, 0xEE,
			0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x02, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x00, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x02, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
			0x00, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02,
			0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
			0x2A, 0x2A, 0x2A, 0x2A, 0x02, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x00, 0xEE, 0xEE, 0xEE, 0x2D, 0x2D,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x02, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
			0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x02, 0xEE,
			0xEE, 0xEE, 0xEE, 0xEE, 0x00, 0xEE, 0xEE, 0xEE)
	);
#endif

#ifdef CONFIG_RTL8192F/*use 8192F default,no document*/
static const struct map_t rtl8192f_pg_txpwr_def_info =
	MAP_ENT(0xB8, 2, 0xFF
		, MAPSEG_ARRAY_ENT(0x10, 14,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE)
		, MAPSEG_ARRAY_ENT(0x3A, 14,
			0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x24, 0xEE, 0xEE)
	);
#endif

const struct map_t *hal_pg_txpwr_def_info(_adapter *adapter)
{
	u8 interface_type = 0;
	const struct map_t *map = NULL;

	interface_type = rtw_get_intf_type(adapter);

	switch (rtw_get_chip_type(adapter)) {
#ifdef CONFIG_RTL8723B
	case RTL8723B:
		map = &rtl8723b_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8703B
	case RTL8703B:
		map = &rtl8703b_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8723D
	case RTL8723D:
		map = &rtl8723d_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8188E
	case RTL8188E:
		map = &rtl8188e_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8188F
	case RTL8188F:
		map = &rtl8188f_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8188GTV
	case RTL8188GTV:
		map = &rtl8188gtv_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8812A
	case RTL8812:
		map = &rtl8812a_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8821A
	case RTL8821:
		map = &rtl8821a_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8192E
	case RTL8192E:
		map = &rtl8192e_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8814A
	case RTL8814A:
		map = &rtl8814a_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8822B
	case RTL8822B:
		map = &rtl8822b_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8821C
	case RTL8821C:
		map = &rtl8821c_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8710B
	case RTL8710B:
		map = &rtl8710b_pg_txpwr_def_info;
		break;
#endif
#ifdef CONFIG_RTL8192F
	case RTL8192F:
		map = &rtl8192f_pg_txpwr_def_info;
		break;
#endif
	}

	if (map == NULL) {
		RTW_ERR("%s: unknown chip_type:%u\n"
			, __func__, rtw_get_chip_type(adapter));
		rtw_warn_on(1);
	}

	return map;
}

static u8 hal_chk_pg_txpwr_info_2g(_adapter *adapter, TxPowerInfo24G *pwr_info)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 path, group, tx_idx;

	if (pwr_info == NULL || !hal_chk_band_cap(adapter, BAND_CAP_2G))
		return _SUCCESS;

	for (path = 0; path < MAX_RF_PATH; path++) {
		if (!HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path))
			continue;
		for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
			if (IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexCCK_Base[path][group])
				|| IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexBW40_Base[path][group]))
				return _FAIL;
		}
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			if (!HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx))
				continue;
			if (IS_PG_TXPWR_DIFF_INVALID(pwr_info->CCK_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW40_Diff[path][tx_idx]))
				return _FAIL;
		}
	}

	return _SUCCESS;
}

static u8 hal_chk_pg_txpwr_info_5g(_adapter *adapter, TxPowerInfo5G *pwr_info)
{
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 path, group, tx_idx;

	if (pwr_info == NULL || !hal_chk_band_cap(adapter, BAND_CAP_5G))
		return _SUCCESS;

	for (path = 0; path < MAX_RF_PATH; path++) {
		if (!HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path))
			continue;
		for (group = 0; group < MAX_CHNL_GROUP_5G; group++)
			if (IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexBW40_Base[path][group]))
				return _FAIL;
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			if (!HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx))
				continue;
			if (IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW40_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW80_Diff[path][tx_idx])
				|| IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW160_Diff[path][tx_idx]))
				return _FAIL;
		}
	}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
	return _SUCCESS;
}

static inline void hal_init_pg_txpwr_info_2g(_adapter *adapter, TxPowerInfo24G *pwr_info)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 path, group, tx_idx;

	if (pwr_info == NULL)
		return;

	_rtw_memset(pwr_info, 0, sizeof(TxPowerInfo24G));

	/* init with invalid value */
	for (path = 0; path < MAX_RF_PATH; path++) {
		for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
			pwr_info->IndexCCK_Base[path][group] = PG_TXPWR_INVALID_BASE;
			pwr_info->IndexBW40_Base[path][group] = PG_TXPWR_INVALID_BASE;
		}
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			pwr_info->CCK_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->OFDM_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW20_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW40_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
		}
	}

	/* init for dummy base and diff */
	for (path = 0; path < MAX_RF_PATH; path++) {
		if (!HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path))
			break;
		/* 2.4G BW40 base has 1 less group than CCK base*/
		pwr_info->IndexBW40_Base[path][MAX_CHNL_GROUP_24G - 1] = 0;

		/* dummy diff */
		pwr_info->CCK_Diff[path][0] = 0; /* 2.4G CCK-1TX */
		pwr_info->BW40_Diff[path][0] = 0; /* 2.4G BW40-1S */
	}
}

static inline void hal_init_pg_txpwr_info_5g(_adapter *adapter, TxPowerInfo5G *pwr_info)
{
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 path, group, tx_idx;

	if (pwr_info == NULL)
		return;

	_rtw_memset(pwr_info, 0, sizeof(TxPowerInfo5G));

	/* init with invalid value */
	for (path = 0; path < MAX_RF_PATH; path++) {
		for (group = 0; group < MAX_CHNL_GROUP_5G; group++)
			pwr_info->IndexBW40_Base[path][group] = PG_TXPWR_INVALID_BASE;
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			pwr_info->OFDM_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW20_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW40_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW80_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
			pwr_info->BW160_Diff[path][tx_idx] = PG_TXPWR_INVALID_DIFF;
		}
	}

	for (path = 0; path < MAX_RF_PATH; path++) {
		if (!HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path))
			break;
		/* dummy diff */
		pwr_info->BW40_Diff[path][0] = 0; /* 5G BW40-1S */
	}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
}

#if DBG_PG_TXPWR_READ
#define LOAD_PG_TXPWR_WARN_COND(_txpwr_src) 1
#else
#define LOAD_PG_TXPWR_WARN_COND(_txpwr_src) (_txpwr_src > PG_TXPWR_SRC_PG_DATA)
#endif

u16 hal_load_pg_txpwr_info_path_2g(
	_adapter *adapter,
	TxPowerInfo24G	*pwr_info,
	u32 path,
	u8 txpwr_src,
	const struct map_t *txpwr_map,
	u16 pg_offset)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u16 offset = pg_offset;
	u8 group, tx_idx;
	u8 val;
	u8 tmp_base;
	s8 tmp_diff;

	if (pwr_info == NULL || !hal_chk_band_cap(adapter, BAND_CAP_2G)) {
		offset += PG_TXPWR_1PATH_BYTE_NUM_2G;
		goto exit;
	}

	if (DBG_PG_TXPWR_READ)
		RTW_INFO("%s [%c] offset:0x%03x\n", __func__, rf_path_char(path), offset);

	for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
		if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path)) {
			tmp_base = map_read8(txpwr_map, offset);
			if (!IS_PG_TXPWR_BASE_INVALID(hal_spec, tmp_base)
				&& IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexCCK_Base[path][group])
			) {
				pwr_info->IndexCCK_Base[path][group] = tmp_base;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 2G G%02d CCK-1T base:%u from %s\n", rf_path_char(path), group, tmp_base, pg_txpwr_src_str(txpwr_src));
			}
		}
		offset++;
	}

	for (group = 0; group < MAX_CHNL_GROUP_24G - 1; group++) {
		if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path)) {
			tmp_base = map_read8(txpwr_map, offset);
			if (!IS_PG_TXPWR_BASE_INVALID(hal_spec, tmp_base)
				&& IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexBW40_Base[path][group])
			) {
				pwr_info->IndexBW40_Base[path][group] =	tmp_base;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 2G G%02d BW40-1S base:%u from %s\n", rf_path_char(path), group, tmp_base, pg_txpwr_src_str(txpwr_src));
			}
		}
		offset++;
	}

	for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
		if (tx_idx == 0) {
			if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
				val = map_read8(txpwr_map, offset);
				tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				) {
					pwr_info->BW20_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G BW20-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
				tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][tx_idx])
				) {
					pwr_info->OFDM_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G OFDM-%dT diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
			}
			offset++;
		} else {
			if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
				val = map_read8(txpwr_map, offset);
				tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW40_Diff[path][tx_idx])
				) {
					pwr_info->BW40_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G BW40-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));

				}
				tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				) {
					pwr_info->BW20_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G BW20-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
			}
			offset++;

			if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
				val = map_read8(txpwr_map, offset);
				tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][tx_idx])
				) {
					pwr_info->OFDM_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G OFDM-%dT diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
				tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->CCK_Diff[path][tx_idx])
				) {
					pwr_info->CCK_Diff[path][tx_idx] =	tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 2G CCK-%dT diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
			}
			offset++;
		}
	}

	if (offset != pg_offset + PG_TXPWR_1PATH_BYTE_NUM_2G) {
		RTW_ERR("%s parse %d bytes != %d\n", __func__, offset - pg_offset, PG_TXPWR_1PATH_BYTE_NUM_2G);
		rtw_warn_on(1);
	}

exit:
	return offset;
}

u16 hal_load_pg_txpwr_info_path_5g(
	_adapter *adapter,
	TxPowerInfo5G	*pwr_info,
	u32 path,
	u8 txpwr_src,
	const struct map_t *txpwr_map,
	u16 pg_offset)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u16 offset = pg_offset;
	u8 group, tx_idx;
	u8 val;
	u8 tmp_base;
	s8 tmp_diff;

#ifdef CONFIG_IEEE80211_BAND_5GHZ
	if (pwr_info == NULL || !hal_chk_band_cap(adapter, BAND_CAP_5G))
#endif
	{
		offset += PG_TXPWR_1PATH_BYTE_NUM_5G;
		goto exit;
	}

#ifdef CONFIG_IEEE80211_BAND_5GHZ
	if (DBG_PG_TXPWR_READ)
		RTW_INFO("%s[%c] eaddr:0x%03x\n", __func__, rf_path_char(path), offset);

	for (group = 0; group < MAX_CHNL_GROUP_5G; group++) {
		if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path)) {
			tmp_base = map_read8(txpwr_map, offset);
			if (!IS_PG_TXPWR_BASE_INVALID(hal_spec, tmp_base)
				&& IS_PG_TXPWR_BASE_INVALID(hal_spec, pwr_info->IndexBW40_Base[path][group])
			) {
				pwr_info->IndexBW40_Base[path][group] = tmp_base;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 5G G%02d BW40-1S base:%u from %s\n", rf_path_char(path), group, tmp_base, pg_txpwr_src_str(txpwr_src));
			}
		}
		offset++;
	}

	for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
		if (tx_idx == 0) {
			if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
				val = map_read8(txpwr_map, offset);
				tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				) {
					pwr_info->BW20_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 5G BW20-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
				tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][tx_idx])
				) {
					pwr_info->OFDM_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 5G OFDM-%dT diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
			}
			offset++;
		} else {
			if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
				val = map_read8(txpwr_map, offset);
				tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW40_Diff[path][tx_idx])
				) {
					pwr_info->BW40_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 5G BW40-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
				tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
				if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
					&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW20_Diff[path][tx_idx])
				) {
					pwr_info->BW20_Diff[path][tx_idx] = tmp_diff;
					if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
						RTW_INFO("[%c] 5G BW20-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
				}
			}
			offset++;
		}
	}

	/* OFDM diff 2T ~ 3T */
	if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, 1)) {
		val = map_read8(txpwr_map, offset);
		tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
		if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
			&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][1])
		) {
			pwr_info->OFDM_Diff[path][1] = tmp_diff;
			if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
				RTW_INFO("[%c] 5G OFDM-%dT diff:%d from %s\n", rf_path_char(path), 2, tmp_diff, pg_txpwr_src_str(txpwr_src));
		}
		if (HAL_SPEC_CHK_TX_CNT(hal_spec, 2)) {
			tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
			if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
				&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][2])
			) {
				pwr_info->OFDM_Diff[path][2] = tmp_diff;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 5G OFDM-%dT diff:%d from %s\n", rf_path_char(path), 3, tmp_diff, pg_txpwr_src_str(txpwr_src));
			}
		}
	}
	offset++;

	/* OFDM diff 4T */
	if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, 3)) {
		val = map_read8(txpwr_map, offset);
		tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
		if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
			&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->OFDM_Diff[path][3])
		) {
			pwr_info->OFDM_Diff[path][3] = tmp_diff;
			if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
				RTW_INFO("[%c] 5G OFDM-%dT diff:%d from %s\n", rf_path_char(path), 4, tmp_diff, pg_txpwr_src_str(txpwr_src));
		}
	}
	offset++;

	for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
		if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path) && HAL_SPEC_CHK_TX_CNT(hal_spec, tx_idx)) {
			val = map_read8(txpwr_map, offset);
			tmp_diff = PG_TXPWR_MSB_DIFF_TO_S8BIT(val);
			if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
				&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW80_Diff[path][tx_idx])
			) {
				pwr_info->BW80_Diff[path][tx_idx] = tmp_diff;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 5G BW80-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
			}
			tmp_diff = PG_TXPWR_LSB_DIFF_TO_S8BIT(val);
			if (!IS_PG_TXPWR_DIFF_INVALID(tmp_diff)
				&& IS_PG_TXPWR_DIFF_INVALID(pwr_info->BW160_Diff[path][tx_idx])
			) {
				pwr_info->BW160_Diff[path][tx_idx] = tmp_diff;
				if (LOAD_PG_TXPWR_WARN_COND(txpwr_src))
					RTW_INFO("[%c] 5G BW160-%dS diff:%d from %s\n", rf_path_char(path), tx_idx + 1, tmp_diff, pg_txpwr_src_str(txpwr_src));
			}
		}
		offset++;
	}

	if (offset != pg_offset + PG_TXPWR_1PATH_BYTE_NUM_5G) {
		RTW_ERR("%s parse %d bytes != %d\n", __func__, offset - pg_offset, PG_TXPWR_1PATH_BYTE_NUM_5G);
		rtw_warn_on(1);
	}

#endif /* #ifdef CONFIG_IEEE80211_BAND_5GHZ */

exit:
	return offset;
}

void hal_load_pg_txpwr_info(
	_adapter *adapter,
	TxPowerInfo24G *pwr_info_2g,
	TxPowerInfo5G *pwr_info_5g,
	u8 *pg_data,
	BOOLEAN AutoLoadFail
)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 path;
	u16 pg_offset;
	u8 txpwr_src = PG_TXPWR_SRC_PG_DATA;
	struct map_t pg_data_map = MAP_ENT(184, 1, 0xFF, MAPSEG_PTR_ENT(0x00, 184, pg_data));
	const struct map_t *txpwr_map = NULL;

	/* init with invalid value and some dummy base and diff */
	hal_init_pg_txpwr_info_2g(adapter, pwr_info_2g);
	hal_init_pg_txpwr_info_5g(adapter, pwr_info_5g);

select_src:
	pg_offset = hal_spec->pg_txpwr_saddr;

	switch (txpwr_src) {
	case PG_TXPWR_SRC_PG_DATA:
		txpwr_map = &pg_data_map;
		break;
	case PG_TXPWR_SRC_IC_DEF:
		txpwr_map = hal_pg_txpwr_def_info(adapter);
		break;
	case PG_TXPWR_SRC_DEF:
	default:
		txpwr_map = &pg_txpwr_def_info;
		break;
	};

	if (txpwr_map == NULL)
		goto end_parse;

	for (path = 0; path < MAX_RF_PATH ; path++) {
		if (!HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path) && !HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path))
			break;
		pg_offset = hal_load_pg_txpwr_info_path_2g(adapter, pwr_info_2g, path, txpwr_src, txpwr_map, pg_offset);
		pg_offset = hal_load_pg_txpwr_info_path_5g(adapter, pwr_info_5g, path, txpwr_src, txpwr_map, pg_offset);
	}

	if (hal_chk_pg_txpwr_info_2g(adapter, pwr_info_2g) == _SUCCESS
		&& hal_chk_pg_txpwr_info_5g(adapter, pwr_info_5g) == _SUCCESS)
		goto exit;

end_parse:
	txpwr_src++;
	if (txpwr_src < PG_TXPWR_SRC_NUM)
		goto select_src;

	if (hal_chk_pg_txpwr_info_2g(adapter, pwr_info_2g) != _SUCCESS
		|| hal_chk_pg_txpwr_info_5g(adapter, pwr_info_5g) != _SUCCESS)
		rtw_warn_on(1);

exit:
	#if DBG_PG_TXPWR_READ
	if (pwr_info_2g)
		dump_pg_txpwr_info_2g(RTW_DBGDUMP, pwr_info_2g, 4, 4);
	if (pwr_info_5g)
		dump_pg_txpwr_info_5g(RTW_DBGDUMP, pwr_info_5g, 4, 4);
	#endif

	return;
}

#ifdef CONFIG_EFUSE_CONFIG_FILE

#define EFUSE_POWER_INDEX_INVALID 0xFF

static u8 _check_phy_efuse_tx_power_info_valid(u8 *pg_data, int base_len, u16 pg_offset)
{
	int ff_cnt = 0;
	int i;

	for (i = 0; i < base_len; i++) {
		if (*(pg_data + pg_offset + i) == 0xFF)
			ff_cnt++;
	}

	if (ff_cnt == 0)
		return _TRUE;
	else if (ff_cnt == base_len)
		return _FALSE;
	else
		return EFUSE_POWER_INDEX_INVALID;
}

int check_phy_efuse_tx_power_info_valid(_adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	u8 *pg_data = hal_data->efuse_eeprom_data;
	u16 pg_offset = hal_spec->pg_txpwr_saddr;
	u8 path;
	u8 valid_2g_path_bmp = 0;
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	u8 valid_5g_path_bmp = 0;
#endif
	int result = _FALSE;

	for (path = 0; path < MAX_RF_PATH; path++) {
		u8 ret = _FALSE;

		if (!HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path) && !HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path))
			break;

		if (HAL_SPEC_CHK_RF_PATH_2G(hal_spec, path)) {
			ret = _check_phy_efuse_tx_power_info_valid(pg_data, PG_TXPWR_BASE_BYTE_NUM_2G, pg_offset);
			if (ret == _TRUE)
				valid_2g_path_bmp |= BIT(path);
			else if (ret == EFUSE_POWER_INDEX_INVALID)
				return _FALSE;
		}
		pg_offset += PG_TXPWR_1PATH_BYTE_NUM_2G;

		#ifdef CONFIG_IEEE80211_BAND_5GHZ
		if (HAL_SPEC_CHK_RF_PATH_5G(hal_spec, path)) {
			ret = _check_phy_efuse_tx_power_info_valid(pg_data, PG_TXPWR_BASE_BYTE_NUM_5G, pg_offset);
			if (ret == _TRUE)
				valid_5g_path_bmp |= BIT(path);
			else if (ret == EFUSE_POWER_INDEX_INVALID)
				return _FALSE;
		}
		#endif
		pg_offset += PG_TXPWR_1PATH_BYTE_NUM_5G;
	}

	if ((hal_chk_band_cap(adapter, BAND_CAP_2G) && valid_2g_path_bmp)
		#ifdef CONFIG_IEEE80211_BAND_5GHZ
		|| (hal_chk_band_cap(adapter, BAND_CAP_5G) && valid_5g_path_bmp)
		#endif
	)
		return _TRUE;

	return _FALSE;
}
#endif /* CONFIG_EFUSE_CONFIG_FILE */

void hal_load_txpwr_info(
	_adapter *adapter,
	TxPowerInfo24G *pwr_info_2g,
	TxPowerInfo5G *pwr_info_5g,
	u8 *pg_data
)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 max_tx_cnt = hal_spec->max_tx_cnt;
	u8 rfpath, ch_idx, group, tx_idx;

	/* load from pg data (or default value) */
	hal_load_pg_txpwr_info(adapter, pwr_info_2g, pwr_info_5g, pg_data, _FALSE);

	/* transform to hal_data */
	for (rfpath = 0; rfpath < MAX_RF_PATH; rfpath++) {

		if (!pwr_info_2g || !HAL_SPEC_CHK_RF_PATH_2G(hal_spec, rfpath))
			goto bypass_2g;

		/* 2.4G base */
		for (ch_idx = 0; ch_idx < CENTER_CH_2G_NUM; ch_idx++) {
			u8 cck_group;

			if (rtw_get_ch_group(ch_idx + 1, &group, &cck_group) != BAND_ON_2_4G)
				continue;

			hal_data->Index24G_CCK_Base[rfpath][ch_idx] = pwr_info_2g->IndexCCK_Base[rfpath][cck_group];
			hal_data->Index24G_BW40_Base[rfpath][ch_idx] = pwr_info_2g->IndexBW40_Base[rfpath][group];
		}

		/* 2.4G diff */
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			if (tx_idx >= max_tx_cnt)
				break;

			hal_data->CCK_24G_Diff[rfpath][tx_idx] = pwr_info_2g->CCK_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->OFDM_24G_Diff[rfpath][tx_idx] = pwr_info_2g->OFDM_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->BW20_24G_Diff[rfpath][tx_idx] = pwr_info_2g->BW20_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->BW40_24G_Diff[rfpath][tx_idx] = pwr_info_2g->BW40_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
		}
bypass_2g:
		;

#ifdef CONFIG_IEEE80211_BAND_5GHZ
		if (!pwr_info_5g || !HAL_SPEC_CHK_RF_PATH_5G(hal_spec, rfpath))
			goto bypass_5g;

		/* 5G base */
		for (ch_idx = 0; ch_idx < CENTER_CH_5G_ALL_NUM; ch_idx++) {
			if (rtw_get_ch_group(center_ch_5g_all[ch_idx], &group, NULL) != BAND_ON_5G)
				continue;
			hal_data->Index5G_BW40_Base[rfpath][ch_idx] = pwr_info_5g->IndexBW40_Base[rfpath][group];
		}

		for (ch_idx = 0 ; ch_idx < CENTER_CH_5G_80M_NUM; ch_idx++) {
			u8 upper, lower;

			if (rtw_get_ch_group(center_ch_5g_80m[ch_idx], &group, NULL) != BAND_ON_5G)
				continue;

			upper = pwr_info_5g->IndexBW40_Base[rfpath][group];
			lower = pwr_info_5g->IndexBW40_Base[rfpath][group + 1];
			hal_data->Index5G_BW80_Base[rfpath][ch_idx] = (upper + lower) / 2;
		}

		/* 5G diff */
		for (tx_idx = 0; tx_idx < MAX_TX_COUNT; tx_idx++) {
			if (tx_idx >= max_tx_cnt)
				break;

			hal_data->OFDM_5G_Diff[rfpath][tx_idx] = pwr_info_5g->OFDM_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->BW20_5G_Diff[rfpath][tx_idx] = pwr_info_5g->BW20_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->BW40_5G_Diff[rfpath][tx_idx] = pwr_info_5g->BW40_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
			hal_data->BW80_5G_Diff[rfpath][tx_idx] = pwr_info_5g->BW80_Diff[rfpath][tx_idx] * hal_spec->pg_txgi_diff_factor;
		}
bypass_5g:
		;
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
	}
}

void dump_hal_txpwr_info_2g(void *sel, _adapter *adapter, u8 rfpath_num, u8 max_tx_cnt)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	int path, ch_idx, tx_idx;

	RTW_PRINT_SEL(sel, "2.4G\n");
	RTW_PRINT_SEL(sel, "CCK-1T base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (ch_idx = 0; ch_idx < CENTER_CH_2G_NUM; ch_idx++)
		_RTW_PRINT_SEL(sel, "%2d ", center_ch_2g[ch_idx]);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (ch_idx = 0; ch_idx < CENTER_CH_2G_NUM; ch_idx++)
			_RTW_PRINT_SEL(sel, "%2u ", hal_data->Index24G_CCK_Base[path][ch_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "CCK diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dT ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->CCK_24G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40-1S base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (ch_idx = 0; ch_idx < CENTER_CH_2G_NUM; ch_idx++)
		_RTW_PRINT_SEL(sel, "%2d ", center_ch_2g[ch_idx]);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (ch_idx = 0; ch_idx < CENTER_CH_2G_NUM; ch_idx++)
			_RTW_PRINT_SEL(sel, "%2u ", hal_data->Index24G_BW40_Base[path][ch_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "OFDM diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dT ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->OFDM_24G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW20 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dS ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->BW20_24G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dS ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->BW40_24G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");
}

void dump_hal_txpwr_info_5g(void *sel, _adapter *adapter, u8 rfpath_num, u8 max_tx_cnt)
{
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	int path, ch_idx, tx_idx;
	u8 dump_section = 0;
	u8 ch_idx_s = 0;

	RTW_PRINT_SEL(sel, "5G\n");
	RTW_PRINT_SEL(sel, "BW40-1S base:\n");
	do {
		#define DUMP_5G_BW40_BASE_SECTION_NUM 3
		u8 end[DUMP_5G_BW40_BASE_SECTION_NUM] = {64, 144, 177};

		RTW_PRINT_SEL(sel, "%4s ", "");
		for (ch_idx = ch_idx_s; ch_idx < CENTER_CH_5G_ALL_NUM; ch_idx++) {
			_RTW_PRINT_SEL(sel, "%3d ", center_ch_5g_all[ch_idx]);
			if (end[dump_section] == center_ch_5g_all[ch_idx])
				break;
		}
		_RTW_PRINT_SEL(sel, "\n");
		for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
			RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
			for (ch_idx = ch_idx_s; ch_idx < CENTER_CH_5G_ALL_NUM; ch_idx++) {
				_RTW_PRINT_SEL(sel, "%3u ", hal_data->Index5G_BW40_Base[path][ch_idx]);
				if (end[dump_section] == center_ch_5g_all[ch_idx])
					break;
			}
			_RTW_PRINT_SEL(sel, "\n");
		}
		RTW_PRINT_SEL(sel, "\n");

		ch_idx_s = ch_idx + 1;
		dump_section++;
		if (dump_section >= DUMP_5G_BW40_BASE_SECTION_NUM)
			break;
	} while (1);

	RTW_PRINT_SEL(sel, "BW80-1S base:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (ch_idx = 0; ch_idx < CENTER_CH_5G_80M_NUM; ch_idx++)
		_RTW_PRINT_SEL(sel, "%3d ", center_ch_5g_80m[ch_idx]);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (ch_idx = 0; ch_idx < CENTER_CH_5G_80M_NUM; ch_idx++)
			_RTW_PRINT_SEL(sel, "%3u ", hal_data->Index5G_BW80_Base[path][ch_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "OFDM diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dT ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->OFDM_5G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW20 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dS ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->BW20_5G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW40 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dS ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->BW40_5G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "BW80 diff:\n");
	RTW_PRINT_SEL(sel, "%4s ", "");
	for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
		_RTW_PRINT_SEL(sel, "%dS ", tx_idx + 1);
	_RTW_PRINT_SEL(sel, "\n");
	for (path = 0; path < MAX_RF_PATH && path < rfpath_num; path++) {
		RTW_PRINT_SEL(sel, "[%c]: ", rf_path_char(path));
		for (tx_idx = RF_1TX; tx_idx < MAX_TX_COUNT && tx_idx < max_tx_cnt; tx_idx++)
			_RTW_PRINT_SEL(sel, "%2d ", hal_data->BW80_5G_Diff[path][tx_idx]);
		_RTW_PRINT_SEL(sel, "\n");
	}
	RTW_PRINT_SEL(sel, "\n");
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
}

/*
* rtw_regsty_get_target_tx_power -
*
* Return dBm or -1 for undefined
*/
s8 rtw_regsty_get_target_tx_power(
	IN	PADAPTER		Adapter,
	IN	u8				Band,
	IN	u8				RfPath,
	IN	RATE_SECTION	RateSection
)
{
	struct registry_priv *regsty = adapter_to_regsty(Adapter);
	s8 value = 0;

	if (RfPath > RF_PATH_D) {
		RTW_PRINT("%s invalid RfPath:%d\n", __func__, RfPath);
		return -1;
	}

	if (Band != BAND_ON_2_4G
		#ifdef CONFIG_IEEE80211_BAND_5GHZ
		&& Band != BAND_ON_5G
		#endif
	) {
		RTW_PRINT("%s invalid Band:%d\n", __func__, Band);
		return -1;
	}

	if (RateSection >= RATE_SECTION_NUM
		#ifdef CONFIG_IEEE80211_BAND_5GHZ
		|| (Band == BAND_ON_5G && RateSection == CCK)
		#endif
	) {
		RTW_PRINT("%s invalid RateSection:%d in Band:%d, RfPath:%d\n", __func__
			, RateSection, Band, RfPath);
		return -1;
	}

	if (Band == BAND_ON_2_4G)
		value = regsty->target_tx_pwr_2g[RfPath][RateSection];
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	else /* BAND_ON_5G */
		value = regsty->target_tx_pwr_5g[RfPath][RateSection - 1];
#endif

	return value;
}

bool rtw_regsty_chk_target_tx_power_valid(_adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	int path, tx_num, band, rs;
	s8 target;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		for (path = 0; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				break;

			for (rs = 0; rs < RATE_SECTION_NUM; rs++) {
				tx_num = rate_section_to_tx_num(rs);
				if (tx_num >= hal_spec->tx_nss_num)
					continue;

				if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
					continue;

				if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
					continue;

				target = rtw_regsty_get_target_tx_power(adapter, band, path, rs);
				if (target == -1) {
					RTW_PRINT("%s return _FALSE for band:%d, path:%d, rs:%d, t:%d\n", __func__, band, path, rs, target);
					return _FALSE;
				}
			}
		}
	}

	return _TRUE;
}

/*
* PHY_GetTxPowerByRateBase -
*
* Return value in unit of TX Gain Index
*/
u8
PHY_GetTxPowerByRateBase(
	IN	PADAPTER		Adapter,
	IN	u8				Band,
	IN	u8				RfPath,
	IN	RATE_SECTION	RateSection
)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	u8 value = 0;

	if (RfPath > RF_PATH_D) {
		RTW_PRINT("%s invalid RfPath:%d\n", __func__, RfPath);
		return 0;
	}

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_PRINT("%s invalid Band:%d\n", __func__, Band);
		return 0;
	}

	if (RateSection >= RATE_SECTION_NUM
		|| (Band == BAND_ON_5G && RateSection == CCK)
	) {
		RTW_PRINT("%s invalid RateSection:%d in Band:%d, RfPath:%d\n", __func__
			, RateSection, Band, RfPath);
		return 0;
	}

	if (Band == BAND_ON_2_4G)
		value = pHalData->TxPwrByRateBase2_4G[RfPath][RateSection];
	else /* BAND_ON_5G */
		value = pHalData->TxPwrByRateBase5G[RfPath][RateSection - 1];

	return value;
}

VOID
phy_SetTxPowerByRateBase(
	IN	PADAPTER		Adapter,
	IN	u8				Band,
	IN	u8				RfPath,
	IN	RATE_SECTION	RateSection,
	IN	u8				Value
)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);

	if (RfPath > RF_PATH_D) {
		RTW_PRINT("%s invalid RfPath:%d\n", __func__, RfPath);
		return;
	}

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_PRINT("%s invalid Band:%d\n", __func__, Band);
		return;
	}

	if (RateSection >= RATE_SECTION_NUM
		|| (Band == BAND_ON_5G && RateSection == CCK)
	) {
		RTW_PRINT("%s invalid RateSection:%d in %sG, RfPath:%d\n", __func__
			, RateSection, (Band == BAND_ON_2_4G) ? "2.4" : "5", RfPath);
		return;
	}

	if (Band == BAND_ON_2_4G)
		pHalData->TxPwrByRateBase2_4G[RfPath][RateSection] = Value;
	else /* BAND_ON_5G */
		pHalData->TxPwrByRateBase5G[RfPath][RateSection - 1] = Value;
}

static inline BOOLEAN phy_is_txpwr_by_rate_undefined_of_band_path(_adapter *adapter, u8 band, u8 path)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	u8 rate_idx = 0;

	for (rate_idx = 0; rate_idx < TX_PWR_BY_RATE_NUM_RATE; rate_idx++) {
		if (hal_data->TxPwrByRateOffset[band][path][rate_idx] != 0)
			goto exit;
	}

exit:
	return rate_idx >= TX_PWR_BY_RATE_NUM_RATE ? _TRUE : _FALSE;
}

static inline void phy_txpwr_by_rate_duplicate_band_path(_adapter *adapter, u8 band, u8 s_path, u8 t_path)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	u8 rate_idx = 0;

	for (rate_idx = 0; rate_idx < TX_PWR_BY_RATE_NUM_RATE; rate_idx++)
		hal_data->TxPwrByRateOffset[band][t_path][rate_idx] = hal_data->TxPwrByRateOffset[band][s_path][rate_idx];
}

static void phy_txpwr_by_rate_chk_for_path_dup(_adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	u8 band, path;
	s8 src_path;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++)
		for (path = RF_PATH_A; path < RF_PATH_MAX; path++)
			hal_data->txpwr_by_rate_undefined_band_path[band][path] = 0;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		for (path = RF_PATH_A; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				continue;

			if (phy_is_txpwr_by_rate_undefined_of_band_path(adapter, band, path))
				hal_data->txpwr_by_rate_undefined_band_path[band][path] = 1;
		}
	}

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		src_path = -1;
		for (path = RF_PATH_A; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				continue;

			/* find src */
			if (src_path == -1 && hal_data->txpwr_by_rate_undefined_band_path[band][path] == 0)
				src_path = path;
		}

		if (src_path == -1) {
			RTW_ERR("%s all power by rate undefined\n", __func__);
			continue;
		}

		for (path = RF_PATH_A; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				continue;

			/* duplicate src to undefined one */
			if (hal_data->txpwr_by_rate_undefined_band_path[band][path] == 1) {
				RTW_INFO("%s duplicate %s [%c] to [%c]\n", __func__
					, band_str(band), rf_path_char(src_path), rf_path_char(path));
				phy_txpwr_by_rate_duplicate_band_path(adapter, band, src_path, path);
			}
		}
	}
}

VOID
phy_StoreTxPowerByRateBase(
	IN	PADAPTER	pAdapter
)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(pAdapter);
	struct registry_priv *regsty = adapter_to_regsty(pAdapter);

	u8 rate_sec_base[RATE_SECTION_NUM] = {
		MGN_11M,
		MGN_54M,
		MGN_MCS7,
		MGN_MCS15,
		MGN_MCS23,
		MGN_MCS31,
		MGN_VHT1SS_MCS7,
		MGN_VHT2SS_MCS7,
		MGN_VHT3SS_MCS7,
		MGN_VHT4SS_MCS7,
	};

	u8 band, path, rs, tx_num, base, index;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(pAdapter, band))
			continue;

		for (path = RF_PATH_A; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				break;

			for (rs = 0; rs < RATE_SECTION_NUM; rs++) {
				tx_num = rate_section_to_tx_num(rs);
				if (tx_num >= hal_spec->tx_nss_num)
					continue;

				if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
					continue;

				if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(pAdapter))
					continue;

				if (regsty->target_tx_pwr_valid == _TRUE)
					base = hal_spec->txgi_pdbm * rtw_regsty_get_target_tx_power(pAdapter, band, path, rs);
				else
					base = _PHY_GetTxPowerByRate(pAdapter, band, path, rate_sec_base[rs]);
				phy_SetTxPowerByRateBase(pAdapter, band, path, rs, base);
			}
		}
	}
}

static u8 get_val_from_dhex(u32 dhex, u8 i)
{
	return (((dhex >> (i * 8 + 4)) & 0xF)) * 10 + ((dhex >> (i * 8)) & 0xF);
}

static u8 get_val_from_hex(u32 hex, u8 i)
{
	return (hex >> (i * 8)) & 0xFF;
}

VOID
PHY_GetRateValuesOfTxPowerByRate(
	IN	PADAPTER pAdapter,
	IN	u32 RegAddr,
	IN	u32 BitMask,
	IN	u32 Value,
	OUT	u8 *Rate,
	OUT	s8 *PwrByRateVal,
	OUT	u8 *RateNum
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(pAdapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;
	u8				index = 0, i = 0;
	u8 (*get_val)(u32, u8);

	if (pDM_Odm->phy_reg_pg_version == 1)
		get_val = get_val_from_dhex;
	else
		get_val = get_val_from_hex;

	switch (RegAddr) {
	case rTxAGC_A_Rate18_06:
	case rTxAGC_B_Rate18_06:
		Rate[0] = MGN_6M;
		Rate[1] = MGN_9M;
		Rate[2] = MGN_12M;
		Rate[3] = MGN_18M;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_A_Rate54_24:
	case rTxAGC_B_Rate54_24:
		Rate[0] = MGN_24M;
		Rate[1] = MGN_36M;
		Rate[2] = MGN_48M;
		Rate[3] = MGN_54M;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_A_CCK1_Mcs32:
		Rate[0] = MGN_1M;
		PwrByRateVal[0] = (s8)get_val(Value, 1);
		*RateNum = 1;
		break;

	case rTxAGC_B_CCK11_A_CCK2_11:
		if (BitMask == 0xffffff00) {
			Rate[0] = MGN_2M;
			Rate[1] = MGN_5_5M;
			Rate[2] = MGN_11M;
			for (i = 1; i < 4; ++i)
				PwrByRateVal[i - 1] = (s8)get_val(Value, i);
			*RateNum = 3;
		} else if (BitMask == 0x000000ff) {
			Rate[0] = MGN_11M;
			PwrByRateVal[0] = (s8)get_val(Value, 0);
			*RateNum = 1;
		}
		break;

	case rTxAGC_A_Mcs03_Mcs00:
	case rTxAGC_B_Mcs03_Mcs00:
		Rate[0] = MGN_MCS0;
		Rate[1] = MGN_MCS1;
		Rate[2] = MGN_MCS2;
		Rate[3] = MGN_MCS3;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs07_Mcs04:
	case rTxAGC_B_Mcs07_Mcs04:
		Rate[0] = MGN_MCS4;
		Rate[1] = MGN_MCS5;
		Rate[2] = MGN_MCS6;
		Rate[3] = MGN_MCS7;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs11_Mcs08:
	case rTxAGC_B_Mcs11_Mcs08:
		Rate[0] = MGN_MCS8;
		Rate[1] = MGN_MCS9;
		Rate[2] = MGN_MCS10;
		Rate[3] = MGN_MCS11;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs15_Mcs12:
	case rTxAGC_B_Mcs15_Mcs12:
		Rate[0] = MGN_MCS12;
		Rate[1] = MGN_MCS13;
		Rate[2] = MGN_MCS14;
		Rate[3] = MGN_MCS15;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case rTxAGC_B_CCK1_55_Mcs32:
		Rate[0] = MGN_1M;
		Rate[1] = MGN_2M;
		Rate[2] = MGN_5_5M;
		for (i = 1; i < 4; ++i)
			PwrByRateVal[i - 1] = (s8)get_val(Value, i);
		*RateNum = 3;
		break;

	case 0xC20:
	case 0xE20:
	case 0x1820:
	case 0x1a20:
		Rate[0] = MGN_1M;
		Rate[1] = MGN_2M;
		Rate[2] = MGN_5_5M;
		Rate[3] = MGN_11M;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC24:
	case 0xE24:
	case 0x1824:
	case 0x1a24:
		Rate[0] = MGN_6M;
		Rate[1] = MGN_9M;
		Rate[2] = MGN_12M;
		Rate[3] = MGN_18M;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC28:
	case 0xE28:
	case 0x1828:
	case 0x1a28:
		Rate[0] = MGN_24M;
		Rate[1] = MGN_36M;
		Rate[2] = MGN_48M;
		Rate[3] = MGN_54M;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC2C:
	case 0xE2C:
	case 0x182C:
	case 0x1a2C:
		Rate[0] = MGN_MCS0;
		Rate[1] = MGN_MCS1;
		Rate[2] = MGN_MCS2;
		Rate[3] = MGN_MCS3;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC30:
	case 0xE30:
	case 0x1830:
	case 0x1a30:
		Rate[0] = MGN_MCS4;
		Rate[1] = MGN_MCS5;
		Rate[2] = MGN_MCS6;
		Rate[3] = MGN_MCS7;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC34:
	case 0xE34:
	case 0x1834:
	case 0x1a34:
		Rate[0] = MGN_MCS8;
		Rate[1] = MGN_MCS9;
		Rate[2] = MGN_MCS10;
		Rate[3] = MGN_MCS11;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC38:
	case 0xE38:
	case 0x1838:
	case 0x1a38:
		Rate[0] = MGN_MCS12;
		Rate[1] = MGN_MCS13;
		Rate[2] = MGN_MCS14;
		Rate[3] = MGN_MCS15;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC3C:
	case 0xE3C:
	case 0x183C:
	case 0x1a3C:
		Rate[0] = MGN_VHT1SS_MCS0;
		Rate[1] = MGN_VHT1SS_MCS1;
		Rate[2] = MGN_VHT1SS_MCS2;
		Rate[3] = MGN_VHT1SS_MCS3;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC40:
	case 0xE40:
	case 0x1840:
	case 0x1a40:
		Rate[0] = MGN_VHT1SS_MCS4;
		Rate[1] = MGN_VHT1SS_MCS5;
		Rate[2] = MGN_VHT1SS_MCS6;
		Rate[3] = MGN_VHT1SS_MCS7;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC44:
	case 0xE44:
	case 0x1844:
	case 0x1a44:
		Rate[0] = MGN_VHT1SS_MCS8;
		Rate[1] = MGN_VHT1SS_MCS9;
		Rate[2] = MGN_VHT2SS_MCS0;
		Rate[3] = MGN_VHT2SS_MCS1;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC48:
	case 0xE48:
	case 0x1848:
	case 0x1a48:
		Rate[0] = MGN_VHT2SS_MCS2;
		Rate[1] = MGN_VHT2SS_MCS3;
		Rate[2] = MGN_VHT2SS_MCS4;
		Rate[3] = MGN_VHT2SS_MCS5;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xC4C:
	case 0xE4C:
	case 0x184C:
	case 0x1a4C:
		Rate[0] = MGN_VHT2SS_MCS6;
		Rate[1] = MGN_VHT2SS_MCS7;
		Rate[2] = MGN_VHT2SS_MCS8;
		Rate[3] = MGN_VHT2SS_MCS9;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xCD8:
	case 0xED8:
	case 0x18D8:
	case 0x1aD8:
		Rate[0] = MGN_MCS16;
		Rate[1] = MGN_MCS17;
		Rate[2] = MGN_MCS18;
		Rate[3] = MGN_MCS19;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xCDC:
	case 0xEDC:
	case 0x18DC:
	case 0x1aDC:
		Rate[0] = MGN_MCS20;
		Rate[1] = MGN_MCS21;
		Rate[2] = MGN_MCS22;
		Rate[3] = MGN_MCS23;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xCE0:
	case 0xEE0:
	case 0x18E0:
	case 0x1aE0:
		Rate[0] = MGN_VHT3SS_MCS0;
		Rate[1] = MGN_VHT3SS_MCS1;
		Rate[2] = MGN_VHT3SS_MCS2;
		Rate[3] = MGN_VHT3SS_MCS3;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xCE4:
	case 0xEE4:
	case 0x18E4:
	case 0x1aE4:
		Rate[0] = MGN_VHT3SS_MCS4;
		Rate[1] = MGN_VHT3SS_MCS5;
		Rate[2] = MGN_VHT3SS_MCS6;
		Rate[3] = MGN_VHT3SS_MCS7;
		for (i = 0; i < 4; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 4;
		break;

	case 0xCE8:
	case 0xEE8:
	case 0x18E8:
	case 0x1aE8:
		Rate[0] = MGN_VHT3SS_MCS8;
		Rate[1] = MGN_VHT3SS_MCS9;
		for (i = 0; i < 2; ++i)
			PwrByRateVal[i] = (s8)get_val(Value, i);
		*RateNum = 2;
		break;

	default:
		RTW_PRINT("Invalid RegAddr 0x%x in %s()\n", RegAddr, __func__);
		break;
	};
}

void
PHY_StoreTxPowerByRateNew(
	IN	PADAPTER	pAdapter,
	IN	u32			Band,
	IN	u32			RfPath,
	IN	u32			RegAddr,
	IN	u32			BitMask,
	IN	u32			Data
)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(pAdapter);
	u8	i = 0, rates[4] = {0}, rateNum = 0;
	s8	PwrByRateVal[4] = {0};

	PHY_GetRateValuesOfTxPowerByRate(pAdapter, RegAddr, BitMask, Data, rates, PwrByRateVal, &rateNum);

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_PRINT("Invalid Band %d\n", Band);
		return;
	}

	if (RfPath > RF_PATH_D) {
		RTW_PRINT("Invalid RfPath %d\n", RfPath);
		return;
	}

	for (i = 0; i < rateNum; ++i) {
		u8 rate_idx = PHY_GetRateIndexOfTxPowerByRate(rates[i]);

		pHalData->TxPwrByRateOffset[Band][RfPath][rate_idx] = PwrByRateVal[i];
	}
}

VOID
PHY_InitTxPowerByRate(
	IN	PADAPTER	pAdapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	u8	band = 0, rfPath = 0, rate = 0, i = 0, j = 0;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; ++band)
		for (rfPath = 0; rfPath < TX_PWR_BY_RATE_NUM_RF; ++rfPath)
				for (rate = 0; rate < TX_PWR_BY_RATE_NUM_RATE; ++rate)
					pHalData->TxPwrByRateOffset[band][rfPath][rate] = 0;
}

VOID
phy_store_tx_power_by_rate(
	IN	PADAPTER	pAdapter,
	IN	u32			Band,
	IN	u32			RfPath,
	IN	u32			TxNum,
	IN	u32			RegAddr,
	IN	u32			BitMask,
	IN	u32			Data
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;

	if (pDM_Odm->phy_reg_pg_version > 0)
		PHY_StoreTxPowerByRateNew(pAdapter, Band, RfPath, RegAddr, BitMask, Data);
	else
		RTW_INFO("Invalid PHY_REG_PG.txt version %d\n",  pDM_Odm->phy_reg_pg_version);

}

VOID
phy_ConvertTxPowerByRateInDbmToRelativeValues(
	IN	PADAPTER	pAdapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	u8			base = 0, i = 0, value = 0,
				band = 0, path = 0, index = 0,
				startIndex = 0, endIndex = 0;
	u8	cckRates[4] = {MGN_1M, MGN_2M, MGN_5_5M, MGN_11M},
		ofdmRates[8] = {MGN_6M, MGN_9M, MGN_12M, MGN_18M, MGN_24M, MGN_36M, MGN_48M, MGN_54M},
		mcs0_7Rates[8] = {MGN_MCS0, MGN_MCS1, MGN_MCS2, MGN_MCS3, MGN_MCS4, MGN_MCS5, MGN_MCS6, MGN_MCS7},
		mcs8_15Rates[8] = {MGN_MCS8, MGN_MCS9, MGN_MCS10, MGN_MCS11, MGN_MCS12, MGN_MCS13, MGN_MCS14, MGN_MCS15},
		mcs16_23Rates[8] = {MGN_MCS16, MGN_MCS17, MGN_MCS18, MGN_MCS19, MGN_MCS20, MGN_MCS21, MGN_MCS22, MGN_MCS23},
		vht1ssRates[10] = {MGN_VHT1SS_MCS0, MGN_VHT1SS_MCS1, MGN_VHT1SS_MCS2, MGN_VHT1SS_MCS3, MGN_VHT1SS_MCS4,
			MGN_VHT1SS_MCS5, MGN_VHT1SS_MCS6, MGN_VHT1SS_MCS7, MGN_VHT1SS_MCS8, MGN_VHT1SS_MCS9},
		vht2ssRates[10] = {MGN_VHT2SS_MCS0, MGN_VHT2SS_MCS1, MGN_VHT2SS_MCS2, MGN_VHT2SS_MCS3, MGN_VHT2SS_MCS4,
			MGN_VHT2SS_MCS5, MGN_VHT2SS_MCS6, MGN_VHT2SS_MCS7, MGN_VHT2SS_MCS8, MGN_VHT2SS_MCS9},
		vht3ssRates[10] = {MGN_VHT3SS_MCS0, MGN_VHT3SS_MCS1, MGN_VHT3SS_MCS2, MGN_VHT3SS_MCS3, MGN_VHT3SS_MCS4,
			MGN_VHT3SS_MCS5, MGN_VHT3SS_MCS6, MGN_VHT3SS_MCS7, MGN_VHT3SS_MCS8, MGN_VHT3SS_MCS9};

	/* RTW_INFO("===>PHY_ConvertTxPowerByRateInDbmToRelativeValues()\n" ); */

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; ++band) {
		for (path = RF_PATH_A; path <= RF_PATH_D; ++path) {
			/* CCK */
			if (band == BAND_ON_2_4G) {
				base = PHY_GetTxPowerByRateBase(pAdapter, band, path, CCK);
				for (i = 0; i < sizeof(cckRates); ++i) {
					value = PHY_GetTxPowerByRate(pAdapter, band, path, cckRates[i]);
					PHY_SetTxPowerByRate(pAdapter, band, path, cckRates[i], value - base);
				}
			}

			/* OFDM */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, OFDM);
			for (i = 0; i < sizeof(ofdmRates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, ofdmRates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, ofdmRates[i], value - base);
			}

			/* HT MCS0~7 */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, HT_1SS);
			for (i = 0; i < sizeof(mcs0_7Rates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, mcs0_7Rates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, mcs0_7Rates[i], value - base);
			}

			/* HT MCS8~15 */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, HT_2SS);
			for (i = 0; i < sizeof(mcs8_15Rates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, mcs8_15Rates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, mcs8_15Rates[i], value - base);
			}

			/* HT MCS16~23 */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, HT_3SS);
			for (i = 0; i < sizeof(mcs16_23Rates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, mcs16_23Rates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, mcs16_23Rates[i], value - base);
			}

			/* VHT 1SS */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, VHT_1SS);
			for (i = 0; i < sizeof(vht1ssRates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, vht1ssRates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, vht1ssRates[i], value - base);
			}

			/* VHT 2SS */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, VHT_2SS);
			for (i = 0; i < sizeof(vht2ssRates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, vht2ssRates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, vht2ssRates[i], value - base);
			}

			/* VHT 3SS */
			base = PHY_GetTxPowerByRateBase(pAdapter, band, path, VHT_3SS);
			for (i = 0; i < sizeof(vht3ssRates); ++i) {
				value = PHY_GetTxPowerByRate(pAdapter, band, path, vht3ssRates[i]);
				PHY_SetTxPowerByRate(pAdapter, band, path, vht3ssRates[i], value - base);
			}
		}
	}

	/* RTW_INFO("<===PHY_ConvertTxPowerByRateInDbmToRelativeValues()\n" ); */
}

/*
  * This function must be called if the value in the PHY_REG_PG.txt(or header)
  * is exact dBm values
  */
VOID
PHY_TxPowerByRateConfiguration(
	IN  PADAPTER			pAdapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	phy_txpwr_by_rate_chk_for_path_dup(pAdapter);
	phy_StoreTxPowerByRateBase(pAdapter);
	phy_ConvertTxPowerByRateInDbmToRelativeValues(pAdapter);
}

VOID
phy_set_tx_power_index_by_rate_section(
	IN	PADAPTER		pAdapter,
	IN	enum rf_path		RFPath,
	IN	u8				Channel,
	IN	u8				RateSection
)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(pAdapter);

	if (RateSection >= RATE_SECTION_NUM) {
		RTW_INFO("Invalid RateSection %d in %s", RateSection, __func__);
		rtw_warn_on(1);
		goto exit;
	}

	if (RateSection == CCK && pHalData->current_band_type != BAND_ON_2_4G)
		goto exit;

	PHY_SetTxPowerIndexByRateArray(pAdapter, RFPath, pHalData->current_channel_bw, Channel,
		rates_by_sections[RateSection].rates, rates_by_sections[RateSection].rate_num);

exit:
	return;
}

BOOLEAN
phy_GetChnlIndex(
	IN	u8	Channel,
	OUT u8	*ChannelIdx
)
{
	u8  i = 0;
	BOOLEAN bIn24G = _TRUE;

	if (Channel <= 14) {
		bIn24G = _TRUE;
		*ChannelIdx = Channel - 1;
	} else {
		bIn24G = _FALSE;

		for (i = 0; i < CENTER_CH_5G_ALL_NUM; ++i) {
			if (center_ch_5g_all[i] == Channel) {
				*ChannelIdx = i;
				return bIn24G;
			}
		}
	}

	return bIn24G;
}

u8
PHY_GetTxPowerIndexBase(
	IN	PADAPTER		pAdapter,
	IN	enum rf_path		RFPath,
	IN	u8				Rate,
	u8 ntx_idx,
	IN	enum channel_width	BandWidth,
	IN	u8				Channel,
	OUT PBOOLEAN		bIn24G
)
{
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(pAdapter);
	struct dm_struct			*pDM_Odm = &pHalData->odmpriv;
	u8					i = 0;	/* default set to 1S */
	u8					txPower = 0;
	u8					chnlIdx = (Channel - 1);

	if (HAL_IsLegalChannel(pAdapter, Channel) == _FALSE) {
		chnlIdx = 0;
		RTW_INFO("Illegal channel!!\n");
	}

	*bIn24G = phy_GetChnlIndex(Channel, &chnlIdx);

	if (0)
		RTW_INFO("[%s] Channel Index: %d\n", (*bIn24G ? "2.4G" : "5G"), chnlIdx);

	if (*bIn24G) {
		if (IS_CCK_RATE(Rate)) {
			/* CCK-nTX */
			txPower = pHalData->Index24G_CCK_Base[RFPath][chnlIdx];
			txPower += pHalData->CCK_24G_Diff[RFPath][RF_1TX];
			if (ntx_idx >= RF_2TX)
				txPower += pHalData->CCK_24G_Diff[RFPath][RF_2TX];
			if (ntx_idx >= RF_3TX)
				txPower += pHalData->CCK_24G_Diff[RFPath][RF_3TX];
			if (ntx_idx >= RF_4TX)
				txPower += pHalData->CCK_24G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		txPower = pHalData->Index24G_BW40_Base[RFPath][chnlIdx];

		/* OFDM-nTX */
		if ((MGN_6M <= Rate && Rate <= MGN_54M) && !IS_CCK_RATE(Rate)) {
			txPower += pHalData->OFDM_24G_Diff[RFPath][RF_1TX];
			if (ntx_idx >= RF_2TX)
				txPower += pHalData->OFDM_24G_Diff[RFPath][RF_2TX];
			if (ntx_idx >= RF_3TX)
				txPower += pHalData->OFDM_24G_Diff[RFPath][RF_3TX];
			if (ntx_idx >= RF_4TX)
				txPower += pHalData->OFDM_24G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* BW20-nS */
		if (BandWidth == CHANNEL_WIDTH_20) {
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_24G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_24G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_24G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_24G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* BW40-nS */
		if (BandWidth == CHANNEL_WIDTH_40) {
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* Willis suggest adopt BW 40M power index while in BW 80 mode */
		if (BandWidth == CHANNEL_WIDTH_80) {
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_24G_Diff[RFPath][RF_4TX];
			goto exit;
		}
	}
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	else {
		if (Rate >= MGN_6M)
			txPower = pHalData->Index5G_BW40_Base[RFPath][chnlIdx];
		else {
			RTW_INFO("===>PHY_GetTxPowerIndexBase: INVALID Rate(0x%02x).\n", Rate);
			goto exit;
		}

		/* OFDM-nTX */
		if ((MGN_6M <= Rate && Rate <= MGN_54M) && !IS_CCK_RATE(Rate)) {
			txPower += pHalData->OFDM_5G_Diff[RFPath][RF_1TX];
			if (ntx_idx >= RF_2TX)
				txPower += pHalData->OFDM_5G_Diff[RFPath][RF_2TX];
			if (ntx_idx >= RF_3TX)
				txPower += pHalData->OFDM_5G_Diff[RFPath][RF_3TX];
			if (ntx_idx >= RF_4TX)
				txPower += pHalData->OFDM_5G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* BW20-nS */
		if (BandWidth == CHANNEL_WIDTH_20) {
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_5G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_5G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_5G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW20_5G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* BW40-nS */
		if (BandWidth == CHANNEL_WIDTH_40) {
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_5G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_5G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_5G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW40_5G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* BW80-nS */
		if (BandWidth == CHANNEL_WIDTH_80) {
			/* get 80MHz cch index */
			for (i = 0; i < CENTER_CH_5G_80M_NUM; ++i) {
				if (center_ch_5g_80m[i] == Channel) {
					chnlIdx = i;
					break;
				}
			}
			if (i >= CENTER_CH_5G_80M_NUM) {
			#ifdef CONFIG_MP_INCLUDED
				if (rtw_mp_mode_check(pAdapter) == _FALSE)
			#endif
					rtw_warn_on(1);
				txPower = 0;
				goto exit;
			}

			txPower = pHalData->Index5G_BW80_Base[RFPath][chnlIdx];

			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  || (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += + pHalData->BW80_5G_Diff[RFPath][RF_1TX];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW80_5G_Diff[RFPath][RF_2TX];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW80_5G_Diff[RFPath][RF_3TX];
			if ((MGN_MCS23 <= Rate && Rate <= MGN_MCS31) || (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += pHalData->BW80_5G_Diff[RFPath][RF_4TX];
			goto exit;
		}

		/* TODO: BW160-nS */
		rtw_warn_on(1);
	}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

exit:
	return txPower;
}

s8
PHY_GetTxPowerTrackingOffset(
	PADAPTER	pAdapter,
	enum rf_path	RFPath,
	u8			Rate
)
{
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(pAdapter);
	struct dm_struct			*pDM_Odm = &pHalData->odmpriv;
	s8	offset = 0;

	if (pDM_Odm->rf_calibrate_info.txpowertrack_control  == _FALSE)
		return offset;

	if ((Rate == MGN_1M) || (Rate == MGN_2M) || (Rate == MGN_5_5M) || (Rate == MGN_11M)) {
		offset = pDM_Odm->rf_calibrate_info.remnant_cck_swing_idx;
		/*RTW_INFO("+Remnant_CCKSwingIdx = 0x%x\n", RFPath, Rate, pRFCalibrateInfo->Remnant_CCKSwingIdx);*/
	} else {
		offset = pDM_Odm->rf_calibrate_info.remnant_ofdm_swing_idx[RFPath];
		/*RTW_INFO("+Remanant_OFDMSwingIdx[RFPath %u][Rate 0x%x] = 0x%x\n", RFPath, Rate, pRFCalibrateInfo->Remnant_OFDMSwingIdx[RFPath]);	*/

	}

	return offset;
}

/*The same as MRateToHwRate in hal_com.c*/
u8
PHY_GetRateIndexOfTxPowerByRate(
	IN	u8		Rate
)
{
	u8	index = 0;
	switch (Rate) {
	case MGN_1M:
		index = 0;
		break;
	case MGN_2M:
		index = 1;
		break;
	case MGN_5_5M:
		index = 2;
		break;
	case MGN_11M:
		index = 3;
		break;
	case MGN_6M:
		index = 4;
		break;
	case MGN_9M:
		index = 5;
		break;
	case MGN_12M:
		index = 6;
		break;
	case MGN_18M:
		index = 7;
		break;
	case MGN_24M:
		index = 8;
		break;
	case MGN_36M:
		index = 9;
		break;
	case MGN_48M:
		index = 10;
		break;
	case MGN_54M:
		index = 11;
		break;
	case MGN_MCS0:
		index = 12;
		break;
	case MGN_MCS1:
		index = 13;
		break;
	case MGN_MCS2:
		index = 14;
		break;
	case MGN_MCS3:
		index = 15;
		break;
	case MGN_MCS4:
		index = 16;
		break;
	case MGN_MCS5:
		index = 17;
		break;
	case MGN_MCS6:
		index = 18;
		break;
	case MGN_MCS7:
		index = 19;
		break;
	case MGN_MCS8:
		index = 20;
		break;
	case MGN_MCS9:
		index = 21;
		break;
	case MGN_MCS10:
		index = 22;
		break;
	case MGN_MCS11:
		index = 23;
		break;
	case MGN_MCS12:
		index = 24;
		break;
	case MGN_MCS13:
		index = 25;
		break;
	case MGN_MCS14:
		index = 26;
		break;
	case MGN_MCS15:
		index = 27;
		break;
	case MGN_MCS16:
		index = 28;
		break;
	case MGN_MCS17:
		index = 29;
		break;
	case MGN_MCS18:
		index = 30;
		break;
	case MGN_MCS19:
		index = 31;
		break;
	case MGN_MCS20:
		index = 32;
		break;
	case MGN_MCS21:
		index = 33;
		break;
	case MGN_MCS22:
		index = 34;
		break;
	case MGN_MCS23:
		index = 35;
		break;
	case MGN_MCS24:
		index = 36;
		break;
	case MGN_MCS25:
		index = 37;
		break;
	case MGN_MCS26:
		index = 38;
		break;
	case MGN_MCS27:
		index = 39;
		break;
	case MGN_MCS28:
		index = 40;
		break;
	case MGN_MCS29:
		index = 41;
		break;
	case MGN_MCS30:
		index = 42;
		break;
	case MGN_MCS31:
		index = 43;
		break;
	case MGN_VHT1SS_MCS0:
		index = 44;
		break;
	case MGN_VHT1SS_MCS1:
		index = 45;
		break;
	case MGN_VHT1SS_MCS2:
		index = 46;
		break;
	case MGN_VHT1SS_MCS3:
		index = 47;
		break;
	case MGN_VHT1SS_MCS4:
		index = 48;
		break;
	case MGN_VHT1SS_MCS5:
		index = 49;
		break;
	case MGN_VHT1SS_MCS6:
		index = 50;
		break;
	case MGN_VHT1SS_MCS7:
		index = 51;
		break;
	case MGN_VHT1SS_MCS8:
		index = 52;
		break;
	case MGN_VHT1SS_MCS9:
		index = 53;
		break;
	case MGN_VHT2SS_MCS0:
		index = 54;
		break;
	case MGN_VHT2SS_MCS1:
		index = 55;
		break;
	case MGN_VHT2SS_MCS2:
		index = 56;
		break;
	case MGN_VHT2SS_MCS3:
		index = 57;
		break;
	case MGN_VHT2SS_MCS4:
		index = 58;
		break;
	case MGN_VHT2SS_MCS5:
		index = 59;
		break;
	case MGN_VHT2SS_MCS6:
		index = 60;
		break;
	case MGN_VHT2SS_MCS7:
		index = 61;
		break;
	case MGN_VHT2SS_MCS8:
		index = 62;
		break;
	case MGN_VHT2SS_MCS9:
		index = 63;
		break;
	case MGN_VHT3SS_MCS0:
		index = 64;
		break;
	case MGN_VHT3SS_MCS1:
		index = 65;
		break;
	case MGN_VHT3SS_MCS2:
		index = 66;
		break;
	case MGN_VHT3SS_MCS3:
		index = 67;
		break;
	case MGN_VHT3SS_MCS4:
		index = 68;
		break;
	case MGN_VHT3SS_MCS5:
		index = 69;
		break;
	case MGN_VHT3SS_MCS6:
		index = 70;
		break;
	case MGN_VHT3SS_MCS7:
		index = 71;
		break;
	case MGN_VHT3SS_MCS8:
		index = 72;
		break;
	case MGN_VHT3SS_MCS9:
		index = 73;
		break;
	case MGN_VHT4SS_MCS0:
		index = 74;
		break;
	case MGN_VHT4SS_MCS1:
		index = 75;
		break;
	case MGN_VHT4SS_MCS2:
		index = 76;
		break;
	case MGN_VHT4SS_MCS3:
		index = 77;
		break;
	case MGN_VHT4SS_MCS4:
		index = 78;
		break;
	case MGN_VHT4SS_MCS5:
		index = 79;
		break;
	case MGN_VHT4SS_MCS6:
		index = 80;
		break;
	case MGN_VHT4SS_MCS7:
		index = 81;
		break;
	case MGN_VHT4SS_MCS8:
		index = 82;
		break;
	case MGN_VHT4SS_MCS9:
		index = 83;
		break;
	default:
		RTW_INFO("Invalid rate 0x%x in %s\n", Rate, __FUNCTION__);
		break;
	};

	return index;
}

s8
_PHY_GetTxPowerByRate(
	IN	PADAPTER	pAdapter,
	IN	u8			Band,
	IN	enum rf_path	RFPath,
	IN	u8			Rate
)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(pAdapter);
	s8 value = 0;
	u8 rateIndex = PHY_GetRateIndexOfTxPowerByRate(Rate);

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_INFO("Invalid band %d in %s\n", Band, __func__);
		goto exit;
	}
	if (RFPath > RF_PATH_D) {
		RTW_INFO("Invalid RfPath %d in %s\n", RFPath, __func__);
		goto exit;
	}
	if (rateIndex >= TX_PWR_BY_RATE_NUM_RATE) {
		RTW_INFO("Invalid RateIndex %d in %s\n", rateIndex, __func__);
		goto exit;
	}

	value = pHalData->TxPwrByRateOffset[Band][RFPath][rateIndex];

exit:
	return value;
}


s8
PHY_GetTxPowerByRate(
	IN	PADAPTER	pAdapter,
	IN	u8			Band,
	IN	enum rf_path	RFPath,
	IN	u8			Rate
)
{
	if (!phy_is_tx_power_by_rate_needed(pAdapter))
		return 0;

	return _PHY_GetTxPowerByRate(pAdapter, Band, RFPath, Rate);
}

VOID
PHY_SetTxPowerByRate(
	IN	PADAPTER	pAdapter,
	IN	u8			Band,
	IN	enum rf_path	RFPath,
	IN	u8			Rate,
	IN	s8			Value
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	u8	rateIndex = PHY_GetRateIndexOfTxPowerByRate(Rate);

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_INFO("Invalid band %d in %s\n", Band, __FUNCTION__);
		return;
	}
	if (RFPath > RF_PATH_D) {
		RTW_INFO("Invalid RfPath %d in %s\n", RFPath, __FUNCTION__);
		return;
	}
	if (rateIndex >= TX_PWR_BY_RATE_NUM_RATE) {
		RTW_INFO("Invalid RateIndex %d in %s\n", rateIndex, __FUNCTION__);
		return;
	}

	pHalData->TxPwrByRateOffset[Band][RFPath][rateIndex] = Value;
}

u8 phy_check_under_survey_ch(_adapter *adapter)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	_adapter *iface;
	struct mlme_ext_priv *mlmeext;
	u8 ret = _FALSE;
	int i;

	for (i = 0; i < dvobj->iface_nums; i++) {
		iface = dvobj->padapters[i];
		if (!iface)
			continue;
		mlmeext = &iface->mlmeextpriv;

		/* check scan state */
		if (mlmeext_scan_state(mlmeext) != SCAN_DISABLE
			&& mlmeext_scan_state(mlmeext) != SCAN_COMPLETE
				&& mlmeext_scan_state(mlmeext) != SCAN_BACKING_OP) {
			ret = _TRUE;
		} else if (mlmeext_scan_state(mlmeext) == SCAN_BACKING_OP
			&& !mlmeext_chk_scan_backop_flags(mlmeext, SS_BACKOP_TX_RESUME)) {
			ret = _TRUE;
		}
	}

	return ret;
}

VOID
phy_set_tx_power_level_by_path(
	IN	PADAPTER	Adapter,
	IN	u8			channel,
	IN	u8			path
)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);
	BOOLEAN bIsIn24G = (pHalData->current_band_type == BAND_ON_2_4G);
	u8 under_survey_ch = phy_check_under_survey_ch(Adapter);


	/* if ( pMgntInfo->RegNByteAccess == 0 ) */
	{
		if (bIsIn24G)
			phy_set_tx_power_index_by_rate_section(Adapter, path, channel, CCK);

		phy_set_tx_power_index_by_rate_section(Adapter, path, channel, OFDM);

		if (!under_survey_ch) {
			phy_set_tx_power_index_by_rate_section(Adapter, path, channel, HT_MCS0_MCS7);

			if (IS_HARDWARE_TYPE_JAGUAR(Adapter) || IS_HARDWARE_TYPE_8814A(Adapter))
				phy_set_tx_power_index_by_rate_section(Adapter, path, channel, VHT_1SSMCS0_1SSMCS9);

			if (pHalData->NumTotalRFPath >= 2) {
				phy_set_tx_power_index_by_rate_section(Adapter, path, channel, HT_MCS8_MCS15);

				if (IS_HARDWARE_TYPE_JAGUAR(Adapter) || IS_HARDWARE_TYPE_8814A(Adapter))
					phy_set_tx_power_index_by_rate_section(Adapter, path, channel, VHT_2SSMCS0_2SSMCS9);

				if (IS_HARDWARE_TYPE_8814A(Adapter)) {
					phy_set_tx_power_index_by_rate_section(Adapter, path, channel, HT_MCS16_MCS23);
					phy_set_tx_power_index_by_rate_section(Adapter, path, channel, VHT_3SSMCS0_3SSMCS9);
				}
			}
		}
	}
}

#ifndef DBG_TX_POWER_IDX
#define DBG_TX_POWER_IDX 0
#endif

VOID
PHY_SetTxPowerIndexByRateArray(
	IN	PADAPTER			pAdapter,
	IN	enum rf_path			RFPath,
	IN	enum channel_width	BandWidth,
	IN	u8					Channel,
	IN	u8					*Rates,
	IN	u8					RateArraySize
)
{
	u32	powerIndex = 0;
	int	i = 0;

	for (i = 0; i < RateArraySize; ++i) {
#if DBG_TX_POWER_IDX
		struct txpwr_idx_comp tic;

		powerIndex = rtw_hal_get_tx_power_index(pAdapter, RFPath, Rates[i], BandWidth, Channel, &tic);
		RTW_INFO("TXPWR: [%c][%s]ch:%u, %s %uT, pwr_idx:%u = %u + (%d=%d:%d) + (%d) + (%d)\n"
			, rf_path_char(RFPath), ch_width_str(BandWidth), Channel, MGN_RATE_STR(Rates[i]), tic.ntx_idx + 1
			, powerIndex, tic.base, (tic.by_rate > tic.limit ? tic.limit : tic.by_rate), tic.by_rate, tic.limit, tic.tpt, tic.ebias);
#else
		powerIndex = phy_get_tx_power_index(pAdapter, RFPath, Rates[i], BandWidth, Channel);
#endif
		PHY_SetTxPowerIndex(pAdapter, powerIndex, RFPath, Rates[i]);
	}
}

#if CONFIG_TXPWR_LIMIT
const char *const _txpwr_lmt_rs_str[] = {
	"CCK",
	"OFDM",
	"HT",
	"VHT",
	"UNKNOWN",
};

static s8
phy_GetChannelIndexOfTxPowerLimit(
	IN	u8			Band,
	IN	u8			Channel
)
{
	s8	channelIndex = -1;
	u8	i = 0;

	if (Band == BAND_ON_2_4G)
		channelIndex = Channel - 1;
	else if (Band == BAND_ON_5G) {
		for (i = 0; i < CENTER_CH_5G_ALL_NUM; ++i) {
			if (center_ch_5g_all[i] == Channel)
				channelIndex = i;
		}
	} else
		RTW_PRINT("Invalid Band %d in %s\n", Band, __func__);

	if (channelIndex == -1)
		RTW_PRINT("Invalid Channel %d of Band %d in %s\n", Channel, Band, __func__);

	return channelIndex;
}

static s8 phy_txpwr_ww_lmt_value(_adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);

	if (hal_spec->txgi_max == 63)
		return -63;
	else if (hal_spec->txgi_max == 127)
		return -128;

	rtw_warn_on(1);
	return -128;
}

/*
* return txpwr limit absolute value
* hsl_spec->txgi_max is returned when NO limit
*/
s8 phy_get_txpwr_lmt_abs(
	IN	PADAPTER			Adapter,
	IN	const char			*regd_name,
	IN	BAND_TYPE			Band,
	IN	enum channel_width		bw,
	u8 tlrs,
	u8 ntx_idx,
	u8 cch,
	u8 lock
)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(Adapter);
	struct rf_ctl_t *rfctl = adapter_to_rfctl(Adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(Adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(Adapter);
	struct txpwr_lmt_ent *ent = NULL;
	_irqL irqL;
	_list *cur, *head;
	s8 ch_idx;
	u8 is_ww_regd = 0;
	s8 ww_lmt_val = phy_txpwr_ww_lmt_value(Adapter);
	s8 lmt = hal_spec->txgi_max;

	if ((Adapter->registrypriv.RegEnableTxPowerLimit == 2 && hal_data->EEPROMRegulatory != 1) ||
		Adapter->registrypriv.RegEnableTxPowerLimit == 0)
		goto exit;

	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G) {
		RTW_ERR("%s invalid band:%u\n", __func__, Band);
		rtw_warn_on(1);
		goto exit;
	}

	if (Band == BAND_ON_5G  && tlrs == TXPWR_LMT_RS_CCK) {
		RTW_ERR("5G has no CCK\n");
		goto exit;
	}

	if (lock)
		_enter_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);

	if (!regd_name) /* no regd_name specified, use currnet */
		regd_name = rfctl->regd_name;

	if (rfctl->txpwr_regd_num == 0
		|| strcmp(regd_name, regd_str(TXPWR_LMT_NONE)) == 0)
		goto release_lock;

	if (strcmp(regd_name, regd_str(TXPWR_LMT_WW)) == 0)
		is_ww_regd = 1;

	if (!is_ww_regd) {
		ent = _rtw_txpwr_lmt_get_by_name(rfctl, regd_name);
		if (!ent)
			goto release_lock;
	}

	ch_idx = phy_GetChannelIndexOfTxPowerLimit(Band, cch);
	if (ch_idx == -1)
		goto release_lock;

	if (Band == BAND_ON_2_4G) {
		if (!is_ww_regd) {
			lmt = ent->lmt_2g[bw][tlrs][ch_idx][ntx_idx];
			if (lmt != ww_lmt_val)
				goto release_lock;
		}

		/* search for min value for WW regd or WW limit */
		lmt = hal_spec->txgi_max;
		head = &rfctl->txpwr_lmt_list;
		cur = get_next(head);
		while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
			ent = LIST_CONTAINOR(cur, struct txpwr_lmt_ent, list);
			cur = get_next(cur);
			if (ent->lmt_2g[bw][tlrs][ch_idx][ntx_idx] != ww_lmt_val)
				lmt = rtw_min(lmt, ent->lmt_2g[bw][tlrs][ch_idx][ntx_idx]);
		}
	}
	#ifdef CONFIG_IEEE80211_BAND_5GHZ
	else if (Band == BAND_ON_5G) {
		if (!is_ww_regd) {
			lmt = ent->lmt_5g[bw][tlrs - 1][ch_idx][ntx_idx];
			if (lmt != ww_lmt_val)
				goto release_lock;
		}

		/* search for min value for WW regd or WW limit */
		lmt = hal_spec->txgi_max;
		head = &rfctl->txpwr_lmt_list;
		cur = get_next(head);
		while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
			ent = LIST_CONTAINOR(cur, struct txpwr_lmt_ent, list);
			cur = get_next(cur);
			if (ent->lmt_5g[bw][tlrs - 1][ch_idx][ntx_idx] != ww_lmt_val)
				lmt = rtw_min(lmt, ent->lmt_5g[bw][tlrs - 1][ch_idx][ntx_idx]);
		}
	}
	#endif

release_lock:
	if (lock)
		_exit_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);

exit:
	return lmt;
}

/*
* return txpwr limit diff value
* hal_spec->txgi_max is returned when NO limit
*/
inline s8 phy_get_txpwr_lmt(_adapter *adapter
	, const char *regd_name
	, BAND_TYPE band, enum channel_width bw
	, u8 rfpath, u8 rs, u8 ntx_idx, u8 cch, u8 lock
)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 tlrs;
	s8 lmt = hal_spec->txgi_max;

	if (IS_CCK_RATE_SECTION(rs))
		tlrs = TXPWR_LMT_RS_CCK;
	else if (IS_OFDM_RATE_SECTION(rs))
		tlrs = TXPWR_LMT_RS_OFDM;
	else if (IS_HT_RATE_SECTION(rs))
		tlrs = TXPWR_LMT_RS_HT;
	else if (IS_VHT_RATE_SECTION(rs))
		tlrs = TXPWR_LMT_RS_VHT;
	else {
		RTW_ERR("%s invalid rs %u\n", __func__, rs);
		rtw_warn_on(1);
		goto exit;
	}

	lmt = phy_get_txpwr_lmt_abs(adapter, regd_name, band, bw, tlrs, ntx_idx, cch, lock);

	if (lmt != hal_spec->txgi_max) {
		/* return diff value */
		lmt = lmt - PHY_GetTxPowerByRateBase(adapter, band, rfpath, rs);
	}

exit:
	return lmt;
}

/*
* May search for secondary channels for min limit
* return txpwr limit diff value
*/
s8
PHY_GetTxPowerLimit(_adapter *adapter
	, const char *regd_name
	, BAND_TYPE band, enum channel_width bw
	, u8 rfpath, u8 rate, u8 ntx_idx, u8 cch)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	BOOLEAN no_sc = _FALSE;
	s8 tlrs = -1, rs = -1;
	s8 lmt = hal_spec->txgi_max;
	u8 tmp_cch = 0;
	u8 tmp_bw;
	u8 bw_bmp = 0;
	s8 min_lmt = hal_spec->txgi_max;
	u8 final_bw = bw, final_cch = cch;
	_irqL irqL;

#ifdef CONFIG_MP_INCLUDED
	/* MP mode channel don't use secondary channel */
	if (rtw_mp_mode_check(adapter) == _TRUE)
		no_sc = _TRUE;
#endif
	if (IS_CCK_RATE(rate)) {
		tlrs = TXPWR_LMT_RS_CCK;
		rs = CCK;
	} else if (IS_OFDM_RATE(rate)) {
		tlrs = TXPWR_LMT_RS_OFDM;
		rs = OFDM;
	} else if (IS_HT_RATE(rate)) {
		tlrs = TXPWR_LMT_RS_HT;
		rs = HT_1SS + (IS_HT1SS_RATE(rate) ? 0 : IS_HT2SS_RATE(rate) ? 1 : IS_HT3SS_RATE(rate) ? 2 : IS_HT4SS_RATE(rate) ? 3 : 0);
	} else if (IS_VHT_RATE(rate)) {
		tlrs = TXPWR_LMT_RS_VHT;
		rs = VHT_1SS + (IS_VHT1SS_RATE(rate) ? 0 : IS_VHT2SS_RATE(rate) ? 1 : IS_VHT3SS_RATE(rate) ? 2 : IS_VHT4SS_RATE(rate) ? 3 : 0);
	} else {
		RTW_ERR("%s invalid rate 0x%x\n", __func__, rate);
		rtw_warn_on(1);
		goto exit;
	}

	if (no_sc == _TRUE) {
		/* use the input center channel and bandwidth directly */
		tmp_cch = cch;
		bw_bmp = ch_width_to_bw_cap(bw);
	} else {
		/*
		* find the possible tx bandwidth bmp for this rate, and then will get center channel for each bandwidth
		* if no possible tx bandwidth bmp, select valid bandwidth up to current RF bandwidth into bmp
		*/
		if (tlrs == TXPWR_LMT_RS_CCK || tlrs == TXPWR_LMT_RS_OFDM)
			bw_bmp = BW_CAP_20M; /* CCK, OFDM only BW 20M */
		else if (tlrs == TXPWR_LMT_RS_HT) {
			bw_bmp = rtw_get_tx_bw_bmp_of_ht_rate(dvobj, rate, bw);
			if (bw_bmp == 0)
				bw_bmp = ch_width_to_bw_cap(bw > CHANNEL_WIDTH_40 ? CHANNEL_WIDTH_40 : bw);
		} else if (tlrs == TXPWR_LMT_RS_VHT) {
			bw_bmp = rtw_get_tx_bw_bmp_of_vht_rate(dvobj, rate, bw);
			if (bw_bmp == 0)
				bw_bmp = ch_width_to_bw_cap(bw > CHANNEL_WIDTH_160 ? CHANNEL_WIDTH_160 : bw);
		} else
			rtw_warn_on(1);
	}

	if (bw_bmp == 0)
		goto exit;

	_enter_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);

	/* loop for each possible tx bandwidth to find minimum limit */
	for (tmp_bw = CHANNEL_WIDTH_20; tmp_bw <= bw; tmp_bw++) {
		if (!(ch_width_to_bw_cap(tmp_bw) & bw_bmp))
			continue;

		if (no_sc == _FALSE) {
			if (tmp_bw == CHANNEL_WIDTH_20)
				tmp_cch = hal_data->cch_20;
			else if (tmp_bw == CHANNEL_WIDTH_40)
				tmp_cch = hal_data->cch_40;
			else if (tmp_bw == CHANNEL_WIDTH_80)
				tmp_cch = hal_data->cch_80;
			else {
				tmp_cch = 0;
				rtw_warn_on(1);
			}
		}

		lmt = phy_get_txpwr_lmt_abs(adapter, regd_name, band, tmp_bw, tlrs, ntx_idx, tmp_cch, 0);

		if (min_lmt >= lmt) {
			min_lmt = lmt;
			final_cch = tmp_cch;
			final_bw = tmp_bw;
		}

	}

	_exit_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);

	if (min_lmt != hal_spec->txgi_max) {
		/* return diff value */
		min_lmt = min_lmt - PHY_GetTxPowerByRateBase(adapter, band, rfpath, rs);
	}

exit:

	if (0) {
		if (final_bw != bw && (IS_HT_RATE(rate) || IS_VHT_RATE(rate)))
			RTW_INFO("%s min_lmt: %s ch%u -> %s ch%u\n"
				, MGN_RATE_STR(rate)
				, ch_width_str(bw), cch
				, ch_width_str(final_bw), final_cch);
	}

	return min_lmt;
}

static void phy_txpwr_lmt_cck_ofdm_mt_chk(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	struct txpwr_lmt_ent *ent;
	_list *cur, *head;
	u8 channel, tlrs, ntx_idx;

	rfctl->txpwr_lmt_2g_cck_ofdm_state = 0;
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	rfctl->txpwr_lmt_5g_cck_ofdm_state = 0;
#endif

	head = &rfctl->txpwr_lmt_list;
	cur = get_next(head);

	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		ent = LIST_CONTAINOR(cur, struct txpwr_lmt_ent, list);
		cur = get_next(cur);

		/* check 2G CCK, OFDM state*/
		for (tlrs = TXPWR_LMT_RS_CCK; tlrs <= TXPWR_LMT_RS_OFDM; tlrs++) {
			for (ntx_idx = RF_1TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {
				for (channel = 0; channel < CENTER_CH_2G_NUM; ++channel) {
					if (ent->lmt_2g[CHANNEL_WIDTH_20][tlrs][channel][ntx_idx] != hal_spec->txgi_max) {
						if (tlrs == TXPWR_LMT_RS_CCK)
							rfctl->txpwr_lmt_2g_cck_ofdm_state |= TXPWR_LMT_HAS_CCK_1T << ntx_idx;
						else
							rfctl->txpwr_lmt_2g_cck_ofdm_state |= TXPWR_LMT_HAS_OFDM_1T << ntx_idx;
						break;
					}
				}
			}
		}

		/* if 2G OFDM multi-TX is not defined, reference HT20 */
		for (channel = 0; channel < CENTER_CH_2G_NUM; ++channel) {
			for (ntx_idx = RF_2TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {
				if (rfctl->txpwr_lmt_2g_cck_ofdm_state & (TXPWR_LMT_HAS_OFDM_1T << ntx_idx))
					continue;
				ent->lmt_2g[CHANNEL_WIDTH_20][TXPWR_LMT_RS_OFDM][channel][ntx_idx] =
					ent->lmt_2g[CHANNEL_WIDTH_20][TXPWR_LMT_RS_HT][channel][ntx_idx];
			}
		}

#ifdef CONFIG_IEEE80211_BAND_5GHZ
		/* check 5G OFDM state*/
		for (ntx_idx = RF_1TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {
			for (channel = 0; channel < CENTER_CH_5G_ALL_NUM; ++channel) {
				if (ent->lmt_5g[CHANNEL_WIDTH_20][TXPWR_LMT_RS_OFDM - 1][channel][ntx_idx] != hal_spec->txgi_max) {
					rfctl->txpwr_lmt_5g_cck_ofdm_state |= TXPWR_LMT_HAS_OFDM_1T << ntx_idx;
					break;
				}
			}
		}

		for (channel = 0; channel < CENTER_CH_5G_ALL_NUM; ++channel) {
			for (ntx_idx = RF_2TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {
				if (rfctl->txpwr_lmt_5g_cck_ofdm_state & (TXPWR_LMT_HAS_OFDM_1T << ntx_idx))
					continue;
				/* if 5G OFDM multi-TX is not defined, reference HT20 */
				ent->lmt_5g[CHANNEL_WIDTH_20][TXPWR_LMT_RS_OFDM - 1][channel][ntx_idx] =
					ent->lmt_5g[CHANNEL_WIDTH_20][TXPWR_LMT_RS_HT - 1][channel][ntx_idx];
			}
		}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
	}
}

#ifdef CONFIG_IEEE80211_BAND_5GHZ
static void phy_txpwr_lmt_cross_ref_ht_vht(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	struct txpwr_lmt_ent *ent;
	_list *cur, *head;
	u8 bw, channel, tlrs, ref_tlrs, ntx_idx;
	int ht_ref_vht_5g_20_40 = 0;
	int vht_ref_ht_5g_20_40 = 0;
	int ht_has_ref_5g_20_40 = 0;
	int vht_has_ref_5g_20_40 = 0;

	rfctl->txpwr_lmt_5g_20_40_ref = 0;

	head = &rfctl->txpwr_lmt_list;
	cur = get_next(head);

	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		ent = LIST_CONTAINOR(cur, struct txpwr_lmt_ent, list);
		cur = get_next(cur);

		for (bw = 0; bw < MAX_5G_BANDWIDTH_NUM; ++bw) {

			for (channel = 0; channel < CENTER_CH_5G_ALL_NUM; ++channel) {

				for (tlrs = TXPWR_LMT_RS_HT; tlrs < TXPWR_LMT_RS_NUM; ++tlrs) {

					/* 5G 20M 40M VHT and HT can cross reference */
					if (bw == CHANNEL_WIDTH_20 || bw == CHANNEL_WIDTH_40) {
						if (tlrs == TXPWR_LMT_RS_HT)
							ref_tlrs = TXPWR_LMT_RS_VHT;
						else if (tlrs == TXPWR_LMT_RS_VHT)
							ref_tlrs = TXPWR_LMT_RS_HT;
						else
							continue;

						for (ntx_idx = RF_1TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {

							if (ent->lmt_5g[bw][ref_tlrs - 1][channel][ntx_idx] == hal_spec->txgi_max)
								continue;

							if (tlrs == TXPWR_LMT_RS_HT)
								ht_has_ref_5g_20_40++;
							else if (tlrs == TXPWR_LMT_RS_VHT)
								vht_has_ref_5g_20_40++;
							else
								continue;

							if (ent->lmt_5g[bw][tlrs - 1][channel][ntx_idx] != hal_spec->txgi_max)
								continue;

							if (tlrs == TXPWR_LMT_RS_HT && ref_tlrs == TXPWR_LMT_RS_VHT)
								ht_ref_vht_5g_20_40++;
							else if (tlrs == TXPWR_LMT_RS_VHT && ref_tlrs == TXPWR_LMT_RS_HT)
								vht_ref_ht_5g_20_40++;

							if (0)
								RTW_INFO("reg:%s, bw:%u, ch:%u, %s-%uT ref %s-%uT\n"
									, ent->regd_name, bw, channel
									, txpwr_lmt_rs_str(tlrs), ntx_idx + 1
									, txpwr_lmt_rs_str(ref_tlrs), ntx_idx + 1);

							ent->lmt_5g[bw][tlrs - 1][channel][ntx_idx] =
								ent->lmt_5g[bw][ref_tlrs - 1][channel][ntx_idx];
						}
					}

				}
			}
		}
	}

	if (0) {
		RTW_INFO("ht_ref_vht_5g_20_40:%d, ht_has_ref_5g_20_40:%d\n", ht_ref_vht_5g_20_40, ht_has_ref_5g_20_40);
		RTW_INFO("vht_ref_ht_5g_20_40:%d, vht_has_ref_5g_20_40:%d\n", vht_ref_ht_5g_20_40, vht_has_ref_5g_20_40);
	}

	/* 5G 20M&40M HT all come from VHT*/
	if (ht_ref_vht_5g_20_40 && ht_has_ref_5g_20_40 == ht_ref_vht_5g_20_40)
		rfctl->txpwr_lmt_5g_20_40_ref |= TXPWR_LMT_REF_HT_FROM_VHT;

	/* 5G 20M&40M VHT all come from HT*/
	if (vht_ref_ht_5g_20_40 && vht_has_ref_5g_20_40 == vht_ref_ht_5g_20_40)
		rfctl->txpwr_lmt_5g_20_40_ref |= TXPWR_LMT_REF_VHT_FROM_HT;
}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

#ifndef DBG_TXPWR_LMT_BAND_CHK
#define DBG_TXPWR_LMT_BAND_CHK 0
#endif

#if DBG_TXPWR_LMT_BAND_CHK
/* check if larger bandwidth limit is less than smaller bandwidth for HT & VHT rate */
void phy_txpwr_limit_bandwidth_chk(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 band, bw, path, tlrs, ntx_idx, cch, offset, scch;
	u8 ch_num, n, i;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		for (bw = CHANNEL_WIDTH_40; bw <= CHANNEL_WIDTH_80; bw++) {
			if (bw >= CHANNEL_WIDTH_160)
				continue;
			if (band == BAND_ON_2_4G && bw >= CHANNEL_WIDTH_80)
				continue;

			if (band == BAND_ON_2_4G)
				ch_num = center_chs_2g_num(bw);
			else
				ch_num = center_chs_5g_num(bw);

			if (ch_num == 0) {
				rtw_warn_on(1);
				break;
			}

			for (tlrs = TXPWR_LMT_RS_HT; tlrs < TXPWR_LMT_RS_NUM; tlrs++) {

				if (band == BAND_ON_2_4G && tlrs == TXPWR_LMT_RS_VHT)
					continue;
				if (band == BAND_ON_5G && tlrs == TXPWR_LMT_RS_CCK)
					continue;
				if (bw > CHANNEL_WIDTH_20 && (tlrs == TXPWR_LMT_RS_CCK || tlrs == TXPWR_LMT_RS_OFDM))
					continue;
				if (bw > CHANNEL_WIDTH_40 && tlrs == TXPWR_LMT_RS_HT)
					continue;
				if (tlrs == TXPWR_LMT_RS_VHT && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
					continue;

				for (ntx_idx = RF_1TX; ntx_idx < MAX_TX_COUNT; ntx_idx++) {
					struct txpwr_lmt_ent *ent;
					_list *cur, *head;

					if (ntx_idx >= hal_spec->tx_nss_num)
						continue;

					/* bypass CCK multi-TX is not defined */
					if (tlrs == TXPWR_LMT_RS_CCK && ntx_idx > RF_1TX) {
						if (band == BAND_ON_2_4G
							&& !(rfctl->txpwr_lmt_2g_cck_ofdm_state & (TXPWR_LMT_HAS_CCK_1T << ntx_idx)))
							continue;
					}

					/* bypass OFDM multi-TX is not defined */
					if (tlrs == TXPWR_LMT_RS_OFDM && ntx_idx > RF_1TX) {
						if (band == BAND_ON_2_4G
							&& !(rfctl->txpwr_lmt_2g_cck_ofdm_state & (TXPWR_LMT_HAS_OFDM_1T << ntx_idx)))
							continue;
						#ifdef CONFIG_IEEE80211_BAND_5GHZ
						if (band == BAND_ON_5G
							&& !(rfctl->txpwr_lmt_5g_cck_ofdm_state & (TXPWR_LMT_HAS_OFDM_1T << ntx_idx)))
							continue;
						#endif
					}

					/* bypass 5G 20M, 40M pure reference */
					#ifdef CONFIG_IEEE80211_BAND_5GHZ
					if (band == BAND_ON_5G && (bw == CHANNEL_WIDTH_20 || bw == CHANNEL_WIDTH_40)) {
						if (rfctl->txpwr_lmt_5g_20_40_ref == TXPWR_LMT_REF_HT_FROM_VHT) {
							if (tlrs == TXPWR_LMT_RS_HT)
								continue;
						} else if (rfctl->txpwr_lmt_5g_20_40_ref == TXPWR_LMT_REF_VHT_FROM_HT) {
							if (tlrs == TXPWR_LMT_RS_VHT && bw <= CHANNEL_WIDTH_40)
								continue;
						}
					}
					#endif

					for (n = 0; n < ch_num; n++) {
						u8 cch_by_bw[3];
						u8 offset_by_bw; /* bitmap, 0 for lower, 1 for upper */
						u8 bw_pos;
						s8 lmt[3];

						if (band == BAND_ON_2_4G)
							cch = center_chs_2g(bw, n);
						else
							cch = center_chs_5g(bw, n);

						if (cch == 0) {
							rtw_warn_on(1);
							break;
						}

						_rtw_memset(cch_by_bw, 0, 3);
						cch_by_bw[bw] = cch;
						offset_by_bw = 0x01;

						do {
							for (bw_pos = bw; bw_pos >= CHANNEL_WIDTH_40; bw_pos--)
								cch_by_bw[bw_pos - 1] = rtw_get_scch_by_cch_offset(cch_by_bw[bw_pos], bw_pos, offset_by_bw & BIT(bw_pos) ? HAL_PRIME_CHNL_OFFSET_UPPER : HAL_PRIME_CHNL_OFFSET_LOWER);

							head = &rfctl->txpwr_lmt_list;
							cur = get_next(head);
							while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
								ent = LIST_CONTAINOR(cur, struct txpwr_lmt_ent, list);
								cur = get_next(cur);

								for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--)
									lmt[bw_pos] = phy_get_txpwr_lmt_abs(adapter, ent->regd_name, band, bw_pos, tlrs, ntx_idx, cch_by_bw[bw_pos], 0);

								for (bw_pos = bw; bw_pos > CHANNEL_WIDTH_20; bw_pos--)
									if (lmt[bw_pos] > lmt[bw_pos - 1])
										break;
								if (bw_pos == CHANNEL_WIDTH_20)
									continue;

								RTW_PRINT_SEL(RTW_DBGDUMP, "[%s][%s][%s][%uT][%-4s] cch:"
									, band_str(band)
									, ch_width_str(bw)
									, txpwr_lmt_rs_str(tlrs)
									, ntx_idx + 1
									, ent->regd_name
								);
								for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--)
									_RTW_PRINT_SEL(RTW_DBGDUMP, "%03u ", cch_by_bw[bw_pos]);
								_RTW_PRINT_SEL(RTW_DBGDUMP, "limit:");
								for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--) {
									if (lmt[bw_pos] == hal_spec->txgi_max)
										_RTW_PRINT_SEL(RTW_DBGDUMP, "N/A ");
									else if (lmt[bw_pos] > -hal_spec->txgi_pdbm && lmt[bw_pos] < 0) /* -1 < value < 0 */
										_RTW_PRINT_SEL(RTW_DBGDUMP, "-0.%d", (rtw_abs(lmt[bw_pos]) % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
									else if (lmt[bw_pos] % hal_spec->txgi_pdbm)
										_RTW_PRINT_SEL(RTW_DBGDUMP, "%2d.%d ", lmt[bw_pos] / hal_spec->txgi_pdbm, (rtw_abs(lmt[bw_pos]) % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
									else
										_RTW_PRINT_SEL(RTW_DBGDUMP, "%2d ", lmt[bw_pos] / hal_spec->txgi_pdbm);
								}
								_RTW_PRINT_SEL(RTW_DBGDUMP, "\n");
							}
							for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--)
								lmt[bw_pos] = phy_get_txpwr_lmt_abs(adapter, regd_str(TXPWR_LMT_WW), band, bw_pos, tlrs, ntx_idx, cch_by_bw[bw_pos], 0);

							for (bw_pos = bw; bw_pos > CHANNEL_WIDTH_20; bw_pos--)
								if (lmt[bw_pos] > lmt[bw_pos - 1])
									break;
							if (bw_pos != CHANNEL_WIDTH_20) {
								RTW_PRINT_SEL(RTW_DBGDUMP, "[%s][%s][%s][%uT][%-4s] cch:"
									, band_str(band)
									, ch_width_str(bw)
									, txpwr_lmt_rs_str(tlrs)
									, ntx_idx + 1
									, regd_str(TXPWR_LMT_WW)
								);
								for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--)
									_RTW_PRINT_SEL(RTW_DBGDUMP, "%03u ", cch_by_bw[bw_pos]);
								_RTW_PRINT_SEL(RTW_DBGDUMP, "limit:");
								for (bw_pos = bw; bw_pos < CHANNEL_WIDTH_160; bw_pos--) {
									if (lmt[bw_pos] == hal_spec->txgi_max)
										_RTW_PRINT_SEL(RTW_DBGDUMP, "N/A ");
									else if (lmt[bw_pos] > -hal_spec->txgi_pdbm && lmt[bw_pos] < 0) /* -1 < value < 0 */
										_RTW_PRINT_SEL(RTW_DBGDUMP, "-0.%d", (rtw_abs(lmt[bw_pos]) % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
									else if (lmt[bw_pos] % hal_spec->txgi_pdbm)
										_RTW_PRINT_SEL(RTW_DBGDUMP, "%2d.%d ", lmt[bw_pos] / hal_spec->txgi_pdbm, (rtw_abs(lmt[bw_pos]) % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
									else
										_RTW_PRINT_SEL(RTW_DBGDUMP, "%2d ", lmt[bw_pos] / hal_spec->txgi_pdbm);
								}
								_RTW_PRINT_SEL(RTW_DBGDUMP, "\n");
							}

							offset_by_bw += 2;
							if (offset_by_bw & BIT(bw + 1))
								break;
						} while (1); /* loop for all ch combinations */
					} /* loop for center channels */
				} /* loop fo each ntx_idx */
			} /* loop for tlrs */
		} /* loop for bandwidth */
	} /* loop for band */
}
#endif /* DBG_TXPWR_LMT_BAND_CHK */

static void phy_txpwr_lmt_post_hdl(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	_irqL irqL;

	_enter_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);

#ifdef CONFIG_IEEE80211_BAND_5GHZ
	if (IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
		phy_txpwr_lmt_cross_ref_ht_vht(adapter);
#endif
	phy_txpwr_lmt_cck_ofdm_mt_chk(adapter);

#if DBG_TXPWR_LMT_BAND_CHK
	phy_txpwr_limit_bandwidth_chk(adapter);
#endif

	_exit_critical_mutex(&rfctl->txpwr_lmt_mutex, &irqL);
}

BOOLEAN
GetS1ByteIntegerFromStringInDecimal(
	IN		char	*str,
	IN OUT	s8		*val
)
{
	u8 negative = 0;
	u16 i = 0;

	*val = 0;

	while (str[i] != '\0') {
		if (i == 0 && (str[i] == '+' || str[i] == '-')) {
			if (str[i] == '-')
				negative = 1;
		} else if (str[i] >= '0' && str[i] <= '9') {
			*val *= 10;
			*val += (str[i] - '0');
		} else
			return _FALSE;
		++i;
	}

	if (negative)
		*val = -*val;

	return _TRUE;
}
#endif /* CONFIG_TXPWR_LIMIT */

/*
* phy_set_tx_power_limit - Parsing TX power limit from phydm array, called by odm_ConfigBB_TXPWR_LMT_XXX in phydm
*/
VOID
phy_set_tx_power_limit(
	IN	struct dm_struct		*pDM_Odm,
	IN	u8				*Regulation,
	IN	u8				*Band,
	IN	u8				*Bandwidth,
	IN	u8				*RateSection,
	IN	u8				*ntx,
	IN	u8				*Channel,
	IN	u8				*PowerLimit
)
{
#if CONFIG_TXPWR_LIMIT
	PADAPTER Adapter = pDM_Odm->adapter;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(Adapter);
	u8 band = 0, bandwidth = 0, tlrs = 0, channel;
	u8 ntx_idx;
	s8 powerLimit = 0, prevPowerLimit, channelIndex;
	s8 ww_lmt_val = phy_txpwr_ww_lmt_value(Adapter);

	if (0)
		RTW_INFO("Index of power limit table [regulation %s][band %s][bw %s][rate section %s][ntx %s][chnl %s][val %s]\n"
			, Regulation, Band, Bandwidth, RateSection, ntx, Channel, PowerLimit);

	if (GetU1ByteIntegerFromStringInDecimal((char *)Channel, &channel) == _FALSE
		|| GetS1ByteIntegerFromStringInDecimal((char *)PowerLimit, &powerLimit) == _FALSE
	) {
		RTW_PRINT("Illegal index of power limit table [ch %s][val %s]\n", Channel, PowerLimit);
		return;
	}

	if (powerLimit != ww_lmt_val) {
		if (powerLimit < -hal_spec->txgi_max || powerLimit > hal_spec->txgi_max)
			RTW_PRINT("Illegal power limit value [ch %s][val %s]\n", Channel, PowerLimit);

		if (powerLimit > hal_spec->txgi_max)
			powerLimit = hal_spec->txgi_max;
		else if (powerLimit < -hal_spec->txgi_max)
			powerLimit =  ww_lmt_val + 1;
	}

	if (eqNByte(RateSection, (u8 *)("CCK"), 3))
		tlrs = TXPWR_LMT_RS_CCK;
	else if (eqNByte(RateSection, (u8 *)("OFDM"), 4))
		tlrs = TXPWR_LMT_RS_OFDM;
	else if (eqNByte(RateSection, (u8 *)("HT"), 2))
		tlrs = TXPWR_LMT_RS_HT;
	else if (eqNByte(RateSection, (u8 *)("VHT"), 3))
		tlrs = TXPWR_LMT_RS_VHT;
	else {
		RTW_PRINT("Wrong rate section:%s\n", RateSection);
		return;
	}

	if (eqNByte(ntx, (u8 *)("1T"), 2))
		ntx_idx = RF_1TX;
	else if (eqNByte(ntx, (u8 *)("2T"), 2))
		ntx_idx = RF_2TX;
	else if (eqNByte(ntx, (u8 *)("3T"), 2))
		ntx_idx = RF_3TX;
	else if (eqNByte(ntx, (u8 *)("4T"), 2))
		ntx_idx = RF_4TX;
	else {
		RTW_PRINT("Wrong tx num:%s\n", ntx);
		return;
	}

	if (eqNByte(Bandwidth, (u8 *)("20M"), 3))
		bandwidth = CHANNEL_WIDTH_20;
	else if (eqNByte(Bandwidth, (u8 *)("40M"), 3))
		bandwidth = CHANNEL_WIDTH_40;
	else if (eqNByte(Bandwidth, (u8 *)("80M"), 3))
		bandwidth = CHANNEL_WIDTH_80;
	else if (eqNByte(Bandwidth, (u8 *)("160M"), 4))
		bandwidth = CHANNEL_WIDTH_160;
	else {
		RTW_PRINT("unknown bandwidth: %s\n", Bandwidth);
		return;
	}

	if (eqNByte(Band, (u8 *)("2.4G"), 4)) {
		band = BAND_ON_2_4G;
		channelIndex = phy_GetChannelIndexOfTxPowerLimit(BAND_ON_2_4G, channel);

		if (channelIndex == -1) {
			RTW_PRINT("unsupported channel: %d at 2.4G\n", channel);
			return;
		}

		if (bandwidth >= MAX_2_4G_BANDWIDTH_NUM) {
			RTW_PRINT("unsupported bandwidth: %s at 2.4G\n", Bandwidth);
			return;
		}

		rtw_txpwr_lmt_add(adapter_to_rfctl(Adapter), Regulation, band, bandwidth, tlrs, ntx_idx, channelIndex, powerLimit);
	}
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	else if (eqNByte(Band, (u8 *)("5G"), 2)) {
		band = BAND_ON_5G;
		channelIndex = phy_GetChannelIndexOfTxPowerLimit(BAND_ON_5G, channel);

		if (channelIndex == -1) {
			RTW_PRINT("unsupported channel: %d at 5G\n", channel);
			return;
		}

		rtw_txpwr_lmt_add(adapter_to_rfctl(Adapter), Regulation, band, bandwidth, tlrs, ntx_idx, channelIndex, powerLimit);
	}
#endif
	else {
		RTW_PRINT("unknown/unsupported band:%s\n", Band);
		return;
	}
#endif
}

u8
phy_get_tx_power_index(
	IN	PADAPTER			pAdapter,
	IN	enum rf_path			RFPath,
	IN	u8					Rate,
	IN	enum channel_width	BandWidth,
	IN	u8					Channel
)
{
	return rtw_hal_get_tx_power_index(pAdapter, RFPath, Rate, BandWidth, Channel, NULL);
}

VOID
PHY_SetTxPowerIndex(
	IN	PADAPTER		pAdapter,
	IN	u32				PowerIndex,
	IN	enum rf_path		RFPath,
	IN	u8				Rate
)
{
	rtw_hal_set_tx_power_index(pAdapter, PowerIndex, RFPath, Rate);
}

void dump_tx_power_idx_title(void *sel, _adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	u8 bw = hal_data->current_channel_bw;

	RTW_PRINT_SEL(sel, "%s", ch_width_str(bw));
	if (bw >= CHANNEL_WIDTH_80)
		_RTW_PRINT_SEL(sel, ", cch80:%u", hal_data->cch_80);
	if (bw >= CHANNEL_WIDTH_40)
		_RTW_PRINT_SEL(sel, ", cch40:%u", hal_data->cch_40);
	_RTW_PRINT_SEL(sel, ", cch20:%u\n", hal_data->cch_20);

	RTW_PRINT_SEL(sel, "%-4s %-9s %2s %-3s %-4s %-3s %-4s %-4s %-3s %-5s\n"
		, "path", "rate", "", "pwr", "base", "", "(byr", "lmt)", "tpt", "ebias");
}

void dump_tx_power_idx_by_path_rs(void *sel, _adapter *adapter, u8 rfpath, u8 rs)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	u8 power_idx;
	struct txpwr_idx_comp tic;
	u8 tx_num, i;
	u8 band = hal_data->current_band_type;
	u8 cch = hal_data->current_channel;
	u8 bw = hal_data->current_channel_bw;

	if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, rfpath))
		return;

	if (rs >= RATE_SECTION_NUM)
		return;

	tx_num = rate_section_to_tx_num(rs);
	if (tx_num >= hal_spec->tx_nss_num || tx_num >= hal_spec->max_tx_cnt)
		return;

	if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
		return;

	if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
		return;

	for (i = 0; i < rates_by_sections[rs].rate_num; i++) {
		power_idx = rtw_hal_get_tx_power_index(adapter, rfpath, rates_by_sections[rs].rates[i], bw, cch, &tic);

		RTW_PRINT_SEL(sel, "%4c %9s %uT %3u %4u %3d (%3d %3d) %3d %5d\n"
			, rf_path_char(rfpath), MGN_RATE_STR(rates_by_sections[rs].rates[i]), tic.ntx_idx + 1
			, power_idx, tic.base, (tic.by_rate > tic.limit ? tic.limit : tic.by_rate), tic.by_rate, tic.limit, tic.tpt, tic.ebias);
	}
}

void dump_tx_power_idx(void *sel, _adapter *adapter)
{
	u8 rfpath, rs;

	dump_tx_power_idx_title(sel, adapter);
	for (rfpath = RF_PATH_A; rfpath < RF_PATH_MAX; rfpath++)
		for (rs = CCK; rs < RATE_SECTION_NUM; rs++)
			dump_tx_power_idx_by_path_rs(sel, adapter, rfpath, rs);
}

bool phy_is_tx_power_limit_needed(_adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct registry_priv *regsty = dvobj_to_regsty(adapter_to_dvobj(adapter));

#if CONFIG_TXPWR_LIMIT
	if (regsty->RegEnableTxPowerLimit == 1
		|| (regsty->RegEnableTxPowerLimit == 2 && hal_data->EEPROMRegulatory == 1))
		return _TRUE;
#endif

	return _FALSE;
}

bool phy_is_tx_power_by_rate_needed(_adapter *adapter)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct registry_priv *regsty = dvobj_to_regsty(adapter_to_dvobj(adapter));

	if (regsty->RegEnableTxPowerByRate == 1
		|| (regsty->RegEnableTxPowerByRate == 2 && hal_data->EEPROMRegulatory != 2))
		return _TRUE;
	return _FALSE;
}

int phy_load_tx_power_by_rate(_adapter *adapter, u8 chk_file)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct registry_priv *regsty = dvobj_to_regsty(adapter_to_dvobj(adapter));
	int ret = _FAIL;

	hal_data->txpwr_by_rate_loaded = 0;
	PHY_InitTxPowerByRate(adapter);

	/* tx power limit is based on tx power by rate */
	hal_data->txpwr_limit_loaded = 0;

#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	if (chk_file
		&& phy_ConfigBBWithPgParaFile(adapter, PHY_FILE_PHY_REG_PG) == _SUCCESS
	) {
		hal_data->txpwr_by_rate_from_file = 1;
		goto post_hdl;
	}
#endif

#ifdef CONFIG_EMBEDDED_FWIMG
	if (HAL_STATUS_SUCCESS == odm_config_bb_with_header_file(&hal_data->odmpriv, CONFIG_BB_PHY_REG_PG)) {
		RTW_INFO("default power by rate loaded\n");
		hal_data->txpwr_by_rate_from_file = 0;
		goto post_hdl;
	}
#endif

	RTW_ERR("%s():Read Tx power by rate fail\n", __func__);
	goto exit;

post_hdl:
	if (hal_data->odmpriv.phy_reg_pg_value_type != PHY_REG_PG_EXACT_VALUE) {
		rtw_warn_on(1);
		goto exit;
	}

	PHY_TxPowerByRateConfiguration(adapter);
	hal_data->txpwr_by_rate_loaded = 1;

	ret = _SUCCESS;

exit:
	return ret;
}

#if CONFIG_TXPWR_LIMIT
int phy_load_tx_power_limit(_adapter *adapter, u8 chk_file)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct registry_priv *regsty = dvobj_to_regsty(adapter_to_dvobj(adapter));
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	int ret = _FAIL;

	hal_data->txpwr_limit_loaded = 0;
	rtw_regd_exc_list_free(rfctl);
	rtw_txpwr_lmt_list_free(rfctl);

	if (!hal_data->txpwr_by_rate_loaded && regsty->target_tx_pwr_valid != _TRUE) {
		RTW_ERR("%s():Read Tx power limit before target tx power is specify\n", __func__);
		goto exit;
	}

#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	if (chk_file
		&& PHY_ConfigRFWithPowerLimitTableParaFile(adapter, PHY_FILE_TXPWR_LMT) == _SUCCESS
	) {
		hal_data->txpwr_limit_from_file = 1;
		goto post_hdl;
	}
#endif

#ifdef CONFIG_EMBEDDED_FWIMG
	if (odm_config_rf_with_header_file(&hal_data->odmpriv, CONFIG_RF_TXPWR_LMT, RF_PATH_A) == HAL_STATUS_SUCCESS) {
		RTW_INFO("default power limit loaded\n");
		hal_data->txpwr_limit_from_file = 0;
		goto post_hdl;
	}
#endif

	RTW_ERR("%s():Read Tx power limit fail\n", __func__);
	goto exit;

post_hdl:
	phy_txpwr_lmt_post_hdl(adapter);
	rtw_txpwr_init_regd(rfctl);
	hal_data->txpwr_limit_loaded = 1;
	ret = _SUCCESS;

exit:
	return ret;
}
#endif /* CONFIG_TXPWR_LIMIT */

void phy_load_tx_power_ext_info(_adapter *adapter, u8 chk_file)
{
	struct registry_priv *regsty = adapter_to_regsty(adapter);

	/* check registy target tx power */
	regsty->target_tx_pwr_valid = rtw_regsty_chk_target_tx_power_valid(adapter);

	/* power by rate and limit */
	if (phy_is_tx_power_by_rate_needed(adapter)
		|| (phy_is_tx_power_limit_needed(adapter) && regsty->target_tx_pwr_valid != _TRUE)
	)
		phy_load_tx_power_by_rate(adapter, chk_file);

#if CONFIG_TXPWR_LIMIT
	if (phy_is_tx_power_limit_needed(adapter))
		phy_load_tx_power_limit(adapter, chk_file);
#endif
}

inline void phy_reload_tx_power_ext_info(_adapter *adapter)
{
	phy_load_tx_power_ext_info(adapter, 1);
}

inline void phy_reload_default_tx_power_ext_info(_adapter *adapter)
{
	phy_load_tx_power_ext_info(adapter, 0);
}

void dump_tx_power_ext_info(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = adapter_to_regsty(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);

	if (regsty->target_tx_pwr_valid == _TRUE)
		RTW_PRINT_SEL(sel, "target_tx_power: from registry\n");
	else if (phy_is_tx_power_by_rate_needed(adapter))
		RTW_PRINT_SEL(sel, "target_tx_power: from power by rate\n");
	else
		RTW_PRINT_SEL(sel, "target_tx_power: unavailable\n");

	RTW_PRINT_SEL(sel, "tx_power_by_rate: %s, %s, %s\n"
		, phy_is_tx_power_by_rate_needed(adapter) ? "enabled" : "disabled"
		, hal_data->txpwr_by_rate_loaded ? "loaded" : "unloaded"
		, hal_data->txpwr_by_rate_from_file ? "file" : "default"
	);

	RTW_PRINT_SEL(sel, "tx_power_limit: %s, %s, %s\n"
		, phy_is_tx_power_limit_needed(adapter) ? "enabled" : "disabled"
		, hal_data->txpwr_limit_loaded ? "loaded" : "unloaded"
		, hal_data->txpwr_limit_from_file ? "file" : "default"
	);
}

void dump_target_tx_power(void *sel, _adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	struct registry_priv *regsty = adapter_to_regsty(adapter);
	int path, tx_num, band, rs;
	u8 target;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		for (path = 0; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				break;

			RTW_PRINT_SEL(sel, "[%s][%c]%s\n", band_str(band), rf_path_char(path)
				, (regsty->target_tx_pwr_valid == _FALSE && hal_data->txpwr_by_rate_undefined_band_path[band][path]) ? "(dup)" : "");

			for (rs = 0; rs < RATE_SECTION_NUM; rs++) {
				tx_num = rate_section_to_tx_num(rs);
				if (tx_num >= hal_spec->tx_nss_num)
					continue;

				if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
					continue;

				if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
					continue;

				target = PHY_GetTxPowerByRateBase(adapter, band, path, rs);

				if (target % hal_spec->txgi_pdbm) {
					_RTW_PRINT_SEL(sel, "%7s: %2d.%d\n", rate_section_str(rs)
						, target / hal_spec->txgi_pdbm, (target % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
				} else {
					_RTW_PRINT_SEL(sel, "%7s: %5d\n", rate_section_str(rs)
						, target / hal_spec->txgi_pdbm);
				}
			}
		}
	}

exit:
	return;
}

void dump_tx_power_by_rate(void *sel, _adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	int path, tx_num, band, n, rs;
	u8 rate_num, max_rate_num, base;
	s8 by_rate_offset;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; band++) {
		if (!hal_is_band_support(adapter, band))
			continue;

		for (path = 0; path < RF_PATH_MAX; path++) {
			if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
				break;

			RTW_PRINT_SEL(sel, "[%s][%c]%s\n", band_str(band), rf_path_char(path)
				, hal_data->txpwr_by_rate_undefined_band_path[band][path] ? "(dup)" : "");

			for (rs = 0; rs < RATE_SECTION_NUM; rs++) {
				tx_num = rate_section_to_tx_num(rs);
				if (tx_num >= hal_spec->tx_nss_num)
					continue;

				if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
					continue;

				if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
					continue;

				if (IS_HARDWARE_TYPE_JAGUAR_AND_JAGUAR2(adapter))
					max_rate_num = 10;
				else
					max_rate_num = 8;
				rate_num = rate_section_rate_num(rs);
				base = PHY_GetTxPowerByRateBase(adapter, band, path, rs);

				RTW_PRINT_SEL(sel, "%7s: ", rate_section_str(rs));

				/* dump power by rate in db */
				for (n = rate_num - 1; n >= 0; n--) {
					by_rate_offset = PHY_GetTxPowerByRate(adapter, band, path, rates_by_sections[rs].rates[n]);

					if ((base + by_rate_offset) % hal_spec->txgi_pdbm) {
						_RTW_PRINT_SEL(sel, "%2d.%d ", (base + by_rate_offset) / hal_spec->txgi_pdbm
							, ((base + by_rate_offset) % hal_spec->txgi_pdbm) * 100 / hal_spec->txgi_pdbm);
					} else
						_RTW_PRINT_SEL(sel, "%5d ", (base + by_rate_offset) / hal_spec->txgi_pdbm);
				}
				for (n = 0; n < max_rate_num - rate_num; n++)
					_RTW_PRINT_SEL(sel, "%5s ", "");

				_RTW_PRINT_SEL(sel, "|");

				/* dump power by rate in offset */
				for (n = rate_num - 1; n >= 0; n--) {
					by_rate_offset = PHY_GetTxPowerByRate(adapter, band, path, rates_by_sections[rs].rates[n]);
					_RTW_PRINT_SEL(sel, "%3d ", by_rate_offset);
				}
				RTW_PRINT_SEL(sel, "\n");

			}
		}
	}
}

/*
 * phy file path is stored in global char array rtw_phy_para_file_path
 * need to care about racing
 */
int rtw_get_phy_file_path(_adapter *adapter, const char *file_name)
{
#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);
	int len = 0;

	if (file_name) {
		len += snprintf(rtw_phy_para_file_path, PATH_LENGTH_MAX, "%s", rtw_phy_file_path);
		#if defined(CONFIG_MULTIDRV) || defined(REALTEK_CONFIG_PATH_WITH_IC_NAME_FOLDER)
		len += snprintf(rtw_phy_para_file_path + len, PATH_LENGTH_MAX - len, "%s/", hal_spec->ic_name);
		#endif
		len += snprintf(rtw_phy_para_file_path + len, PATH_LENGTH_MAX - len, "%s", file_name);

		return _TRUE;
	}
#endif
	return _FALSE;
}

#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
int
phy_ConfigMACWithParaFile(
	IN	PADAPTER	Adapter,
	IN	char		*pFileName
)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;
	char	*szLine, *ptmp;
	u32	u4bRegOffset, u4bRegValue, u4bMove;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_MAC_PARA_FILE))
		return rtStatus;

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if ((pHalData->mac_reg_len == 0) && (pHalData->mac_reg == NULL)) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pHalData->mac_reg = rtw_zvmalloc(rlen);
				if (pHalData->mac_reg) {
					_rtw_memcpy(pHalData->mac_reg, pHalData->para_file_buf, rlen);
					pHalData->mac_reg_len = rlen;
				} else
					RTW_INFO("%s mac_reg alloc fail !\n", __FUNCTION__);
			}
		}
	} else {
		if ((pHalData->mac_reg_len != 0) && (pHalData->mac_reg != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pHalData->mac_reg, pHalData->mac_reg_len);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		ptmp = pHalData->para_file_buf;
		for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
			if (!IsCommentString(szLine)) {
				/* Get 1st hex value as register offset */
				if (GetHexValueFromString(szLine, &u4bRegOffset, &u4bMove)) {
					if (u4bRegOffset == 0xffff) {
						/* Ending. */
						break;
					}

					/* Get 2nd hex value as register value. */
					szLine += u4bMove;
					if (GetHexValueFromString(szLine, &u4bRegValue, &u4bMove))
						rtw_write8(Adapter, u4bRegOffset, (u8)u4bRegValue);
				}
			}
		}
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}

int
phy_ConfigBBWithParaFile(
	IN	PADAPTER	Adapter,
	IN	char		*pFileName,
	IN	u32			ConfigType
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;
	char	*szLine, *ptmp;
	u32	u4bRegOffset, u4bRegValue, u4bMove;
	char	*pBuf = NULL;
	u32	*pBufLen = NULL;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_BB_PARA_FILE))
		return rtStatus;

	switch (ConfigType) {
	case CONFIG_BB_PHY_REG:
		pBuf = pHalData->bb_phy_reg;
		pBufLen = &pHalData->bb_phy_reg_len;
		break;
	case CONFIG_BB_AGC_TAB:
		pBuf = pHalData->bb_agc_tab;
		pBufLen = &pHalData->bb_agc_tab_len;
		break;
	default:
		RTW_INFO("Unknown ConfigType!! %d\r\n", ConfigType);
		break;
	}

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if ((pBufLen != NULL) && (*pBufLen == 0) && (pBuf == NULL)) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pBuf = rtw_zvmalloc(rlen);
				if (pBuf) {
					_rtw_memcpy(pBuf, pHalData->para_file_buf, rlen);
					*pBufLen = rlen;

					switch (ConfigType) {
					case CONFIG_BB_PHY_REG:
						pHalData->bb_phy_reg = pBuf;
						break;
					case CONFIG_BB_AGC_TAB:
						pHalData->bb_agc_tab = pBuf;
						break;
					}
				} else
					RTW_INFO("%s(): ConfigType %d  alloc fail !\n", __FUNCTION__, ConfigType);
			}
		}
	} else {
		if ((pBufLen != NULL) && (*pBufLen != 0) && (pBuf != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pBuf, *pBufLen);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		ptmp = pHalData->para_file_buf;
		for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
			if (!IsCommentString(szLine)) {
				/* Get 1st hex value as register offset. */
				if (GetHexValueFromString(szLine, &u4bRegOffset, &u4bMove)) {
					if (u4bRegOffset == 0xffff) {
						/* Ending. */
						break;
					} else if (u4bRegOffset == 0xfe || u4bRegOffset == 0xffe) {
#ifdef CONFIG_LONG_DELAY_ISSUE
						rtw_msleep_os(50);
#else
						rtw_mdelay_os(50);
#endif
					} else if (u4bRegOffset == 0xfd)
						rtw_mdelay_os(5);
					else if (u4bRegOffset == 0xfc)
						rtw_mdelay_os(1);
					else if (u4bRegOffset == 0xfb)
						rtw_udelay_os(50);
					else if (u4bRegOffset == 0xfa)
						rtw_udelay_os(5);
					else if (u4bRegOffset == 0xf9)
						rtw_udelay_os(1);

					/* Get 2nd hex value as register value. */
					szLine += u4bMove;
					if (GetHexValueFromString(szLine, &u4bRegValue, &u4bMove)) {
						/* RTW_INFO("[BB-ADDR]%03lX=%08lX\n", u4bRegOffset, u4bRegValue); */
						phy_set_bb_reg(Adapter, u4bRegOffset, bMaskDWord, u4bRegValue);

						if (u4bRegOffset == 0xa24)
							pHalData->odmpriv.rf_calibrate_info.rega24 = u4bRegValue;

						/* Add 1us delay between BB/RF register setting. */
						rtw_udelay_os(1);
					}
				}
			}
		}
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}

VOID
phy_DecryptBBPgParaFile(
	PADAPTER		Adapter,
	char			*buffer
)
{
	u32	i = 0, j = 0;
	u8	map[95] = {0};
	u8	currentChar;
	char	*BufOfLines, *ptmp;

	/* RTW_INFO("=====>phy_DecryptBBPgParaFile()\n"); */
	/* 32 the ascii code of the first visable char, 126 the last one */
	for (i = 0; i < 95; ++i)
		map[i] = (u8)(94 - i);

	ptmp = buffer;
	i = 0;
	for (BufOfLines = GetLineFromBuffer(ptmp); BufOfLines != NULL; BufOfLines = GetLineFromBuffer(ptmp)) {
		/* RTW_INFO("Encrypted Line: %s\n", BufOfLines); */

		for (j = 0; j < strlen(BufOfLines); ++j) {
			currentChar = BufOfLines[j];

			if (currentChar == '\0')
				break;

			currentChar -= (u8)((((i + j) * 3) % 128));

			BufOfLines[j] = map[currentChar - 32] + 32;
		}
		/* RTW_INFO("Decrypted Line: %s\n", BufOfLines ); */
		if (strlen(BufOfLines) != 0)
			i++;
		BufOfLines[strlen(BufOfLines)] = '\n';
	}
}

#ifndef DBG_TXPWR_BY_RATE_FILE_PARSE
#define DBG_TXPWR_BY_RATE_FILE_PARSE 0
#endif

int
phy_ParseBBPgParaFile(
	PADAPTER		Adapter,
	char			*buffer
)
{
	int	rtStatus = _FAIL;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(Adapter);
	char	*szLine, *ptmp;
	u32	u4bRegOffset, u4bRegMask, u4bRegValue;
	u32	u4bMove;
	BOOLEAN firstLine = _TRUE;
	u8	tx_num = 0;
	u8	band = 0, rf_path = 0;

	if (Adapter->registrypriv.RegDecryptCustomFile == 1)
		phy_DecryptBBPgParaFile(Adapter, buffer);

	ptmp = buffer;
	for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
		if (isAllSpaceOrTab(szLine, sizeof(*szLine)))
			continue;

		if (!IsCommentString(szLine)) {
			/* Get header info (relative value or exact value) */
			if (firstLine) {
				if (eqNByte(szLine, (u8 *)("#[v1]"), 5)
					|| eqNByte(szLine, (u8 *)("#[v2]"), 5))
					pHalData->odmpriv.phy_reg_pg_version = szLine[3] - '0';
				else {
					RTW_ERR("The format in PHY_REG_PG are invalid %s\n", szLine);
					goto exit;
				}

				if (eqNByte(szLine + 5, (u8 *)("[Exact]#"), 8)) {
					pHalData->odmpriv.phy_reg_pg_value_type = PHY_REG_PG_EXACT_VALUE;
					firstLine = _FALSE;
					continue;
				} else {
					RTW_ERR("The values in PHY_REG_PG are invalid %s\n", szLine);
					goto exit;
				}
			}

			if (pHalData->odmpriv.phy_reg_pg_version > 0) {
				u32	index = 0, cnt = 0;

				if (eqNByte(szLine, "0xffff", 6))
					break;

				if (!eqNByte("#[END]#", szLine, 7)) {
					/* load the table label info */
					if (szLine[0] == '#') {
						index = 0;
						if (eqNByte(szLine, "#[2.4G]" , 7)) {
							band = BAND_ON_2_4G;
							index += 8;
						} else if (eqNByte(szLine, "#[5G]", 5)) {
							band = BAND_ON_5G;
							index += 6;
						} else {
							RTW_ERR("Invalid band %s in PHY_REG_PG.txt\n", szLine);
							goto exit;
						}

						rf_path = szLine[index] - 'A';
						if (DBG_TXPWR_BY_RATE_FILE_PARSE)
							RTW_INFO(" Table label Band %d, RfPath %d\n", band, rf_path );
					} else { /* load rows of tables */
						if (szLine[1] == '1')
							tx_num = RF_1TX;
						else if (szLine[1] == '2')
							tx_num = RF_2TX;
						else if (szLine[1] == '3')
							tx_num = RF_3TX;
						else if (szLine[1] == '4')
							tx_num = RF_4TX;
						else {
							RTW_ERR("Invalid row in PHY_REG_PG.txt '%c'(%d)\n", szLine[1], szLine[1]);
							goto exit;
						}

						while (szLine[index] != ']')
							++index;
						++index;/* skip ] */

						/* Get 2nd hex value as register offset. */
						szLine += index;
						if (GetHexValueFromString(szLine, &u4bRegOffset, &u4bMove))
							szLine += u4bMove;
						else
							goto exit;

						/* Get 2nd hex value as register mask. */
						if (GetHexValueFromString(szLine, &u4bRegMask, &u4bMove))
							szLine += u4bMove;
						else
							goto exit;

						if (pHalData->odmpriv.phy_reg_pg_value_type == PHY_REG_PG_EXACT_VALUE) {
							u32	combineValue = 0;
							u8	integer = 0, fraction = 0;

							if (GetFractionValueFromString(szLine, &integer, &fraction, &u4bMove))
								szLine += u4bMove;
							else
								goto exit;

							integer *= hal_spec->txgi_pdbm;
							integer += ((u16)fraction * (u16)hal_spec->txgi_pdbm) / 100;
							if (pHalData->odmpriv.phy_reg_pg_version == 1)
								combineValue |= (((integer / 10) << 4) + (integer % 10));
							else
								combineValue |= integer;

							if (GetFractionValueFromString(szLine, &integer, &fraction, &u4bMove))
								szLine += u4bMove;
							else
								goto exit;

							integer *= hal_spec->txgi_pdbm;
							integer += ((u16)fraction * (u16)hal_spec->txgi_pdbm) / 100;
							combineValue <<= 8;
							if (pHalData->odmpriv.phy_reg_pg_version == 1)
								combineValue |= (((integer / 10) << 4) + (integer % 10));
							else
								combineValue |= integer;

							if (GetFractionValueFromString(szLine, &integer, &fraction, &u4bMove))
								szLine += u4bMove;
							else
								goto exit;

							integer *= hal_spec->txgi_pdbm;
							integer += ((u16)fraction * (u16)hal_spec->txgi_pdbm) / 100;
							combineValue <<= 8;
							if (pHalData->odmpriv.phy_reg_pg_version == 1)
								combineValue |= (((integer / 10) << 4) + (integer % 10));
							else
								combineValue |= integer;

							if (GetFractionValueFromString(szLine, &integer, &fraction, &u4bMove))
								szLine += u4bMove;
							else
								goto exit;

							integer *= hal_spec->txgi_pdbm;
							integer += ((u16)fraction * (u16)hal_spec->txgi_pdbm) / 100;
							combineValue <<= 8;
							if (pHalData->odmpriv.phy_reg_pg_version == 1)
								combineValue |= (((integer / 10) << 4) + (integer % 10));
							else
								combineValue |= integer;

							phy_store_tx_power_by_rate(Adapter, band, rf_path, tx_num, u4bRegOffset, u4bRegMask, combineValue);

							if (DBG_TXPWR_BY_RATE_FILE_PARSE)
								RTW_INFO("addr:0x%3x mask:0x%08x %dTx = 0x%08x\n", u4bRegOffset, u4bRegMask, tx_num + 1, combineValue);
						}
					}
				}
			}
		}
	}

	rtStatus = _SUCCESS;

exit:
	RTW_INFO("%s return %d\n", __func__, rtStatus);
	return rtStatus;
}

int
phy_ConfigBBWithPgParaFile(
	IN	PADAPTER	Adapter,
	IN	const char	*pFileName)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_BB_PG_PARA_FILE))
		return rtStatus;

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if (pHalData->bb_phy_reg_pg == NULL) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pHalData->bb_phy_reg_pg = rtw_zvmalloc(rlen);
				if (pHalData->bb_phy_reg_pg) {
					_rtw_memcpy(pHalData->bb_phy_reg_pg, pHalData->para_file_buf, rlen);
					pHalData->bb_phy_reg_pg_len = rlen;
				} else
					RTW_INFO("%s bb_phy_reg_pg alloc fail !\n", __FUNCTION__);
			}
		}
	} else {
		if ((pHalData->bb_phy_reg_pg_len != 0) && (pHalData->bb_phy_reg_pg != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pHalData->bb_phy_reg_pg, pHalData->bb_phy_reg_pg_len);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		/* RTW_INFO("phy_ConfigBBWithPgParaFile(): read %s ok\n", pFileName); */
		rtStatus = phy_ParseBBPgParaFile(Adapter, pHalData->para_file_buf);
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}

#if (MP_DRIVER == 1)

int
phy_ConfigBBWithMpParaFile(
	IN	PADAPTER	Adapter,
	IN	char		*pFileName
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;
	char	*szLine, *ptmp;
	u32	u4bRegOffset, u4bRegValue, u4bMove;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_BB_MP_PARA_FILE))
		return rtStatus;

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if ((pHalData->bb_phy_reg_mp_len == 0) && (pHalData->bb_phy_reg_mp == NULL)) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pHalData->bb_phy_reg_mp = rtw_zvmalloc(rlen);
				if (pHalData->bb_phy_reg_mp) {
					_rtw_memcpy(pHalData->bb_phy_reg_mp, pHalData->para_file_buf, rlen);
					pHalData->bb_phy_reg_mp_len = rlen;
				} else
					RTW_INFO("%s bb_phy_reg_mp alloc fail !\n", __FUNCTION__);
			}
		}
	} else {
		if ((pHalData->bb_phy_reg_mp_len != 0) && (pHalData->bb_phy_reg_mp != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pHalData->bb_phy_reg_mp, pHalData->bb_phy_reg_mp_len);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		/* RTW_INFO("phy_ConfigBBWithMpParaFile(): read %s ok\n", pFileName); */

		ptmp = pHalData->para_file_buf;
		for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
			if (!IsCommentString(szLine)) {
				/* Get 1st hex value as register offset. */
				if (GetHexValueFromString(szLine, &u4bRegOffset, &u4bMove)) {
					if (u4bRegOffset == 0xffff) {
						/* Ending. */
						break;
					} else if (u4bRegOffset == 0xfe || u4bRegOffset == 0xffe) {
#ifdef CONFIG_LONG_DELAY_ISSUE
						rtw_msleep_os(50);
#else
						rtw_mdelay_os(50);
#endif
					} else if (u4bRegOffset == 0xfd)
						rtw_mdelay_os(5);
					else if (u4bRegOffset == 0xfc)
						rtw_mdelay_os(1);
					else if (u4bRegOffset == 0xfb)
						rtw_udelay_os(50);
					else if (u4bRegOffset == 0xfa)
						rtw_udelay_os(5);
					else if (u4bRegOffset == 0xf9)
						rtw_udelay_os(1);

					/* Get 2nd hex value as register value. */
					szLine += u4bMove;
					if (GetHexValueFromString(szLine, &u4bRegValue, &u4bMove)) {
						/* RTW_INFO("[ADDR]%03lX=%08lX\n", u4bRegOffset, u4bRegValue); */
						phy_set_bb_reg(Adapter, u4bRegOffset, bMaskDWord, u4bRegValue);

						/* Add 1us delay between BB/RF register setting. */
						rtw_udelay_os(1);
					}
				}
			}
		}
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}

#endif

int
PHY_ConfigRFWithParaFile(
	IN	PADAPTER	Adapter,
	IN	char		*pFileName,
	IN	enum rf_path		eRFPath
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;
	char	*szLine, *ptmp;
	u32	u4bRegOffset, u4bRegValue, u4bMove;
	u16	i;
	char	*pBuf = NULL;
	u32	*pBufLen = NULL;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_RF_PARA_FILE))
		return rtStatus;

	switch (eRFPath) {
	case RF_PATH_A:
		pBuf = pHalData->rf_radio_a;
		pBufLen = &pHalData->rf_radio_a_len;
		break;
	case RF_PATH_B:
		pBuf = pHalData->rf_radio_b;
		pBufLen = &pHalData->rf_radio_b_len;
		break;
	default:
		RTW_INFO("Unknown RF path!! %d\r\n", eRFPath);
		break;
	}

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if ((pBufLen != NULL) && (*pBufLen == 0) && (pBuf == NULL)) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pBuf = rtw_zvmalloc(rlen);
				if (pBuf) {
					_rtw_memcpy(pBuf, pHalData->para_file_buf, rlen);
					*pBufLen = rlen;

					switch (eRFPath) {
					case RF_PATH_A:
						pHalData->rf_radio_a = pBuf;
						break;
					case RF_PATH_B:
						pHalData->rf_radio_b = pBuf;
						break;
					default:
						RTW_INFO("Unknown RF path!! %d\r\n", eRFPath);
						break;
					}
				} else
					RTW_INFO("%s(): eRFPath=%d  alloc fail !\n", __FUNCTION__, eRFPath);
			}
		}
	} else {
		if ((pBufLen != NULL) && (*pBufLen != 0) && (pBuf != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pBuf, *pBufLen);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		/* RTW_INFO("%s(): read %s successfully\n", __FUNCTION__, pFileName); */

		ptmp = pHalData->para_file_buf;
		for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
			if (!IsCommentString(szLine)) {
				/* Get 1st hex value as register offset. */
				if (GetHexValueFromString(szLine, &u4bRegOffset, &u4bMove)) {
					if (u4bRegOffset == 0xfe || u4bRegOffset == 0xffe) {
						/* Deay specific ms. Only RF configuration require delay.												 */
#ifdef CONFIG_LONG_DELAY_ISSUE
						rtw_msleep_os(50);
#else
						rtw_mdelay_os(50);
#endif
					} else if (u4bRegOffset == 0xfd) {
						/* delay_ms(5); */
						for (i = 0; i < 100; i++)
							rtw_udelay_os(MAX_STALL_TIME);
					} else if (u4bRegOffset == 0xfc) {
						/* delay_ms(1); */
						for (i = 0; i < 20; i++)
							rtw_udelay_os(MAX_STALL_TIME);
					} else if (u4bRegOffset == 0xfb)
						rtw_udelay_os(50);
					else if (u4bRegOffset == 0xfa)
						rtw_udelay_os(5);
					else if (u4bRegOffset == 0xf9)
						rtw_udelay_os(1);
					else if (u4bRegOffset == 0xffff)
						break;

					/* Get 2nd hex value as register value. */
					szLine += u4bMove;
					if (GetHexValueFromString(szLine, &u4bRegValue, &u4bMove)) {
						phy_set_rf_reg(Adapter, eRFPath, u4bRegOffset, bRFRegOffsetMask, u4bRegValue);

						/* Temp add, for frequency lock, if no delay, that may cause */
						/* frequency shift, ex: 2412MHz => 2417MHz */
						/* If frequency shift, the following action may works. */
						/* Fractional-N table in radio_a.txt */
						/* 0x2a 0x00001		 */ /* channel 1 */
						/* 0x2b 0x00808		frequency divider. */
						/* 0x2b 0x53333 */
						/* 0x2c 0x0000c */
						rtw_udelay_os(1);
					}
				}
			}
		}
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}

VOID
initDeltaSwingIndexTables(
	PADAPTER	Adapter,
	char		*Band,
	char		*Path,
	char		*Sign,
	char		*Channel,
	char		*Rate,
	char		*Data
)
{
#define STR_EQUAL_5G(_band, _path, _sign, _rate, _chnl) \
	((strcmp(Band, _band) == 0) && (strcmp(Path, _path) == 0) && (strcmp(Sign, _sign) == 0) &&\
	 (strcmp(Rate, _rate) == 0) && (strcmp(Channel, _chnl) == 0)\
	)
#define STR_EQUAL_2G(_band, _path, _sign, _rate) \
	((strcmp(Band, _band) == 0) && (strcmp(Path, _path) == 0) && (strcmp(Sign, _sign) == 0) &&\
	 (strcmp(Rate, _rate) == 0)\
	)

#define STORE_SWING_TABLE(_array, _iteratedIdx) \
	do {	\
	for (token = strsep(&Data, delim); token != NULL; token = strsep(&Data, delim)) {\
		sscanf(token, "%d", &idx);\
		_array[_iteratedIdx++] = (u8)idx;\
	} } while (0)\

	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;
	struct dm_rf_calibration_struct	*pRFCalibrateInfo = &(pDM_Odm->rf_calibrate_info);
	u32	j = 0;
	char	*token;
	char	delim[] = ",";
	u32	idx = 0;

	/* RTW_INFO("===>initDeltaSwingIndexTables(): Band: %s;\nPath: %s;\nSign: %s;\nChannel: %s;\nRate: %s;\n, Data: %s;\n",  */
	/*	Band, Path, Sign, Channel, Rate, Data); */

	if (STR_EQUAL_2G("2G", "A", "+", "CCK"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_p, j);
	else if (STR_EQUAL_2G("2G", "A", "-", "CCK"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_n, j);
	else if (STR_EQUAL_2G("2G", "B", "+", "CCK"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_p, j);
	else if (STR_EQUAL_2G("2G", "B", "-", "CCK"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_n, j);
	else if (STR_EQUAL_2G("2G", "A", "+", "ALL"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2ga_p, j);
	else if (STR_EQUAL_2G("2G", "A", "-", "ALL"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2ga_n, j);
	else if (STR_EQUAL_2G("2G", "B", "+", "ALL"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2gb_p, j);
	else if (STR_EQUAL_2G("2G", "B", "-", "ALL"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_2gb_n, j);
	else if (STR_EQUAL_5G("5G", "A", "+", "ALL", "0"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_p[0], j);
	else if (STR_EQUAL_5G("5G", "A", "-", "ALL", "0"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_n[0], j);
	else if (STR_EQUAL_5G("5G", "B", "+", "ALL", "0"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_p[0], j);
	else if (STR_EQUAL_5G("5G", "B", "-", "ALL", "0"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_n[0], j);
	else if (STR_EQUAL_5G("5G", "A", "+", "ALL", "1"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_p[1], j);
	else if (STR_EQUAL_5G("5G", "A", "-", "ALL", "1"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_n[1], j);
	else if (STR_EQUAL_5G("5G", "B", "+", "ALL", "1"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_p[1], j);
	else if (STR_EQUAL_5G("5G", "B", "-", "ALL", "1"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_n[1], j);
	else if (STR_EQUAL_5G("5G", "A", "+", "ALL", "2"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_p[2], j);
	else if (STR_EQUAL_5G("5G", "A", "-", "ALL", "2"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_n[2], j);
	else if (STR_EQUAL_5G("5G", "B", "+", "ALL", "2"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_p[2], j);
	else if (STR_EQUAL_5G("5G", "B", "-", "ALL", "2"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_n[2], j);
	else if (STR_EQUAL_5G("5G", "A", "+", "ALL", "3"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_p[3], j);
	else if (STR_EQUAL_5G("5G", "A", "-", "ALL", "3"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5ga_n[3], j);
	else if (STR_EQUAL_5G("5G", "B", "+", "ALL", "3"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_p[3], j);
	else if (STR_EQUAL_5G("5G", "B", "-", "ALL", "3"))
		STORE_SWING_TABLE(pRFCalibrateInfo->delta_swing_table_idx_5gb_n[3], j);
	else
		RTW_INFO("===>initDeltaSwingIndexTables(): The input is invalid!!\n");
}

int
PHY_ConfigRFWithTxPwrTrackParaFile(
	IN	PADAPTER		Adapter,
	IN	char			*pFileName
)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	struct dm_struct			*pDM_Odm = &pHalData->odmpriv;
	struct dm_rf_calibration_struct		*pRFCalibrateInfo = &(pDM_Odm->rf_calibrate_info);
	int	rlen = 0, rtStatus = _FAIL;
	char	*szLine, *ptmp;
	u32	i = 0, j = 0;
	char	c = 0;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_RF_TXPWR_TRACK_PARA_FILE))
		return rtStatus;

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if ((pHalData->rf_tx_pwr_track_len == 0) && (pHalData->rf_tx_pwr_track == NULL)) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pHalData->rf_tx_pwr_track = rtw_zvmalloc(rlen);
				if (pHalData->rf_tx_pwr_track) {
					_rtw_memcpy(pHalData->rf_tx_pwr_track, pHalData->para_file_buf, rlen);
					pHalData->rf_tx_pwr_track_len = rlen;
				} else
					RTW_INFO("%s rf_tx_pwr_track alloc fail !\n", __FUNCTION__);
			}
		}
	} else {
		if ((pHalData->rf_tx_pwr_track_len != 0) && (pHalData->rf_tx_pwr_track != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pHalData->rf_tx_pwr_track, pHalData->rf_tx_pwr_track_len);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		/* RTW_INFO("%s(): read %s successfully\n", __FUNCTION__, pFileName); */

		ptmp = pHalData->para_file_buf;
		for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
			if (!IsCommentString(szLine)) {
				char	band[5] = "", path[5] = "", sign[5]  = "";
				char	chnl[5] = "", rate[10] = "";
				char	data[300] = ""; /* 100 is too small */

				if (strlen(szLine) < 10 || szLine[0] != '[')
					continue;

				strncpy(band, szLine + 1, 2);
				strncpy(path, szLine + 5, 1);
				strncpy(sign, szLine + 8, 1);

				i = 10; /* szLine+10 */
				if (!ParseQualifiedString(szLine, &i, rate, '[', ']')) {
					/* RTW_INFO("Fail to parse rate!\n"); */
				}
				if (!ParseQualifiedString(szLine, &i, chnl, '[', ']')) {
					/* RTW_INFO("Fail to parse channel group!\n"); */
				}
				while (szLine[i] != '{' && i < strlen(szLine))
					i++;
				if (!ParseQualifiedString(szLine, &i, data, '{', '}')) {
					/* RTW_INFO("Fail to parse data!\n"); */
				}

				initDeltaSwingIndexTables(Adapter, band, path, sign, chnl, rate, data);
			}
		}
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);
#if 0
	for (i = 0; i < DELTA_SWINGIDX_SIZE; ++i) {
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2ga_p[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2ga_p[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2ga_n[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2ga_n[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2gb_p[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2gb_p[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2gb_n[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2gb_n[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_p[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_p[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_n[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2g_cck_a_n[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_p[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_p[i]);
		RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_n[%d] = %d\n", i, pRFCalibrateInfo->delta_swing_table_idx_2g_cck_b_n[i]);

		for (j = 0; j < 3; ++j) {
			RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_5ga_p[%d][%d] = %d\n", j, i, pRFCalibrateInfo->delta_swing_table_idx_5ga_p[j][i]);
			RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_5ga_n[%d][%d] = %d\n", j, i, pRFCalibrateInfo->delta_swing_table_idx_5ga_n[j][i]);
			RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_5gb_p[%d][%d] = %d\n", j, i, pRFCalibrateInfo->delta_swing_table_idx_5gb_p[j][i]);
			RTW_INFO("pRFCalibrateInfo->delta_swing_table_idx_5gb_n[%d][%d] = %d\n", j, i, pRFCalibrateInfo->delta_swing_table_idx_5gb_n[j][i]);
		}
	}
#endif
	return rtStatus;
}

#if CONFIG_TXPWR_LIMIT

#ifndef DBG_TXPWR_LMT_FILE_PARSE
#define DBG_TXPWR_LMT_FILE_PARSE 0
#endif

#define PARSE_RET_NO_HDL	0
#define PARSE_RET_SUCCESS	1
#define PARSE_RET_FAIL		2

/*
* @@Ver=2.0
* or
* @@DomainCode=0x28, Regulation=C6
* or
* @@CountryCode=GB, Regulation=C7
*/
static u8 parse_reg_exc_config(_adapter *adapter, char *szLine)
{
#define VER_PREFIX "Ver="
#define DOMAIN_PREFIX "DomainCode=0x"
#define COUNTRY_PREFIX "CountryCode="
#define REG_PREFIX "Regulation="

	const u8 ver_prefix_len = strlen(VER_PREFIX);
	const u8 domain_prefix_len = strlen(DOMAIN_PREFIX);
	const u8 country_prefix_len = strlen(COUNTRY_PREFIX);
	const u8 reg_prefix_len = strlen(REG_PREFIX);
	u32 i, i_val_s, i_val_e;
	u32 j;
	u8 domain = 0xFF;
	char *country = NULL;
	u8 parse_reg = 0;

	if (szLine[0] != '@' || szLine[1] != '@')
		return PARSE_RET_NO_HDL;

	i = 2;
	if (strncmp(szLine + i, VER_PREFIX, ver_prefix_len) == 0)
		; /* nothing to do */
	else if (strncmp(szLine + i, DOMAIN_PREFIX, domain_prefix_len) == 0) {
		/* get string after domain prefix to ',' */
		i += domain_prefix_len;
		i_val_s = i;
		while (szLine[i] != ',') {
			if (szLine[i] == '\0')
				return PARSE_RET_FAIL;
			i++;
		}
		i_val_e = i;

		/* check if all hex */
		for (j = i_val_s; j < i_val_e; j++)
			if (IsHexDigit(szLine[j]) == _FALSE)
				return PARSE_RET_FAIL;

		/* get value from hex string */
		if (sscanf(szLine + i_val_s, "%hhx", &domain) != 1)
			return PARSE_RET_FAIL;

		parse_reg = 1;
	} else if (strncmp(szLine + i, COUNTRY_PREFIX, country_prefix_len) == 0) {
		/* get string after country prefix to ',' */
		i += country_prefix_len;
		i_val_s = i;
		while (szLine[i] != ',') {
			if (szLine[i] == '\0')
				return PARSE_RET_FAIL;
			i++;
		}
		i_val_e = i;

		if (i_val_e - i_val_s != 2)
			return PARSE_RET_FAIL;

		/* check if all alpha */
		for (j = i_val_s; j < i_val_e; j++)
			if (is_alpha(szLine[j]) == _FALSE)
				return PARSE_RET_FAIL;

		country = szLine + i_val_s;

		parse_reg = 1;

	} else
		return PARSE_RET_FAIL;

	if (parse_reg) {
		/* move to 'R' */
		while (szLine[i] != 'R') {
			if (szLine[i] == '\0')
				return PARSE_RET_FAIL;
			i++;
		}

		/* check if matching regulation prefix */
		if (strncmp(szLine + i, REG_PREFIX, reg_prefix_len) != 0)
			return PARSE_RET_FAIL;

		/* get string after regulation prefix ending with space */
		i += reg_prefix_len;
		i_val_s = i;
		while (szLine[i] != ' ' && szLine[i] != '\t' && szLine[i] != '\0')
			i++;

		if (i == i_val_s)
			return PARSE_RET_FAIL;

		rtw_regd_exc_add_with_nlen(adapter_to_rfctl(adapter), country, domain, szLine + i_val_s, i - i_val_s);
	}

	return PARSE_RET_SUCCESS;
}

static int
phy_ParsePowerLimitTableFile(
	PADAPTER		Adapter,
	char			*buffer
)
{
#define LD_STAGE_EXC_MAPPING	0
#define LD_STAGE_TAB_DEFINE		1
#define LD_STAGE_TAB_START		2
#define LD_STAGE_COLUMN_DEFINE	3
#define LD_STAGE_CH_ROW			4

	int	rtStatus = _FAIL;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(Adapter);
	struct dm_struct	*pDM_Odm = &(pHalData->odmpriv);
	u8	loadingStage = LD_STAGE_EXC_MAPPING;
	u32	i = 0, forCnt = 0;
	char	*szLine, *ptmp;
	char band[10], bandwidth[10], rateSection[10], ntx[10], colNumBuf[10];
	char **regulation = NULL;
	u8	colNum = 0;

	if (Adapter->registrypriv.RegDecryptCustomFile == 1)
		phy_DecryptBBPgParaFile(Adapter, buffer);

	ptmp = buffer;
	for (szLine = GetLineFromBuffer(ptmp); szLine != NULL; szLine = GetLineFromBuffer(ptmp)) {
		if (isAllSpaceOrTab(szLine, sizeof(*szLine)))
			continue;
		if (IsCommentString(szLine))
			continue;

		if (loadingStage == LD_STAGE_EXC_MAPPING) {
			if (szLine[0] == '#' || szLine[1] == '#') {
				loadingStage = LD_STAGE_TAB_DEFINE;
				if (DBG_TXPWR_LMT_FILE_PARSE)
					dump_regd_exc_list(RTW_DBGDUMP, adapter_to_rfctl(Adapter));
			} else {
				if (parse_reg_exc_config(Adapter, szLine) == PARSE_RET_FAIL) {
					RTW_ERR("Fail to parse regulation exception ruls!\n");
					goto exit;
				}
				continue;
			}
		}

		if (loadingStage == LD_STAGE_TAB_DEFINE) {
			/* read "##	2.4G, 20M, 1T, CCK" */
			if (szLine[0] != '#' || szLine[1] != '#')
				continue;

			/* skip the space */
			i = 2;
			while (szLine[i] == ' ' || szLine[i] == '\t')
				++i;

			szLine[--i] = ' '; /* return the space in front of the regulation info */

			/* Parse the label of the table */
			_rtw_memset((PVOID) band, 0, 10);
			_rtw_memset((PVOID) bandwidth, 0, 10);
			_rtw_memset((PVOID) ntx, 0, 10);
			_rtw_memset((PVOID) rateSection, 0, 10);
			if (!ParseQualifiedString(szLine, &i, band, ' ', ',')) {
				RTW_ERR("Fail to parse band!\n");
				goto exit;
			}
			if (!ParseQualifiedString(szLine, &i, bandwidth, ' ', ',')) {
				RTW_ERR("Fail to parse bandwidth!\n");
				goto exit;
			}
			if (!ParseQualifiedString(szLine, &i, ntx, ' ', ',')) {
				RTW_ERR("Fail to parse ntx!\n");
				goto exit;
			}
			if (!ParseQualifiedString(szLine, &i, rateSection, ' ', ',')) {
				RTW_ERR("Fail to parse rate!\n");
				goto exit;
			}

			loadingStage = LD_STAGE_TAB_START;
		} else if (loadingStage == LD_STAGE_TAB_START) {
			/* read "##	START" */
			if (szLine[0] != '#' || szLine[1] != '#')
				continue;

			/* skip the space */
			i = 2;
			while (szLine[i] == ' ' || szLine[i] == '\t')
				++i;

			if (!eqNByte((u8 *)(szLine + i), (u8 *)("START"), 5)) {
				RTW_ERR("Missing \"##   START\" label\n");
				goto exit;
			}

			loadingStage = LD_STAGE_COLUMN_DEFINE;
		} else if (loadingStage == LD_STAGE_COLUMN_DEFINE) {
			/* read "##	#5#	FCC	ETSI	MKK	IC	KCC" */
			if (szLine[0] != '#' || szLine[1] != '#')
				continue;

			/* skip the space */
			i = 2;
			while (szLine[i] == ' ' || szLine[i] == '\t')
				++i;

			_rtw_memset((PVOID) colNumBuf, 0, 10);
			if (!ParseQualifiedString(szLine, &i, colNumBuf, '#', '#')) {
				RTW_ERR("Fail to parse column number!\n");
				goto exit;
			}
			if (!GetU1ByteIntegerFromStringInDecimal(colNumBuf, &colNum)) {
				RTW_ERR("Column number \"%s\" is not unsigned decimal\n", colNumBuf);
				goto exit;
			}
			if (colNum == 0) {
				RTW_ERR("Column number is 0\n");
				goto exit;
			}

			if (DBG_TXPWR_LMT_FILE_PARSE)
				RTW_PRINT("[%s][%s][%s][%s] column num:%d\n", band, bandwidth, rateSection, ntx, colNum);

			regulation = (char **)rtw_zmalloc(sizeof(char *) * colNum);
			if (!regulation) {
				RTW_ERR("Regulation alloc fail\n");
				goto exit;
			}

			for (forCnt = 0; forCnt < colNum; ++forCnt) {
				u32 i_ns;

				/* skip the space */
				while (szLine[i] == ' ' || szLine[i] == '\t')
					i++;
				i_ns = i;

				while (szLine[i] != ' ' && szLine[i] != '\t' && szLine[i] != '\0')
					i++;

				regulation[forCnt] = (char *)rtw_malloc(i - i_ns + 1);
				if (!regulation[forCnt]) {
					RTW_ERR("Regulation alloc fail\n");
					goto exit;
				}

				_rtw_memcpy(regulation[forCnt], szLine + i_ns, i - i_ns);
				regulation[forCnt][i - i_ns] = '\0';
			}

			if (DBG_TXPWR_LMT_FILE_PARSE) {
				RTW_PRINT("column name:");
				for (forCnt = 0; forCnt < colNum; ++forCnt)
					_RTW_PRINT(" %s", regulation[forCnt]);
				_RTW_PRINT("\n");
			}

			loadingStage = LD_STAGE_CH_ROW;
		} else if (loadingStage == LD_STAGE_CH_ROW) {
			char	channel[10] = {0}, powerLimit[10] = {0};
			u8	cnt = 0;

			/* the table ends */
			if (szLine[0] == '#' && szLine[1] == '#') {
				i = 2;
				while (szLine[i] == ' ' || szLine[i] == '\t')
					++i;

				if (eqNByte((u8 *)(szLine + i), (u8 *)("END"), 3)) {
					loadingStage = LD_STAGE_TAB_DEFINE;
					if (regulation) {
						for (forCnt = 0; forCnt < colNum; ++forCnt) {
							if (regulation[forCnt]) {
								rtw_mfree(regulation[forCnt], strlen(regulation[forCnt]) + 1);
								regulation[forCnt] = NULL;
							}
						}
						rtw_mfree((u8 *)regulation, sizeof(char *) * colNum);
						regulation = NULL;
					}
					colNum = 0;
					continue;
				} else {
					RTW_ERR("Missing \"##   END\" label\n");
					goto exit;
				}
			}

			if ((szLine[0] != 'c' && szLine[0] != 'C') ||
				(szLine[1] != 'h' && szLine[1] != 'H')
			) {
				RTW_WARN("Wrong channel prefix: '%c','%c'(%d,%d)\n", szLine[0], szLine[1], szLine[0], szLine[1]);
				continue;
			}
			i = 2;/* move to the  location behind 'h' */

			/* load the channel number */
			cnt = 0;
			while (szLine[i] >= '0' && szLine[i] <= '9') {
				channel[cnt] = szLine[i];
				++cnt;
				++i;
			}
			/* RTW_INFO("chnl %s!\n", channel); */

			for (forCnt = 0; forCnt < colNum; ++forCnt) {
				/* skip the space between channel number and the power limit value */
				while (szLine[i] == ' ' || szLine[i] == '\t')
					++i;

				/* load the power limit value */
				_rtw_memset((PVOID) powerLimit, 0, 10);

				if (szLine[i] == 'W' && szLine[i + 1] == 'W') {
					/*
					* case "WW" assign special ww value
					* means to get minimal limit in other regulations at same channel
					*/
					s8 ww_value = phy_txpwr_ww_lmt_value(Adapter);

					sprintf(powerLimit, "%d", ww_value);
					i += 2;

				} else if (szLine[i] == 'N' && szLine[i + 1] == 'A') {
					/*
					* case "NA" assign max txgi value
					* means no limitation
					*/
					sprintf(powerLimit, "%d", hal_spec->txgi_max);
					i += 2;

				} else if ((szLine[i] >= '0' && szLine[i] <= '9') || szLine[i] == '.'
					|| szLine[i] == '+' || szLine[i] == '-'
				){
					/* case of dBm value */
					u8 integer = 0, fraction = 0, negative = 0;
					u32 u4bMove;
					s8 lmt = 0;

					if (szLine[i] == '+' || szLine[i] == '-') {
						if (szLine[i] == '-')
							negative = 1;
						i++;
					}

					if (GetFractionValueFromString(&szLine[i], &integer, &fraction, &u4bMove))
						i += u4bMove;
					else {
						RTW_ERR("Limit \"%s\" is not valid decimal\n", &szLine[i]);
						goto exit;
					}

					/* transform to string of value in unit of txgi */
					lmt = integer * hal_spec->txgi_pdbm + ((u16)fraction * (u16)hal_spec->txgi_pdbm) / 100;
					if (negative)
						lmt = -lmt;
					sprintf(powerLimit, "%d", lmt);

				} else {
					RTW_ERR("Wrong limit expression \"%c%c\"(%d, %d)\n"
						, szLine[i], szLine[i + 1], szLine[i], szLine[i + 1]);
					goto exit;
				}

				/* store the power limit value */
				phy_set_tx_power_limit(pDM_Odm, (u8 *)regulation[forCnt], (u8 *)band,
					(u8 *)bandwidth, (u8 *)rateSection, (u8 *)ntx, (u8 *)channel, (u8 *)powerLimit);

			}
		}
	}

	rtStatus = _SUCCESS;

exit:
	if (regulation) {
		for (forCnt = 0; forCnt < colNum; ++forCnt) {
			if (regulation[forCnt]) {
				rtw_mfree(regulation[forCnt], strlen(regulation[forCnt]) + 1);
				regulation[forCnt] = NULL;
			}
		}
		rtw_mfree((u8 *)regulation, sizeof(char *) * colNum);
		regulation = NULL;
	}

	RTW_INFO("%s return %d\n", __func__, rtStatus);
	return rtStatus;
}

int
PHY_ConfigRFWithPowerLimitTableParaFile(
	IN	PADAPTER	Adapter,
	IN	const char	*pFileName
)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	int	rlen = 0, rtStatus = _FAIL;

	if (!(Adapter->registrypriv.load_phy_file & LOAD_RF_TXPWR_LMT_PARA_FILE))
		return rtStatus;

	_rtw_memset(pHalData->para_file_buf, 0, MAX_PARA_FILE_BUF_LEN);

	if (pHalData->rf_tx_pwr_lmt == NULL) {
		rtw_get_phy_file_path(Adapter, pFileName);
		if (rtw_is_file_readable(rtw_phy_para_file_path) == _TRUE) {
			rlen = rtw_retrieve_from_file(rtw_phy_para_file_path, pHalData->para_file_buf, MAX_PARA_FILE_BUF_LEN);
			if (rlen > 0) {
				rtStatus = _SUCCESS;
				pHalData->rf_tx_pwr_lmt = rtw_zvmalloc(rlen);
				if (pHalData->rf_tx_pwr_lmt) {
					_rtw_memcpy(pHalData->rf_tx_pwr_lmt, pHalData->para_file_buf, rlen);
					pHalData->rf_tx_pwr_lmt_len = rlen;
				} else
					RTW_INFO("%s rf_tx_pwr_lmt alloc fail !\n", __FUNCTION__);
			}
		}
	} else {
		if ((pHalData->rf_tx_pwr_lmt_len != 0) && (pHalData->rf_tx_pwr_lmt != NULL)) {
			_rtw_memcpy(pHalData->para_file_buf, pHalData->rf_tx_pwr_lmt, pHalData->rf_tx_pwr_lmt_len);
			rtStatus = _SUCCESS;
		} else
			RTW_INFO("%s(): Critical Error !!!\n", __FUNCTION__);
	}

	if (rtStatus == _SUCCESS) {
		/* RTW_INFO("%s(): read %s ok\n", __FUNCTION__, pFileName); */
		rtStatus = phy_ParsePowerLimitTableFile(Adapter, pHalData->para_file_buf);
	} else
		RTW_INFO("%s(): No File %s, Load from HWImg Array!\n", __FUNCTION__, pFileName);

	return rtStatus;
}
#endif /* CONFIG_TXPWR_LIMIT */

void phy_free_filebuf_mask(_adapter *padapter, u8 mask)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	if (pHalData->mac_reg && (mask & LOAD_MAC_PARA_FILE)) {
		rtw_vmfree(pHalData->mac_reg, pHalData->mac_reg_len);
		pHalData->mac_reg = NULL;
	}
	if (mask & LOAD_BB_PARA_FILE) {
		if (pHalData->bb_phy_reg) {
			rtw_vmfree(pHalData->bb_phy_reg, pHalData->bb_phy_reg_len);
			pHalData->bb_phy_reg = NULL;
		}
		if (pHalData->bb_agc_tab) {
			rtw_vmfree(pHalData->bb_agc_tab, pHalData->bb_agc_tab_len);
			pHalData->bb_agc_tab = NULL;
		}
	}
	if (pHalData->bb_phy_reg_pg && (mask & LOAD_BB_PG_PARA_FILE)) {
		rtw_vmfree(pHalData->bb_phy_reg_pg, pHalData->bb_phy_reg_pg_len);
		pHalData->bb_phy_reg_pg = NULL;
	}
	if (pHalData->bb_phy_reg_mp && (mask & LOAD_BB_MP_PARA_FILE)) {
		rtw_vmfree(pHalData->bb_phy_reg_mp, pHalData->bb_phy_reg_mp_len);
		pHalData->bb_phy_reg_mp = NULL;
	}
	if (mask & LOAD_RF_PARA_FILE) {
		if (pHalData->rf_radio_a) {
			rtw_vmfree(pHalData->rf_radio_a, pHalData->rf_radio_a_len);
			pHalData->rf_radio_a = NULL;
		}
		if (pHalData->rf_radio_b) {
			rtw_vmfree(pHalData->rf_radio_b, pHalData->rf_radio_b_len);
			pHalData->rf_radio_b = NULL;
		}
	}
	if (pHalData->rf_tx_pwr_track && (mask & LOAD_RF_TXPWR_TRACK_PARA_FILE)) {
		rtw_vmfree(pHalData->rf_tx_pwr_track, pHalData->rf_tx_pwr_track_len);
		pHalData->rf_tx_pwr_track = NULL;
	}
	if (pHalData->rf_tx_pwr_lmt && (mask & LOAD_RF_TXPWR_LMT_PARA_FILE)) {
		rtw_vmfree(pHalData->rf_tx_pwr_lmt, pHalData->rf_tx_pwr_lmt_len);
		pHalData->rf_tx_pwr_lmt = NULL;
	}
}

inline void phy_free_filebuf(_adapter *padapter)
{
	phy_free_filebuf_mask(padapter, 0xFF);
}

#endif
