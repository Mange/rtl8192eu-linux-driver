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
#ifndef __RTL8703B_DM_H__
#define __RTL8703B_DM_H__
//============================================================
// Description:
//
// This file is for 8703B dynamic mechanism only
//
//
//============================================================

//============================================================
// structure and define
//============================================================

//============================================================
// function prototype
//============================================================

void rtl8703b_init_dm_priv(PADAPTER padapter);
void rtl8703b_deinit_dm_priv(PADAPTER padapter);

void rtl8703b_InitHalDm(PADAPTER padapter);
void rtl8703b_HalDmWatchDog(PADAPTER padapter);
void rtl8703b_HalDmWatchDog_in_LPS(PADAPTER padapter);
void rtl8703b_hal_dm_in_lps(PADAPTER padapter);


#endif

