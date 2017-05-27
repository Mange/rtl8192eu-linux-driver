/******************************************************************************
 *
 * Copyright(c) 2013 Realtek Corporation. All rights reserved.
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
/*
 * Description:
 *	This file can be applied to following platforms:
 *	CONFIG_PLATFORM_ARM_SUNXI Series platform 
 *	
 */
 
#include <drv_types.h>

#ifdef CONFIG_PLATFORM_ARM_SUNxI
extern int sunxi_usb_disable_hcd(__u32 usbc_no);
extern int sunxi_usb_enable_hcd(__u32 usbc_no);
extern int sunxi_wlan_get_usb_index(void);
extern void sunxi_wlan_set_power(int on);
#endif

#if defined(CONFIG_PLATFORM_ARM_SUN6I) || defined(CONFIG_PLATFORM_ARM_SUN7I)
extern int sw_usb_disable_hcd(__u32 usbc_no);
extern int sw_usb_enable_hcd(__u32 usbc_no);
extern void wifi_pm_power(int on);
static script_item_u item;
#endif

#ifdef CONFIG_PLATFORM_ARM_SUN8I
extern int sunxi_usb_disable_hcd(__u32 usbc_no);
extern int sunxi_usb_enable_hcd(__u32 usbc_no);
extern void wifi_pm_power(int on);
static script_item_u item;
#endif


int platform_wifi_power_on(void)
{
	int ret = 0;
	int usb_no = sunxi_wlan_get_usb_index();

	printk("platform_wifi_power_on(), usb_index: %d\n", usb_no);
#ifdef CONFIG_PLATFORM_ARM_SUNxI
#ifndef CONFIG_RTL8723A
	{
		/* ----------get usb_wifi_usbc_num------------- */
		/*
		ret = script_parser_fetch("usb_wifi_para", "usb_wifi_usbc_num", (int *)&usb_wifi_host, 64);
		if(ret != 0){
			DBG_8192C("ERR: script_parser_fetch usb_wifi_usbc_num failed\n");
			ret = -ENOMEM;
			goto exit;
		}
		DBG_8192C("sw_usb_enable_hcd: usbc_num = %d\n", usb_wifi_host);
		sw_usb_enable_hcd(usb_wifi_host);
		*/
		sunxi_wlan_set_power(1);
		mdelay(100);
		sunxi_usb_enable_hcd(usb_no);
	}
#endif //CONFIG_RTL8723A
#endif //CONFIG_PLATFORM_ARM_SUNxI

#if defined(CONFIG_PLATFORM_ARM_SUN6I) || defined(CONFIG_PLATFORM_ARM_SUN7I)
	{
		script_item_value_type_e type;

		type = script_get_item("wifi_para", "wifi_usbc_id", &item);
		if(SCIRPT_ITEM_VALUE_TYPE_INT != type){
			printk("ERR: script_get_item wifi_usbc_id failed\n");
			ret = -ENOMEM;
			goto exit;
		}

		printk("sw_usb_enable_hcd: usbc_num = %d\n", item.val);
		wifi_pm_power(1);
		mdelay(10);
	
		#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
		sw_usb_enable_hcd(item.val);
		#endif
	}
#endif //defined(CONFIG_PLATFORM_ARM_SUN6I) || defined(CONFIG_PLATFORM_ARM_SUN7I)

#if defined(CONFIG_PLATFORM_ARM_SUN8I)
	{
		script_item_value_type_e type;

		type = script_get_item("wifi_para", "wifi_usbc_id", &item);
		if(SCIRPT_ITEM_VALUE_TYPE_INT != type){
			printk("ERR: script_get_item wifi_usbc_id failed\n");
			ret = -ENOMEM;
			goto exit;
		}

		printk("sw_usb_enable_hcd: usbc_num = %d\n", item.val);
		wifi_pm_power(1);
		mdelay(10);
	
		#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
		sunxi_usb_enable_hcd(item.val);
		#endif
	}
#endif //CONFIG_PLATFORM_ARM_SUN8I

exit:
	return ret;
}

void platform_wifi_power_off(void)
{
	int usb_no = sunxi_wlan_get_usb_index();

	printk("platform_wifi_power_off(), usb_index: %d\n", usb_no);
#ifdef CONFIG_PLATFORM_ARM_SUNxI
#ifndef CONFIG_RTL8723A
	//DBG_8192C("sw_usb_disable_hcd: usbc_num = %d\n", usb_wifi_host);
	//sw_usb_disable_hcd(usb_wifi_host);
	sunxi_usb_disable_hcd(usb_no);
	mdelay(100);
	sunxi_wlan_set_power(0);
#endif //ifndef CONFIG_RTL8723A
#endif	//CONFIG_PLATFORM_ARM_SUNxI

#if defined(CONFIG_PLATFORM_ARM_SUN6I) || defined(CONFIG_PLATFORM_ARM_SUN7I)
	#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
	sw_usb_disable_hcd(item.val);
	#endif
	wifi_pm_power(0);
#endif //defined(CONFIG_PLATFORM_ARM_SUN6I) || defined(CONFIG_PLATFORM_ARM_SUN7I)

#if defined(CONFIG_PLATFORM_ARM_SUN8I)
	#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
	sunxi_usb_disable_hcd(item.val);
	#endif
	wifi_pm_power(0);
#endif //defined(CONFIG_PLATFORM_ARM_SUN8I) 

}

