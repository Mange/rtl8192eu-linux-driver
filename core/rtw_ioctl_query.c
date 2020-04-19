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
#define _RTW_IOCTL_QUERY_C_

#include <drv_types.h>


#ifdef PLATFORM_WINDOWS
/*
 * Added for WPA2-PSK, by Annie, 2005-09-20.
 *   */
u8
query_802_11_capability(
	_adapter		*Adapter,
	u8			*pucBuf,
	u32		*pulOutLen
)
{
	static NDIS_802_11_AUTHENTICATION_ENCRYPTION szAuthEnc[] = {
		{Ndis802_11AuthModeOpen, Ndis802_11EncryptionDisabled},
		{Ndis802_11AuthModeOpen, Ndis802_11Encryption1Enabled},
		{Ndis802_11AuthModeShared, Ndis802_11EncryptionDisabled},
		{Ndis802_11AuthModeShared, Ndis802_11Encryption1Enabled},
		{Ndis802_11AuthModeWPA, Ndis802_11Encryption2Enabled},
		{Ndis802_11AuthModeWPA, Ndis802_11Encryption3Enabled},
		{Ndis802_11AuthModeWPAPSK, Ndis802_11Encryption2Enabled},
		{Ndis802_11AuthModeWPAPSK, Ndis802_11Encryption3Enabled},
		{Ndis802_11AuthModeWPANone, Ndis802_11Encryption2Enabled},
		{Ndis802_11AuthModeWPANone, Ndis802_11Encryption3Enabled},
		{Ndis802_11AuthModeWPA2, Ndis802_11Encryption2Enabled},
		{Ndis802_11AuthModeWPA2, Ndis802_11Encryption3Enabled},
		{Ndis802_11AuthModeWPA2PSK, Ndis802_11Encryption2Enabled},
		{Ndis802_11AuthModeWPA2PSK, Ndis802_11Encryption3Enabled}
	};
	static ULONG	ulNumOfPairSupported = sizeof(szAuthEnc) / sizeof(NDIS_802_11_AUTHENTICATION_ENCRYPTION);
	NDIS_802_11_CAPABILITY *pCap = (NDIS_802_11_CAPABILITY *)pucBuf;
	u8	*pucAuthEncryptionSupported = (u8 *) pCap->AuthenticationEncryptionSupported;


	pCap->Length = sizeof(NDIS_802_11_CAPABILITY);
	if (ulNumOfPairSupported > 1)
		pCap->Length +=	(ulNumOfPairSupported - 1) * sizeof(NDIS_802_11_AUTHENTICATION_ENCRYPTION);

	pCap->Version = 2;
	pCap->NoOfPMKIDs = NUM_PMKID_CACHE;
	pCap->NoOfAuthEncryptPairsSupported = ulNumOfPairSupported;

	if (sizeof(szAuthEnc) <= 240)		/* 240 = 256 - 4*4	 */ { /* SecurityInfo.szCapability: only 256 bytes in size. */
		_rtw_memcpy(pucAuthEncryptionSupported, (u8 *)szAuthEnc,  sizeof(szAuthEnc));
		*pulOutLen = pCap->Length;
		return _TRUE;
	} else {
		*pulOutLen = 0;
		return _FALSE;
	}
}

u8 query_802_11_association_information(_adapter *padapter, PNDIS_802_11_ASSOCIATION_INFORMATION	pAssocInfo)
{
	struct wlan_network *tgt_network;
	struct	mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	struct	security_priv  *psecuritypriv = &(padapter->securitypriv);
	WLAN_BSSID_EX	*psecnetwork = (WLAN_BSSID_EX *)&pmlmepriv->cur_network.network;
	u8	*pDest = (u8 *)pAssocInfo + sizeof(NDIS_802_11_ASSOCIATION_INFORMATION);
	unsigned char i, *auth_ie, *supp_ie;

	/* NdisZeroMemory(pAssocInfo, sizeof(NDIS_802_11_ASSOCIATION_INFORMATION)); */
	_rtw_memset(pAssocInfo, 0, sizeof(NDIS_802_11_ASSOCIATION_INFORMATION));
	/* pAssocInfo->Length = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION); */

	/* ------------------------------------------------------ */
	/* Association Request related information */
	/* ------------------------------------------------------ */
	/* Req_1. AvailableRequestFixedIEs */
	if (psecnetwork != NULL) {

		pAssocInfo->AvailableRequestFixedIEs |= NDIS_802_11_AI_REQFI_CAPABILITIES | NDIS_802_11_AI_REQFI_CURRENTAPADDRESS;
		pAssocInfo->RequestFixedIEs.Capabilities = (unsigned short) *&psecnetwork->IEs[10];
		_rtw_memcpy(pAssocInfo->RequestFixedIEs.CurrentAPAddress,
			    &psecnetwork->MacAddress, 6);

		pAssocInfo->OffsetRequestIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION);

		if (check_fwstate(pmlmepriv, _FW_UNDER_LINKING | _FW_LINKED) == _TRUE) {

			if (psecuritypriv->ndisauthtype >= Ndis802_11AuthModeWPA2)
				pDest[0] = 48;		/* RSN Information Element */
			else
				pDest[0] = 221;	/* WPA(SSN) Information Element */

			supp_ie = &psecuritypriv->supplicant_ie[0];

			i = 13;	/* 0~11 is fixed information element		 */
			while ((i < supp_ie[0]) && (i < 256)) {
				if ((unsigned char)supp_ie[i] == pDest[0]) {
					_rtw_memcpy((u8 *)(pDest),
						    &supp_ie[i],
						    supp_ie[1 + i] + 2);

					break;
				}

				i = i + supp_ie[i + 1] + 2;
				if (supp_ie[1 + i] == 0)
					i = i + 1;

			}


			pAssocInfo->RequestIELength += (2 + supp_ie[1 + i]); /* (2 + psecnetwork->IEs[1+i]+4); */

		}



	}


	/* ------------------------------------------------------ */
	/* Association Response related information */
	/* ------------------------------------------------------ */

	if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE) {
		tgt_network = &(pmlmepriv->cur_network);
		if (tgt_network != NULL) {
			pAssocInfo->AvailableResponseFixedIEs =
				NDIS_802_11_AI_RESFI_CAPABILITIES
				| NDIS_802_11_AI_RESFI_ASSOCIATIONID
				;

			pAssocInfo->ResponseFixedIEs.Capabilities = (unsigned short) *&tgt_network->network.IEs[10];
			pAssocInfo->ResponseFixedIEs.StatusCode = 0;
			pAssocInfo->ResponseFixedIEs.AssociationId = (unsigned short) tgt_network->aid;

			pDest = (u8 *)pAssocInfo + sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAssocInfo->RequestIELength;
			auth_ie = &psecuritypriv->authenticator_ie[0];


			i = auth_ie[0] - 12;
			if (i > 0) {
				_rtw_memcpy((u8 *)&pDest[0], &auth_ie[1], i);
				pAssocInfo->ResponseIELength = i;
			}


			pAssocInfo->OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAssocInfo->RequestIELength;


		}
	}

	return _TRUE;
}
#endif
