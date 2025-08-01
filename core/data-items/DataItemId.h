/* Copyright (c) 2015-2017,2021 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation, nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
Changes from Qualcomm Innovation Center are provided under the following license:

Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __DATAITEMID_H__
#define __DATAITEMID_H__

/**
 * Enumeration of Data Item types
 * When add/remove/update changes are made to Data Items, this file needs to be updated
 * accordingly
 */
typedef enum e_DataItemId {
    INVALID_DATA_ITEM_ID = -1,
    // 0 - 4
    AIRPLANEMODE_DATA_ITEM_ID,
    ENH_DATA_ITEM_ID,
    GPSSTATE_DATA_ITEM_ID,
    NLPSTATUS_DATA_ITEM_ID,
    WIFIHARDWARESTATE_DATA_ITEM_ID,
    // 5 - 9
    NETWORKINFO_DATA_ITEM_ID,
    RILVERSION_DATA_ITEM_ID,
    RILSERVICEINFO_DATA_ITEM_ID,
    RILCELLINFO_DATA_ITEM_ID,
    SERVICESTATUS_DATA_ITEM_ID,
    // 10 - 14
    MODEL_DATA_ITEM_ID,
    MANUFACTURER_DATA_ITEM_ID,
    VOICECALL_DATA_ITEM,
    ASSISTED_GPS_DATA_ITEM_ID,
    SCREEN_STATE_DATA_ITEM_ID,
    // 15 - 19
    POWER_CONNECTED_STATE_DATA_ITEM_ID,
    TIMEZONE_CHANGE_DATA_ITEM_ID,
    TIME_CHANGE_DATA_ITEM_ID,
    WIFI_SUPPLICANT_STATUS_DATA_ITEM_ID,
    SHUTDOWN_STATE_DATA_ITEM_ID,
    // 20 - 24
    TAC_DATA_ITEM_ID, // deprecated
    MCCMNC_DATA_ITEM_ID,
    BATTERY_LEVEL_DATA_ITEM_ID,
    IN_EMERGENCY_CALL_DATA_ITEM_ID,
    OEM_GTP_UPLOAD_TRIGGER_READY_ITEM_ID,

    // 25 - 29
    MAX_DATA_ITEM_ID,
    PRECISE_LOCATION_ENABLED_DATA_ITEM_ID,
    TRACKING_STARTED_DATA_ITEM_ID,
    NTRIP_STARTED_DATA_ITEM_ID,
    LOC_FEATURE_STATUS_DATA_ITEM_ID,

    // 30 - 34
    NETWORK_POSITIONING_STARTED_DATA_ITEM_ID,
    QESDK_WWAN_FEATURE_STATUS_DATA_ITEM_ID,
    QESDK_WWAN_CS_CONSENT_SRC_DATA_ITEM_ID,

    MAX_DATA_ITEM_ID_1_1,
} DataItemId;

#endif // #ifndef __DATAITEMID_H__
