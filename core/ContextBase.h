/* Copyright (c) 2011-2017, 2020-2021 The Linux Foundation. All rights reserved.
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

/*
Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#ifndef __LOC_CONTEXT_BASE__
#define __LOC_CONTEXT_BASE__

#include <stdbool.h>
#include <ctype.h>
#include <loc_pla.h>
#include <MsgTask.h>
#include <LocApiBase.h>
#include <LBSProxyBase.h>
#include <loc_cfg.h>
#include <unordered_map>

/* GPS.conf support */
/* NOTE: the implementaiton of the parser casts number
   fields to 32 bit. To ensure all 'n' fields working,
   they must all be 32 bit fields. */
typedef struct loc_gps_cfg_s
{
    uint32_t       INTERMEDIATE_POS;
    uint32_t       ACCURACY_THRES;
    uint32_t       SUPL_VER;
    uint32_t       SUPL_MODE;
    uint32_t       SUPL_ES;
    uint32_t       CAPABILITIES;
    uint32_t       LPP_PROFILE;
    char           XTRA_SERVER_1[LOC_MAX_PARAM_STRING];
    char           XTRA_SERVER_2[LOC_MAX_PARAM_STRING];
    char           XTRA_SERVER_3[LOC_MAX_PARAM_STRING];
    uint32_t       USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL;
    uint32_t       NMEA_PROVIDER;
    char           NMEA_REPORT_RATE[LOC_MAX_PARAM_NAME];
    GnssConfigGpsLock   GPS_LOCK;
    uint32_t       A_GLONASS_POS_PROTOCOL_SELECT;
    uint32_t       AGPS_CERT_WRITABLE_MASK;
    uint32_t       AGPS_CONFIG_INJECT;
    uint32_t       LPPE_CP_TECHNOLOGY;
    uint32_t       LPPE_UP_TECHNOLOGY;
    uint32_t       EXTERNAL_DR_ENABLED;
    char           SUPL_HOST[LOC_MAX_PARAM_STRING];
    uint32_t       SUPL_PORT;
    uint32_t       MODEM_TYPE;
    char           MO_SUPL_HOST[LOC_MAX_PARAM_STRING];
    uint32_t       MO_SUPL_PORT;
    uint32_t       CONSTRAINED_TIME_UNCERTAINTY_ENABLED;
    double         CONSTRAINED_TIME_UNCERTAINTY_THRESHOLD;
    uint32_t       CONSTRAINED_TIME_UNCERTAINTY_ENERGY_BUDGET;
    uint32_t       POSITION_ASSISTED_CLOCK_ESTIMATOR_ENABLED;
    uint32_t       CP_MTLR_ES;
    uint32_t       GNSS_DEPLOYMENT;
    uint32_t       CUSTOM_NMEA_GGA_FIX_QUALITY_ENABLED;
    uint32_t       NI_SUPL_DENY_ON_NFW_LOCKED;
    uint32_t       ENABLE_NMEA_PRINT;
    uint32_t       NMEA_TAG_BLOCK_GROUPING_ENABLED;
} loc_gps_cfg_s_type;

/* NOTE: the implementation of the parser casts number
   fields to 32 bit. To ensure all 'n' fields working,
   they must all be 32 bit fields. */
/* Meanwhile, *_valid fields are 8 bit fields, and 'f'
   fields are double. Rigid as they are, it is the
   the status quo, until the parsing mechanism is
   changed, that is. */
typedef struct
{
    uint8_t        GYRO_BIAS_RANDOM_WALK_VALID;
    double         GYRO_BIAS_RANDOM_WALK;
    uint32_t       SENSOR_ACCEL_BATCHES_PER_SEC;
    uint32_t       SENSOR_ACCEL_SAMPLES_PER_BATCH;
    uint32_t       SENSOR_GYRO_BATCHES_PER_SEC;
    uint32_t       SENSOR_GYRO_SAMPLES_PER_BATCH;
    uint32_t       SENSOR_ACCEL_BATCHES_PER_SEC_HIGH;
    uint32_t       SENSOR_ACCEL_SAMPLES_PER_BATCH_HIGH;
    uint32_t       SENSOR_GYRO_BATCHES_PER_SEC_HIGH;
    uint32_t       SENSOR_GYRO_SAMPLES_PER_BATCH_HIGH;
    uint32_t       SENSOR_CONTROL_MODE;
    uint32_t       SENSOR_ALGORITHM_CONFIG_MASK;
    uint8_t        ACCEL_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         ACCEL_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        ANGLE_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         ANGLE_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        RATE_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         RATE_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY;
} loc_sap_cfg_s_type;

// data struct to hold izat process info
struct izat_process_info {
   bool valueAddedProcessEnabled;
   bool gtpDaemonEnabled;
   bool slimDaemonEnabled;
   bool eDgnssDaemonEnabled;
   bool engineServiceEnabled;
   EngineServiceInfo engineServiceInfo;
};

using namespace loc_util;

namespace loc_core {

class LocAdapterBase;

class ContextBase {
    static LBSProxyBase* getLBSProxy(const char* libName);
    LocApiBase* createLocApi(LOC_API_ADAPTER_EVENT_MASK_T excludedMask);
    static const loc_param_s_type mGps_conf_table[];
    static const loc_param_s_type mSap_conf_table[];
    static uint32_t mAntennaInfoVectorSize;

protected:
    const LBSProxyBase* mLBSProxy;
    const MsgTask* mMsgTask;
    LocApiBase* mLocApi;
    LocApiProxyBase *mLocApiProxy;

public:
    ContextBase(const MsgTask* msgTask,
                LOC_API_ADAPTER_EVENT_MASK_T exMask,
                const char* libName);
    inline virtual ~ContextBase() {
        if (nullptr != mLocApi) {
            mLocApi->destroy();
            mLocApi = nullptr;
        }
        if (nullptr != mLBSProxy) {
            delete mLBSProxy;
            mLBSProxy = nullptr;
        }
    }

    inline const MsgTask* getMsgTask() { return mMsgTask; }
    inline LocApiBase* getLocApi() { return mLocApi; }
    inline LocApiProxyBase* getLocApiProxy() { return mLocApiProxy; }
    inline bool hasAgpsExtendedCapabilities() { return mLBSProxy->hasAgpsExtendedCapabilities(); }
    inline bool hasNativeXtraClient() { return mLBSProxy->hasNativeXtraClient(); }
    inline void modemPowerVote(bool power) const { return mLBSProxy->modemPowerVote(power); }
    inline const LBSProxyBase* getLBSProxyBase() { return mLBSProxy; }
    inline IzatDevId_t getIzatDevId() const {
        return mLBSProxy->getIzatDevId();
    }
    inline void sendMsg(const LocMsg *msg) { getMsgTask()->sendMsg(msg); }
    inline bool checkFeatureStatus(int* fids,
            LocFeatureStatus* status, uint32_t idCount, bool directQwesCall = false) const {
        return mLocApiProxy->checkFeatureStatus(fids, status, idCount, directQwesCall);
    }
    static loc_gps_cfg_s_type mGps_conf;
    static loc_sap_cfg_s_type mSap_conf;
    static izat_process_info   mIzat_process_conf;
    static bool sIsEngineCapabilitiesKnown;
    static uint64_t sSupportedMsgMask;
    static uint8_t sFeaturesSupported[MAX_FEATURE_LENGTH];
    static bool sGnssMeasurementSupported;
    static GnssNMEARptRate sNmeaReportRate;
    static LocationCapabilitiesMask sQwesFeatureMask;
    static LocationHwCapabilitiesMask sHwCapabilitiesMask;

    static void readConfig();
    static void readIZatConfForValueAddedProcess();
    static uint32_t getCarrierCapabilities();
    void setEngineCapabilities(uint64_t supportedMsgMask,
            uint8_t *featureList, bool gnssMeasurementSupported);

    static inline bool isEngineCapabilitiesKnown() {
        return sIsEngineCapabilitiesKnown;
    }

    static inline bool isMessageSupported(LocCheckingMessagesID msgID) {

        // confirm if msgID is not larger than the number of bits in
        // mSupportedMsg
        if ((uint64_t)msgID > (sizeof(sSupportedMsgMask) << 3)) {
            return false;
        } else {
            uint32_t messageChecker = 1 << msgID;
            return (messageChecker & sSupportedMsgMask) == messageChecker;
        }
    }

    /*
        Check if a feature is supported
    */
    static bool isFeatureSupported(uint8_t featureVal);

    /*
        Check if gnss measurement is supported
    */
    static bool gnssConstellationConfig();

    /*
        set QWES feature status info
    */
    static inline void setQwesFeatureStatus(
            const std::unordered_map<LocationQwesFeatureType, bool> &featureMap) {
       std::unordered_map<LocationQwesFeatureType, bool>::const_iterator itr;
       static LocationQwesFeatureType locQwesFeatType[LOCATION_QWES_FEATURE_TYPE_MAX];
       for (itr = featureMap.begin(); itr != featureMap.end(); ++itr) {
           LOC_LOGi("Feature : %d isValid: %d", itr->first, itr->second);
           locQwesFeatType[itr->first] = itr->second;
           switch (itr->first) {
               case LOCATION_QWES_FEATURE_TYPE_CARRIER_PHASE:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_CARRIER_PHASE_BIT;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_CARRIER_PHASE_BIT;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_SV_POLYNOMIAL:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_SV_POLYNOMIAL_BIT;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_SV_POLYNOMIAL_BIT;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_GNSS_SINGLE_FREQUENCY:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_GNSS_SINGLE_FREQUENCY;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_GNSS_SINGLE_FREQUENCY;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_SV_EPH:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_SV_EPHEMERIS_BIT;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_SV_EPHEMERIS_BIT;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_GNSS_MULTI_FREQUENCY:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_GNSS_MULTI_FREQUENCY;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_GNSS_MULTI_FREQUENCY;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_PPE:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_PPE;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_PPE;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_QDR2:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_QDR2;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_QDR2;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_QDR3:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_QDR3;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_QDR3;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_VPE:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_VPE;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_VPE;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_DGNSS:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_DGNSS;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_DGNSS;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_RSSI_POSITIONING:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_WIFI_RSSI_POSITIONING;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_WIFI_RSSI_POSITIONING;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_RTT_POSITIONING:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_WIFI_RTT_POSITIONING;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_WIFI_RTT_POSITIONING;
                   }
               break;
               case LOCATION_QWES_FEATURE_NLOS_ML20:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_NLOS_ML20;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_NLOS_ML20;
                   }
                   break;
               case LOCATION_QWES_FEATURE_TYPE_WWAN_STANDARD_POSITIONING:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_WWAN_STANDARD_POSITIONING;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_WWAN_STANDARD_POSITIONING;
                   }
               break;
               case LOCATION_QWES_FEATURE_TYPE_WWAN_PREMIUM_POSITIONING:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_WWAN_PREMIUM_POSITIONING;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_WWAN_PREMIUM_POSITIONING;
                   }
               break;
               case LOCATION_QWES_FEATURE_STATUS_GNSS_NHZ:
                   if (itr->second) {
                       sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_GNSS_NHZ;
                   } else {
                       sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_GNSS_NHZ;
                   }
               break;
           }
       }

       // Set CV2X basic when time freq and tunc is set
       // CV2X_BASIC  = LOCATION_QWES_FEATURE_TYPE_TIME_FREQUENCY &
       //       LOCATION_QWES_FEATURE_TYPE_TIME_UNCERTAINTY

       // Set CV2X premium when time freq and tunc is set
       // CV2X_PREMIUM = CV2X_BASIC & LOCATION_QWES_FEATURE_TYPE_QDR3 &
       //       LOCATION_QWES_FEATURE_TYPE_CLOCK_ESTIMATE

       bool cv2xBasicEnabled = (1 == locQwesFeatType[LOCATION_QWES_FEATURE_TYPE_TIME_FREQUENCY]) &&
            (1 == locQwesFeatType[LOCATION_QWES_FEATURE_TYPE_TIME_UNCERTAINTY]);
       bool cv2xPremiumEnabled = cv2xBasicEnabled &&
            (1 == locQwesFeatType[LOCATION_QWES_FEATURE_TYPE_QDR3]) &&
            (1 == locQwesFeatType[LOCATION_QWES_FEATURE_TYPE_CLOCK_ESTIMATE]);

       LOC_LOGd("CV2X_BASIC:%d, CV2X_PREMIUM:%d", cv2xBasicEnabled, cv2xPremiumEnabled);
       if (cv2xBasicEnabled) {
            sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_CV2X_LOCATION_BASIC;
       } else {
            sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_CV2X_LOCATION_BASIC;
       }
       if (cv2xPremiumEnabled) {
            sQwesFeatureMask |= LOCATION_CAPABILITIES_QWES_CV2X_LOCATION_PREMIUM;
       } else {
            sQwesFeatureMask &= ~LOCATION_CAPABILITIES_QWES_CV2X_LOCATION_PREMIUM;
       }
    }

    /*
        get QWES feature status info
    */
    static inline LocationCapabilitiesMask getQwesFeatureStatus() {
        return (ContextBase::sQwesFeatureMask);
    }

    /*
        set HW feature status info
    */
    static inline void setHwCapabilities(const LocationHwCapabilitiesMask& mask) {
        sHwCapabilitiesMask |= mask;
    }

    /*
        get HW feature status info
    */
    static inline LocationHwCapabilitiesMask getHwCapabilitiesMask() {
        return (ContextBase::sHwCapabilitiesMask);
    }

    static inline bool isAntennaInfoAvailable() {
        return mAntennaInfoVectorSize != 0;
    }
};

struct LocApiResponse: LocMsg {
    private:
        ContextBase& mContext;
        std::function<void (LocationError err)> mProcImpl;
        inline virtual void proc() const {
            mProcImpl(mLocationError);
        }
    protected:
        LocationError mLocationError;
    public:
        inline LocApiResponse(ContextBase& context,
                              std::function<void (LocationError err)> procImpl ) :
                              mContext(context), mProcImpl(procImpl) {}

        void returnToSender(const LocationError err) {
            mLocationError = err;
            mContext.sendMsg(this);
        }
};

struct LocApiCollectiveResponse: LocMsg {
    private:
        ContextBase& mContext;
        std::function<void (std::vector<LocationError> errs)> mProcImpl;
        inline virtual void proc() const {
            mProcImpl(mLocationErrors);
        }
    protected:
        std::vector<LocationError> mLocationErrors;
    public:
        inline LocApiCollectiveResponse(ContextBase& context,
                              std::function<void (std::vector<LocationError> errs)> procImpl ) :
                              mContext(context), mProcImpl(procImpl) {}
        inline virtual ~LocApiCollectiveResponse() {
        }

        void returnToSender(std::vector<LocationError>& errs) {
            mLocationErrors = errs;
            mContext.sendMsg(this);
        }
};


template <typename DATA>
struct LocApiResponseData: LocMsg {
    private:
        ContextBase& mContext;
        std::function<void (LocationError err, DATA data)> mProcImpl;
        inline virtual void proc() const {
            mProcImpl(mLocationError, mData);
        }
    protected:
        LocationError mLocationError;
        DATA mData;
    public:
        inline LocApiResponseData(ContextBase& context,
                              std::function<void (LocationError err, DATA data)> procImpl ) :
                              mContext(context), mProcImpl(procImpl) {}

        void returnToSender(const LocationError err, const DATA data) {
            mLocationError = err;
            mData = data;
            mContext.sendMsg(this);
        }
};


} // namespace loc_core

#endif //__LOC_CONTEXT_BASE__
