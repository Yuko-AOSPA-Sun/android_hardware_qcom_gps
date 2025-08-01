/* Copyright (c) 2011-2014, 2016-2021 The Linux Foundation. All rights reserved.
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

#define LOG_NDEBUG 0 //Define to enable LOGV
#define LOG_TAG "LocSvc_LocApiBase"

#include <dlfcn.h>
#include <inttypes.h>
#include <gps_extended_c.h>
#include <LocApiBase.h>
#include <LocAdapterBase.h>
#include <log_util.h>
#include <LocContext.h>
#include <loc_misc_utils.h>

#ifdef PTP_SUPPORTED
#include <gptp_helper.h>
#endif

namespace loc_core {

#define MSEC_IN_ONE_WEEK 604800000LL

#define TO_ALL_LOCADAPTERS(call) TO_ALL_ADAPTERS(mLocAdapters, (call))
#define TO_1ST_HANDLING_LOCADAPTERS(call) TO_1ST_HANDLING_ADAPTER(mLocAdapters, (call))

int hexcode(char *hexstring, int string_size,
            const char *data, int data_size)
{
   int i;
   for (i = 0; i < data_size; i++)
   {
      char ch = data[i];
      if (i*2 + 3 <= string_size)
      {
         snprintf(&hexstring[i*2], 3, "%02X", ch);
      }
      else {
         break;
      }
   }
   return i;
}

int decodeAddress(char *addr_string, int string_size,
                   const char *data, int data_size)
{
    const char addr_prefix = 0x91;
    int i, idxOutput = 0;

    if (!data || !addr_string) { return 0; }

    if (data[0] != addr_prefix)
    {
        LOC_LOGW("decodeAddress: address prefix is not 0x%x but 0x%x", addr_prefix, data[0]);
        addr_string[0] = '\0';
        return 0; // prefix not correct
    }

    for (i = 1; i < data_size; i++)
    {
        unsigned char ch = data[i], low = ch & 0x0F, hi = ch >> 4;
        if (low <= 9 && idxOutput < string_size - 1) { addr_string[idxOutput++] = low + '0'; }
        if (hi <= 9 && idxOutput < string_size - 1) { addr_string[idxOutput++] = hi + '0'; }
    }

    addr_string[idxOutput] = '\0'; // Terminates the string

    return idxOutput;
}

struct LocSsrMsg : public LocMsg {
    LocApiBase* mLocApi;
    inline LocSsrMsg(LocApiBase* locApi) :
        LocMsg(), mLocApi(locApi)
    {
        locallog();
    }
    inline virtual void proc() const {
        mLocApi->close();
        if (LOC_API_ADAPTER_ERR_SUCCESS == mLocApi->open(mLocApi->getEvtMask())) {
            // Notify adapters that engine up after SSR
            mLocApi->handleEngineUpEvent();
        }
    }
    inline void locallog() const {
        LOC_LOGV("LocSsrMsg");
    }
    inline virtual void log() const {
        locallog();
    }
};

struct LocOpenMsg : public LocMsg {
    LocApiBase* mLocApi;
    LocAdapterBase* mAdapter;
    inline LocOpenMsg(LocApiBase* locApi, LocAdapterBase* adapter = nullptr) :
            LocMsg(), mLocApi(locApi), mAdapter(adapter)
    {
        locallog();
    }
    inline virtual void proc() const {
        if (LOC_API_ADAPTER_ERR_SUCCESS == mLocApi->open(mLocApi->getEvtMask()) &&
            nullptr != mAdapter) {
            mAdapter->handleEngineUpEvent();
        }
    }
    inline void locallog() const {
        LOC_LOGv("LocOpen Mask: %" PRIx64 "\n", mLocApi->getEvtMask());
    }
    inline virtual void log() const {
        locallog();
    }
};

struct LocCloseMsg : public LocMsg {
    LocApiBase* mLocApi;
    inline LocCloseMsg(LocApiBase* locApi) :
        LocMsg(), mLocApi(locApi)
    {
        locallog();
    }
    inline virtual void proc() const {
        mLocApi->close();
    }
    inline void locallog() const {
    }
    inline virtual void log() const {
        locallog();
    }
};

MsgTask* LocApiBase::mMsgTask = nullptr;
volatile int32_t LocApiBase::mMsgTaskRefCount = 0;

LocApiBase::LocApiBase(LOC_API_ADAPTER_EVENT_MASK_T excludedMask,
                       ContextBase* context) :
    mContext(context),
    mMask(0), mExcludedMask(excludedMask), mEngineLockState(ENGINE_LOCK_STATE_DISABLED) {
    memset(mLocAdapters, 0, sizeof(mLocAdapters));

    android_atomic_inc(&mMsgTaskRefCount);
    if (nullptr == mMsgTask) {
        mMsgTask = new MsgTask("LocApiMsgTask");
    }
}

LOC_API_ADAPTER_EVENT_MASK_T LocApiBase::getEvtMask()
{
    LOC_API_ADAPTER_EVENT_MASK_T mask = 0;

    TO_ALL_LOCADAPTERS(mask |= mLocAdapters[i]->getEvtMask());

    return mask & ~mExcludedMask;
}

bool LocApiBase::isMaster()
{
    bool isMaster = false;

    for (int i = 0;
            !isMaster && i < MAX_ADAPTERS && NULL != mLocAdapters[i];
            i++) {
        isMaster |= mLocAdapters[i]->isAdapterMaster();
    }
    return isMaster;
}

bool LocApiBase::isInSession()
{
    bool inSession = false;

    for (int i = 0;
         !inSession && i < MAX_ADAPTERS && NULL != mLocAdapters[i];
         i++) {
        inSession = mLocAdapters[i]->isInSession();
    }

    return inSession;
}

bool LocApiBase::needReport(const UlpLocation& ulpLocation,
                            enum loc_sess_status status,
                            LocPosTechMask techMask)
{
    bool reported = false;

    if (LOC_SESS_INTERMEDIATE == ContextBase::mGps_conf.INTERMEDIATE_POS) {
        // if intermediate fix is allowed, we will report out intermediate or final fixes
        // when one of below two conditions are met:
        // 1: if accuracy level is do not care, report out all intermediate or final fixes
        // 2: otherwise, the accuracy level will need to be valid and less than threshold
        if (LOC_SESS_FAILURE != status) {
            if ((ContextBase::mGps_conf.ACCURACY_THRES != 0) &&
                    (((ulpLocation.gpsLocation.flags & LOC_GPS_LOCATION_HAS_ACCURACY) == 0) ||
                     (ulpLocation.gpsLocation.accuracy >= ContextBase::mGps_conf.ACCURACY_THRES))) {
                reported = false;
            } else {
                reported = true;
            }
        }
    } else {
        // intermediate fix is not allowed, only can report out final fixes
        if (LOC_SESS_SUCCESS == status) {
            // this is a final fix with satellite and/or sensor contribution
            LocPosTechMask mask =
                LOC_POS_TECH_MASK_SATELLITE | LOC_POS_TECH_MASK_SENSORS;
#ifndef __ANDROID__
            // Include propagated GPS fix if not on Android target
            mask |=  LOC_POS_TECH_MASK_PROPAGATED;
#endif
            reported = (mask & techMask);
        }
    }

    return reported;
}

void LocApiBase::addAdapter(LocAdapterBase* adapter)
{
    for (int i = 0; i < MAX_ADAPTERS && mLocAdapters[i] != adapter; i++) {
        if (mLocAdapters[i] == NULL) {
            mLocAdapters[i] = adapter;
            sendMsg(new LocOpenMsg(this,  adapter));
            break;
        }
    }
}

void LocApiBase::removeAdapter(LocAdapterBase* adapter)
{
    for (int i = 0;
         i < MAX_ADAPTERS && NULL != mLocAdapters[i];
         i++) {
        if (mLocAdapters[i] == adapter) {
            mLocAdapters[i] = NULL;

            // shift the rest of the adapters up so that the pointers
            // in the array do not have holes.  This should be more
            // performant, because the array maintenance is much much
            // less frequent than event handlings, which need to linear
            // search all the adapters
            int j = i;
            while (++i < MAX_ADAPTERS && mLocAdapters[i] != NULL);

            // i would be MAX_ADAPTERS or point to a NULL
            i--;
            // i now should point to a none NULL adapter within valid
            // range although i could be equal to j, but it won't hurt.
            // No need to check it, as it gains nothing.
            mLocAdapters[j] = mLocAdapters[i];
            // this makes sure that we exit the for loop
            mLocAdapters[i] = NULL;

            // if we have an empty list of adapters
            if (0 == i) {
                sendMsg(new LocCloseMsg(this));
            } else {
                // else we need to remove the bit
                sendMsg(new LocOpenMsg(this));
            }
        }
    }
}

void LocApiBase::updateEvtMask()
{
    sendMsg(new LocOpenMsg(this));
}

void LocApiBase::updateNmeaMask(uint32_t mask)
{
    struct LocSetNmeaMsg : public LocMsg {
        LocApiBase* mLocApi;
        uint32_t mMask;
        inline LocSetNmeaMsg(LocApiBase* locApi, uint32_t mask) :
            LocMsg(), mLocApi(locApi), mMask(mask)
        {
            locallog();
        }
        inline virtual void proc() const {
            mLocApi->setNMEATypesSync(mMask);
        }
        inline void locallog() const {
            LOC_LOGv("LocSyncNmea NmeaMask: %" PRIx32 "\n", mMask);
        }
        inline virtual void log() const {
            locallog();
        }
    };

    sendMsg(new LocSetNmeaMsg(this, mask));
}

void LocApiBase::handleEngineUpEvent()
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->handleEngineUpEvent());
}

void LocApiBase::handleEngineDownEvent()
{    // This will take care of renegotiating the loc handle
    sendMsg(new LocSsrMsg(this));

    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->handleEngineDownEvent());
}

void LocApiBase::reportPosition(UlpLocation& location,
                                GpsLocationExtended& locationExtended,
                                enum loc_sess_status status,
                                LocPosTechMask loc_technology_mask,
                                GnssDataNotification* pDataNotify,
                                int msInWeek)
{
    // print the location info before delivering
    LOC_LOGd("\n  flags: 0x%x\n  source: %d\n  latitude: %f\n  longitude: %f\n  "
           "altitude: %f\n  speed: %f\n  bearing: %f\n  accuracy: %f\n  "
           "timestamp: %" PRId64 "\n  "
           "session status: %d\n  technology mask: 0x%x\n  time bias unc %f msec\n  "
           "SV used in fix (gps/glo/bds/gal/qzss/navic) : \n"
           "(0x%" PRIx64 "/0x%" PRIx64 "/0x%" PRIx64 "/0x%" PRIx64 "/0x%" PRIx64 "/0x%" PRIx64 ")",
             location.gpsLocation.flags, location.position_source,
             location.gpsLocation.latitude, location.gpsLocation.longitude,
             location.gpsLocation.altitude, location.gpsLocation.speed,
             location.gpsLocation.bearing, location.gpsLocation.accuracy,
             location.gpsLocation.timestamp, status, loc_technology_mask,
             locationExtended.gnssSystemTime.u.gpsSystemTime.systemClkTimeUncMs,
             locationExtended.gnss_sv_used_ids.gps_sv_used_ids_mask,
             locationExtended.gnss_sv_used_ids.glo_sv_used_ids_mask,
             locationExtended.gnss_sv_used_ids.bds_sv_used_ids_mask,
             locationExtended.gnss_sv_used_ids.gal_sv_used_ids_mask,
             locationExtended.gnss_sv_used_ids.qzss_sv_used_ids_mask,
             locationExtended.gnss_sv_used_ids.navic_sv_used_ids_mask);
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(
        mLocAdapters[i]->reportPositionEvent(location, locationExtended,
                                             status, loc_technology_mask,
                                             pDataNotify, msInWeek)
    );
}

void LocApiBase::reportWwanZppFix(LocGpsLocation &zppLoc)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->reportWwanZppFix(zppLoc));
}

void LocApiBase::reportZppBestAvailableFix(LocGpsLocation &zppLoc,
        GpsLocationExtended &location_extended, LocPosTechMask tech_mask)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportZppBestAvailableFix(zppLoc,
            location_extended, tech_mask));
}

void LocApiBase::requestOdcpi(OdcpiRequestInfo& request)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->requestOdcpiEvent(request));
}

void LocApiBase::reportGnssEngEnergyConsumedEvent(uint64_t energyConsumedSinceFirstBoot)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportGnssEngEnergyConsumedEvent(
            energyConsumedSinceFirstBoot));
}

void LocApiBase::reportDeleteAidingDataEvent(GnssAidingData& aidingData) {
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->reportDeleteAidingDataEvent(aidingData));
}

void LocApiBase::reportKlobucharIonoModel(GnssKlobucharIonoModel & ionoModel) {
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->reportKlobucharIonoModelEvent(ionoModel));
}

void LocApiBase::reportGnssAdditionalSystemInfo(GnssAdditionalSystemInfo& additionalSystemInfo) {
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->reportGnssAdditionalSystemInfoEvent(
            additionalSystemInfo));
}

void LocApiBase::sendNfwNotification(GnssNfwNotification& notification)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportNfwNotificationEvent(notification));

}

void LocApiBase::reportSv(GnssSvNotification& svNotify)
{
    const char* constellationString[] = { "Unknown", "GPS", "SBAS", "GLONASS",
        "QZSS", "BEIDOU", "GALILEO", "NAVIC" };

    // print the SV info before delivering
    LOC_LOGv("num sv: %u\n"
        "      sv: constellation svid  cN0  bbCN0"
        "  elevation  azimuth  carrierFreq  gloFreq flags signalType",
        svNotify.count);
    for (size_t i = 0; i < svNotify.count && i < GNSS_SV_MAX; i++) {
        if (svNotify.gnssSvs[i].type >
            sizeof(constellationString) / sizeof(constellationString[0]) - 1) {
            svNotify.gnssSvs[i].type = GNSS_SV_TYPE_UNKNOWN;
        }
        // Display what we report to clients
        LOC_LOGV(" %03zu: %*s %02d  %2.2f  %2.2f  %3.2f  %3.2f %10.2f %u 0x%02X 0x%2X",
            i,
            13,
            constellationString[svNotify.gnssSvs[i].type],
            svNotify.gnssSvs[i].svId,
            svNotify.gnssSvs[i].cN0Dbhz,
            svNotify.gnssSvs[i].basebandCarrierToNoiseDbHz,
            svNotify.gnssSvs[i].elevation,
            svNotify.gnssSvs[i].azimuth,
            svNotify.gnssSvs[i].carrierFrequencyHz,
            svNotify.gnssSvs[i].gloFrequency,
            svNotify.gnssSvs[i].gnssSvOptionsMask,
            svNotify.gnssSvs[i].gnssSignalTypeMask);
    }
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(
        mLocAdapters[i]->reportSvEvent(svNotify)
        );
}

void LocApiBase::reportSvPolynomial(GnssSvPolynomial &svPolynomial)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(
        mLocAdapters[i]->reportSvPolynomialEvent(svPolynomial)
    );
}

void LocApiBase::reportSvEphemeris(GnssSvEphemerisReport & svEphemeris)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(
        mLocAdapters[i]->reportSvEphemerisEvent(svEphemeris)
    );
}

void LocApiBase::reportStatus(LocGpsStatusValue status)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportStatus(status));
}

void LocApiBase::reportData(GnssDataNotification& dataNotify, int msInWeek)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportDataEvent(dataNotify, msInWeek));
}

void LocApiBase::reportNmea(const char* nmea, int length)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportNmeaEvent(nmea, length));
}

void LocApiBase::reportXtraServer(const char* url1, const char* url2,
                                  const char* url3, const int maxlength)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->reportXtraServer(url1, url2, url3, maxlength));

}

void LocApiBase::reportLocationSystemInfo(const LocationSystemInfo& locationSystemInfo)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportLocationSystemInfoEvent(locationSystemInfo));
}

void LocApiBase::reportDcMessage(const GnssDcReportInfo& dcReport) {
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportDcMessage(dcReport));
}

void LocApiBase::reportSignalTypeCapabilities(const GnssCapabNotification& gnssCapabNotification) {
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportSignalTypeCapabilities(gnssCapabNotification));
}

void LocApiBase::reportModemGnssQesdkFeatureStatus(const ModemGnssQesdkFeatureMask& mask) {
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportModemGnssQesdkFeatureStatus(mask));
}

void LocApiBase::reportNtnStatusEvent(LocationError status,
        const GnssSignalTypeMask& gpsSignalTypeConfigMask, bool isSetResponse) {
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportNtnStatusEvent(
                status, gpsSignalTypeConfigMask, isSetResponse));
}

void LocApiBase::reportNtnConfigUpdateEvent(const GnssSignalTypeMask& gpsSignalTypeConfigMask) {
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportNtnConfigUpdateEvent(gpsSignalTypeConfigMask));
}

void LocApiBase::reportQwesCapabilities
(
    const std::unordered_map<LocationQwesFeatureType, bool> &featureMap
)
{
    //Set Qwes feature status map in ContextBase
    ContextBase::setQwesFeatureStatus(featureMap);
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportQwesCapabilities(featureMap));
}
void LocApiBase::requestXtraData()
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->requestXtraData());
}

void LocApiBase::requestTime()
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->requestTime());
}

void LocApiBase::requestLocation()
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->requestLocation());
}

void LocApiBase::requestATL(int connHandle, LocAGpsType agps_type,
                            LocApnTypeMask apn_type_mask, SubId sub_id)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(
            mLocAdapters[i]->requestATL(connHandle, agps_type, apn_type_mask, sub_id));
}

void LocApiBase::releaseATL(int connHandle)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(mLocAdapters[i]->releaseATL(connHandle));
}

void LocApiBase::requestNiNotify(GnssNiNotification &notify, const void* data,
                                 const LocInEmergency emergencyState)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_1ST_HANDLING_LOCADAPTERS(
            mLocAdapters[i]->requestNiNotifyEvent(notify,
                                                  data,
                                                  emergencyState));
}

void* LocApiBase :: getSibling()
    DEFAULT_IMPL(NULL)

LocApiProxyBase* LocApiBase :: getLocApiProxy()
    DEFAULT_IMPL(NULL)

void LocApiBase::reportGnssMeasurements(GnssMeasurements& gnssMeasurements, int msInWeek)
{
    // loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportGnssMeasurementsEvent(gnssMeasurements, msInWeek));
}

void LocApiBase::reportGnssSvIdConfig(const GnssSvIdConfig& config)
{
    // Print the config
    LOC_LOGv("gloBlacklistSvMask: %" PRIu64 ", bdsBlacklistSvMask: %" PRIu64 ",\n"
             "qzssBlacklistSvMask: %" PRIu64 ", galBlacklistSvMask: %" PRIu64 ",\n"
              "navicBlacklistSvMask: %" PRIu64,
             config.gloBlacklistSvMask, config.bdsBlacklistSvMask,
             config.qzssBlacklistSvMask, config.galBlacklistSvMask, config.navicBlacklistSvMask);

    // Loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportGnssSvIdConfigEvent(config));
}

void LocApiBase::reportGnssSvTypeConfig(const GnssSvTypeConfig& config)
{
    // Print the config
    LOC_LOGv("blacklistedMask: %" PRIu64 ", enabledMask: %" PRIu64,
             config.blacklistedSvTypesMask, config.enabledSvTypesMask);

    // Loop through adapters, and deliver to all adapters.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportGnssSvTypeConfigEvent(config));
}

void LocApiBase::geofenceBreach(size_t count, uint32_t* hwIds, Location& location,
                                GeofenceBreachType breachType, uint64_t timestamp)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->geofenceBreachEvent(count, hwIds, location, breachType,
                                                            timestamp));
}

void LocApiBase::geofenceStatus(GeofenceStatusAvailable available)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->geofenceStatusEvent(available));
}

void LocApiBase::reportDBTPosition(UlpLocation &location, GpsLocationExtended &locationExtended,
                                   enum loc_sess_status status, LocPosTechMask loc_technology_mask)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportPositionEvent(location, locationExtended, status,
                                                            loc_technology_mask));
}

void LocApiBase::reportLocations(Location* locations, size_t count, BatchingMode batchingMode)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportLocationsEvent(locations, count, batchingMode));
}

void LocApiBase::reportCompletedTrips(uint32_t accumulated_distance)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportCompletedTripsEvent(accumulated_distance));
}

void LocApiBase::handleBatchStatusEvent(BatchingStatus batchStatus)
{
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportBatchStatusChangeEvent(batchStatus));
}

void LocApiBase::reportGnssConfig(uint32_t sessionId, const GnssConfig& gnssConfig)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportGnssConfigEvent(sessionId, gnssConfig));
}

void LocApiBase::reportLatencyInfo(GnssLatencyInfo& gnssLatencyInfo)
{
    // loop through adapters, and deliver to the first handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportLatencyInfoEvent(gnssLatencyInfo));
}

void LocApiBase::reportEngineLockStatus(EngineLockState engineLockState) {
    // loop through adapters, and deliver to the All handling adapter.
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->handleEngineLockStatusEvent(engineLockState));
}

void LocApiBase::reportEngDebugDataInfo(GnssEngineDebugDataInfo& gnssEngineDebugDataInfo) {
    TO_ALL_LOCADAPTERS(mLocAdapters[i]->reportEngDebugDataInfoEvent(gnssEngineDebugDataInfo));
}

enum loc_api_adapter_err LocApiBase::
   open(LOC_API_ADAPTER_EVENT_MASK_T /*mask*/)
DEFAULT_IMPL(LOC_API_ADAPTER_ERR_SUCCESS)

enum loc_api_adapter_err LocApiBase::
    close()
DEFAULT_IMPL(LOC_API_ADAPTER_ERR_SUCCESS)

void LocApiBase::
    deleteAidingData(const GnssAidingData& /*data*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    injectPosition(double /*latitude*/, double /*longitude*/, float /*accuracy*/,
                   bool /*onDemandCpi*/)
DEFAULT_IMPL()

void LocApiBase::
    injectPosition(const Location& /*location*/, bool /*onDemandCpi*/)
DEFAULT_IMPL()

void LocApiBase::
    injectPosition(const GnssLocationInfoNotification & /*locationInfo*/, bool /*onDemandCpi*/)
DEFAULT_IMPL()

void LocApiBase::
    injectPositionAndCivicAddress(const Location& location, const GnssCivicAddress& addr)
DEFAULT_IMPL()

void LocApiBase::
    setTime(LocGpsUtcTime /*time*/, int64_t /*timeReference*/, int /*uncertainty*/)
DEFAULT_IMPL()

void LocApiBase::
   atlOpenStatus(int /*handle*/, int /*is_succ*/, char* /*apn*/, uint32_t /*apnLen*/,
                 AGpsBearerType /*bear*/, LocAGpsType /*agpsType*/,
                 LocApnTypeMask /*mask*/)
DEFAULT_IMPL()

void LocApiBase::
    atlCloseStatus(int /*handle*/, int /*is_succ*/)
DEFAULT_IMPL()

LocationError LocApiBase::
    setServerSync(const char* /*url*/, int /*len*/, LocServerType /*type*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

LocationError LocApiBase::
    setServerSync(unsigned int /*ip*/, int /*port*/, LocServerType /*type*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::
    informNiResponse(GnssNiResponse /*userResponse*/, const void* /*passThroughData*/)
DEFAULT_IMPL()

LocationError LocApiBase::
    setSUPLVersionSync(GnssConfigSuplVersion /*version*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

enum loc_api_adapter_err LocApiBase::
    setNMEATypesSync (uint32_t /*typesMask*/)
DEFAULT_IMPL(LOC_API_ADAPTER_ERR_SUCCESS)

LocationError LocApiBase::
    setLPPConfigSync(GnssConfigLppProfileMask /*profileMask*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)


enum loc_api_adapter_err LocApiBase::
    setSensorPropertiesSync(bool /*gyroBiasVarianceRandomWalk_valid*/,
                        float /*gyroBiasVarianceRandomWalk*/,
                        bool /*accelBiasVarianceRandomWalk_valid*/,
                        float /*accelBiasVarianceRandomWalk*/,
                        bool /*angleBiasVarianceRandomWalk_valid*/,
                        float /*angleBiasVarianceRandomWalk*/,
                        bool /*rateBiasVarianceRandomWalk_valid*/,
                        float /*rateBiasVarianceRandomWalk*/,
                        bool /*velocityBiasVarianceRandomWalk_valid*/,
                        float /*velocityBiasVarianceRandomWalk*/)
DEFAULT_IMPL(LOC_API_ADAPTER_ERR_SUCCESS)

enum loc_api_adapter_err LocApiBase::
    setSensorPerfControlConfigSync(int /*controlMode*/,
                               int /*accelSamplesPerBatch*/,
                               int /*accelBatchesPerSec*/,
                               int /*gyroSamplesPerBatch*/,
                               int /*gyroBatchesPerSec*/,
                               int /*accelSamplesPerBatchHigh*/,
                               int /*accelBatchesPerSecHigh*/,
                               int /*gyroSamplesPerBatchHigh*/,
                               int /*gyroBatchesPerSecHigh*/,
                               int /*algorithmConfig*/)
DEFAULT_IMPL(LOC_API_ADAPTER_ERR_SUCCESS)

LocationError LocApiBase::
    setAGLONASSProtocolSync(GnssConfigAGlonassPositionProtocolMask /*aGlonassProtocol*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

LocationError LocApiBase::
    setLPPeProtocolCpSync(GnssConfigLppeControlPlaneMask /*lppeCP*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

LocationError LocApiBase::
    setLPPeProtocolUpSync(GnssConfigLppeUserPlaneMask /*lppeUP*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

GnssConfigSuplVersion LocApiBase::convertSuplVersion(const uint32_t /*suplVersion*/)
DEFAULT_IMPL(GNSS_CONFIG_SUPL_VERSION_1_0_0)

GnssConfigLppeControlPlaneMask LocApiBase::convertLppeCp(const uint32_t /*lppeControlPlaneMask*/)
DEFAULT_IMPL(0)

GnssConfigLppeUserPlaneMask LocApiBase::convertLppeUp(const uint32_t /*lppeUserPlaneMask*/)
DEFAULT_IMPL(0)

LocationError LocApiBase::setEmergencyExtensionWindowSync(
        const uint32_t /*emergencyExtensionSeconds*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::setMeasurementCorrections(
        const GnssMeasurementCorrections& /*gnssMeasurementCorrections*/)
DEFAULT_IMPL()

void LocApiBase::
   getWwanZppFix()
DEFAULT_IMPL()

void LocApiBase::
   getBestAvailableZppFix()
DEFAULT_IMPL()

bool LocApiBase::
   getBestAvailableZppFixSync(LocGpsLocation &zppLoc, LocPosTechMask &tech_mask)
DEFAULT_IMPL(false)

LocationError LocApiBase::
    setGpsLockSync(GnssConfigGpsLock /*lock*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::
    requestForAidingData(GnssAidingDataSvMask /*svDataMask*/)
DEFAULT_IMPL()

LocationError LocApiBase::
    setXtraVersionCheckSync(uint32_t /*check*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

LocationError LocApiBase::setBlacklistSvSync(const GnssSvIdConfig& /*config*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::setBlacklistSv(const GnssSvIdConfig& /*config*/,
                                LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::getBlacklistSv()
DEFAULT_IMPL()

void LocApiBase::setConstellationControl(const GnssSvTypeConfig& /*config*/,
                                         LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::getConstellationControl()
DEFAULT_IMPL()

void LocApiBase::resetConstellationControl(LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    setConstrainedTuncMode(bool /*enabled*/,
                           float /*tuncConstraint*/,
                           uint32_t /*energyBudget*/,
                           LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    setPositionAssistedClockEstimatorMode(bool /*enabled*/,
                                          LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::getGnssEnergyConsumed()
DEFAULT_IMPL()


void LocApiBase::addGeofence(uint32_t /*clientId*/, const GeofenceOption& /*options*/,
        const GeofenceInfo& /*info*/,
        LocApiResponseData<LocApiGeofenceData>* /*adapterResponseData*/)
DEFAULT_IMPL()

void LocApiBase::removeGeofence(uint32_t /*hwId*/, uint32_t /*clientId*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::pauseGeofence(uint32_t /*hwId*/, uint32_t /*clientId*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::resumeGeofence(uint32_t /*hwId*/, uint32_t /*clientId*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::modifyGeofence(uint32_t /*hwId*/, uint32_t /*clientId*/,
         const GeofenceOption& /*options*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::startTimeBasedTracking(const TrackingOptions& /*options*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::stopTimeBasedTracking(LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::startDistanceBasedTracking(uint32_t /*sessionId*/,
        const LocationOptions& /*options*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::stopDistanceBasedTracking(uint32_t /*sessionId*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::startBatching(uint32_t /*sessionId*/, const LocationOptions& /*options*/,
        uint32_t /*accuracy*/, uint32_t /*timeout*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::stopBatching(uint32_t /*sessionId*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::startOutdoorTripBatchingSync(uint32_t /*tripDistance*/,
        uint32_t /*tripTbf*/, uint32_t /*timeout*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::startOutdoorTripBatching(uint32_t /*tripDistance*/, uint32_t /*tripTbf*/,
        uint32_t /*timeout*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::reStartOutdoorTripBatching(uint32_t /*ongoingTripDistance*/,
        uint32_t /*ongoingTripInterval*/, uint32_t /*batchingTimeout,*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::stopOutdoorTripBatchingSync(bool /*deallocBatchBuffer*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::stopOutdoorTripBatching(bool /*deallocBatchBuffer*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::getBatchedLocationsSync(size_t /*count*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::getBatchedLocations(size_t /*count*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::getBatchedTripLocationsSync(size_t /*count*/,
        uint32_t /*accumulatedDistance*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::getBatchedTripLocations(size_t /*count*/, uint32_t /*accumulatedDistance*/,
        LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::queryAccumulatedTripDistanceSync(uint32_t& /*accumulated_trip_distance*/,
        uint32_t& /*numOfBatchedPositions*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::queryAccumulatedTripDistance(
        LocApiResponseData<LocApiBatchData>* /*adapterResponseData*/)
DEFAULT_IMPL()

void LocApiBase::setBatchSize(size_t /*size*/)
DEFAULT_IMPL()

void LocApiBase::setTripBatchSize(size_t /*size*/)
DEFAULT_IMPL()

void LocApiBase::addToCallQueue(LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::updateSystemPowerState(PowerStateType /*powerState*/)
DEFAULT_IMPL()

void LocApiBase::updatePowerConnectState(bool /*connected*/)
DEFAULT_IMPL()

void LocApiBase::
    configRobustLocation(bool /*enabled*/,
                         bool /*enableForE911*/,
                         LocApiResponse* /*adapterResponse*/,
                         bool /*enableForE911Valid*/)
DEFAULT_IMPL()

void LocApiBase::
    getRobustLocationConfig(uint32_t /*sessionId*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    configMinGpsWeek(uint16_t /*minGpsWeek*/,
                     LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    getMinGpsWeek(uint32_t /*sessionId*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

LocationError LocApiBase::
    setParameterSync(const GnssConfig& /*gnssConfig*/)
DEFAULT_IMPL(LOCATION_ERROR_SUCCESS)

void LocApiBase::
    getParameter(uint32_t /*sessionId*/, GnssConfigFlagsMask /*flags*/,
                 LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    configConstellationMultiBand(const GnssSvTypeConfig& /*secondaryBandConfig*/,
                                 LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::
    getConstellationMultiBandConfig(uint32_t /*sessionId*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::setTribandState(bool /*enabled*/)
DEFAULT_IMPL()

void LocApiBase::
    configPrecisePositioning(uint32_t featureId, bool enable, const std::string& appHash,
            LocApiResponse* /*adpterResponse*/)
DEFAULT_IMPL()

void LocApiBase::configMerkleTree(mgpOsnmaPublicKeyAndMerkleTreeStruct* /*merkleTree*/,
            LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::configOsnmaEnablement(bool /*enable*/, LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::getNtnConfigSignalMask(LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

void LocApiBase::setNtnConfigSignalMask(GnssSignalTypeMask /*gpsSignalTypeConfigMask*/,
            LocApiResponse* /*adapterResponse*/)
DEFAULT_IMPL()

int64_t RealtimeEstimator::getElapsedRealtimeEstimateNanos(int64_t curDataTimeNanos,
            bool isCurDataTimeTrustable, int64_t tbfNanos) {
    //The algorithm works follow below steps:
    //When isCurDataTimeTrustable is meet (means Modem timestamp is already stable),
    //1, Wait for mFixTimeStablizationThreshold fixes; While waiting for modem time
    //   stable, we set the traveltime to a default value;
    //2, When the mFixTimeStablizationThreshold fix comes, we think now the mode time
    //   is already stable, calculate the initial AP-Modem clock diff(mCurrentClockDiff)
    //   using formula:
    //   mCurrentClockDiff = currentTimeNanos - locationTimeNanos - currentTravelTimeNanos
    //3, since then, when the nth fix comes,
    //   3.1 First update mCurrentClockDiff using below formula:
    //        mCurrentClockDiff = mCurrentClockDiff + (currentTimeNanos - sinceBootTimeNanos)
    //                - (mPrevUtcTimeNanos - mPrevBootTimeNanos)
    //   3.2 Calculate currentTravelTimeNanos:
    //        currentTravelTimeNanos = currentTimeNanos - locationTimeNanos - mCurrentClockDiff
    //4, It is possible that locationTimeNanos will jump,
    //   reset mFixTimeStablizationThreshold to default value, jump to step 2 to continue.

    int64_t currentTravelTimeNanos = mInitialTravelTime;
    struct timespec currentTime = {};
    int64_t sinceBootTimeNanos = 0;
    if (getCurrentTime(currentTime, sinceBootTimeNanos)) {
        if (isCurDataTimeTrustable) {
            if (tbfNanos > 0 && tbfNanos != curDataTimeNanos - mPrevDataTimeNanos) {
                mFixTimeStablizationThreshold = 5;
            }
            int64_t currentTimeNanos = (int64_t)currentTime.tv_sec*1000000000 + currentTime.tv_nsec;
            LOC_LOGv("sinceBootTimeNanos:%" PRIi64 " currentTimeNanos:%" PRIi64 ""
                     " locationTimeNanos:%" PRIi64 "",
                     sinceBootTimeNanos, currentTimeNanos, curDataTimeNanos);
            if (mFixTimeStablizationThreshold == 0) {
                currentTravelTimeNanos = mInitialTravelTime;
                mCurrentClockDiff = currentTimeNanos - curDataTimeNanos - currentTravelTimeNanos;
            } else if (mFixTimeStablizationThreshold < 0) {
                mCurrentClockDiff = mCurrentClockDiff + (currentTimeNanos - sinceBootTimeNanos)
                        - (mPrevUtcTimeNanos - mPrevBootTimeNanos);
                currentTravelTimeNanos = currentTimeNanos - curDataTimeNanos - mCurrentClockDiff;
            }

            mPrevUtcTimeNanos = currentTimeNanos;
            mPrevBootTimeNanos = sinceBootTimeNanos;
            mPrevDataTimeNanos = curDataTimeNanos;
            mFixTimeStablizationThreshold--;
        }
    } else {
        return -1;
    }
    LOC_LOGv("Estimated travel time: %" PRIi64 "", currentTravelTimeNanos);
    return (sinceBootTimeNanos - currentTravelTimeNanos);
}

void RealtimeEstimator::reset() {
    mCurrentClockDiff = 0;
    mPrevDataTimeNanos = 0;
    mPrevUtcTimeNanos = 0;
    mPrevBootTimeNanos = 0;
    mFixTimeStablizationThreshold = 5;
    memset(&mTimePairPVTReport, 0, sizeof(mTimePairPVTReport));
    memset(&mTimePairMeasReport, 0, sizeof(mTimePairMeasReport));
}

int64_t RealtimeEstimator::getElapsedRealtimeQtimer(int64_t qtimerTicksAtOrigin) {
    struct timespec currentTime = {};
    int64_t sinceBootTimeNanos = 0;
    int64_t elapsedRealTimeNanos = 0;

    if (getCurrentTime(currentTime, sinceBootTimeNanos)) {
       uint64_t qtimerDiff = 0;
       uint64_t qTimerTickCount = getQTimerTickCount();
       if (qTimerTickCount >= qtimerTicksAtOrigin) {
           qtimerDiff = qTimerTickCount - qtimerTicksAtOrigin;
       }
       LOC_LOGd("sinceBootTimeNanos:%" PRIi64 " qtimerTicksAtOrigin=%" PRIi64 ""
                " qTimerTickCount=%" PRIi64 " qtimerDiff=%" PRIi64 "",
                sinceBootTimeNanos, qtimerTicksAtOrigin, qTimerTickCount, qtimerDiff);
       uint64_t qTimerDiffNanos = qTimerTicksToNanos(double(qtimerDiff));

       /* If the time difference between Qtimer on modem side and Qtimer on AP side
          is greater than one second we assume this is a dual-SoC device such as
          Kona and will try to get Qtimer on modem side and on AP side and
          will adjust our difference accordingly */
       if (qTimerDiffNanos > 1000000000) {
           uint64_t qtimerDelta = getQTimerDeltaNanos();
           if (qTimerDiffNanos >= qtimerDelta) {
               qTimerDiffNanos -= qtimerDelta;
           }
       }

       LOC_LOGd("Qtimer travel time: %" PRIi64 "", qTimerDiffNanos);
       if (sinceBootTimeNanos >= qTimerDiffNanos) {
           elapsedRealTimeNanos = sinceBootTimeNanos - qTimerDiffNanos;
       } else {
           elapsedRealTimeNanos = -1;
       }
    } else {
        elapsedRealTimeNanos = -1;
    }
    return elapsedRealTimeNanos;
}

void RealtimeEstimator::saveGpsTimeAndQtimerPairInPvtReport(
        const GpsLocationExtended& locationExtended,
        enum loc_sess_status status) {

    // Use GPS timestamp and qtimer tick for 1Hz PVT report or Final fixes for association
    if (locationExtended.isReportTimeAccurate() &&
            ((locationExtended.gnssSystemTime.u.gpsSystemTime.systemMsec % 1000 == 0) ||
             (LOC_SESS_SUCCESS == status))) {
        LOC_LOGv("save time association from PVT report with gps time %u %u, "
                 "qtimer %" PRIi64 " %f ",
                 locationExtended.gnssSystemTime.u.gpsSystemTime.systemWeek,
                 locationExtended.gnssSystemTime.u.gpsSystemTime.systemMsec,
                 locationExtended.systemTick, locationExtended.systemTickUnc);
        mTimePairPVTReport.gpsTime.gpsWeek =
                locationExtended.gnssSystemTime.u.gpsSystemTime.systemWeek;
        mTimePairPVTReport.gpsTime.gpsTimeOfWeekMs =
                locationExtended.gnssSystemTime.u.gpsSystemTime.systemMsec;
        mTimePairPVTReport.qtimerTick = locationExtended.systemTick;
        mTimePairPVTReport.timeUncMsec = locationExtended.systemTickUnc;
    }
}

void RealtimeEstimator::saveGpsTimeAndQtimerPairInMeasReport(
        const GnssSvMeasurementSet& svMeasurementSet) {

    const GnssSvMeasurementHeader& svMeasSetHeader = svMeasurementSet.svMeasSetHeader;

    // Use 1Hz measurement report timestamp and qtimer tick for association
    if ((svMeasurementSet.isNhz == false) &&
            (svMeasSetHeader.flags & GNSS_SV_MEAS_HEADER_HAS_GPS_SYSTEM_TIME) &&
            (svMeasSetHeader.gpsSystemTime.hasAccurateTime() == true) &&
            (svMeasSetHeader.flags & GNSS_SV_MEAS_HEADER_HAS_REF_COUNT_TICKS) &&
            (svMeasurementSet.svMeasSetHeader.refCountTicks != 0) &&
            (svMeasSetHeader.flags & GNSS_SV_MEAS_HEADER_HAS_REF_COUNT_TICKS_UNC) &&
            (svMeasurementSet.svMeasSetHeader.refCountTicksUnc != 0.0f)) {
        LOC_LOGv("save time association from meas report with gps time %u %u, "
                 "qtimer %" PRIi64 " %f ",
                 svMeasSetHeader.gpsSystemTime.systemWeek,
                 svMeasSetHeader.gpsSystemTime.systemMsec,
                 svMeasurementSet.svMeasSetHeader.refCountTicks,
                 svMeasurementSet.svMeasSetHeader.refCountTicksUnc);
            mTimePairMeasReport.gpsTime.gpsWeek = svMeasSetHeader.gpsSystemTime.systemWeek;
            mTimePairMeasReport.gpsTime.gpsTimeOfWeekMs = svMeasSetHeader.gpsSystemTime.systemMsec;
            mTimePairMeasReport.qtimerTick = svMeasurementSet.svMeasSetHeader.refCountTicks;
            mTimePairMeasReport.timeUncMsec = svMeasurementSet.svMeasSetHeader.refCountTicksUnc;
        }
    }

bool RealtimeEstimator::fillAdditionalTimestamps(
        const GpsLocationExtended& locationExtended,
        int64_t &bootTimeNsAtOrigin, float &bootTimeUnc,
        uint64_t &gptpTime, bool &gPTPValidity) {
    struct timespec curBootTime = {};
    int64_t curBootTimeNs = 0;
    int64_t curQTimerNSec = 0;
    int64_t qtimerNsecAtOrigin = 0;
    int64_t gpsTimeDiffMsec = 0;
    GpsTimeQtimerTickPair timePair;

    // We have valid association
    if (mTimePairMeasReport.gpsTime.gpsWeek != 0) {
        timePair = mTimePairMeasReport;
        LOC_LOGv("use meas time association");
    } else if (mTimePairPVTReport.gpsTime.gpsWeek != 0) {
        LOC_LOGv("use PVT time association");
        timePair = mTimePairPVTReport;
    } else {
        return false;
    }

    if (locationExtended.gnssSystemTime.hasAccurateGpsTime() == false ||
            (locationExtended.flags & GPS_LOCATION_EXTENDED_HAS_GPS_TIME) == 0 ||
            // 65535 GPS week from modem means unknown
            locationExtended.gpsTime.gpsWeek == UNKNOWN_GPS_WEEK_NUM) {
          return false;
    }

    int64_t timePairQtimerNsec = (timePair.qtimerTick / 192) * 10000;
    const GPSTimeStruct& gpsTimeAtOrigin = locationExtended.gpsTime;
    int64_t originMsec = (int64_t)gpsTimeAtOrigin.gpsWeek * (int64_t)MSEC_IN_ONE_WEEK +
                         (int64_t)gpsTimeAtOrigin.gpsTimeOfWeekMs;
    int64_t timePairMsec = (int64_t)timePair.gpsTime.gpsWeek * (int64_t)MSEC_IN_ONE_WEEK +
                            (int64_t)timePair.gpsTime.gpsTimeOfWeekMs;

    gpsTimeDiffMsec = originMsec - timePairMsec;

    qtimerNsecAtOrigin = timePairQtimerNsec + gpsTimeDiffMsec * 1000000;

    clock_gettime(CLOCK_BOOTTIME, &curBootTime);
    curBootTimeNs = ((int64_t)curBootTime.tv_sec) * 1000000000 + (int64_t)curBootTime.tv_nsec;
    // qtimer freq: 19200000, so
    // so 1 tick equals 1000,000,000/19,200,000 ns = 10000/192
    curQTimerNSec = (getQTimerTickCount() / 192) * 10000;
    bootTimeNsAtOrigin = curBootTimeNs - (curQTimerNSec - qtimerNsecAtOrigin);

    bootTimeUnc = timePair.timeUncMsec;
#ifdef PTP_SUPPORTED
    if (gptpGetPtpTimeFromQTimeNs(&gptpTime, qtimerNsecAtOrigin)) {
        gPTPValidity = true;
    }
#endif

    LOC_LOGv("gpsTimeAtOrigin (%d, %d), timepair: gps (%d, %d), "
             "qtimer nsec =%" PRIi64 ", curQTimerNSec=%" PRIi64 " qtimerNsecAtOrigin=%" PRIi64 ""
             " curBoottimeNSec=%" PRIi64 " bootimeNsecAtOrigin=%" PRIi64 ", boottime unc =%f"
             " gptp Time =%" PRIu64 " gPTPValidity = %d",
             gpsTimeAtOrigin.gpsWeek, gpsTimeAtOrigin.gpsTimeOfWeekMs,
             timePair.gpsTime.gpsWeek, timePair.gpsTime.gpsTimeOfWeekMs,
             timePairQtimerNsec, curQTimerNSec, qtimerNsecAtOrigin,
             curBootTimeNs, bootTimeNsAtOrigin, bootTimeUnc, gptpTime, gPTPValidity);

    if (bootTimeNsAtOrigin > 0) {
        return true;
    } else {
        return false;
    }
}

bool RealtimeEstimator::getCurrentTime(
        struct timespec& currentTime, int64_t& sinceBootTimeNanos)
{
    struct timespec sinceBootTime = {};
    struct timespec sinceBootTimeTest = {};
    bool clockGetTimeSuccess = false;
    const uint32_t MAX_TIME_DELTA_VALUE_NANOS = 2000000; // 2 milli-seconds
    const uint32_t MAX_GET_TIME_COUNT = 20;
    /* Attempt to get CLOCK_REALTIME and CLOCK_BOOTIME in succession without an interruption
    or context switch (for up to MAX_GET_TIME_COUNT times) to avoid errors in the calculation */
    for (uint32_t i = 0; i < MAX_GET_TIME_COUNT; i++) {
        if (clock_gettime(CLOCK_BOOTTIME, &sinceBootTime) != 0) {
            break;
        };
        if (clock_gettime(CLOCK_REALTIME, &currentTime) != 0) {
            break;
        }
        if (clock_gettime(CLOCK_BOOTTIME, &sinceBootTimeTest) != 0) {
            break;
        };
        sinceBootTimeNanos = (int64_t)sinceBootTime.tv_sec * 1000000000 + sinceBootTime.tv_nsec;
        int64_t sinceBootTimeTestNanos =
            (int64_t)sinceBootTimeTest.tv_sec * 1000000000 + sinceBootTimeTest.tv_nsec;
        int64_t sinceBootTimeDeltaNanos = sinceBootTimeTestNanos - sinceBootTimeNanos;

        /* sinceBootTime and sinceBootTimeTest should have a close value if there was no
        interruption or context switch between clock_gettime for CLOCK_BOOTIME and
        clock_gettime for CLOCK_REALTIME */
        if (sinceBootTimeDeltaNanos < MAX_TIME_DELTA_VALUE_NANOS) {
            clockGetTimeSuccess = true;
            break;
        } else {
            LOC_LOGd("Delta:%" PRIi64 "ns time too large, retry number #%u...",
                     sinceBootTimeDeltaNanos, i + 1);
        }
    }
    return clockGetTimeSuccess;
}

} // namespace loc_core
