/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
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

#define LOG_NDEBUG 0
#define LOG_TAG "LocSvc_MeasurementAPIClient"

#include <log_util.h>
#include <loc_cfg.h>
#include <inttypes.h>

#include "LocationUtil.h"
#include "MeasurementAPIClient.h"
#include <loc_misc_utils.h>

namespace android {
namespace hardware {
namespace gnss {
namespace V2_1 {
namespace implementation {

using ::android::hardware::gnss::V1_0::IGnssMeasurement;
using ::android::hardware::gnss::V2_0::IGnssMeasurementCallback;

static void convertGnssData(const GnssMeasurementsNotification& in,
        V1_0::IGnssMeasurementCallback::GnssData& out);
static void convertGnssData_1_1(const GnssMeasurementsNotification& in,
        V1_1::IGnssMeasurementCallback::GnssData& out);
static void convertGnssData_2_0(const GnssMeasurementsNotification& in,
        V2_0::IGnssMeasurementCallback::GnssData& out);
static void convertGnssData_2_1(const GnssMeasurementsNotification& in,
        V2_1::IGnssMeasurementCallback::GnssData& out);
static void convertGnssMeasurement(const GnssMeasurementsData& in,
        V1_0::IGnssMeasurementCallback::GnssMeasurement& out);
static void convertGnssClock(const GnssMeasurementsClock& in,
        IGnssMeasurementCallback::GnssClock& out);
static void convertGnssClock_2_1(const GnssMeasurementsClock& in,
        V2_1::IGnssMeasurementCallback::GnssClock& out);
static void convertGnssMeasurementsCodeType(const GnssMeasurementsCodeType& inCodeType,
        const char* inOtherCodeTypeName,
        ::android::hardware::hidl_string& out);
static void convertGnssMeasurementsAccumulatedDeltaRangeState(
        const GnssMeasurementsAdrStateMask& in,
        ::android::hardware::hidl_bitfield
                <V1_1::IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState>& out);
static void convertGnssMeasurementsState(const GnssMeasurementsStateMask& in,
        ::android::hardware::hidl_bitfield
                <V2_0::IGnssMeasurementCallback::GnssMeasurementState>& out);
static void convertElapsedRealtimeNanos(const GnssMeasurementsNotification& in,
        ::android::hardware::gnss::V2_0::ElapsedRealtime& elapsedRealtimeNanos);

MeasurementAPIClient::MeasurementAPIClient() :
    mGnssMeasurementCbIface(nullptr),
    mGnssMeasurementCbIface_1_1(nullptr),
    mGnssMeasurementCbIface_2_0(nullptr),
    mGnssMeasurementCbIface_2_1(nullptr),
    mTracking(false)
{
    LOC_LOGD("%s]: ()", __FUNCTION__);
}

MeasurementAPIClient::~MeasurementAPIClient()
{
    LOC_LOGD("%s]: ()", __FUNCTION__);
}

void MeasurementAPIClient::clearInterfaces()
{
    mGnssMeasurementCbIface = nullptr;
    mGnssMeasurementCbIface_1_1 = nullptr;
    mGnssMeasurementCbIface_2_0 = nullptr;
    mGnssMeasurementCbIface_2_1 = nullptr;
}

Return<IGnssMeasurement::GnssMeasurementStatus>
MeasurementAPIClient::startTracking(
        GnssPowerMode powerMode, uint32_t timeBetweenMeasurement)
{
    LocationCallbacks locationCallbacks;
    memset(&locationCallbacks, 0, sizeof(LocationCallbacks));
    locationCallbacks.size = sizeof(LocationCallbacks);

    if (mGnssMeasurementCbIface_2_1 != nullptr ||
        mGnssMeasurementCbIface_2_0 != nullptr ||
        mGnssMeasurementCbIface_1_1 != nullptr ||
        mGnssMeasurementCbIface != nullptr) {
        locationCallbacks.gnssMeasurementsCb =
            [this](const GnssMeasurementsNotification &gnssMeasurementsNotification) {
                onGnssMeasurementsCb(gnssMeasurementsNotification);
            };
    }

    locAPISetCallbacks(locationCallbacks);

    TrackingOptions options;
    options.size = sizeof(TrackingOptions);
    options.minInterval = 1000;
    options.powerMode = powerMode;
    options.tbm = timeBetweenMeasurement;

    mTracking = true;
    LOC_LOGd("(powermode: %d) (tbm %d)", (int)powerMode, timeBetweenMeasurement);

    locAPIStartTracking(options);
    return IGnssMeasurement::GnssMeasurementStatus::SUCCESS;
}

// for GpsMeasurementInterface
void MeasurementAPIClient::measurementClose() {
    LOC_LOGD("%s]: ()", __FUNCTION__);
    mTracking = false;
    clearInterfaces();
    locAPIStopTracking();

    // Clear measurement callback
    LocationCallbacks locationCallbacks;
    memset(&locationCallbacks, 0, sizeof(LocationCallbacks));
    locationCallbacks.size = sizeof(LocationCallbacks);
    locAPISetCallbacks(locationCallbacks);
}

// callbacks
void MeasurementAPIClient::onGnssMeasurementsCb(
        const GnssMeasurementsNotification &gnssMeasurementsNotification)
{
    LOC_LOGD("%s]: (count: %u active: %d)",
            __FUNCTION__, gnssMeasurementsNotification.count, mTracking);
    if (mTracking) {
        mMutex.lock();
        sp<V1_0::IGnssMeasurementCallback> gnssMeasurementCbIface = nullptr;
        sp<V1_1::IGnssMeasurementCallback> gnssMeasurementCbIface_1_1 = nullptr;
        sp<V2_0::IGnssMeasurementCallback> gnssMeasurementCbIface_2_0 = nullptr;
        sp<V2_1::IGnssMeasurementCallback> gnssMeasurementCbIface_2_1 = nullptr;
        if (mGnssMeasurementCbIface_2_1 != nullptr) {
            gnssMeasurementCbIface_2_1 = mGnssMeasurementCbIface_2_1;
        } else if (mGnssMeasurementCbIface_2_0 != nullptr) {
            gnssMeasurementCbIface_2_0 = mGnssMeasurementCbIface_2_0;
        } else if (mGnssMeasurementCbIface_1_1 != nullptr) {
            gnssMeasurementCbIface_1_1 = mGnssMeasurementCbIface_1_1;
        } else if (mGnssMeasurementCbIface != nullptr) {
            gnssMeasurementCbIface = mGnssMeasurementCbIface;
        }
        mMutex.unlock();

        if (gnssMeasurementCbIface_2_1 != nullptr) {
            V2_1::IGnssMeasurementCallback::GnssData gnssData;
            convertGnssData_2_1(gnssMeasurementsNotification, gnssData);
            auto r = gnssMeasurementCbIface_2_1->gnssMeasurementCb_2_1(gnssData);
            if (!r.isOk()) {
                LOC_LOGE("%s] Error from gnssMeasurementCb description=%s",
                    __func__, r.description().c_str());
            }
        } else if (gnssMeasurementCbIface_2_0 != nullptr) {
            V2_0::IGnssMeasurementCallback::GnssData gnssData;
            convertGnssData_2_0(gnssMeasurementsNotification, gnssData);
            auto r = gnssMeasurementCbIface_2_0->gnssMeasurementCb_2_0(gnssData);
            if (!r.isOk()) {
                LOC_LOGE("%s] Error from gnssMeasurementCb description=%s",
                    __func__, r.description().c_str());
            }
        } else if (gnssMeasurementCbIface_1_1 != nullptr) {
            V1_1::IGnssMeasurementCallback::GnssData gnssData;
            convertGnssData_1_1(gnssMeasurementsNotification, gnssData);
            auto r = gnssMeasurementCbIface_1_1->gnssMeasurementCb(gnssData);
            if (!r.isOk()) {
                LOC_LOGE("%s] Error from gnssMeasurementCb description=%s",
                    __func__, r.description().c_str());
            }
        } else if (gnssMeasurementCbIface != nullptr) {
            V1_0::IGnssMeasurementCallback::GnssData gnssData;
            convertGnssData(gnssMeasurementsNotification, gnssData);
            auto r = gnssMeasurementCbIface->GnssMeasurementCb(gnssData);
            if (!r.isOk()) {
                LOC_LOGE("%s] Error from GnssMeasurementCb description=%s",
                    __func__, r.description().c_str());
            }
        }
    }
}

static void convertGnssMeasurement(const GnssMeasurementsData& in,
        V1_0::IGnssMeasurementCallback::GnssMeasurement& out)
{
    memset(&out, 0, sizeof(out));
    if (in.flags & GNSS_MEASUREMENTS_DATA_SIGNAL_TO_NOISE_RATIO_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_SNR;
    if (in.flags & GNSS_MEASUREMENTS_DATA_CARRIER_FREQUENCY_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_FREQUENCY;
    if (in.flags & GNSS_MEASUREMENTS_DATA_CARRIER_CYCLES_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_CYCLES;
    if (in.flags & GNSS_MEASUREMENTS_DATA_CARRIER_PHASE_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_PHASE;
    if (in.flags & GNSS_MEASUREMENTS_DATA_CARRIER_PHASE_UNCERTAINTY_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_PHASE_UNCERTAINTY;
    if (in.flags & GNSS_MEASUREMENTS_DATA_AUTOMATIC_GAIN_CONTROL_BIT)
        out.flags |= IGnssMeasurementCallback::GnssMeasurementFlags::HAS_AUTOMATIC_GAIN_CONTROL;
    convertGnssSvid(in, out.svid);
    convertGnssConstellationType(in.svType, out.constellation);
    out.timeOffsetNs = in.timeOffsetNs;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_CODE_LOCK_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_CODE_LOCK;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_BIT_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BIT_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_SUBFRAME_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SUBFRAME_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_TOW_DECODED_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_TOW_DECODED;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_MSEC_AMBIGUOUS_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_MSEC_AMBIGUOUS;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_SYMBOL_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SYMBOL_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_GLO_STRING_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_STRING_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_GLO_TOD_DECODED_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_TOD_DECODED;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_BDS_D2_BIT_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BDS_D2_BIT_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_BDS_D2_SUBFRAME_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BDS_D2_SUBFRAME_SYNC;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_GAL_E1BC_CODE_LOCK_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1BC_CODE_LOCK;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_GAL_E1C_2ND_CODE_LOCK_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1C_2ND_CODE_LOCK;
    if (in.stateMask & GNSS_MEASUREMENTS_STATE_GAL_E1B_PAGE_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1B_PAGE_SYNC;
    if (in.stateMask &  GNSS_MEASUREMENTS_STATE_SBAS_SYNC_BIT)
        out.state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SBAS_SYNC;
    out.receivedSvTimeInNs = in.receivedSvTimeNs;
    out.receivedSvTimeUncertaintyInNs = in.receivedSvTimeUncertaintyNs;
    out.cN0DbHz = in.carrierToNoiseDbHz;
    out.pseudorangeRateMps = in.pseudorangeRateMps;
    out.pseudorangeRateUncertaintyMps = in.pseudorangeRateUncertaintyMps;
    if (in.adrStateMask & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_VALID_BIT)
        out.accumulatedDeltaRangeState |=
            IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_VALID;
    if (in.adrStateMask & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_RESET_BIT)
        out.accumulatedDeltaRangeState |=
            IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_RESET;
    if (in.adrStateMask & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_CYCLE_SLIP_BIT)
        out.accumulatedDeltaRangeState |=
            IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_CYCLE_SLIP;
    out.accumulatedDeltaRangeM = in.adrMeters;
    out.accumulatedDeltaRangeUncertaintyM = in.adrUncertaintyMeters;
    out.carrierFrequencyHz = in.carrierFrequencyHz;
    out.carrierCycles = in.carrierCycles;
    out.carrierPhase = in.carrierPhase;
    out.carrierPhaseUncertainty = in.carrierPhaseUncertainty;
    uint8_t indicator =
        static_cast<uint8_t>(IGnssMeasurementCallback::GnssMultipathIndicator::INDICATOR_UNKNOWN);
    if (in.multipathIndicator & GNSS_MEASUREMENTS_MULTIPATH_INDICATOR_PRESENT)
        indicator |= IGnssMeasurementCallback::GnssMultipathIndicator::INDICATOR_PRESENT;
    if (in.multipathIndicator & GNSS_MEASUREMENTS_MULTIPATH_INDICATOR_NOT_PRESENT)
        indicator |= IGnssMeasurementCallback::GnssMultipathIndicator::INDICATIOR_NOT_PRESENT;
    out.multipathIndicator =
        static_cast<IGnssMeasurementCallback::GnssMultipathIndicator>(indicator);
    out.snrDb = in.signalToNoiseRatioDb;
    out.agcLevelDb = in.agcLevelDb;
}

static void convertGnssClock(const GnssMeasurementsClock& in,
        IGnssMeasurementCallback::GnssClock& out)
{
    memset(&out, 0, sizeof(out));
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_LEAP_SECOND_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_LEAP_SECOND;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_TIME_UNCERTAINTY_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_TIME_UNCERTAINTY;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_FULL_BIAS_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_FULL_BIAS;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_BIAS_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_BIAS;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_BIAS_UNCERTAINTY_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_BIAS_UNCERTAINTY;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_DRIFT_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_DRIFT;
    if (in.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_DRIFT_UNCERTAINTY_BIT)
        out.gnssClockFlags |= IGnssMeasurementCallback::GnssClockFlags::HAS_DRIFT_UNCERTAINTY;
    out.leapSecond = in.leapSecond;
    out.timeNs = in.timeNs;
    out.timeUncertaintyNs = in.timeUncertaintyNs;
    out.fullBiasNs = in.fullBiasNs;
    out.biasNs = in.biasNs;
    out.biasUncertaintyNs = in.biasUncertaintyNs;
    out.driftNsps = in.driftNsps;
    out.driftUncertaintyNsps = in.driftUncertaintyNsps;
    out.hwClockDiscontinuityCount = in.hwClockDiscontinuityCount;
}

static void convertGnssClock_2_1(const GnssMeasurementsClock& in,
        V2_1::IGnssMeasurementCallback::GnssClock& out)
{
    memset(&out, 0, sizeof(out));
    convertGnssClock(in, out.v1_0);
    convertGnssConstellationType(in.referenceSignalTypeForIsb.svType,
            out.referenceSignalTypeForIsb.constellation);
    out.referenceSignalTypeForIsb.carrierFrequencyHz =
            in.referenceSignalTypeForIsb.carrierFrequencyHz;
    convertGnssMeasurementsCodeType(in.referenceSignalTypeForIsb.codeType,
            in.referenceSignalTypeForIsb.otherCodeTypeName,
            out.referenceSignalTypeForIsb.codeType);
}

static void convertGnssData(const GnssMeasurementsNotification& in,
        V1_0::IGnssMeasurementCallback::GnssData& out)
{
    memset(&out, 0, sizeof(out));
    out.measurementCount = in.count;
    if (out.measurementCount > static_cast<uint32_t>(V1_0::GnssMax::SVS_COUNT)) {
        LOC_LOGW("%s]: Too many measurement %u. Clamps to %d.",
                __FUNCTION__,  out.measurementCount, V1_0::GnssMax::SVS_COUNT);
        out.measurementCount = static_cast<uint32_t>(V1_0::GnssMax::SVS_COUNT);
    }
    for (size_t i = 0; i < out.measurementCount; i++) {
        convertGnssMeasurement(in.measurements[i], out.measurements[i]);
    }
    convertGnssClock(in.clock, out.clock);
}

static void convertGnssData_1_1(const GnssMeasurementsNotification& in,
        V1_1::IGnssMeasurementCallback::GnssData& out)
{
    memset(&out, 0, sizeof(out));
    out.measurements.resize(in.count);
    for (size_t i = 0; i < in.count; i++) {
        convertGnssMeasurement(in.measurements[i], out.measurements[i].v1_0);
        convertGnssMeasurementsAccumulatedDeltaRangeState(in.measurements[i].adrStateMask,
                out.measurements[i].accumulatedDeltaRangeState);
    }
    convertGnssClock(in.clock, out.clock);
}

static void convertGnssData_2_0(const GnssMeasurementsNotification& in,
        V2_0::IGnssMeasurementCallback::GnssData& out)
{
    memset(&out, 0, sizeof(out));
    out.measurements.resize(in.count);
    for (size_t i = 0; i < in.count; i++) {
        convertGnssMeasurement(in.measurements[i], out.measurements[i].v1_1.v1_0);
        convertGnssConstellationType(in.measurements[i].svType, out.measurements[i].constellation);
        convertGnssMeasurementsCodeType(in.measurements[i].codeType,
            in.measurements[i].otherCodeTypeName,
            out.measurements[i].codeType);
        convertGnssMeasurementsAccumulatedDeltaRangeState(in.measurements[i].adrStateMask,
                out.measurements[i].v1_1.accumulatedDeltaRangeState);
        convertGnssMeasurementsState(in.measurements[i].stateMask, out.measurements[i].state);
    }
    convertGnssClock(in.clock, out.clock);
    convertElapsedRealtimeNanos(in, out.elapsedRealtime);
}

static void convertGnssMeasurementsCodeType(const GnssMeasurementsCodeType& inCodeType,
        const char* inOtherCodeTypeName, ::android::hardware::hidl_string& out)
{
    switch(inCodeType) {
        case GNSS_MEASUREMENTS_CODE_TYPE_A:
            out = "A";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_B:
            out = "B";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_C:
            out = "C";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_I:
            out = "I";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_L:
            out = "L";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_M:
            out = "M";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_P:
            out = "P";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_Q:
            out = "Q";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_S:
            out = "S";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_W:
            out = "W";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_X:
            out = "X";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_Y:
            out = "Y";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_Z:
            out = "Z";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_N:
            out = "N";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_D:
            out = "D";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_E:
            out = "E";
            break;
        case GNSS_MEASUREMENTS_CODE_TYPE_OTHER:
        default:
            out = inOtherCodeTypeName;
            break;
    }
}

static void convertGnssMeasurementsAccumulatedDeltaRangeState(
        const GnssMeasurementsAdrStateMask& in,
        ::android::hardware::hidl_bitfield
                <V1_1::IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState>& out)
{
    memset(&out, 0, sizeof(out));
    if (in & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_VALID_BIT)
        out |= IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_VALID;
    if (in & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_RESET_BIT)
        out |= IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_RESET;
    if (in & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_CYCLE_SLIP_BIT)
        out |= IGnssMeasurementCallback::GnssAccumulatedDeltaRangeState::ADR_STATE_CYCLE_SLIP;
    if (in & GNSS_MEASUREMENTS_ACCUMULATED_DELTA_RANGE_STATE_HALF_CYCLE_RESOLVED_BIT)
        out |= IGnssMeasurementCallback::
                GnssAccumulatedDeltaRangeState::ADR_STATE_HALF_CYCLE_RESOLVED;
}

static void convertGnssMeasurementsState(const GnssMeasurementsStateMask& in,
        ::android::hardware::hidl_bitfield
                <V2_0::IGnssMeasurementCallback::GnssMeasurementState>& out)
{
    memset(&out, 0, sizeof(out));
    if (in & GNSS_MEASUREMENTS_STATE_CODE_LOCK_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_CODE_LOCK;
    if (in & GNSS_MEASUREMENTS_STATE_BIT_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BIT_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_SUBFRAME_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SUBFRAME_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_TOW_DECODED_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_TOW_DECODED;
    if (in & GNSS_MEASUREMENTS_STATE_MSEC_AMBIGUOUS_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_MSEC_AMBIGUOUS;
    if (in & GNSS_MEASUREMENTS_STATE_SYMBOL_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SYMBOL_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_GLO_STRING_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_STRING_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_GLO_TOD_DECODED_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_TOD_DECODED;
    if (in & GNSS_MEASUREMENTS_STATE_BDS_D2_BIT_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BDS_D2_BIT_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_BDS_D2_SUBFRAME_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_BDS_D2_SUBFRAME_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_GAL_E1BC_CODE_LOCK_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1BC_CODE_LOCK;
    if (in & GNSS_MEASUREMENTS_STATE_GAL_E1C_2ND_CODE_LOCK_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1C_2ND_CODE_LOCK;
    if (in & GNSS_MEASUREMENTS_STATE_GAL_E1B_PAGE_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GAL_E1B_PAGE_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_SBAS_SYNC_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_SBAS_SYNC;
    if (in & GNSS_MEASUREMENTS_STATE_TOW_KNOWN_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_TOW_KNOWN;
    if (in & GNSS_MEASUREMENTS_STATE_GLO_TOD_KNOWN_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_TOD_KNOWN;
    if (in & GNSS_MEASUREMENTS_STATE_2ND_CODE_LOCK_BIT)
        out |= IGnssMeasurementCallback::GnssMeasurementState::STATE_2ND_CODE_LOCK;
}

static void convertGnssData_2_1(const GnssMeasurementsNotification& in,
        V2_1::IGnssMeasurementCallback::GnssData& out)
{
    memset(&out, 0, sizeof(out));
    out.measurements.resize(in.count);
    for (size_t i = 0; i < in.count; i++) {
        out.measurements[i].flags = 0;
        convertGnssMeasurement(in.measurements[i], out.measurements[i].v2_0.v1_1.v1_0);
        convertGnssConstellationType(in.measurements[i].svType,
                out.measurements[i].v2_0.constellation);
        convertGnssMeasurementsCodeType(in.measurements[i].codeType,
                in.measurements[i].otherCodeTypeName,
                out.measurements[i].v2_0.codeType);
        convertGnssMeasurementsAccumulatedDeltaRangeState(in.measurements[i].adrStateMask,
                out.measurements[i].v2_0.v1_1.accumulatedDeltaRangeState);
        convertGnssMeasurementsState(in.measurements[i].stateMask,
                out.measurements[i].v2_0.state);
        out.measurements[i].basebandCN0DbHz = in.measurements[i].basebandCarrierToNoiseDbHz;

        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_SIGNAL_TO_NOISE_RATIO_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_SNR;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_CARRIER_FREQUENCY_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_FREQUENCY;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_CARRIER_CYCLES_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_CYCLES;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_CARRIER_PHASE_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_CARRIER_PHASE;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_CARRIER_PHASE_UNCERTAINTY_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::
                        GnssMeasurementFlags::HAS_CARRIER_PHASE_UNCERTAINTY;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_AUTOMATIC_GAIN_CONTROL_BIT) {
            out.measurements[i].flags |=
                V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_AUTOMATIC_GAIN_CONTROL;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_FULL_ISB_BIT) {
            out.measurements[i].fullInterSignalBiasNs = in.measurements[i].fullInterSignalBiasNs;
            out.measurements[i].flags |=
                    V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_FULL_ISB;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_FULL_ISB_UNCERTAINTY_BIT) {
            out.measurements[i].fullInterSignalBiasUncertaintyNs =
                    in.measurements[i].fullInterSignalBiasUncertaintyNs;
            out.measurements[i].flags |=
                    V2_1::IGnssMeasurementCallback::
                            GnssMeasurementFlags::HAS_FULL_ISB_UNCERTAINTY;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_SATELLITE_ISB_BIT) {
            out.measurements[i].satelliteInterSignalBiasNs =
                    in.measurements[i].satelliteInterSignalBiasNs;
            out.measurements[i].flags |=
                    V2_1::IGnssMeasurementCallback::GnssMeasurementFlags::HAS_SATELLITE_ISB;
        }
        if (in.measurements[i].flags & GNSS_MEASUREMENTS_DATA_SATELLITE_ISB_UNCERTAINTY_BIT) {
            out.measurements[i].satelliteInterSignalBiasUncertaintyNs =
                    in.measurements[i].satelliteInterSignalBiasUncertaintyNs;
            out.measurements[i].flags |=
                    V2_1::IGnssMeasurementCallback::
                            GnssMeasurementFlags::HAS_SATELLITE_ISB_UNCERTAINTY;
        }
    }
    convertGnssClock_2_1(in.clock, out.clock);
    convertElapsedRealtimeNanos(in, out.elapsedRealtime);
}

static void convertElapsedRealtimeNanos(const GnssMeasurementsNotification& in,
        ::android::hardware::gnss::V2_0::ElapsedRealtime& elapsedRealtime)
{
    if (in.clock.flags & GNSS_MEASUREMENTS_CLOCK_FLAGS_ELAPSED_REAL_TIME_BIT) {
        elapsedRealtime.flags |= V2_0::ElapsedRealtimeFlags::HAS_TIMESTAMP_NS;
        elapsedRealtime.timestampNs = in.clock.elapsedRealTime;
        elapsedRealtime.flags |= V2_0::ElapsedRealtimeFlags::HAS_TIME_UNCERTAINTY_NS;
        elapsedRealtime.timeUncertaintyNs = in.clock.elapsedRealTimeUnc;
        LOC_LOGd("elapsedRealtime.timestampNs=%" PRIi64 ""
                 " elapsedRealtime.timeUncertaintyNs=%" PRIi64 " elapsedRealtime.flags=0x%X",
                 elapsedRealtime.timestampNs,
                 elapsedRealtime.timeUncertaintyNs, elapsedRealtime.flags);
    }
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
