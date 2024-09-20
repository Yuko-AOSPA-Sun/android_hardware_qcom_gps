/* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "LocSvc_SystemStatus"

#include <inttypes.h>
#include <string>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <loc_pla.h>
#include <log_util.h>
#include <loc_nmea.h>
#include <DataItemsFactoryProxy.h>
#include <SystemStatus.h>
#include <SystemStatusOsObserver.h>
#include <DataItemConcreteTypes.h>

namespace loc_core
{
/******************************************************************************
 SystemStatusTimeAndClock
******************************************************************************/
SystemStatusTimeAndClock::SystemStatusTimeAndClock(const GnssEngineDebugDataInfo& info) :
    mGpsWeek(info.gpsWeek),
    mGpsTowMs(info.gpsTowMs),
    mTimeValid(info.timeValid),
    mTimeSource(info.sourceOfTime),
    mTimeUnc(info.clkTimeUnc),
    mClockFreqBias(info.clkFreqBias),
    mClockFreqBiasUnc(info.clkFreqUnc),
    mLeapSeconds(info.leapSecondInfo.leapSec),
    mLeapSecUnc(info.leapSecondInfo.leapSecUnc),
    mTimeUncNs(info.clkTimeUnc * 1000000LL)
{
}

bool SystemStatusTimeAndClock::equals(const SystemStatusItemBase& peer) {
    if ((mGpsWeek != ((const SystemStatusTimeAndClock&)peer).mGpsWeek) ||
        (mGpsTowMs != ((const SystemStatusTimeAndClock&)peer).mGpsTowMs) ||
        (mTimeValid != ((const SystemStatusTimeAndClock&)peer).mTimeValid) ||
        (mTimeSource != ((const SystemStatusTimeAndClock&)peer).mTimeSource) ||
        (mTimeUnc != ((const SystemStatusTimeAndClock&)peer).mTimeUnc) ||
        (mClockFreqBias != ((const SystemStatusTimeAndClock&)peer).mClockFreqBias) ||
        (mClockFreqBiasUnc != ((const SystemStatusTimeAndClock&)peer).mClockFreqBiasUnc) ||
        (mLeapSeconds != ((const SystemStatusTimeAndClock&)peer).mLeapSeconds) ||
        (mLeapSecUnc != ((const SystemStatusTimeAndClock&)peer).mLeapSecUnc) ||
        (mTimeUncNs != ((const SystemStatusTimeAndClock&)peer).mTimeUncNs)) {
        return false;
    }
    return true;
}

void SystemStatusTimeAndClock::dump()
{
    LOC_LOGV("TimeAndClock: u=%ld:%ld g=%d:%d v=%d ts=%d tu=%d b=%d bu=%d ls=%d lu=%d un=%" PRIu64,
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mGpsWeek,
             mGpsTowMs,
             mTimeValid,
             mTimeSource,
             mTimeUnc,
             mClockFreqBias,
             mClockFreqBiasUnc,
             mLeapSeconds,
             mLeapSecUnc,
             mTimeUncNs);
}

/******************************************************************************
 SystemStatusXoState
******************************************************************************/
SystemStatusXoState::SystemStatusXoState(const GnssEngineDebugDataInfo& info) :
    mXoState(info.xoState)
{
}

bool SystemStatusXoState::equals(const SystemStatusItemBase& peer) {
    if (mXoState != ((const SystemStatusXoState&)peer).mXoState) {
        return false;
    }
    return true;
}

void SystemStatusXoState::dump()
{
    LOC_LOGV("XoState: u=%ld:%ld x=%d",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mXoState);
}

/******************************************************************************
 SystemStatusRfAndParams
******************************************************************************/
SystemStatusRfAndParams::SystemStatusRfAndParams(const GnssEngineDebugDataInfo& info) :
    mJammedSignalsMask(info.jammedSignalsMask),
    mJammerGps(0),
    mJammerGlo(0),
    mJammerBds(0),
    mJammerGal(0) {

    if (info.jammerInd.size() > 0) {
         mJammerGps = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GPS_L1CA];
         mJammerGlo = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GLONASS_G1];
         mJammerBds = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_BEIDOU_B1_I];
         mJammerGal = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GALILEO_E1_C];
    }

    mJammerInd = std::move(info.jammerInd);
}

bool SystemStatusRfAndParams::equals(const SystemStatusItemBase& peer) {
    if ((mJammerGps != ((const SystemStatusRfAndParams&)peer).mJammerGps) ||
        (mJammerGlo != ((const SystemStatusRfAndParams&)peer).mJammerGlo) ||
        (mJammerBds != ((const SystemStatusRfAndParams&)peer).mJammerBds) ||
        (mJammerGal != ((const SystemStatusRfAndParams&)peer).mJammerGal) ||
        (mJammedSignalsMask != ((const SystemStatusRfAndParams&)peer).mJammedSignalsMask)) {
        return false;
    }
    return true;
}

void SystemStatusRfAndParams::dump()
{
    LOC_LOGV("RfAndParams: u=%ld:%ld jgp=%d jgl=%d jbd=%d jga=%d ",
             mUtcTime.tv_sec, mUtcTime.tv_nsec, mJammerGps, mJammerGlo,
             mJammerBds, mJammerGal);
}

/******************************************************************************
 SystemStatusErrRecovery
******************************************************************************/
SystemStatusErrRecovery::SystemStatusErrRecovery(const GnssEngineDebugDataInfo& info) :
    mRecErrorRecovery(info.rcvrErrRecovery)
{
}

bool SystemStatusErrRecovery::equals(const SystemStatusItemBase& peer) {
    if (mRecErrorRecovery != ((const SystemStatusErrRecovery&)peer).mRecErrorRecovery) {
        return false;
    }
    return true;
}

void SystemStatusErrRecovery::dump()
{
    LOC_LOGV("ErrRecovery: u=%ld:%ld e=%d",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mRecErrorRecovery);
}

/******************************************************************************
 SystemStatusInjectedPosition
******************************************************************************/
SystemStatusInjectedPosition::SystemStatusInjectedPosition(const GnssEngineDebugDataInfo& info) :
    mEpiValidity(info.epiValidity),
    mEpiLat(info.epiLat),
    mEpiLon(info.epiLon),
    mEpiAlt(info.epiAlt),
    mEpiHepe(info.epiHepe),
    mEpiAltUnc(info.epiAltUnc),
    mEpiSrc(info.epiSrc)
{
}

bool SystemStatusInjectedPosition::equals(const SystemStatusItemBase& peer) {
    if ((mEpiValidity != ((const SystemStatusInjectedPosition&)peer).mEpiValidity) ||
        (mEpiLat != ((const SystemStatusInjectedPosition&)peer).mEpiLat) ||
        (mEpiLon != ((const SystemStatusInjectedPosition&)peer).mEpiLon) ||
        (mEpiAlt != ((const SystemStatusInjectedPosition&)peer).mEpiAlt) ||
        (mEpiHepe != ((const SystemStatusInjectedPosition&)peer).mEpiHepe) ||
        (mEpiAltUnc != ((const SystemStatusInjectedPosition&)peer).mEpiAltUnc) ||
        (mEpiSrc != ((const SystemStatusInjectedPosition&)peer).mEpiSrc)) {
        return false;
    }
    return true;
}

void SystemStatusInjectedPosition::dump()
{
    LOC_LOGV("InjectedPosition: u=%ld:%ld v=%x la=%f lo=%f al=%f he=%f au=%f es=%d",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mEpiValidity,
             mEpiLat,
             mEpiLon,
             mEpiAlt,
             mEpiHepe,
             mEpiAltUnc,
             mEpiSrc);
}

/******************************************************************************
 SystemStatusBestPosition
******************************************************************************/
SystemStatusBestPosition::SystemStatusBestPosition(const GnssEngineDebugDataInfo& info) :
    mValid(true),
    mBestLat(info.bestPosLat),
    mBestLon(info.bestPosLon),
    mBestAlt(info.bestPosAlt),
    mBestHepe(info.bestPosHepe),
    mBestAltUnc(info.bestPosAltUnc)
{
}

bool SystemStatusBestPosition::equals(const SystemStatusItemBase& peer) {
    if ((mBestLat != ((const SystemStatusBestPosition&)peer).mBestLat) ||
        (mBestLon != ((const SystemStatusBestPosition&)peer).mBestLon) ||
        (mBestAlt != ((const SystemStatusBestPosition&)peer).mBestAlt) ||
        (mBestHepe != ((const SystemStatusBestPosition&)peer).mBestHepe) ||
        (mBestAltUnc != ((const SystemStatusBestPosition&)peer).mBestAltUnc)) {
        return false;
    }
    return true;
}

void SystemStatusBestPosition::dump()
{
    LOC_LOGV("BestPosition: u=%ld:%ld la=%f lo=%f al=%f he=%f au=%f",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mBestLat,
             mBestLon,
             mBestAlt,
             mBestHepe,
             mBestAltUnc);
}

/******************************************************************************
 SystemStatusXtra
******************************************************************************/
SystemStatusXtra::SystemStatusXtra(const GnssEngineDebugDataInfo& info) :
    mXtraValidMask(info.xtraValidMask),
    mGpsXtraAge(info.gpsXtraAge),
    mGloXtraAge(info.gloXtraAge),
    mBdsXtraAge(info.bdsXtraAge),
    mGalXtraAge(info.galXtraAge),
    mQzssXtraAge(info.qzssXtraAge),
    mNavicXtraAge(info.navicXtraAge),
    mGpsXtraValid(info.gpsXtraMask),
    mGloXtraValid(info.gloXtraMask),
    mBdsXtraValid(info.bdsXtraMask),
    mGalXtraValid(info.galXtraMask),
    mQzssXtraValid(info.qzssXtraMask),
    mNavicXtraValid(info.navicXtraMask)
{
}

bool SystemStatusXtra::equals(const SystemStatusItemBase& peer) {
    if ((mXtraValidMask != ((const SystemStatusXtra&)peer).mXtraValidMask) ||
        (mGpsXtraAge != ((const SystemStatusXtra&)peer).mGpsXtraAge) ||
        (mGloXtraAge != ((const SystemStatusXtra&)peer).mGloXtraAge) ||
        (mBdsXtraAge != ((const SystemStatusXtra&)peer).mBdsXtraAge) ||
        (mGalXtraAge != ((const SystemStatusXtra&)peer).mGalXtraAge) ||
        (mQzssXtraAge != ((const SystemStatusXtra&)peer).mQzssXtraAge) ||
        (mNavicXtraAge != ((const SystemStatusXtra&)peer).mNavicXtraAge) ||
        (mGpsXtraValid != ((const SystemStatusXtra&)peer).mGpsXtraValid) ||
        (mGloXtraValid != ((const SystemStatusXtra&)peer).mGloXtraValid) ||
        (mBdsXtraValid != ((const SystemStatusXtra&)peer).mBdsXtraValid) ||
        (mGalXtraValid != ((const SystemStatusXtra&)peer).mGalXtraValid) ||
        (mQzssXtraValid != ((const SystemStatusXtra&)peer).mQzssXtraValid) ||
        (mNavicXtraValid != ((const SystemStatusXtra&)peer).mNavicXtraValid)) {
        return false;
    }
    return true;
}

void SystemStatusXtra::dump()
{
    LOC_LOGV("SystemStatusXtra: u=%ld:%ld m=%x a=%d:%d:%d:%d:%d v=%x:%x:%" PRIx64 ":%" PRIx64":%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mXtraValidMask,
             mGpsXtraAge,
             mGloXtraAge,
             mBdsXtraAge,
             mGalXtraAge,
             mQzssXtraAge,
             mGpsXtraValid,
             mGloXtraValid,
             mBdsXtraValid,
             mGalXtraValid,
             mQzssXtraValid);
}

/******************************************************************************
 SystemStatusEphemeris
******************************************************************************/
SystemStatusEphemeris::SystemStatusEphemeris(const GnssEngineDebugDataInfo& info) :
    mGpsEpheValid(info.gpsEphMask),
    mGloEpheValid(info.gloEphMask),
    mBdsEpheValid(info.bdsEphMask),
    mGalEpheValid(info.galEphMask),
    mQzssEpheValid(info.qzssEphMask),
    mNavicEpheValid(info.navicEphMask)
{
}

bool SystemStatusEphemeris::equals(const SystemStatusItemBase& peer) {
    if ((mGpsEpheValid != ((const SystemStatusEphemeris&)peer).mGpsEpheValid) ||
        (mGloEpheValid != ((const SystemStatusEphemeris&)peer).mGloEpheValid) ||
        (mBdsEpheValid != ((const SystemStatusEphemeris&)peer).mBdsEpheValid) ||
        (mGalEpheValid != ((const SystemStatusEphemeris&)peer).mGalEpheValid) ||
        (mQzssEpheValid != ((const SystemStatusEphemeris&)peer).mQzssEpheValid) ||
        (mNavicEpheValid != ((const SystemStatusEphemeris&)peer).mNavicEpheValid)) {
        return false;
    }
    return true;
}

void SystemStatusEphemeris::dump()
{
    LOC_LOGV("Ephemeris: u=%ld:%ld ev=%x:%x:%" PRIx64 ":%" PRIx64 ":%x%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mGpsEpheValid,
             mGloEpheValid,
             mBdsEpheValid,
             mGalEpheValid,
             mQzssEpheValid,
             mNavicEpheValid);
}

/******************************************************************************
 SystemStatusSvHealth
******************************************************************************/
SystemStatusSvHealth::SystemStatusSvHealth(const GnssEngineDebugDataInfo& info) :
    mGpsUnknownMask(info.gpsHealthUnknownMask),
    mGloUnknownMask(info.gloHealthUnknownMask),
    mBdsUnknownMask(info.bdsHealthUnknownMask),
    mGalUnknownMask(info.galHealthUnknownMask),
    mQzssUnknownMask(info.qzssHealthUnknownMask),
    mNavicUnknownMask(info.navicHealthUnknownMask),
    mGpsGoodMask(info.gpsHealthGoodMask),
    mGloGoodMask(info.gloHealthGoodMask),
    mBdsGoodMask(info.bdsHealthGoodMask),
    mGalGoodMask(info.galHealthGoodMask),
    mQzssGoodMask(info.qzssHealthGoodMask),
    mNavicGoodMask(info.navicHealthGoodMask),
    mGpsBadMask(info.gpsHealthBadMask),
    mGloBadMask(info.gloHealthBadMask),
    mBdsBadMask(info.bdsHealthBadMask),
    mGalBadMask(info.galHealthBadMask),
    mQzssBadMask(info.qzssHealthBadMask),
    mNavicBadMask(info.navicHealthBadMask)
{
}

bool SystemStatusSvHealth::equals(const SystemStatusItemBase& peer) {
    if ((mGpsUnknownMask != ((const SystemStatusSvHealth&)peer).mGpsUnknownMask) ||
        (mGloUnknownMask != ((const SystemStatusSvHealth&)peer).mGloUnknownMask) ||
        (mBdsUnknownMask != ((const SystemStatusSvHealth&)peer).mBdsUnknownMask) ||
        (mGalUnknownMask != ((const SystemStatusSvHealth&)peer).mGalUnknownMask) ||
        (mQzssUnknownMask != ((const SystemStatusSvHealth&)peer).mQzssUnknownMask) ||
        (mNavicUnknownMask != ((const SystemStatusSvHealth&)peer).mNavicUnknownMask) ||
        (mGpsGoodMask != ((const SystemStatusSvHealth&)peer).mGpsGoodMask) ||
        (mGloGoodMask != ((const SystemStatusSvHealth&)peer).mGloGoodMask) ||
        (mBdsGoodMask != ((const SystemStatusSvHealth&)peer).mBdsGoodMask) ||
        (mGalGoodMask != ((const SystemStatusSvHealth&)peer).mGalGoodMask) ||
        (mQzssGoodMask != ((const SystemStatusSvHealth&)peer).mQzssGoodMask) ||
        (mNavicGoodMask != ((const SystemStatusSvHealth&)peer).mNavicGoodMask) ||
        (mGpsBadMask != ((const SystemStatusSvHealth&)peer).mGpsBadMask) ||
        (mGloBadMask != ((const SystemStatusSvHealth&)peer).mGloBadMask) ||
        (mBdsBadMask != ((const SystemStatusSvHealth&)peer).mBdsBadMask) ||
        (mGalBadMask != ((const SystemStatusSvHealth&)peer).mGalBadMask) ||
        (mQzssBadMask != ((const SystemStatusSvHealth&)peer).mQzssBadMask) ||
        (mNavicBadMask != ((const SystemStatusSvHealth&)peer).mNavicBadMask)) {
        return false;
    }
    return true;
}

void SystemStatusSvHealth::dump()
{
    LOC_LOGV("SvHealth: u=%ld:%ld \
             u=%x:%x:%" PRIx64 ":%" PRIx64 ":%x:%x\
             g=%x:%x:%" PRIx64 ":%" PRIx64 ":%x:%x \
             b=%x:%x:%" PRIx64 ":%" PRIx64 ":%x:%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mGpsUnknownMask,
             mGloUnknownMask,
             mBdsUnknownMask,
             mGalUnknownMask,
             mQzssUnknownMask,
             mNavicUnknownMask,
             mGpsGoodMask,
             mGloGoodMask,
             mBdsGoodMask,
             mGalGoodMask,
             mQzssGoodMask,
             mNavicGoodMask,
             mGpsBadMask,
             mGloBadMask,
             mBdsBadMask,
             mGalBadMask,
             mQzssBadMask,
             mNavicBadMask);
}

/******************************************************************************
 SystemStatusPdr
******************************************************************************/
SystemStatusPdr::SystemStatusPdr(const GnssEngineDebugDataInfo& info) :
    mFixInfoMask(info.fixInfoMask)
{
}

bool SystemStatusPdr::equals(const SystemStatusItemBase& peer) {
    if (mFixInfoMask != ((const SystemStatusPdr&)peer).mFixInfoMask) {
        return false;
    }
    return true;
}

void SystemStatusPdr::dump()
{
    LOC_LOGV("Pdr: u=%ld:%ld m=%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mFixInfoMask);
}

/******************************************************************************
 SystemStatusNavData
******************************************************************************/
SystemStatusNavData::SystemStatusNavData(const GnssEngineDebugDataInfo& info)
{
   memset(mNav, 0, sizeof(mNav));
   for (int i = 0; i < info.navDataLen; i++) {
        GnssNavDataInfo navInfo  = info.navData[i];
        int offset = 0;
        if (0 == navInfo.gnssSvId) continue;
        // GPS
        if (navInfo.gnssSvId >= GPS_SV_ID_MIN && navInfo.gnssSvId <= GPS_SV_ID_MAX) {
            offset = GPS_SV_INDEX_OFFSET + navInfo.gnssSvId - GPS_SV_ID_MIN;
        }
        // GLO
        if (navInfo.gnssSvId >= GLO_SV_ID_MIN && navInfo.gnssSvId <= GLO_SV_ID_MAX) {
            offset = GLO_SV_INDEX_OFFSET + navInfo.gnssSvId - GLO_SV_ID_MIN;
        }
        // BDS
        if (navInfo.gnssSvId >= BDS_SV_ID_MIN && navInfo.gnssSvId <= BDS_SV_ID_MAX) {
            offset = BDS_SV_INDEX_OFFSET + navInfo.gnssSvId - BDS_SV_ID_MIN;
        }
        // GAL
        if (navInfo.gnssSvId >= GAL_SV_ID_MIN && navInfo.gnssSvId <= GAL_SV_ID_MAX) {
            offset = GAL_SV_INDEX_OFFSET + navInfo.gnssSvId - GAL_SV_ID_MIN;
        }
        // QZSS
        if (navInfo.gnssSvId >= QZSS_SV_ID_MIN && navInfo.gnssSvId <= QZSS_SV_ID_MAX) {
            offset = QZSS_SV_INDEX_OFFSET + navInfo.gnssSvId - QZSS_SV_ID_MIN;
        }
        // Navic
        if (navInfo.gnssSvId >= NAVIC_SV_ID_MIN && navInfo.gnssSvId <= NAVIC_SV_ID_MAX) {
            offset = NAVIC_SV_INDEX_OFFSET + navInfo.gnssSvId - NAVIC_SV_ID_MIN;
        }
        mNav[offset].mType   = GnssEphemerisType(navInfo.type);
        mNav[offset].mSource = GnssEphemerisSource(navInfo.src);
        mNav[offset].mAgeSec = navInfo.age;
   }
}

bool SystemStatusNavData::equals(const SystemStatusItemBase& peer) {
    for (uint32_t i=0; i<SV_ALL_NUM; i++) {
        if ((mNav[i].mType != ((const SystemStatusNavData&)peer).mNav[i].mType) ||
            (mNav[i].mSource != ((const SystemStatusNavData&)peer).mNav[i].mSource) ||
            (mNav[i].mAgeSec != ((const SystemStatusNavData&)peer).mNav[i].mAgeSec)) {
            return false;
        }
    }
    return true;
}

void SystemStatusNavData::dump()
{
    LOC_LOGV("NavData: u=%ld:%ld",
            mUtcTime.tv_sec, mUtcTime.tv_nsec);
    for (uint32_t i=0; i<SV_ALL_NUM; i++) {
        LOC_LOGV("i=%d type=%d src=%d age=%d",
            i, mNav[i].mType, mNav[i].mSource, mNav[i].mAgeSec);
    }
}

/******************************************************************************
 SystemStatusPositionFailure
******************************************************************************/
SystemStatusPositionFailure::SystemStatusPositionFailure(const GnssEngineDebugDataInfo& info) :
    mFixInfoMask(info.fixStatusMask),
    mHepeLimit(info.fixHepeLimit)
{
}

bool SystemStatusPositionFailure::equals(const SystemStatusItemBase& peer) {
    if ((mFixInfoMask != ((const SystemStatusPositionFailure&)peer).mFixInfoMask) ||
        (mHepeLimit != ((const SystemStatusPositionFailure&)peer).mHepeLimit)) {
        return false;
    }
    return true;
}

void SystemStatusPositionFailure::dump()
{
    LOC_LOGV("PositionFailure: u=%ld:%ld m=%d h=%d",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mFixInfoMask,
             mHepeLimit);
}

/******************************************************************************
 SystemStatusLocation
******************************************************************************/
bool SystemStatusLocation::equals(const SystemStatusItemBase& peer) {
    if ((mLocation.gpsLocation.latitude !=
                ((const SystemStatusLocation&)peer).mLocation.gpsLocation.latitude) ||
        (mLocation.gpsLocation.longitude !=
                ((const SystemStatusLocation&)peer).mLocation.gpsLocation.longitude) ||
        (mLocation.gpsLocation.altitude !=
                ((const SystemStatusLocation&)peer).mLocation.gpsLocation.altitude)) {
        return false;
    }
    return true;
}

void SystemStatusLocation::dump()
{
    LOC_LOGV("Location: lat=%f lon=%f alt=%f spd=%f",
             mLocation.gpsLocation.latitude,
             mLocation.gpsLocation.longitude,
             mLocation.gpsLocation.altitude,
             mLocation.gpsLocation.speed);
}

/******************************************************************************
 SystemStatus
******************************************************************************/
pthread_mutex_t   SystemStatus::mMutexSystemStatus = PTHREAD_MUTEX_INITIALIZER;
SystemStatus*     SystemStatus::mInstance = NULL;

SystemStatus* SystemStatus::getInstance(const MsgTask* msgTask)
{
    pthread_mutex_lock(&mMutexSystemStatus);

    if (!mInstance) {
        // Instantiating for the first time. msgTask should not be NULL
        if (msgTask == NULL) {
            LOC_LOGE("SystemStatus: msgTask is NULL!!");
            pthread_mutex_unlock(&mMutexSystemStatus);
            return NULL;
        }
        mInstance = new (nothrow) SystemStatus(msgTask);
        LOC_LOGD("SystemStatus::getInstance:%p. Msgtask:%p", mInstance, msgTask);
    }

    pthread_mutex_unlock(&mMutexSystemStatus);
    return mInstance;
}

void SystemStatus::destroyInstance()
{
    delete mInstance;
    mInstance = NULL;
}

void SystemStatus::resetNetworkInfo() {
    for (int i=0; i<mCache.mNetworkInfo.size(); ++i) {
        // Reset all the cached NetworkInfo Items as disconnected
        eventConnectionStatus(false, mCache.mNetworkInfo[i].mDataItem.mType,
                mCache.mNetworkInfo[i].mDataItem.mRoaming,
                mCache.mNetworkInfo[i].mDataItem.mNetworkHandle,
                mCache.mNetworkInfo[i].mDataItem.mApn);
    }
}

IOsObserver* SystemStatus::getOsObserver()
{
    return &mSysStatusObsvr;
}

SystemStatus::SystemStatus(const MsgTask* msgTask) :
    mSysStatusObsvr(this, msgTask), mTracking(false) {
    int result = 0;
    ENTRY_LOG ();
    mCache.mLocation.clear();

    mCache.mTimeAndClock.clear();
    mCache.mXoState.clear();
    mCache.mRfAndParams.clear();
    mCache.mErrRecovery.clear();

    mCache.mInjectedPosition.clear();
    mCache.mBestPosition.clear();
    mCache.mXtra.clear();
    mCache.mEphemeris.clear();
    mCache.mSvHealth.clear();
    mCache.mPdr.clear();
    mCache.mNavData.clear();

    mCache.mPositionFailure.clear();

    mCache.mENH.clear();
    mCache.mGPSState.clear();
    mCache.mWifiHardwareState.clear();
    mCache.mNetworkInfo.clear();
    mCache.mRilServiceInfo.clear();
    mCache.mRilCellInfo.clear();
    mCache.mServiceStatus.clear();
    mCache.mModel.clear();
    mCache.mManufacturer.clear();
    mCache.mPowerConnectState.clear();
    mCache.mTimeZoneChange.clear();
    mCache.mTimeChange.clear();
    mCache.mWifiSupplicantStatus.clear();
    mCache.mMccMnc.clear();

    EXIT_LOG_WITH_ERROR ("%d",result);
}

/******************************************************************************
 SystemStatus - storing dataitems
******************************************************************************/
template <typename TYPE_REPORT, typename TYPE_ITEM>
bool SystemStatus::setIteminReport(TYPE_REPORT& report, TYPE_ITEM&& s)
{
    if (s.ignore()) {
        return false;
    }
    if (!report.empty() && report.back().equals(static_cast<TYPE_ITEM&>(s.collate(report.back())))) {
        // there is no change - just update reported timestamp
        report.back().mUtcReported = s.mUtcReported;
        return false;
    }

    // first event or updated
    report.push_back(s);
    if (report.size() > s.maxItem) {
        report.erase(report.begin());
    }
    return true;
}

template <typename TYPE_REPORT, typename TYPE_ITEM>
void SystemStatus::setDefaultIteminReport(TYPE_REPORT& report, const TYPE_ITEM& s)
{
    report.push_back(s);
    if (report.size() > s.maxItem) {
        report.erase(report.begin());
    }
}

template <typename TYPE_REPORT, typename TYPE_ITEM>
void SystemStatus::getIteminReport(TYPE_REPORT& reportout, const TYPE_ITEM& c) const
{
    reportout.clear();
    if (c.size() >= 1) {
        reportout.push_back(c.back());
        reportout.back().dump();
    }
}

void SystemStatus::setEngineDebugDataInfo(const GnssEngineDebugDataInfo& gnssEngineDebugDataInfo) {
    pthread_mutex_lock(&mMutexSystemStatus);
    LOC_LOGd("setEngine data");
    setIteminReport(mCache.mTimeAndClock, SystemStatusTimeAndClock(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mXoState, SystemStatusXoState(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mRfAndParams, SystemStatusRfAndParams(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mErrRecovery, SystemStatusErrRecovery(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mInjectedPosition,
                    SystemStatusInjectedPosition(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mBestPosition, SystemStatusBestPosition(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mXtra, SystemStatusXtra(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mEphemeris, SystemStatusEphemeris(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mSvHealth, SystemStatusSvHealth(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mPdr, SystemStatusPdr(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mNavData, SystemStatusNavData(gnssEngineDebugDataInfo));
    setIteminReport(mCache.mPositionFailure, SystemStatusPositionFailure(gnssEngineDebugDataInfo));
    pthread_mutex_unlock(&mMutexSystemStatus);
}


/******************************************************************************
@brief      API to set report position data into internal buffer

@param[In]  UlpLocation

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventPosition(const UlpLocation& location,
                                 const GpsLocationExtended& locationEx)
{
    bool ret = false;
    pthread_mutex_lock(&mMutexSystemStatus);

    ret = setIteminReport(mCache.mLocation, SystemStatusLocation(location, locationEx));
    LOC_LOGV("eventPosition - lat=%f lon=%f alt=%f speed=%f",
             location.gpsLocation.latitude,
             location.gpsLocation.longitude,
             location.gpsLocation.altitude,
             location.gpsLocation.speed);

    pthread_mutex_unlock(&mMutexSystemStatus);
    return ret;
}

/******************************************************************************
@brief      API to set report DataItem event into internal buffer

@param[In]  DataItem

@return     true when info is updatated
******************************************************************************/
bool SystemStatus::eventDataItemNotify(IDataItemCore* dataitem)
{
    bool ret = false;
    pthread_mutex_lock(&mMutexSystemStatus);
    switch(dataitem->getId())
    {
        case ENH_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mENH,
                    SystemStatusENH(*(static_cast<ENHDataItem*>(dataitem))));
            break;
        case GPSSTATE_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mGPSState,
                    SystemStatusGpsState(*(static_cast<GPSStateDataItem*>(dataitem))));
            break;
        case WIFIHARDWARESTATE_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mWifiHardwareState, SystemStatusWifiHardwareState(
                        *(static_cast<WifiHardwareStateDataItem*>(dataitem))));
            break;
        case NETWORKINFO_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mNetworkInfo,
                    SystemStatusNetworkInfo(*(static_cast<NetworkInfoDataItem*>(dataitem))));
            // Update latest mAllTypes/mAllNetworkHandles of original obj to notify clients
            if (ret) {
                (static_cast<NetworkInfoDataItem*>(dataitem))->mAllTypes =
                        mCache.mNetworkInfo.back().mDataItem.mAllTypes;
                memcpy((static_cast<NetworkInfoDataItem*>(dataitem))->mAllNetworkHandles,
                        mCache.mNetworkInfo.back().mDataItem.mAllNetworkHandles, sizeof((
                        static_cast<NetworkInfoDataItem*>(dataitem))->mAllNetworkHandles));
            }
            break;
        case RILSERVICEINFO_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mRilServiceInfo,
                    SystemStatusServiceInfo(*(static_cast<RilServiceInfoDataItem*>(dataitem))));
            break;
        case RILCELLINFO_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mRilCellInfo,
                    SystemStatusRilCellInfo(*(static_cast<RilCellInfoDataItem*>(dataitem))));
            break;
        case SERVICESTATUS_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mServiceStatus,
                    SystemStatusServiceStatus(*(static_cast<ServiceStatusDataItem*>(dataitem))));
            break;
        case MODEL_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mModel,
                    SystemStatusModel(*(static_cast<ModelDataItem*>(dataitem))));
            break;
        case MANUFACTURER_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mManufacturer,
                    SystemStatusManufacturer(*(static_cast<ManufacturerDataItem*>(dataitem))));
            break;
        case IN_EMERGENCY_CALL_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mInEmergencyCall,
                    SystemStatusInEmergencyCall(
                        *(static_cast<InEmergencyCallDataItem*>(dataitem))));
            break;
        case POWER_CONNECTED_STATE_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mPowerConnectState, SystemStatusPowerConnectState(
                        *(static_cast<PowerConnectStateDataItem*>(dataitem))));
            break;
        case TIMEZONE_CHANGE_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mTimeZoneChange,
                    SystemStatusTimeZoneChange(*(static_cast<TimeZoneChangeDataItem*>(dataitem))));
            break;
        case TIME_CHANGE_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mTimeChange,
                    SystemStatusTimeChange(*(static_cast<TimeChangeDataItem*>(dataitem))));
            break;
        case WIFI_SUPPLICANT_STATUS_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mWifiSupplicantStatus, SystemStatusWifiSupplicantStatus(
                        *(static_cast<WifiSupplicantStatusDataItem*>(dataitem))));
            break;
        case MCCMNC_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mMccMnc,
                    SystemStatusMccMnc(*(static_cast<MccmncDataItem*>(dataitem))));
            break;
        case TRACKING_STARTED_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mTrackingStarted,
                    SystemStatusTrackingStarted(
                        *(static_cast<TrackingStartedDataItem*>(dataitem))));
            break;
        case NTRIP_STARTED_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mNtripStarted,
                    SystemStatusNtripStarted(
                        *(static_cast<NtripStartedDataItem*>(dataitem))));
            break;
        case PRECISE_LOCATION_ENABLED_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mPreciseLocationEnabled,
                    SystemStatusPreciseLocationEnabled(
                        *(static_cast<PreciseLocationEnabledDataItem*>(dataitem))));
            break;
        case LOC_FEATURE_STATUS_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mLocFeatureStatus,
                    SystemStatusLocFeatureStatus(
                        *(static_cast<LocFeatureStatusDataItem*>(dataitem))));
            break;
        case NETWORK_POSITIONING_STARTED_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mNlpSessionStarted,
                    SystemStatusNlpSessionStarted(
                        *(static_cast<NlpSessionStartedDataItem*>(dataitem))));
            break;
        case QESDK_WWAN_FEATURE_STATUS_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mQesdkWwanFeatureStatus,
                    SystemStatusQesdkWwanFeatureStatus(
                        *(static_cast<QesdkWwanFeatureStatusDataItem*>(dataitem))));
            break;
        case QESDK_WWAN_CS_CONSENT_SRC_DATA_ITEM_ID:
            ret = setIteminReport(mCache.mQesdkWwanCsConsentSrc,
                    SystemStatusQesdkWwanCsConsentSrc(
                        *(static_cast<QesdkWwanCsConsentSrcDataItem*>(dataitem))));
            break;
        default:
            break;
    }
    pthread_mutex_unlock(&mMutexSystemStatus);
    LOC_LOGv("DataItemId: %d, whether to record dateitem in cache: %d", dataitem->getId(), ret);
    return ret;
}

/******************************************************************************
@brief      API to get report data into a given buffer

@param[In]  reference to report buffer
@param[In]  bool flag to identify latest only or entire buffer

@return     true when successfully done
******************************************************************************/
bool SystemStatus::getReport(SystemStatusReports& report, bool isLatestOnly,
        bool inSessionOnly) const {
    pthread_mutex_lock(&mMutexSystemStatus);
    if (inSessionOnly && !mTracking) {
        pthread_mutex_unlock(&mMutexSystemStatus);
        return true;
    }

    if (isLatestOnly) {
        // push back only the latest report and return it
        getIteminReport(report.mLocation, mCache.mLocation);

        getIteminReport(report.mTimeAndClock, mCache.mTimeAndClock);
        getIteminReport(report.mXoState, mCache.mXoState);
        getIteminReport(report.mRfAndParams, mCache.mRfAndParams);
        getIteminReport(report.mErrRecovery, mCache.mErrRecovery);

        getIteminReport(report.mInjectedPosition, mCache.mInjectedPosition);
        getIteminReport(report.mBestPosition, mCache.mBestPosition);
        getIteminReport(report.mXtra, mCache.mXtra);
        getIteminReport(report.mEphemeris, mCache.mEphemeris);
        getIteminReport(report.mSvHealth, mCache.mSvHealth);
        getIteminReport(report.mPdr, mCache.mPdr);
        getIteminReport(report.mNavData, mCache.mNavData);

        getIteminReport(report.mPositionFailure, mCache.mPositionFailure);

        getIteminReport(report.mENH, mCache.mENH);
        getIteminReport(report.mGPSState, mCache.mGPSState);
        getIteminReport(report.mWifiHardwareState, mCache.mWifiHardwareState);
        getIteminReport(report.mNetworkInfo, mCache.mNetworkInfo);
        getIteminReport(report.mRilServiceInfo, mCache.mRilServiceInfo);
        getIteminReport(report.mRilCellInfo, mCache.mRilCellInfo);
        getIteminReport(report.mServiceStatus, mCache.mServiceStatus);
        getIteminReport(report.mModel, mCache.mModel);
        getIteminReport(report.mManufacturer, mCache.mManufacturer);
        getIteminReport(report.mPowerConnectState, mCache.mPowerConnectState);
        getIteminReport(report.mTimeZoneChange, mCache.mTimeZoneChange);
        getIteminReport(report.mTimeChange, mCache.mTimeChange);
        getIteminReport(report.mWifiSupplicantStatus, mCache.mWifiSupplicantStatus);
        getIteminReport(report.mMccMnc, mCache.mMccMnc);
    }
    else {
        // copy entire reports and return them
        report.mLocation.clear();

        report.mTimeAndClock.clear();
        report.mXoState.clear();
        report.mRfAndParams.clear();
        report.mErrRecovery.clear();

        report.mInjectedPosition.clear();
        report.mBestPosition.clear();
        report.mXtra.clear();
        report.mEphemeris.clear();
        report.mSvHealth.clear();
        report.mPdr.clear();
        report.mNavData.clear();

        report.mPositionFailure.clear();

        report.mENH.clear();
        report.mGPSState.clear();
        report.mWifiHardwareState.clear();
        report.mNetworkInfo.clear();
        report.mRilServiceInfo.clear();
        report.mRilCellInfo.clear();
        report.mServiceStatus.clear();
        report.mModel.clear();
        report.mManufacturer.clear();
        report.mPowerConnectState.clear();
        report.mTimeZoneChange.clear();
        report.mTimeChange.clear();
        report.mWifiSupplicantStatus.clear();
        report.mMccMnc.clear();

        report = mCache;
    }

    pthread_mutex_unlock(&mMutexSystemStatus);
    return true;
}

/******************************************************************************
@brief      API to set default report data

@param[In]  none

@return     true when successfully done
******************************************************************************/
bool SystemStatus::setDefaultGnssEngineStates(void)
{
    pthread_mutex_lock(&mMutexSystemStatus);

    setDefaultIteminReport(mCache.mLocation, SystemStatusLocation());

    setDefaultIteminReport(mCache.mTimeAndClock, SystemStatusTimeAndClock());
    setDefaultIteminReport(mCache.mXoState, SystemStatusXoState());
    setDefaultIteminReport(mCache.mRfAndParams, SystemStatusRfAndParams());
    setDefaultIteminReport(mCache.mErrRecovery, SystemStatusErrRecovery());

    setDefaultIteminReport(mCache.mInjectedPosition, SystemStatusInjectedPosition());
    setDefaultIteminReport(mCache.mBestPosition, SystemStatusBestPosition());
    setDefaultIteminReport(mCache.mXtra, SystemStatusXtra());
    setDefaultIteminReport(mCache.mEphemeris, SystemStatusEphemeris());
    setDefaultIteminReport(mCache.mSvHealth, SystemStatusSvHealth());
    setDefaultIteminReport(mCache.mPdr, SystemStatusPdr());
    setDefaultIteminReport(mCache.mNavData, SystemStatusNavData());

    setDefaultIteminReport(mCache.mPositionFailure, SystemStatusPositionFailure());

    pthread_mutex_unlock(&mMutexSystemStatus);
    return true;
}

/******************************************************************************
@brief      API to handle connection status update event from GnssRil

@param[In]  Connection status

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventConnectionStatus(bool connected, int8_t type,
                                         bool roaming, NetworkHandle networkHandle,
                                         const string& apn)
{
    // send networkinof dataitem to systemstatus observer clients
    SystemStatusNetworkInfo s(type, "", "", connected, roaming,
                              (uint64_t) networkHandle, apn);
    mSysStatusObsvr.notify({&s.mDataItem});

    return true;
}

/******************************************************************************
@brief      API to update power connect state

@param[In]  power connect status

@return     true when successfully done
******************************************************************************/
bool SystemStatus::updatePowerConnectState(bool charging)
{
    SystemStatusPowerConnectState s(charging);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update ENH

@param[In]  user consent

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventOptInStatus(bool userConsent)
{
    SystemStatusENH s(userConsent, ENHDataItem::FIELD_CONSENT);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update Region

@param[In]  region

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventRegionStatus(bool region)
{
    SystemStatusENH s(region, ENHDataItem::FIELD_REGION);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to notify emergency call

@param[In]  is emergency call

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventInEmergencyCall(bool isEmergency)
{
    SystemStatusInEmergencyCall s(isEmergency);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update precise location state

@param[In]  precise Location state

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventPreciseLocation(bool preciseLocation) {
    SystemStatusPreciseLocationEnabled s(preciseLocation);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update Ntrip started state

@param[In]  Ntrip started state

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventNtripStarted(bool ntripStarted) {
    SystemStatusNtripStarted s(ntripStarted);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update engine tracking state

@param[In]  tracking state

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventSetTracking(bool tracking, bool updateSysStatusTrkState) {
    pthread_mutex_lock(&mMutexSystemStatus);
    if (updateSysStatusTrkState) {
        mTracking = tracking;
    }
    SystemStatusTrackingStarted s(tracking);
    mSysStatusObsvr.notify({&s.mDataItem});
    pthread_mutex_unlock(&mMutexSystemStatus);
    return true;
}

/******************************************************************************
@brief      API to update Location feature QWES status

@param[In]  Location feature QWES status

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventLocFeatureStatus(std::unordered_set<int> fids) {
    SystemStatusLocFeatureStatus  s(fids);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}
/******************************************************************************
@brief      API to update network positioning session state

@param[In]  session state

@return     true when successfully done
******************************************************************************/
bool SystemStatus::eventNlpSessionStatus(bool nlpStarted) {
    SystemStatusNlpSessionStarted s(nlpStarted);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

/******************************************************************************
@brief      API to update gps enable state

@param[In]  enable state

@return     true when successfully done
******************************************************************************/

bool SystemStatus::eventGpsEnabled(bool gpsEnabled) {
    SystemStatusGpsState  s(gpsEnabled);
    mSysStatusObsvr.notify({&s.mDataItem});
    return true;
}

} // namespace loc_core

