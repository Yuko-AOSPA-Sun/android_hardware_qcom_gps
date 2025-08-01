/* Copyright (c) 2017-2021 The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation nor the names of its
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
 */

/*
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef LOCATION_INTERFACE_H
#define LOCATION_INTERFACE_H

#include <LocationAPI.h>
#include <gps_extended_c.h>
#include <functional>

/* Used for callback to deliver GNSS energy consumed */
/** @fn
    @brief Used by query API that retrieves energy consumed by
           modem GNSS engine.

    @param gnssEnergyConsumedFromFirstBoot:
            Energy consumed by the GNSS engine since the first bootup
            in units of 0.1 milli watt seconds.
            A value of 0xffffffffffffffff indicates an invalid reading.
*/
typedef std::function<void(
    uint64_t gnssEnergyConsumedFromFirstBoot
)> GnssEnergyConsumedCallback;

typedef void (*removeClientCompleteCallback)(LocationAPI* client);

struct GnssInterface {
    size_t size;
    void (*initialize)(void);
    void (*deinitialize)(void);
    void (*addClient)(LocationAPI* client, const LocationCallbacks& callbacks);
    void (*removeClient)(LocationAPI* client, removeClientCompleteCallback rmClientCb);
    void (*requestCapabilities)(LocationAPI* client);
    uint32_t (*startTracking)(LocationAPI* client, const TrackingOptions&);
    void (*updateTrackingOptions)(LocationAPI* client, uint32_t id, const TrackingOptions&);
    void (*stopTracking)(LocationAPI* client, uint32_t id);
    void (*gnssNiResponse)(LocationAPI* client, uint32_t id, GnssNiResponse response);
    void (*setControlCallbacks)(LocationControlCallbacks& controlCallbacks);
    uint32_t (*enable)(LocationTechnologyType techType);
    void (*disable)(uint32_t id);
    uint32_t* (*gnssUpdateConfig)(const GnssConfig& config);
    uint32_t* (*gnssGetConfig)(GnssConfigFlagsMask config);
    void (*gnssUpdateSvTypeConfig)(const GnssSvTypeConfig& config);
    void (*gnssGetSvTypeConfig)(GnssSvTypeConfigCallback& callback);
    void (*gnssResetSvTypeConfig)();
    uint32_t (*gnssDeleteAidingData)(const GnssAidingData& data);
    void (*gnssUpdateXtraThrottle)(const bool enabled);
    void (*injectLocation)(double latitude, double longitude, float accuracy);
    void (*injectTime)(int64_t time, int64_t timeReference, int32_t uncertainty);
    void (*agpsInit)(const AgpsCbInfo& cbInfo);
    void (*agpsDataConnOpen)(AGpsExtType agpsType, const char* apnName, int apnLen, int ipType);
    void (*agpsDataConnClosed)(AGpsExtType agpsType);
    void (*agpsDataConnFailed)(AGpsExtType agpsType);
    void (*getDebugReport)(GnssDebugReport& report);
    void (*updateConnectionStatus)(bool connected, int8_t type, bool roaming,
                                   NetworkHandle networkHandle, const std::string& apn);
    void (*odcpiInit)(const odcpiRequestCallback& callback, OdcpiPrioritytype priority,
           OdcpiCallbackTypeMask typeMask);
    void (*deRegisterOdcpi)(OdcpiPrioritytype priority, OdcpiCallbackTypeMask typeMask);
    void (*odcpiInject)(const Location& location);
    void (*blockCPI)(double latitude, double longitude, float accuracy,
                     int blockDurationMsec, double latLonDiffThreshold);
    void (*setEsStatusCallback)(std::function<void(bool)> esStatusCb);
    void (*getGnssEnergyConsumed)(GnssEnergyConsumedCallback energyConsumedCb);
    void (*enableNfwLocationAccess)(const std::vector<std::string>& enabledNfws);
    void (*nfwInit)(const NfwCbInfo& cbInfo);
    void (*getPowerStateChanges)(std::function<void(bool)> powerStateCb);
    void (*injectLocationExt)(const GnssLocationInfoNotification &locationInfo);
    void (*updateBatteryStatus)(bool charging);
    void (*updateSystemPowerState)(PowerStateType systemPowerState);
    uint32_t (*setConstrainedTunc) (bool enable, float tuncConstraint, uint32_t energyBudget);
    uint32_t (*setPositionAssistedClockEstimator) (bool enable);
    uint32_t (*gnssUpdateSvConfig)(const GnssSvTypeConfig& constellationEnablementConfig,
                                   const GnssSvIdConfig&   blacklistSvConfig);
    uint32_t (*configLeverArm)(const LeverArmConfigInfo& configInfo);
    bool (*measCorrInit)(const measCorrSetCapabilitiesCallback setCapabilitiesCb);
    bool (*measCorrSetCorrections)(const GnssMeasurementCorrections& gnssMeasCorr);
    void (*measCorrClose)();
    uint32_t (*getAntennaInfo)(AntennaInfoCallback* antennaInfoCallback);
    uint32_t (*configRobustLocation)(bool enable, bool enableForE911);
    uint32_t (*configMinGpsWeek)(uint16_t minGpsWeek);
    uint32_t (*configDeadReckoningEngineParams)(const DeadReckoningEngineConfig& dreConfig);
    void (*updateNTRIPGGAConsent)(bool consentAccepted);
    void (*enablePPENtripStream)(const GnssNtripConnectionParams& params, bool enableRTKEngine);
    void (*disablePPENtripStream)();
    uint32_t (*gnssUpdateSecondaryBandConfig)(const GnssSvTypeConfig& secondaryBandConfig);
    uint32_t (*gnssGetSecondaryBandConfig)();
    void (*resetNetworkInfo)();
    uint32_t (*configEngineRunState)(PositioningEngineMask engType,
                                     LocEngineRunState engState);
    uint32_t (*configOutputNmeaTypes)(GnssNmeaTypesMask enabledNmeaTypes,
                                      GnssGeodeticDatumType nmeaDatumType,
                                      LocReqEngineTypeMask locReqEngTypeMask);
    void (*powerIndicationInit)(const powerIndicationCb powerIndicationCallback);
    void (*powerIndicationRequest)();
    void (*setAddressRequestCb)(std::function<void(const Location&)> addressRequestCb);
    void (*injectLocationAndAddr)(const Location& location, const GnssCivicAddress& addr);
    uint32_t (*setOptInStatus)(bool userConsent);
    uint32_t (*configEngineIntegrityRisk)(PositioningEngineMask engineType, uint32_t integrityRisk);
    uint32_t (*configXtraParams) (bool enable, const XtraConfigParams& configParams);
    uint32_t (*gnssGetXtraStatus)();
    uint32_t (*gnssRegisterXtraStatusUpdate)(bool registerUpdate);
    void (*configPrecisePositioning)(uint32_t featureId, bool enable, const std::string& appHash);
    uint32_t (*configMerkleTree) (const char * merkleTreeConfigBuffer, int bufferLength);
    uint32_t (*configOsnmaEnablement) (bool enable);
    uint32_t (*gnssInjectMmfData) (const GnssMapMatchedData& data);
    uint32_t (*configureUserConsentForXtra) (const bool xtraUserConsent);
    void (*set3rdPartyNtnCapability)(bool isCapable);
    void (*getNtnConfigSignalMask)();
    void (*setNtnConfigSignalMask)(GnssSignalTypeMask gpsSignalTypeConfigMask);
};

struct BatchingInterface {
    size_t size;
    void (*initialize)(void);
    void (*deinitialize)(void);
    void (*addClient)(LocationAPI* client, const LocationCallbacks& callbacks);
    void (*removeClient)(LocationAPI* client, removeClientCompleteCallback rmClientCb);
    void (*requestCapabilities)(LocationAPI* client);
    uint32_t (*startBatching)(LocationAPI* client, const BatchingOptions&);
    void (*stopBatching)(LocationAPI* client, uint32_t id);
    void (*updateBatchingOptions)(LocationAPI* client, uint32_t id, const BatchingOptions&);
    void (*getBatchedLocations)(LocationAPI* client, uint32_t id, size_t count);
    int32_t (*getBatchSize)(LocationAPI* client);
    void (*updateSystemPowerState)(PowerStateType systemPowerState);
};

struct GeofenceInterface {
    size_t size;
    void (*initialize)(void);
    void (*deinitialize)(void);
    void (*addClient)(LocationAPI* client, const LocationCallbacks& callbacks);
    void (*removeClient)(LocationAPI* client, removeClientCompleteCallback rmClientCb);
    void (*requestCapabilities)(LocationAPI* client);
    uint32_t* (*addGeofences)(LocationAPI* client, size_t count, GeofenceOption*, GeofenceInfo*);
    void (*removeGeofences)(LocationAPI* client, size_t count, uint32_t* ids);
    void (*modifyGeofences)(LocationAPI* client, size_t count, uint32_t* ids,
                            GeofenceOption* options);
    void (*pauseGeofences)(LocationAPI* client, size_t count, uint32_t* ids);
    void (*resumeGeofences)(LocationAPI* client, size_t count, uint32_t* ids);
    void (*updateSystemPowerState)(PowerStateType systemPowerState);
};

#endif /* LOCATION_INTERFACE_H */
