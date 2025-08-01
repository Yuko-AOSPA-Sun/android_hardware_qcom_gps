/* Copyright (c) 2017-2019, 2021, The Linux Foundation. All rights reserved.
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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef BATCHING_ADAPTER_H
#define BATCHING_ADAPTER_H

#include <LocAdapterBase.h>
#include <LocContext.h>
#include <LocationAPI.h>
#include <map>

using namespace loc_core;

class BatchingAdapter : public LocAdapterBase {

    /* ==== BATCHING ======================================================================= */
    typedef struct {
        uint32_t accumulatedDistanceOngoingBatch;
        uint32_t accumulatedDistanceThisTrip;
        uint32_t accumulatedDistanceOnTripRestart;
        uint32_t tripDistance;
        uint32_t tripTBFInterval;
    } TripSessionStatus;
    typedef std::map<uint32_t, TripSessionStatus> TripSessionStatusMap;
    typedef std::map<LocationSessionKey, BatchingOptions> BatchingSessionMap;

    BatchingSessionMap mBatchingSessions;
    TripSessionStatusMap mTripSessions;
    uint32_t mOngoingTripDistance;
    uint32_t mOngoingTripTBFInterval;
    bool mTripWithOngoingTBFDropped;
    bool mTripWithOngoingTripDistanceDropped;
    PowerStateType mSystemPowerState;

    void startTripBatchingMultiplex(LocationAPI* client, uint32_t sessionId,
                                    const BatchingOptions& batchingOptions);
    void stopTripBatchingMultiplex(LocationAPI* client, uint32_t sessionId,
                                   bool restartNeeded,
                                   const BatchingOptions& batchOptions,
                                   bool eraseSession = true);
    inline void stopTripBatchingMultiplex(LocationAPI* client, uint32_t id,
                                             bool eraseSession = true) {
        BatchingOptions batchOptions;
        stopTripBatchingMultiplex(client, id, false, batchOptions, eraseSession);
    };
    void stopTripBatchingMultiplexCommon(LocationError err,
                                         LocationAPI* client,
                                         uint32_t sessionId,
                                         bool restartNeeded,
                                         const BatchingOptions& batchOptions,
                                         bool eraseSession = true);
    void restartTripBatching(bool queryAccumulatedDistance, uint32_t accDist = 0,
                             uint32_t numbatchedPos = 0);
    void printTripReport();

    /* ==== CONFIGURATION ================================================================== */
    uint32_t mBatchingTimeout;
    uint32_t mBatchingAccuracy;
    size_t mBatchSize;
    size_t mTripBatchSize;

protected:

    /* ==== CLIENT ========================================================================= */
    virtual void updateClientsEventMask();
    virtual void stopClientSessions(LocationAPI* client, bool eraseSession = true);

public:
    BatchingAdapter();
    virtual ~BatchingAdapter() {}

    /* ==== SSR ============================================================================ */
    /* ======== EVENTS ====(Called from QMI Thread)========================================= */
    virtual void handleEngineUpEvent();
    /* ======== UTILITIES ================================================================== */
    void restartSessions();

    /* ==== BATCHING ======================================================================= */
    /* ======== COMMANDS ====(Called from Client Thread)==================================== */
    uint32_t startBatchingCommand(LocationAPI* client, const BatchingOptions &batchOptions);
    void updateBatchingOptionsCommand(
            LocationAPI* client, uint32_t id, const BatchingOptions& batchOptions);
    void stopBatchingCommand(LocationAPI* client, uint32_t id);
    void getBatchedLocationsCommand(LocationAPI* client, uint32_t id, size_t count);
    inline int32_t getBatchSizeCommand(LocationAPI* client) { return mBatchSize; }
    void updateSystemPowerStateCommand(PowerStateType systemPowerState);
    /* ======== RESPONSES ================================================================== */
    void reportResponse(LocationAPI* client, LocationError err, uint32_t sessionId);
    /* ======== UTILITIES ================================================================== */
    bool hasBatchingCallback(LocationAPI* client);
    bool isBatchingSession(LocationAPI* client, uint32_t sessionId);
    bool isTripSession(uint32_t sessionId);
    void saveBatchingSession(LocationAPI* client, uint32_t sessionId,
                             const BatchingOptions& batchingOptions);
    void eraseBatchingSession(LocationAPI* client, uint32_t sessionId);
    uint32_t autoReportBatchingSessionsCount();
    void startBatching(LocationAPI* client, uint32_t sessionId,
                       const BatchingOptions& batchingOptions);
    void stopBatching(LocationAPI* client, uint32_t sessionId, bool restartNeeded,
                      const BatchingOptions& batchOptions, bool eraseSession = true);
    void stopBatching(LocationAPI* client, uint32_t sessionId, bool eraseSession = true) {
        BatchingOptions batchOptions;
        stopBatching(client, sessionId, false, batchOptions, eraseSession);
    };
    void suspendBatchingSessions();
    void updateSystemPowerState(PowerStateType systemPowerState);

    /* ==== REPORTS ======================================================================== */
    virtual void handleEngineLockStatusEvent(EngineLockState engineLockState);
    void handleEngineLockStatus(EngineLockState engineLockState);
    /* ======== EVENTS ====(Called from QMI Thread)========================================= */
    void reportLocationsEvent(const Location* locations, size_t count,
            BatchingMode batchingMode);
    void reportCompletedTripsEvent(uint32_t accumulatedDistance);
    void reportBatchStatusChangeEvent(BatchingStatus batchStatus);
    /* ======== UTILITIES ================================================================== */
    void reportLocations(Location* locations, size_t count, BatchingMode batchingMode);
    void reportBatchStatusChange(BatchingStatus batchStatus,
            std::list<uint32_t> & completedTripsList);

    /* ==== CONFIGURATION ================================================================== */
    /* ======== COMMANDS ====(Called from Client Thread)==================================== */
    /* ======== UTILITIES ================================================================== */
    void setBatchSize(size_t batchSize) { mBatchSize = batchSize; }
    size_t getBatchSize() { return mBatchSize; }
    void setTripBatchSize(size_t batchSize) { mTripBatchSize = batchSize; }
    size_t getTripBatchSize() { return mTripBatchSize; }
    void setBatchingTimeout(uint32_t batchingTimeout) { mBatchingTimeout = batchingTimeout; }
    uint32_t getBatchingTimeout() { return mBatchingTimeout; }
    void setBatchingAccuracy(uint32_t accuracy) { mBatchingAccuracy = accuracy; }
    uint32_t getBatchingAccuracy() { return mBatchingAccuracy; }
};

#endif /* BATCHING_ADAPTER_H */
