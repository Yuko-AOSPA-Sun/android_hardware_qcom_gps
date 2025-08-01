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

#ifndef XTRA_SYSTEM_STATUS_OBS_H
#define XTRA_SYSTEM_STATUS_OBS_H

#include <cinttypes>
#include <MsgTask.h>
#include <LocIpc.h>
#include <LocTimer.h>
#include <stdlib.h>

using namespace std;
using namespace loc_util;
using loc_core::IOsObserver;
using loc_core::IDataItemObserver;
using loc_core::IDataItemCore;

struct StartDgnssNtripParams {
    GnssNtripConnectionParams ntripParams;
    string                    nmea;
    bool enableRTKEngine;

    void clear() {
        ntripParams.hostNameOrIp.clear();
        ntripParams.mountPoint.clear();
        ntripParams.username.clear();
        ntripParams.password.clear();
        ntripParams.port = 0;
        ntripParams.useSSL = false;
        ntripParams.requiresNmeaLocation = false;
        ntripParams.nmeaUpdateInterval = 0;
        enableRTKEngine = false;
        nmea.clear();
    }
};

class GnssAdapter;
class XtraIpcListener;

class XtraSystemStatusObserver : public IDataItemObserver {
    friend XtraIpcListener;

public :
    // constructor & destructor
    XtraSystemStatusObserver(GnssAdapter* adapter, IOsObserver* sysStatObs, const MsgTask* msgTask);
    inline virtual ~XtraSystemStatusObserver() {
        subscribe(false);
        mIpc.stopNonBlockingListening();
    }

    void init();
    // IDataItemObserver overrides
    inline virtual void getName(string& name);
    virtual void notify(const unordered_set<IDataItemCore*>& dlist);

    bool updateLockStatus(GnssConfigGpsLock lock);
    bool updateConnections(uint64_t allConnections,
            loc_core::NetworkInfoType* networkHandleInfo, bool roaming);
    bool updateMccMnc(const string& mccmncCountry);
    bool updateXtraThrottle(const bool enabled);
    bool updatePowerState(const PowerStateType powerState);
    inline const MsgTask* getMsgTask() { return mMsgTask; }
    void subscribe(bool yes);
    bool notifySessionStart();
    bool onStatusRequested(int32_t xtraStatusUpdated);
    void startDgnssSource(const StartDgnssNtripParams& params);
    void restartDgnssSource();
    void stopDgnssSource();
    void updateNmeaToDgnssServer(const string& nmea);

    bool updateXtraConfig(bool enabled, const XtraConfigParams& configParams);
    bool getXtraStatus(uint32_t sessionId);
    bool registerXtraStatusUpdate(uint32_t sessionId, bool registerUpdate);
    bool updateXtraDataDeletion();
    bool updateXtraUserConsent(bool userConsent);
    bool set3rdPartyNtnCapability(bool enabled);

private:
    GnssAdapter*   mAdapter;
    IOsObserver*   mSystemStatusObsrvr;
    const MsgTask* mMsgTask;
    GnssConfigGpsLock mGpsLock;
    LocIpc mIpc;
    uint64_t mConnections;
    loc_core::NetworkInfoType mNetworkHandle[MAX_NETWORK_HANDLES];
    bool mRoaming;
    string mTac;
    string mMccmnc;
    bool mXtraThrottle;
    PowerStateType mPowerState;
    bool mReqStatusReceived;
    bool mIsConnectivityStatusKnown;
    shared_ptr<LocIpcSender> mXtraSender;
    shared_ptr<LocIpcSender> mDgnssSender;
    string mNtripParamsString;
    bool mRegisterForXtraStatus;

    class DelayLocTimer : public LocTimer {
        LocIpcSender& mXtraSender;
        LocIpcSender& mDgnssSender;
    public:
        DelayLocTimer(LocIpcSender& xtraSender, LocIpcSender& dgnssSender) :
                mXtraSender(xtraSender), mDgnssSender(dgnssSender) {}
        void timeOutCallback() override {
            LocIpc::send(mXtraSender, (const uint8_t*)"halinit", sizeof("halinit"));
            LocIpc::send(mDgnssSender, (const uint8_t*)"halinit", sizeof("halinit"));
        }
    } mDelayLocTimer;
};

#endif
