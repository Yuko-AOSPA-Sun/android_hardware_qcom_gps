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
 SystemStatusNmeaBase - base class for all NMEA parsers
******************************************************************************/
class SystemStatusNmeaBase
{
protected:
    std::vector<std::string> mField;

    SystemStatusNmeaBase(const char *str_in, uint32_t len_in)
    {
        // check size and talker
        if (!loc_nmea_is_debug(str_in, len_in)) {
            return;
        }

        std::string parser(str_in);
        std::string::size_type index = 0;

        // verify checksum field
        index = parser.find("*");
        if (index == std::string::npos) {
            return;
        }
        parser[index] = ',';

        // tokenize parser
        while (1) {
            std::string str;
            index = parser.find(",");
            if (index == std::string::npos) {
                break;
            }
            str = parser.substr(0, index);
            parser = parser.substr(index + 1);
            mField.push_back(str);
        }
    }

    virtual ~SystemStatusNmeaBase() { }

public:
    static const uint32_t NMEA_MINSIZE = DEBUG_NMEA_MINSIZE;
    static const uint32_t NMEA_MAXSIZE = DEBUG_NMEA_MAXSIZE;
};

/******************************************************************************
 SystemStatusPQWM1
******************************************************************************/

class SystemStatusPQWM1
{
public:
    uint16_t mGpsWeek;    // x1
    uint32_t mGpsTowMs;   // x2
    uint8_t  mTimeValid;  // x3
    uint8_t  mTimeSource; // x4
    int32_t  mTimeUnc;    // x5
    int32_t  mClockFreqBias; // x6
    int32_t  mClockFreqBiasUnc; // x7
    uint8_t  mXoState;    // x8
    int32_t  mPgaGain;    // x9
    uint32_t mGpsBpAmpI;  // xA
    uint32_t mGpsBpAmpQ;  // xB
    uint32_t mAdcI;       // xC
    uint32_t mAdcQ;       // xD
    uint32_t mJammerGps;  // xE
    uint32_t mJammerGlo;  // xF
    uint32_t mJammerBds;  // x10
    uint32_t mJammerGal;  // x11
    uint32_t mRecErrorRecovery; // x12
    int32_t  mLeapSeconds;// x17
    int32_t  mLeapSecUnc; // x18
    uint32_t mGloBpAmpI;  // x19
    uint32_t mGloBpAmpQ;  // x1A
    uint32_t mBdsBpAmpI;  // x1B
    uint32_t mBdsBpAmpQ;  // x1C
    uint32_t mGalBpAmpI;  // x1D
    uint32_t mGalBpAmpQ;  // x1E
    uint64_t mTimeUncNs;  // x1F
    uint32_t mJammedSignalsMask;
    std::vector<int32_t> mJammerInd;
    SystemStatusPQWM1(const GnssEngineDebugDataInfo& info);
    SystemStatusPQWM1() = default;
};

SystemStatusPQWM1::SystemStatusPQWM1(const GnssEngineDebugDataInfo& info) {
    mGpsWeek = info.gpsWeek;
    mGpsTowMs = info.gpsTowMs;
    mTimeValid = info.timeValid;
    mTimeSource = info.sourceOfTime;
    mTimeUnc = info.clkTimeUnc;
    mClockFreqBias = info.clkFreqBias;
    mClockFreqBiasUnc = info.clkFreqUnc;
    mXoState = info.xoState;
    mRecErrorRecovery = info.rcvrErrRecovery;
    if (info.jammerInd.size() > 0) {
        mJammerGps = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GPS_L1CA];
        mJammerGlo = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GLONASS_G1];
        mJammerBds = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_BEIDOU_B1_I];
        mJammerGal = info.jammerInd[GNSS_LOC_SIGNAL_TYPE_GALILEO_E1_C];
    }
    mLeapSeconds = info.leapSecondInfo.leapSec;
    mLeapSecUnc = info.leapSecondInfo.leapSecUnc;
    mTimeUncNs = info.clkTimeUnc * 1000000LL;
    mJammedSignalsMask = info.jammedSignalsMask;
    mJammerInd = std::move(info.jammerInd);
}

// parser
class SystemStatusPQWM1parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eGpsWeek = 1,
        eGpsTowMs = 2,
        eTimeValid = 3,
        eTimeSource = 4,
        eTimeUnc = 5,
        eClockFreqBias = 6,
        eClockFreqBiasUnc = 7,
        eXoState = 8,
        ePgaGain = 9,
        eGpsBpAmpI = 10,
        eGpsBpAmpQ = 11,
        eAdcI = 12,
        eAdcQ = 13,
        eJammerGps = 14,
        eJammerGlo = 15,
        eJammerBds = 16,
        eJammerGal = 17,
        eRecErrorRecovery = 18,
        eAgcGps = 19,
        eAgcGlo = 20,
        eAgcBds = 21,
        eAgcGal = 22,
        eMax0 = eAgcGal,
        eLeapSeconds = 23,
        eLeapSecUnc = 24,
        eGloBpAmpI = 25,
        eGloBpAmpQ = 26,
        eBdsBpAmpI = 27,
        eBdsBpAmpQ = 28,
        eGalBpAmpI = 29,
        eGalBpAmpQ = 30,
        eTimeUncNs = 31,
        eMax
    };
    SystemStatusPQWM1 mM1;

public:
    inline uint16_t   getGpsWeek()    { return mM1.mGpsWeek; }
    inline uint32_t   getGpsTowMs()   { return mM1.mGpsTowMs; }
    inline uint8_t    getTimeValid()  { return mM1.mTimeValid; }
    inline uint8_t    getTimeSource() { return mM1.mTimeSource; }
    inline int32_t    getTimeUnc()    { return mM1.mTimeUnc; }
    inline int32_t    getClockFreqBias() { return mM1.mClockFreqBias; }
    inline int32_t    getClockFreqBiasUnc() { return mM1.mClockFreqBiasUnc; }
    inline uint8_t    getXoState()    { return mM1.mXoState;}
    inline int32_t    getPgaGain()    { return mM1.mPgaGain;          }
    inline uint32_t   getGpsBpAmpI()  { return mM1.mGpsBpAmpI;        }
    inline uint32_t   getGpsBpAmpQ()  { return mM1.mGpsBpAmpQ;        }
    inline uint32_t   getAdcI()       { return mM1.mAdcI;             }
    inline uint32_t   getAdcQ()       { return mM1.mAdcQ;             }
    inline uint32_t   getJammerGps()  { return mM1.mJammerGps;        }
    inline uint32_t   getJammerGlo()  { return mM1.mJammerGlo;        }
    inline uint32_t   getJammerBds()  { return mM1.mJammerBds;        }
    inline uint32_t   getJammerGal()  { return mM1.mJammerGal;        }
    inline uint32_t   getRecErrorRecovery() { return mM1.mRecErrorRecovery; }
    inline int32_t    getLeapSeconds(){ return mM1.mLeapSeconds; }
    inline int32_t    getLeapSecUnc() { return mM1.mLeapSecUnc; }
    inline uint32_t   getGloBpAmpI()  { return mM1.mGloBpAmpI; }
    inline uint32_t   getGloBpAmpQ()  { return mM1.mGloBpAmpQ; }
    inline uint32_t   getBdsBpAmpI()  { return mM1.mBdsBpAmpI; }
    inline uint32_t   getBdsBpAmpQ()  { return mM1.mBdsBpAmpQ; }
    inline uint32_t   getGalBpAmpI()  { return mM1.mGalBpAmpI; }
    inline uint32_t   getGalBpAmpQ()  { return mM1.mGalBpAmpQ; }
    inline uint64_t   getTimeUncNs()  { return mM1.mTimeUncNs; }

    SystemStatusPQWM1parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        memset(&mM1, 0, sizeof(mM1));
        if (mField.size() <= eMax0) {
            LOC_LOGE("PQWM1parser - invalid size=%zu", mField.size());
            mM1.mTimeValid = 0;
            return;
        }
        mM1.mGpsWeek = atoi(mField[eGpsWeek].c_str());
        mM1.mGpsTowMs = atoi(mField[eGpsTowMs].c_str());
        mM1.mTimeValid = atoi(mField[eTimeValid].c_str());
        mM1.mTimeSource = atoi(mField[eTimeSource].c_str());
        mM1.mTimeUnc = atoi(mField[eTimeUnc].c_str());
        mM1.mClockFreqBias = atoi(mField[eClockFreqBias].c_str());
        mM1.mClockFreqBiasUnc = atoi(mField[eClockFreqBiasUnc].c_str());
        mM1.mXoState = atoi(mField[eXoState].c_str());
        mM1.mPgaGain = atoi(mField[ePgaGain].c_str());
        mM1.mGpsBpAmpI = atoi(mField[eGpsBpAmpI].c_str());
        mM1.mGpsBpAmpQ = atoi(mField[eGpsBpAmpQ].c_str());
        mM1.mAdcI = atoi(mField[eAdcI].c_str());
        mM1.mAdcQ = atoi(mField[eAdcQ].c_str());
        mM1.mJammerGps = atoi(mField[eJammerGps].c_str());
        mM1.mJammerGlo = atoi(mField[eJammerGlo].c_str());
        mM1.mJammerBds = atoi(mField[eJammerBds].c_str());
        mM1.mJammerGal = atoi(mField[eJammerGal].c_str());
        mM1.mRecErrorRecovery = atoi(mField[eRecErrorRecovery].c_str());
        if (mField.size() > eLeapSecUnc) {
            mM1.mLeapSeconds = atoi(mField[eLeapSeconds].c_str());
            mM1.mLeapSecUnc = atoi(mField[eLeapSecUnc].c_str());
        }
        if (mField.size() > eGalBpAmpQ) {
            mM1.mGloBpAmpI = atoi(mField[eGloBpAmpI].c_str());
            mM1.mGloBpAmpQ = atoi(mField[eGloBpAmpQ].c_str());
            mM1.mBdsBpAmpI = atoi(mField[eBdsBpAmpI].c_str());
            mM1.mBdsBpAmpQ = atoi(mField[eBdsBpAmpQ].c_str());
            mM1.mGalBpAmpI = atoi(mField[eGalBpAmpI].c_str());
            mM1.mGalBpAmpQ = atoi(mField[eGalBpAmpQ].c_str());
        }
        if (mField.size() > eTimeUncNs) {
            mM1.mTimeUncNs = strtoull(mField[eTimeUncNs].c_str(), nullptr, 10);
        }
    }

    inline SystemStatusPQWM1& get() { return mM1;} //getparser
};

/******************************************************************************
 SystemStatusPQWP1
******************************************************************************/
class SystemStatusPQWP1
{
public:
    uint8_t  mEpiValidity; // x4
    float    mEpiLat;    // x5
    float    mEpiLon;    // x6
    float    mEpiAlt;    // x7
    float    mEpiHepe;   // x8
    float    mEpiAltUnc; // x9
    uint8_t  mEpiSrc;    // x10
    SystemStatusPQWP1() = default;
    SystemStatusPQWP1(const GnssEngineDebugDataInfo& info);
};

SystemStatusPQWP1::SystemStatusPQWP1(const GnssEngineDebugDataInfo& info) {
    mEpiValidity = info.epiValidity;
    mEpiLat = info.epiLat;
    mEpiLon = info.epiLon;
    mEpiAlt = info.epiAlt;
    mEpiHepe = info.epiHepe;
    mEpiAltUnc = info.epiAltUnc;
    mEpiSrc = info.epiSrc;
}

class SystemStatusPQWP1parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eEpiValidity = 2,
        eEpiLat = 3,
        eEpiLon = 4,
        eEpiAlt = 5,
        eEpiHepe = 6,
        eEpiAltUnc = 7,
        eEpiSrc = 8,
        eMax
    };
    SystemStatusPQWP1 mP1;

public:
    inline uint8_t    getEpiValidity() { return mP1.mEpiValidity;      }
    inline float      getEpiLat() { return mP1.mEpiLat;           }
    inline float      getEpiLon() { return mP1.mEpiLon;           }
    inline float      getEpiAlt() { return mP1.mEpiAlt;           }
    inline float      getEpiHepe() { return mP1.mEpiHepe;          }
    inline float      getEpiAltUnc() { return mP1.mEpiAltUnc;        }
    inline uint8_t    getEpiSrc() { return mP1.mEpiSrc;           }

    SystemStatusPQWP1parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP1, 0, sizeof(mP1));
        mP1.mEpiValidity = strtol(mField[eEpiValidity].c_str(), NULL, 16);
        mP1.mEpiLat = atof(mField[eEpiLat].c_str());
        mP1.mEpiLon = atof(mField[eEpiLon].c_str());
        mP1.mEpiAlt = atof(mField[eEpiAlt].c_str());
        mP1.mEpiHepe = atoi(mField[eEpiHepe].c_str());
        mP1.mEpiAltUnc = atof(mField[eEpiAltUnc].c_str());
        mP1.mEpiSrc = atoi(mField[eEpiSrc].c_str());
    }

    inline SystemStatusPQWP1& get() { return mP1;}
};

/******************************************************************************
 SystemStatusPQWP2
******************************************************************************/
class SystemStatusPQWP2
{
public:
    float    mBestLat;   // x4
    float    mBestLon;   // x5
    float    mBestAlt;   // x6
    float    mBestHepe;  // x7
    float    mBestAltUnc; // x8
    SystemStatusPQWP2() = default;
    SystemStatusPQWP2(const GnssEngineDebugDataInfo& info);
};

SystemStatusPQWP2::SystemStatusPQWP2(const GnssEngineDebugDataInfo& info){
    mBestLat = info.bestPosLat;
    mBestLon = info.bestPosLon;
    mBestAlt = info.bestPosAlt;
    mBestHepe = info.bestPosHepe;
    mBestAltUnc = info.bestPosAltUnc;
}


class SystemStatusPQWP2parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eBestLat = 2,
        eBestLon = 3,
        eBestAlt = 4,
        eBestHepe = 5,
        eBestAltUnc = 6,
        eMax
    };
    SystemStatusPQWP2 mP2;

public:
    inline float      getBestLat() { return mP2.mBestLat;          }
    inline float      getBestLon() { return mP2.mBestLon;          }
    inline float      getBestAlt() { return mP2.mBestAlt;          }
    inline float      getBestHepe() { return mP2.mBestHepe;         }
    inline float      getBestAltUnc() { return mP2.mBestAltUnc;       }

    SystemStatusPQWP2parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP2, 0, sizeof(mP2));
        mP2.mBestLat = atof(mField[eBestLat].c_str());
        mP2.mBestLon = atof(mField[eBestLon].c_str());
        mP2.mBestAlt = atof(mField[eBestAlt].c_str());
        mP2.mBestHepe = atof(mField[eBestHepe].c_str());
        mP2.mBestAltUnc = atof(mField[eBestAltUnc].c_str());
    }

    inline SystemStatusPQWP2& get() { return mP2;}
};

/******************************************************************************
 SystemStatusPQWP3
******************************************************************************/
class SystemStatusPQWP3
{
public:
    uint8_t   mXtraValidMask;
    uint32_t  mGpsXtraAge;
    uint32_t  mGloXtraAge;
    uint32_t  mBdsXtraAge;
    uint32_t  mGalXtraAge;
    uint32_t  mQzssXtraAge;
    uint32_t  mNavicXtraAge;
    uint32_t  mGpsXtraValid;
    uint32_t  mGloXtraValid;
    uint64_t  mBdsXtraValid;
    uint64_t  mGalXtraValid;
    uint8_t   mQzssXtraValid;
    uint32_t  mNavicXtraValid;
    SystemStatusPQWP3() = default;
    SystemStatusPQWP3(const GnssEngineDebugDataInfo& info);
};

SystemStatusPQWP3::SystemStatusPQWP3(const GnssEngineDebugDataInfo& info){
    mXtraValidMask = info.xtraValidMask;
    mGpsXtraAge = info.gpsXtraAge;
    mGloXtraAge = info.gloXtraAge;
    mBdsXtraAge = info.bdsXtraAge;
    mGalXtraAge = info.galXtraAge;
    mQzssXtraAge = info.qzssXtraAge;
    mNavicXtraAge = info.navicXtraAge;
    mGpsXtraValid = info.gpsXtraMask;
    mGloXtraValid = info.gloXtraMask;
    mBdsXtraValid = info.bdsXtraMask;
    mGalXtraValid = info.galXtraMask;
    mQzssXtraValid = info.qzssXtraMask;
    mNavicXtraValid = info.navicXtraMask;
}

class SystemStatusPQWP3parser : public SystemStatusNmeaBase
{
private:
    // todo: update for navic once available
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eXtraValidMask = 2,
        eGpsXtraAge = 3,
        eGloXtraAge = 4,
        eBdsXtraAge = 5,
        eGalXtraAge = 6,
        eQzssXtraAge = 7,
        eGpsXtraValid = 8,
        eGloXtraValid = 9,
        eBdsXtraValid = 10,
        eGalXtraValid = 11,
        eQzssXtraValid = 12,
        eMax
    };
    SystemStatusPQWP3 mP3;

public:
    inline uint8_t    getXtraValid() { return mP3.mXtraValidMask;   }
    inline uint32_t   getGpsXtraAge() { return mP3.mGpsXtraAge;       }
    inline uint32_t   getGloXtraAge() { return mP3.mGloXtraAge;       }
    inline uint32_t   getBdsXtraAge() { return mP3.mBdsXtraAge;       }
    inline uint32_t   getGalXtraAge() { return mP3.mGalXtraAge;       }
    inline uint32_t   getQzssXtraAge() { return mP3.mQzssXtraAge;      }
    inline uint32_t   getNavicXtraAge() { return mP3.mNavicXtraAge;     }
    inline uint32_t   getGpsXtraValid() { return mP3.mGpsXtraValid;     }
    inline uint32_t   getGloXtraValid() { return mP3.mGloXtraValid;     }
    inline uint64_t   getBdsXtraValid() { return mP3.mBdsXtraValid;     }
    inline uint64_t   getGalXtraValid() { return mP3.mGalXtraValid;     }
    inline uint8_t    getQzssXtraValid() { return mP3.mQzssXtraValid;    }
    inline uint32_t   getNavicXtraValid() { return mP3.mNavicXtraValid;     }

    SystemStatusPQWP3parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP3, 0, sizeof(mP3));
        // todo: update for navic once available
        mP3.mXtraValidMask = strtol(mField[eXtraValidMask].c_str(), NULL, 16);
        mP3.mGpsXtraAge = atoi(mField[eGpsXtraAge].c_str());
        mP3.mGloXtraAge = atoi(mField[eGloXtraAge].c_str());
        mP3.mBdsXtraAge = atoi(mField[eBdsXtraAge].c_str());
        mP3.mGalXtraAge = atoi(mField[eGalXtraAge].c_str());
        mP3.mQzssXtraAge = atoi(mField[eQzssXtraAge].c_str());
        mP3.mGpsXtraValid = strtol(mField[eGpsXtraValid].c_str(), NULL, 16);
        mP3.mGloXtraValid = strtol(mField[eGloXtraValid].c_str(), NULL, 16);
        mP3.mBdsXtraValid = strtol(mField[eBdsXtraValid].c_str(), NULL, 16);
        mP3.mGalXtraValid = strtol(mField[eGalXtraValid].c_str(), NULL, 16);
        mP3.mQzssXtraValid = strtol(mField[eQzssXtraValid].c_str(), NULL, 16);
    }

    inline SystemStatusPQWP3& get() { return mP3;}
};

/******************************************************************************
 SystemStatusPQWP4
******************************************************************************/
class SystemStatusPQWP4
{
public:
    uint32_t  mGpsEpheValid;
    uint32_t  mGloEpheValid;
    uint64_t  mBdsEpheValid;
    uint64_t  mGalEpheValid;
    uint8_t   mQzssEpheValid;
    uint32_t  mNavicEpheValid;

    SystemStatusPQWP4() = default;
    SystemStatusPQWP4(const GnssEngineDebugDataInfo& info);
};

SystemStatusPQWP4::SystemStatusPQWP4(const GnssEngineDebugDataInfo& info) {
    mGpsEpheValid = info.gpsEphMask;
    mGloEpheValid = info.gloEphMask;
    mBdsEpheValid = info.bdsEphMask;
    mGalEpheValid = info.galEphMask;
    mQzssEpheValid = info.qzssEphMask;
    mNavicEpheValid = info.navicEphMask;
}

class SystemStatusPQWP4parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eGpsEpheValid = 2,
        eGloEpheValid = 3,
        eBdsEpheValid = 4,
        eGalEpheValid = 5,
        eQzssEpheValid = 6,
        eMax
    };
    SystemStatusPQWP4 mP4;

public:
    inline uint32_t   getGpsEpheValid() { return mP4.mGpsEpheValid;     }
    inline uint32_t   getGloEpheValid() { return mP4.mGloEpheValid;     }
    inline uint64_t   getBdsEpheValid() { return mP4.mBdsEpheValid;     }
    inline uint64_t   getGalEpheValid() { return mP4.mGalEpheValid;     }
    inline uint8_t    getQzssEpheValid() { return mP4.mQzssEpheValid;    }

    SystemStatusPQWP4parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP4, 0, sizeof(mP4));
        mP4.mGpsEpheValid = strtol(mField[eGpsEpheValid].c_str(), NULL, 16);
        mP4.mGloEpheValid = strtol(mField[eGloEpheValid].c_str(), NULL, 16);
        mP4.mBdsEpheValid = strtol(mField[eBdsEpheValid].c_str(), NULL, 16);
        mP4.mGalEpheValid = strtol(mField[eGalEpheValid].c_str(), NULL, 16);
        mP4.mQzssEpheValid = strtol(mField[eQzssEpheValid].c_str(), NULL, 16);
    }

    inline SystemStatusPQWP4& get() { return mP4;}
};

/******************************************************************************
 SystemStatusPQWP5
******************************************************************************/
class SystemStatusPQWP5
{
public:
    uint32_t  mGpsUnknownMask;
    uint32_t  mGloUnknownMask;
    uint64_t  mBdsUnknownMask;
    uint64_t  mGalUnknownMask;
    uint8_t   mQzssUnknownMask;
    uint32_t  mNavicUnknownMask;
    uint32_t  mGpsGoodMask;
    uint32_t  mGloGoodMask;
    uint64_t  mBdsGoodMask;
    uint64_t  mGalGoodMask;
    uint8_t   mQzssGoodMask;
    uint32_t  mNavicGoodMask;
    uint32_t  mGpsBadMask;
    uint32_t  mGloBadMask;
    uint64_t  mBdsBadMask;
    uint64_t  mGalBadMask;
    uint8_t   mQzssBadMask;
    uint32_t  mNavicBadMask;
    SystemStatusPQWP5() = default;
    SystemStatusPQWP5(const GnssEngineDebugDataInfo& info);
};


SystemStatusPQWP5::SystemStatusPQWP5(const GnssEngineDebugDataInfo& info){
    mGpsUnknownMask = info.gpsHealthUnknownMask;
    mGloUnknownMask = info.gloHealthUnknownMask;
    mBdsUnknownMask = info.bdsHealthUnknownMask;
    mGalUnknownMask = info.galHealthUnknownMask;
    mQzssUnknownMask = info.qzssHealthUnknownMask;
    mNavicUnknownMask = info.navicHealthUnknownMask;
    mGpsGoodMask = info.gpsHealthGoodMask;
    mGloGoodMask = info.gloHealthGoodMask;
    mBdsGoodMask = info.bdsHealthGoodMask;
    mGalGoodMask = info.galHealthGoodMask;
    mQzssGoodMask = info.qzssHealthGoodMask;
    mNavicGoodMask = info.navicHealthGoodMask;
    mGpsBadMask = info.gpsHealthBadMask;
    mGloBadMask = info.gloHealthBadMask;
    mBdsBadMask = info.bdsHealthBadMask;
    mGalBadMask = info.galHealthBadMask;
    mQzssBadMask = info.qzssHealthBadMask;
    mNavicBadMask = info.navicHealthBadMask;
}

class SystemStatusPQWP5parser : public SystemStatusNmeaBase
{
private:
    // todo: update for navic once available
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eGpsUnknownMask = 2,
        eGloUnknownMask = 3,
        eBdsUnknownMask = 4,
        eGalUnknownMask = 5,
        eQzssUnknownMask = 6,
        eGpsGoodMask = 7,
        eGloGoodMask = 8,
        eBdsGoodMask = 9,
        eGalGoodMask = 10,
        eQzssGoodMask = 11,
        eGpsBadMask = 12,
        eGloBadMask = 13,
        eBdsBadMask = 14,
        eGalBadMask = 15,
        eQzssBadMask = 16,
        eMax
    };
    SystemStatusPQWP5 mP5;

public:
    inline uint32_t   getGpsUnknownMask() { return mP5.mGpsUnknownMask;   }
    inline uint32_t   getGloUnknownMask() { return mP5.mGloUnknownMask;   }
    inline uint64_t   getBdsUnknownMask() { return mP5.mBdsUnknownMask;   }
    inline uint64_t   getGalUnknownMask() { return mP5.mGalUnknownMask;   }
    inline uint8_t    getQzssUnknownMask() { return mP5.mQzssUnknownMask;  }
    inline uint32_t   getNavicUnknownMask() { return mP5.mNavicUnknownMask;   }
    inline uint32_t   getGpsGoodMask() { return mP5.mGpsGoodMask;      }
    inline uint32_t   getGloGoodMask() { return mP5.mGloGoodMask;      }
    inline uint64_t   getBdsGoodMask() { return mP5.mBdsGoodMask;      }
    inline uint64_t   getGalGoodMask() { return mP5.mGalGoodMask;      }
    inline uint8_t    getQzssGoodMask() { return mP5.mQzssGoodMask;     }
    inline uint32_t   getNavicGoodMask() { return mP5.mNavicGoodMask;      }
    inline uint32_t   getGpsBadMask() { return mP5.mGpsBadMask;       }
    inline uint32_t   getGloBadMask() { return mP5.mGloBadMask;       }
    inline uint64_t   getBdsBadMask() { return mP5.mBdsBadMask;       }
    inline uint64_t   getGalBadMask() { return mP5.mGalBadMask;       }
    inline uint8_t    getQzssBadMask() { return mP5.mQzssBadMask;      }
    inline uint32_t   getNavicBadMask() { return mP5.mNavicBadMask;       }

    SystemStatusPQWP5parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP5, 0, sizeof(mP5));
        // todo: update for navic once available
        mP5.mGpsUnknownMask = strtol(mField[eGpsUnknownMask].c_str(), NULL, 16);
        mP5.mGloUnknownMask = strtol(mField[eGloUnknownMask].c_str(), NULL, 16);
        mP5.mBdsUnknownMask = strtol(mField[eBdsUnknownMask].c_str(), NULL, 16);
        mP5.mGalUnknownMask = strtol(mField[eGalUnknownMask].c_str(), NULL, 16);
        mP5.mQzssUnknownMask = strtol(mField[eQzssUnknownMask].c_str(), NULL, 16);
        mP5.mGpsGoodMask = strtol(mField[eGpsGoodMask].c_str(), NULL, 16);
        mP5.mGloGoodMask = strtol(mField[eGloGoodMask].c_str(), NULL, 16);
        mP5.mBdsGoodMask = strtol(mField[eBdsGoodMask].c_str(), NULL, 16);
        mP5.mGalGoodMask = strtol(mField[eGalGoodMask].c_str(), NULL, 16);
        mP5.mQzssGoodMask = strtol(mField[eQzssGoodMask].c_str(), NULL, 16);
        mP5.mGpsBadMask = strtol(mField[eGpsBadMask].c_str(), NULL, 16);
        mP5.mGloBadMask = strtol(mField[eGloBadMask].c_str(), NULL, 16);
        mP5.mBdsBadMask = strtol(mField[eBdsBadMask].c_str(), NULL, 16);
        mP5.mGalBadMask = strtol(mField[eGalBadMask].c_str(), NULL, 16);
        mP5.mQzssBadMask = strtol(mField[eQzssBadMask].c_str(), NULL, 16);
    }

    inline SystemStatusPQWP5& get() { return mP5;}
};

/******************************************************************************
 SystemStatusPQWP6parser
******************************************************************************/
class SystemStatusPQWP6
{
public:
    uint32_t  mFixInfoMask;
    SystemStatusPQWP6() = default;
    inline SystemStatusPQWP6(const GnssEngineDebugDataInfo& info) {
        mFixInfoMask = info.fixInfoMask;
    }
};

class SystemStatusPQWP6parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eFixInfoMask = 2,
        eMax
    };
    SystemStatusPQWP6 mP6;

public:
    inline uint32_t   getFixInfoMask() { return mP6.mFixInfoMask;      }

    SystemStatusPQWP6parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mP6, 0, sizeof(mP6));
        mP6.mFixInfoMask = strtol(mField[eFixInfoMask].c_str(), NULL, 16);
    }

    inline SystemStatusPQWP6& get() { return mP6;}
};

/******************************************************************************
 SystemStatusPQWP7parser
******************************************************************************/
class SystemStatusPQWP7
{
public:
    SystemStatusNav mNav[SV_ALL_NUM];
    SystemStatusPQWP7() = default;
    SystemStatusPQWP7(const GnssEngineDebugDataInfo& gnssEngineDebugDataInfo);
};

SystemStatusPQWP7::SystemStatusPQWP7(const GnssEngineDebugDataInfo& gnssEngineDebugDataInfo) {
    memset(mNav, 0, SV_ALL_NUM * sizeof(SystemStatusNav));
    for (int i = 0; i < GNSS_MAX_SV_INFO_LIST_SIZE; i++) {
        GnssNavDataInfo navInfo  = gnssEngineDebugDataInfo.navData[i];
        int offset = 0;
        if (0 == navInfo.gnssSvId) continue;
        // GPS
        if (navInfo.gnssSvId >= GPS_MIN && navInfo.gnssSvId < (GPS_MIN + GPS_NUM)) {
            offset = navInfo.gnssSvId - GPS_MIN;
        }
        // GLO
        if (navInfo.gnssSvId >= GLO_MIN && navInfo.gnssSvId < (GLO_MIN + GLO_NUM)) {
            offset = GPS_NUM + navInfo.gnssSvId - GLO_MIN;
        }
        // BDS
        if (navInfo.gnssSvId >= BDS_MIN && navInfo.gnssSvId < (BDS_MIN + BDS_NUM)) {
            offset = GPS_NUM + GLO_NUM + navInfo.gnssSvId - BDS_MIN;
        }
        // GAL
        if (navInfo.gnssSvId >= GAL_MIN && navInfo.gnssSvId < (GAL_MIN + GAL_NUM)) {
            offset = GPS_NUM + GLO_NUM + BDS_NUM + navInfo.gnssSvId - GAL_MIN;
        }
        // QZSS
        if (navInfo.gnssSvId >= QZSS_MIN && navInfo.gnssSvId < (QZSS_MIN + QZSS_NUM)) {
            offset = GPS_NUM + GLO_NUM + BDS_NUM + GAL_NUM + navInfo.gnssSvId - QZSS_MIN;
        }
        //Navic
        if (navInfo.gnssSvId >= NAVIC_MIN && navInfo.gnssSvId < (NAVIC_MIN + NAVIC_NUM)) {
            offset = GPS_NUM + GLO_NUM + BDS_NUM + GAL_NUM + QZSS_NUM +
                navInfo.gnssSvId - NAVIC_MIN;
        }
        mNav[offset].mType   = GnssEphemerisType(navInfo.type);
        mNav[offset].mSource = GnssEphemerisSource(navInfo.src);
        mNav[offset].mAgeSec = navInfo.age;
    }
}

class SystemStatusPQWP7parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eMin = 2 + SV_ALL_NUM_MIN*3,
        eMax = 2 + SV_ALL_NUM*3
    };
    SystemStatusPQWP7 mP7;

public:
    SystemStatusPQWP7parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        uint32_t svLimit = SV_ALL_NUM;
        if (mField.size() < eMin) {
            LOC_LOGE("PQWP7parser - invalid size=%zu", mField.size());
            return;
        }
        if (mField.size() < eMax) {
            // Try reducing limit, accounting for possibly missing NAVIC support
            svLimit = SV_ALL_NUM_MIN;
        }

        memset(mP7.mNav, 0, sizeof(mP7.mNav));
        for (uint32_t i=0; i<svLimit; i++) {
            mP7.mNav[i].mType   = GnssEphemerisType(atoi(mField[i*3+2].c_str()));
            mP7.mNav[i].mSource = GnssEphemerisSource(atoi(mField[i*3+3].c_str()));
            mP7.mNav[i].mAgeSec = atoi(mField[i*3+4].c_str());
        }
    }

    inline SystemStatusPQWP7& get() { return mP7;}
};

/******************************************************************************
 SystemStatusPQWS1parser
******************************************************************************/
class SystemStatusPQWS1
{
public:
    uint32_t  mFixInfoMask;
    uint32_t  mHepeLimit;
    SystemStatusPQWS1() = default;
    inline SystemStatusPQWS1(const GnssEngineDebugDataInfo& info) {
        mFixInfoMask = info.fixStatusMask;
        mHepeLimit = info.fixHepeLimit;
    }
};

class SystemStatusPQWS1parser : public SystemStatusNmeaBase
{
private:
    enum
    {
        eTalker = 0,
        eUtcTime = 1,
        eFixInfoMask = 2,
        eHepeLimit = 3,
        eMax
    };
    SystemStatusPQWS1 mS1;

public:
    inline uint16_t   getFixInfoMask() { return mS1.mFixInfoMask;      }
    inline uint32_t   getHepeLimit()   { return mS1.mHepeLimit;      }

    SystemStatusPQWS1parser(const char *str_in, uint32_t len_in)
        : SystemStatusNmeaBase(str_in, len_in)
    {
        if (mField.size() < eMax) {
            return;
        }
        memset(&mS1, 0, sizeof(mS1));
        mS1.mFixInfoMask = atoi(mField[eFixInfoMask].c_str());
        mS1.mHepeLimit = atoi(mField[eHepeLimit].c_str());
    }

    inline SystemStatusPQWS1& get() { return mS1;}
};

/******************************************************************************
 SystemStatusTimeAndClock
******************************************************************************/
SystemStatusTimeAndClock::SystemStatusTimeAndClock(const SystemStatusPQWM1& nmea) :
    mGpsWeek(nmea.mGpsWeek),
    mGpsTowMs(nmea.mGpsTowMs),
    mTimeValid(nmea.mTimeValid),
    mTimeSource(nmea.mTimeSource),
    mTimeUnc(nmea.mTimeUnc),
    mClockFreqBias(nmea.mClockFreqBias),
    mClockFreqBiasUnc(nmea.mClockFreqBiasUnc),
    mLeapSeconds(nmea.mLeapSeconds),
    mLeapSecUnc(nmea.mLeapSecUnc),
    mTimeUncNs(nmea.mTimeUncNs)
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
    return;
}

/******************************************************************************
 SystemStatusXoState
******************************************************************************/
SystemStatusXoState::SystemStatusXoState(const SystemStatusPQWM1& nmea) :
    mXoState(nmea.mXoState)
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
    return;
}

/******************************************************************************
 SystemStatusRfAndParams
******************************************************************************/
SystemStatusRfAndParams::SystemStatusRfAndParams(const SystemStatusPQWM1& nmea) :
    mPgaGain(nmea.mPgaGain),
    mGpsBpAmpI(nmea.mGpsBpAmpI),
    mGpsBpAmpQ(nmea.mGpsBpAmpQ),
    mAdcI(nmea.mAdcI),
    mAdcQ(nmea.mAdcQ),
    mJammerGps(nmea.mJammerGps),
    mJammerGlo(nmea.mJammerGlo),
    mJammerBds(nmea.mJammerBds),
    mJammerGal(nmea.mJammerGal),
    mGloBpAmpI(nmea.mGloBpAmpI),
    mGloBpAmpQ(nmea.mGloBpAmpQ),
    mBdsBpAmpI(nmea.mBdsBpAmpI),
    mBdsBpAmpQ(nmea.mBdsBpAmpQ),
    mGalBpAmpI(nmea.mGalBpAmpI),
    mGalBpAmpQ(nmea.mGalBpAmpQ),
    mJammedSignalsMask(nmea.mJammedSignalsMask),
    mJammerInd(std::move(nmea.mJammerInd)) {}

bool SystemStatusRfAndParams::equals(const SystemStatusItemBase& peer) {
    if ((mPgaGain != ((const SystemStatusRfAndParams&)peer).mPgaGain) ||
        (mGpsBpAmpI != ((const SystemStatusRfAndParams&)peer).mGpsBpAmpI) ||
        (mGpsBpAmpQ != ((const SystemStatusRfAndParams&)peer).mGpsBpAmpQ) ||
        (mAdcI != ((const SystemStatusRfAndParams&)peer).mAdcI) ||
        (mAdcQ != ((const SystemStatusRfAndParams&)peer).mAdcQ) ||
        (mJammerGps != ((const SystemStatusRfAndParams&)peer).mJammerGps) ||
        (mJammerGlo != ((const SystemStatusRfAndParams&)peer).mJammerGlo) ||
        (mJammerBds != ((const SystemStatusRfAndParams&)peer).mJammerBds) ||
        (mJammerGal != ((const SystemStatusRfAndParams&)peer).mJammerGal) ||
        (mGloBpAmpI != ((const SystemStatusRfAndParams&)peer).mGloBpAmpI) ||
        (mGloBpAmpQ != ((const SystemStatusRfAndParams&)peer).mGloBpAmpQ) ||
        (mBdsBpAmpI != ((const SystemStatusRfAndParams&)peer).mBdsBpAmpI) ||
        (mBdsBpAmpQ != ((const SystemStatusRfAndParams&)peer).mBdsBpAmpQ) ||
        (mGalBpAmpI != ((const SystemStatusRfAndParams&)peer).mGalBpAmpI) ||
        (mGalBpAmpQ != ((const SystemStatusRfAndParams&)peer).mGalBpAmpQ) ||
        (mJammedSignalsMask != ((const SystemStatusRfAndParams&)peer).mJammedSignalsMask)) {
        return false;
    }
    return true;
}

void SystemStatusRfAndParams::dump()
{
    LOC_LOGV("RfAndParams: u=%ld:%ld p=%d bi=%d bq=%d ai=%d aq=%d "
             "jgp=%d jgl=%d jbd=%d jga=%d ",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mPgaGain,
             mGpsBpAmpI,
             mGpsBpAmpQ,
             mAdcI,
             mAdcQ,
             mJammerGps,
             mJammerGlo,
             mJammerBds,
             mJammerGal);
    return;
}

/******************************************************************************
 SystemStatusErrRecovery
******************************************************************************/
SystemStatusErrRecovery::SystemStatusErrRecovery(const SystemStatusPQWM1& nmea) :
    mRecErrorRecovery(nmea.mRecErrorRecovery)
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
    return;
}

/******************************************************************************
 SystemStatusInjectedPosition
******************************************************************************/
SystemStatusInjectedPosition::SystemStatusInjectedPosition(const SystemStatusPQWP1& nmea) :
    mEpiValidity(nmea.mEpiValidity),
    mEpiLat(nmea.mEpiLat),
    mEpiLon(nmea.mEpiLon),
    mEpiAlt(nmea.mEpiAlt),
    mEpiHepe(nmea.mEpiHepe),
    mEpiAltUnc(nmea.mEpiAltUnc),
    mEpiSrc(nmea.mEpiSrc)
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
    return;
}

/******************************************************************************
 SystemStatusBestPosition
******************************************************************************/
SystemStatusBestPosition::SystemStatusBestPosition(const SystemStatusPQWP2& nmea) :
    mValid(true),
    mBestLat(nmea.mBestLat),
    mBestLon(nmea.mBestLon),
    mBestAlt(nmea.mBestAlt),
    mBestHepe(nmea.mBestHepe),
    mBestAltUnc(nmea.mBestAltUnc)
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
    return;
}

/******************************************************************************
 SystemStatusXtra
******************************************************************************/
SystemStatusXtra::SystemStatusXtra(const SystemStatusPQWP3& nmea) :
    mXtraValidMask(nmea.mXtraValidMask),
    mGpsXtraAge(nmea.mGpsXtraAge),
    mGloXtraAge(nmea.mGloXtraAge),
    mBdsXtraAge(nmea.mBdsXtraAge),
    mGalXtraAge(nmea.mGalXtraAge),
    mQzssXtraAge(nmea.mQzssXtraAge),
    mNavicXtraAge(nmea.mNavicXtraAge),
    mGpsXtraValid(nmea.mGpsXtraValid),
    mGloXtraValid(nmea.mGloXtraValid),
    mBdsXtraValid(nmea.mBdsXtraValid),
    mGalXtraValid(nmea.mGalXtraValid),
    mQzssXtraValid(nmea.mQzssXtraValid),
    mNavicXtraValid(nmea.mNavicXtraValid)
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
    return;
}

/******************************************************************************
 SystemStatusEphemeris
******************************************************************************/
SystemStatusEphemeris::SystemStatusEphemeris(const SystemStatusPQWP4& nmea) :
    mGpsEpheValid(nmea.mGpsEpheValid),
    mGloEpheValid(nmea.mGloEpheValid),
    mBdsEpheValid(nmea.mBdsEpheValid),
    mGalEpheValid(nmea.mGalEpheValid),
    mQzssEpheValid(nmea.mQzssEpheValid),
    mNavicEpheValid(nmea.mNavicEpheValid)
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
    LOC_LOGV("Ephemeris: u=%ld:%ld ev=%x:%x:%" PRIx64 ":%" PRIx64 ":%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mGpsEpheValid,
             mGloEpheValid,
             mBdsEpheValid,
             mGalEpheValid,
             mQzssEpheValid);
    return;
}

/******************************************************************************
 SystemStatusSvHealth
******************************************************************************/
SystemStatusSvHealth::SystemStatusSvHealth(const SystemStatusPQWP5& nmea) :
    mGpsUnknownMask(nmea.mGpsUnknownMask),
    mGloUnknownMask(nmea.mGloUnknownMask),
    mBdsUnknownMask(nmea.mBdsUnknownMask),
    mGalUnknownMask(nmea.mGalUnknownMask),
    mQzssUnknownMask(nmea.mQzssUnknownMask),
    mNavicUnknownMask(nmea.mNavicUnknownMask),
    mGpsGoodMask(nmea.mGpsGoodMask),
    mGloGoodMask(nmea.mGloGoodMask),
    mBdsGoodMask(nmea.mBdsGoodMask),
    mGalGoodMask(nmea.mGalGoodMask),
    mQzssGoodMask(nmea.mQzssGoodMask),
    mNavicGoodMask(nmea.mNavicGoodMask),
    mGpsBadMask(nmea.mGpsBadMask),
    mGloBadMask(nmea.mGloBadMask),
    mBdsBadMask(nmea.mBdsBadMask),
    mGalBadMask(nmea.mGalBadMask),
    mQzssBadMask(nmea.mQzssBadMask),
    mNavicBadMask(nmea.mNavicBadMask)
{
}

bool SystemStatusSvHealth::equals(const SystemStatusItemBase& peer) {
    if ((mGpsUnknownMask != ((const SystemStatusSvHealth&)peer).mGpsUnknownMask) ||
        (mGloUnknownMask != ((const SystemStatusSvHealth&)peer).mGloUnknownMask) ||
        (mBdsUnknownMask != ((const SystemStatusSvHealth&)peer).mBdsUnknownMask) ||
        (mGalUnknownMask != ((const SystemStatusSvHealth&)peer).mGalUnknownMask) ||
        (mQzssUnknownMask != ((const SystemStatusSvHealth&)peer).mQzssUnknownMask) ||
        (mGpsGoodMask != ((const SystemStatusSvHealth&)peer).mGpsGoodMask) ||
        (mGloGoodMask != ((const SystemStatusSvHealth&)peer).mGloGoodMask) ||
        (mBdsGoodMask != ((const SystemStatusSvHealth&)peer).mBdsGoodMask) ||
        (mGalGoodMask != ((const SystemStatusSvHealth&)peer).mGalGoodMask) ||
        (mQzssGoodMask != ((const SystemStatusSvHealth&)peer).mQzssGoodMask) ||
        (mGpsBadMask != ((const SystemStatusSvHealth&)peer).mGpsBadMask) ||
        (mGloBadMask != ((const SystemStatusSvHealth&)peer).mGloBadMask) ||
        (mBdsBadMask != ((const SystemStatusSvHealth&)peer).mBdsBadMask) ||
        (mGalBadMask != ((const SystemStatusSvHealth&)peer).mGalBadMask) ||
        (mQzssBadMask != ((const SystemStatusSvHealth&)peer).mQzssBadMask)) {
        return false;
    }
    return true;
}

void SystemStatusSvHealth::dump()
{
    LOC_LOGV("SvHealth: u=%ld:%ld \
             u=%x:%x:%" PRIx64 ":%" PRIx64 ":%x \
             g=%x:%x:%" PRIx64 ":%" PRIx64 ":%x \
             b=%x:%x:%" PRIx64 ":%" PRIx64 ":%x",
             mUtcTime.tv_sec, mUtcTime.tv_nsec,
             mGpsUnknownMask,
             mGloUnknownMask,
             mBdsUnknownMask,
             mGalUnknownMask,
             mQzssUnknownMask,
             mGpsGoodMask,
             mGloGoodMask,
             mBdsGoodMask,
             mGalGoodMask,
             mQzssGoodMask,
             mGpsBadMask,
             mGloBadMask,
             mBdsBadMask,
             mGalBadMask,
             mQzssBadMask);
    return;
}

/******************************************************************************
 SystemStatusPdr
******************************************************************************/
SystemStatusPdr::SystemStatusPdr(const SystemStatusPQWP6& nmea) :
    mFixInfoMask(nmea.mFixInfoMask)
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
    return;
}

/******************************************************************************
 SystemStatusNavData
******************************************************************************/
SystemStatusNavData::SystemStatusNavData(const SystemStatusPQWP7& nmea)
{
   for (uint32_t i=0; i<SV_ALL_NUM; i++) {
        mNav[i] = nmea.mNav[i];
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
    return;
}

/******************************************************************************
 SystemStatusPositionFailure
******************************************************************************/
SystemStatusPositionFailure::SystemStatusPositionFailure(const SystemStatusPQWS1& nmea) :
    mFixInfoMask(nmea.mFixInfoMask),
    mHepeLimit(nmea.mHepeLimit)
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
    return;
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
    return;
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

/******************************************************************************
@brief      API to set report data into internal buffer

@param[In]  data pointer to the NMEA string
@param[In]  len  length of the NMEA string

@return     true when the NMEA is consumed by the method.
******************************************************************************/
bool SystemStatus::setNmeaString(const char *data, uint32_t len)
{
    if (!loc_nmea_is_debug(data, len)) {
        return false;
    }

    char buf[SystemStatusNmeaBase::NMEA_MAXSIZE + 1] = { 0 };
    strlcpy(buf, data, sizeof(buf));

    pthread_mutex_lock(&mMutexSystemStatus);

    // parse the received nmea strings here
    if (0 == strncmp(data, "$PQWM1", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        SystemStatusPQWM1 s = SystemStatusPQWM1parser(buf, len).get();
        setIteminReport(mCache.mTimeAndClock, SystemStatusTimeAndClock(s));
        setIteminReport(mCache.mXoState, SystemStatusXoState(s));
        setIteminReport(mCache.mRfAndParams, SystemStatusRfAndParams(s));
        setIteminReport(mCache.mErrRecovery, SystemStatusErrRecovery(s));
    }
    else if (0 == strncmp(data, "$PQWP1", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mInjectedPosition,
                SystemStatusInjectedPosition(SystemStatusPQWP1parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP2", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mBestPosition,
                SystemStatusBestPosition(SystemStatusPQWP2parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP3", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mXtra,
                SystemStatusXtra(SystemStatusPQWP3parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP4", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mEphemeris,
                SystemStatusEphemeris(SystemStatusPQWP4parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP5", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mSvHealth,
                SystemStatusSvHealth(SystemStatusPQWP5parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP6", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mPdr,
                SystemStatusPdr(SystemStatusPQWP6parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWP7", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mNavData,
                SystemStatusNavData(SystemStatusPQWP7parser(buf, len).get()));
    }
    else if (0 == strncmp(data, "$PQWS1", SystemStatusNmeaBase::NMEA_MINSIZE)) {
        setIteminReport(mCache.mPositionFailure,
                SystemStatusPositionFailure(SystemStatusPQWS1parser(buf, len).get()));
    }
    else {
        // do nothing
    }

    pthread_mutex_unlock(&mMutexSystemStatus);
    return true;
}


void SystemStatus::setEngineDebugDataInfo(const GnssEngineDebugDataInfo& gnssEngineDebugDataInfo) {
    pthread_mutex_lock(&mMutexSystemStatus);
    LOC_LOGd("setEngine data");
    SystemStatusPQWM1 s(gnssEngineDebugDataInfo);
    setIteminReport(mCache.mTimeAndClock, SystemStatusTimeAndClock(s));
    setIteminReport(mCache.mXoState, SystemStatusXoState(s));
    setIteminReport(mCache.mRfAndParams, SystemStatusRfAndParams(s));
    setIteminReport(mCache.mErrRecovery, SystemStatusErrRecovery(s));
    setIteminReport(mCache.mInjectedPosition,
            SystemStatusInjectedPosition(SystemStatusPQWP1(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mBestPosition,
            SystemStatusBestPosition(SystemStatusPQWP2(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mXtra,
            SystemStatusXtra(SystemStatusPQWP3(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mEphemeris,
            SystemStatusEphemeris(SystemStatusPQWP4(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mSvHealth,
            SystemStatusSvHealth(SystemStatusPQWP5(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mPdr,
            SystemStatusPdr(SystemStatusPQWP6(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mNavData,
            SystemStatusNavData(SystemStatusPQWP7(gnssEngineDebugDataInfo)));
    setIteminReport(mCache.mPositionFailure,
            SystemStatusPositionFailure(SystemStatusPQWS1(gnssEngineDebugDataInfo)));
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

