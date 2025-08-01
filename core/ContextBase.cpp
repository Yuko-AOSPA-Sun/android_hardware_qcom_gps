/* Copyright (c) 2011-2014,2016-2017,2020-2021 The Linux Foundation. All rights reserved.
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

#define LOG_NDEBUG 0
#define LOG_TAG "LocSvc_CtxBase"

#include <dlfcn.h>
#include <unistd.h>
#include <ContextBase.h>
#include <msg_q.h>
#include <loc_target.h>
#include <loc_pla.h>
#include <loc_log.h>

namespace loc_core {

#define SLL_LOC_API_LIB_NAME "libsynergy_loc_api.so"
#define LOC_APIV2_0_LIB_NAME "libloc_api_v02.so"

loc_gps_cfg_s_type ContextBase::mGps_conf {};
loc_sap_cfg_s_type ContextBase::mSap_conf {};
izat_process_info ContextBase:: mIzat_process_conf {};

bool ContextBase::sIsEngineCapabilitiesKnown = false;
uint64_t ContextBase::sSupportedMsgMask = 0;
bool ContextBase::sGnssMeasurementSupported = false;
uint8_t ContextBase::sFeaturesSupported[MAX_FEATURE_LENGTH];
GnssNMEARptRate ContextBase::sNmeaReportRate = GNSS_NMEA_REPORT_RATE_NHZ;
LocationCapabilitiesMask ContextBase::sQwesFeatureMask = 0;
LocationCapabilitiesMask ContextBase::sHwCapabilitiesMask = 0;

const loc_param_s_type ContextBase::mGps_conf_table[] =
{
  {"GPS_LOCK",                       &mGps_conf.GPS_LOCK,                       NULL, 'n'},
  {"SUPL_VER",                       &mGps_conf.SUPL_VER,                       NULL, 'n'},
  {"LPP_PROFILE",                    &mGps_conf.LPP_PROFILE,                    NULL, 'n'},
  {"A_GLONASS_POS_PROTOCOL_SELECT",  &mGps_conf.A_GLONASS_POS_PROTOCOL_SELECT,  NULL, 'n'},
  {"LPPE_CP_TECHNOLOGY",             &mGps_conf.LPPE_CP_TECHNOLOGY,             NULL, 'n'},
  {"LPPE_UP_TECHNOLOGY",             &mGps_conf.LPPE_UP_TECHNOLOGY,             NULL, 'n'},
  {"AGPS_CERT_WRITABLE_MASK",        &mGps_conf.AGPS_CERT_WRITABLE_MASK,        NULL, 'n'},
  {"SUPL_MODE",                      &mGps_conf.SUPL_MODE,                      NULL, 'n'},
  {"SUPL_ES",                        &mGps_conf.SUPL_ES,                        NULL, 'n'},
  {"INTERMEDIATE_POS",               &mGps_conf.INTERMEDIATE_POS,               NULL, 'n'},
  {"ACCURACY_THRES",                 &mGps_conf.ACCURACY_THRES,                 NULL, 'n'},
  {"NMEA_PROVIDER",                  &mGps_conf.NMEA_PROVIDER,                  NULL, 'n'},
  {"NMEA_REPORT_RATE",               &mGps_conf.NMEA_REPORT_RATE,               NULL, 's'},
  {"CAPABILITIES",                   &mGps_conf.CAPABILITIES,                   NULL, 'n'},
  {"XTRA_SERVER_1",                  &mGps_conf.XTRA_SERVER_1,                  NULL, 's'},
  {"XTRA_SERVER_2",                  &mGps_conf.XTRA_SERVER_2,                  NULL, 's'},
  {"XTRA_SERVER_3",                  &mGps_conf.XTRA_SERVER_3,                  NULL, 's'},
  {"USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL",
           &mGps_conf.USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL,          NULL, 'n'},
  {"AGPS_CONFIG_INJECT",             &mGps_conf.AGPS_CONFIG_INJECT,             NULL, 'n'},
  {"EXTERNAL_DR_ENABLED",            &mGps_conf.EXTERNAL_DR_ENABLED,                  NULL, 'n'},
  {"SUPL_HOST",                      &mGps_conf.SUPL_HOST,                      NULL, 's'},
  {"SUPL_PORT",                      &mGps_conf.SUPL_PORT,                      NULL, 'n'},
  {"MODEM_TYPE",                     &mGps_conf.MODEM_TYPE,                     NULL, 'n' },
  {"MO_SUPL_HOST",                   &mGps_conf.MO_SUPL_HOST,                   NULL, 's' },
  {"MO_SUPL_PORT",                   &mGps_conf.MO_SUPL_PORT,                   NULL, 'n' },
  {"CONSTRAINED_TIME_UNCERTAINTY_ENABLED",
           &mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_ENABLED,      NULL, 'n'},
  {"CONSTRAINED_TIME_UNCERTAINTY_THRESHOLD",
           &mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_THRESHOLD,    NULL, 'f'},
  {"CONSTRAINED_TIME_UNCERTAINTY_ENERGY_BUDGET",
           &mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_ENERGY_BUDGET, NULL, 'n'},
  {"POSITION_ASSISTED_CLOCK_ESTIMATOR_ENABLED",
           &mGps_conf.POSITION_ASSISTED_CLOCK_ESTIMATOR_ENABLED, NULL, 'n'},
  {"CP_MTLR_ES",                     &mGps_conf.CP_MTLR_ES,                     NULL, 'n' },
  {"GNSS_DEPLOYMENT",  &mGps_conf.GNSS_DEPLOYMENT, NULL, 'n'},
  {"CUSTOM_NMEA_GGA_FIX_QUALITY_ENABLED",
           &mGps_conf.CUSTOM_NMEA_GGA_FIX_QUALITY_ENABLED, NULL, 'n'},
  {"NMEA_TAG_BLOCK_GROUPING_ENABLED", &mGps_conf.NMEA_TAG_BLOCK_GROUPING_ENABLED, NULL, 'n'},
  {"NI_SUPL_DENY_ON_NFW_LOCKED",  &mGps_conf.NI_SUPL_DENY_ON_NFW_LOCKED, NULL, 'n'},
  {"ENABLE_NMEA_PRINT",  &mGps_conf.ENABLE_NMEA_PRINT, NULL, 'n'},
};

const loc_param_s_type ContextBase::mSap_conf_table[] =
{
  {"GYRO_BIAS_RANDOM_WALK",          &mSap_conf.GYRO_BIAS_RANDOM_WALK,          &mSap_conf.GYRO_BIAS_RANDOM_WALK_VALID, 'f'},
  {"ACCEL_RANDOM_WALK_SPECTRAL_DENSITY",     &mSap_conf.ACCEL_RANDOM_WALK_SPECTRAL_DENSITY,    &mSap_conf.ACCEL_RANDOM_WALK_SPECTRAL_DENSITY_VALID, 'f'},
  {"ANGLE_RANDOM_WALK_SPECTRAL_DENSITY",     &mSap_conf.ANGLE_RANDOM_WALK_SPECTRAL_DENSITY,    &mSap_conf.ANGLE_RANDOM_WALK_SPECTRAL_DENSITY_VALID, 'f'},
  {"RATE_RANDOM_WALK_SPECTRAL_DENSITY",      &mSap_conf.RATE_RANDOM_WALK_SPECTRAL_DENSITY,     &mSap_conf.RATE_RANDOM_WALK_SPECTRAL_DENSITY_VALID, 'f'},
  {"VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY",  &mSap_conf.VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY, &mSap_conf.VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY_VALID, 'f'},
  {"SENSOR_ACCEL_BATCHES_PER_SEC",   &mSap_conf.SENSOR_ACCEL_BATCHES_PER_SEC,   NULL, 'n'},
  {"SENSOR_ACCEL_SAMPLES_PER_BATCH", &mSap_conf.SENSOR_ACCEL_SAMPLES_PER_BATCH, NULL, 'n'},
  {"SENSOR_GYRO_BATCHES_PER_SEC",    &mSap_conf.SENSOR_GYRO_BATCHES_PER_SEC,    NULL, 'n'},
  {"SENSOR_GYRO_SAMPLES_PER_BATCH",  &mSap_conf.SENSOR_GYRO_SAMPLES_PER_BATCH,  NULL, 'n'},
  {"SENSOR_ACCEL_BATCHES_PER_SEC_HIGH",   &mSap_conf.SENSOR_ACCEL_BATCHES_PER_SEC_HIGH,   NULL, 'n'},
  {"SENSOR_ACCEL_SAMPLES_PER_BATCH_HIGH", &mSap_conf.SENSOR_ACCEL_SAMPLES_PER_BATCH_HIGH, NULL, 'n'},
  {"SENSOR_GYRO_BATCHES_PER_SEC_HIGH",    &mSap_conf.SENSOR_GYRO_BATCHES_PER_SEC_HIGH,    NULL, 'n'},
  {"SENSOR_GYRO_SAMPLES_PER_BATCH_HIGH",  &mSap_conf.SENSOR_GYRO_SAMPLES_PER_BATCH_HIGH,  NULL, 'n'},
  {"SENSOR_CONTROL_MODE",            &mSap_conf.SENSOR_CONTROL_MODE,            NULL, 'n'},
  {"SENSOR_ALGORITHM_CONFIG_MASK",   &mSap_conf.SENSOR_ALGORITHM_CONFIG_MASK,   NULL, 'n'}
};

uint32_t ContextBase::mAntennaInfoVectorSize = 0;

void ContextBase::readConfig()
{
    static bool confReadDone = false;
    if (!confReadDone) {
        confReadDone = true;
        /*Defaults for gps.conf*/
        mGps_conf.INTERMEDIATE_POS = 0;
        mGps_conf.ACCURACY_THRES = 0;
        mGps_conf.NMEA_PROVIDER = 0;
#ifdef FEATURE_AUTOMOTIVE
        mGps_conf.GPS_LOCK = GNSS_CONFIG_GPS_LOCK_MO_AND_NI & (~GNSS_CONFIG_GPS_LOCK_NFW_V2X);
#else
        mGps_conf.GPS_LOCK = GNSS_CONFIG_GPS_LOCK_MO_AND_NI;
#endif
        mGps_conf.SUPL_VER = 0x10000;
        mGps_conf.SUPL_MODE = 0x1;
        mGps_conf.SUPL_ES = 0;
        mGps_conf.CP_MTLR_ES = 0;
        mGps_conf.SUPL_HOST[0] = 0;
        mGps_conf.SUPL_PORT = 0;
        mGps_conf.CAPABILITIES = 0x7;
        /* LTE Positioning Profile configuration is disable by default*/
        mGps_conf.LPP_PROFILE = 0;
        /*By default no positioning protocol is selected on A-GLONASS system*/
        mGps_conf.A_GLONASS_POS_PROTOCOL_SELECT = 0;
        /*Use emergency PDN by default*/
        mGps_conf.USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL = 1;
        /* By default no LPPe CP technology is enabled*/
        mGps_conf.LPPE_CP_TECHNOLOGY = 0;
        /* By default no LPPe UP technology is enabled*/
        mGps_conf.LPPE_UP_TECHNOLOGY = 0;
        /* By default we use unknown modem type*/
        mGps_conf.MODEM_TYPE = 2;

        /*Defaults for sap.conf*/
        mSap_conf.GYRO_BIAS_RANDOM_WALK = 0;
        mSap_conf.SENSOR_ACCEL_BATCHES_PER_SEC = 2;
        mSap_conf.SENSOR_ACCEL_SAMPLES_PER_BATCH = 5;
        mSap_conf.SENSOR_GYRO_BATCHES_PER_SEC = 2;
        mSap_conf.SENSOR_GYRO_SAMPLES_PER_BATCH = 5;
        mSap_conf.SENSOR_ACCEL_BATCHES_PER_SEC_HIGH = 4;
        mSap_conf.SENSOR_ACCEL_SAMPLES_PER_BATCH_HIGH = 25;
        mSap_conf.SENSOR_GYRO_BATCHES_PER_SEC_HIGH = 4;
        mSap_conf.SENSOR_GYRO_SAMPLES_PER_BATCH_HIGH = 25;
        mSap_conf.SENSOR_CONTROL_MODE = 0; /* AUTO */
        mSap_conf.SENSOR_ALGORITHM_CONFIG_MASK = 0; /* INS Disabled = FALSE*/
        /* Values MUST be set by OEMs in configuration for sensor-assisted
          navigation to work. There are NO default values */
        mSap_conf.ACCEL_RANDOM_WALK_SPECTRAL_DENSITY = 0;
        mSap_conf.ANGLE_RANDOM_WALK_SPECTRAL_DENSITY = 0;
        mSap_conf.RATE_RANDOM_WALK_SPECTRAL_DENSITY = 0;
        mSap_conf.VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY = 0;
        mSap_conf.GYRO_BIAS_RANDOM_WALK_VALID = 0;
        mSap_conf.ACCEL_RANDOM_WALK_SPECTRAL_DENSITY_VALID = 0;
        mSap_conf.ANGLE_RANDOM_WALK_SPECTRAL_DENSITY_VALID = 0;
        mSap_conf.RATE_RANDOM_WALK_SPECTRAL_DENSITY_VALID = 0;
        mSap_conf.VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY_VALID = 0;

        /* None of the 10 slots for agps certificates are writable by default */
        mGps_conf.AGPS_CERT_WRITABLE_MASK = 0;

        /* inject supl config to modem with config values from config.xml or gps.conf, default 1 */
        mGps_conf.AGPS_CONFIG_INJECT = 1;

        /* default configuration value of constrained time uncertainty mode:
           feature disabled, time uncertainty threshold defined by modem,
           and unlimited power budget */
#ifdef FEATURE_AUTOMOTIVE
        mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_ENABLED = 1;
#else
        mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_ENABLED = 0;
#endif
        mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_THRESHOLD = 0.0;
        mGps_conf.CONSTRAINED_TIME_UNCERTAINTY_ENERGY_BUDGET = 0;

        /* default configuration value of position assisted clock estimator mode */
        mGps_conf.POSITION_ASSISTED_CLOCK_ESTIMATOR_ENABLED = 0;
        /* default configuration QTI GNSS H/W */
        mGps_conf.GNSS_DEPLOYMENT = 0;
        mGps_conf.CUSTOM_NMEA_GGA_FIX_QUALITY_ENABLED = 0;
        /* default NMEA Tag Block Grouping is disabled */
        mGps_conf.NMEA_TAG_BLOCK_GROUPING_ENABLED = 0;
        /* default configuration for NI_SUPL_DENY_ON_NFW_LOCKED */
        mGps_conf.NI_SUPL_DENY_ON_NFW_LOCKED = 1;
        /* By default NMEA Printing is disabled */
        mGps_conf.ENABLE_NMEA_PRINT = 0;

        UTIL_READ_CONF(LOC_PATH_GPS_CONF, mGps_conf_table);
        UTIL_READ_CONF(LOC_PATH_SAP_CONF, mSap_conf_table);

        loc_param_s_type ant_info_vector_table[] =
        {
            { "ANTENNA_INFO_VECTOR_SIZE", &mAntennaInfoVectorSize, NULL, 'n' }
        };
        UTIL_READ_CONF(LOC_PATH_ANT_CORR, ant_info_vector_table);

        if (strncmp(mGps_conf.NMEA_REPORT_RATE, "1HZ", sizeof(mGps_conf.NMEA_REPORT_RATE)) == 0) {
            /* NMEA reporting is configured at 1Hz*/
            sNmeaReportRate = GNSS_NMEA_REPORT_RATE_1HZ;
        } else {
            sNmeaReportRate = GNSS_NMEA_REPORT_RATE_NHZ;
        }
        LOC_LOGI("%s] GNSS Deployment: %s", __FUNCTION__,
                ((mGps_conf.GNSS_DEPLOYMENT == QCSR_SS5_ENABLED) ? "SS5" :
                ((mGps_conf.GNSS_DEPLOYMENT == PDS_API_ENABLED) ? "QFUSION" : "QGNSS")));

        switch (getTargetGnssType(loc_get_target())) {
          case GNSS_GSS:
          case GNSS_AUTO:
             // For APQ targets, MSA/MSB capabilities should be reset
             mGps_conf.CAPABILITIES &= ~(LOC_GPS_CAPABILITY_MSA | LOC_GPS_CAPABILITY_MSB);
             break;
          default:
             break;
        }

        readIZatConfForValueAddedProcess();
    }
}

void ContextBase::readIZatConfForValueAddedProcess() {
    bool retVal = true;
    const char *error = nullptr;
    unsigned int processListLength = 0;
    loc_process_info_s_type* processInfoList = nullptr;

    int rc = loc_read_process_conf(LOC_PATH_IZAT_CONF, &processListLength,
                                   &processInfoList);
    if (rc != 0) {
        LOC_LOGe("failed to parse conf file for value added process");
        return;
    }

    // go over the conf table to see whether any plugin daemon is enabled
    for (unsigned int i = 0; i < processListLength; i++) {
        LOC_LOGi("process %s, enabled %d",
                 processInfoList[i].name[0], processInfoList[i].proc_status);
        if (processInfoList[i].proc_status == ENABLED) {
            mIzat_process_conf.valueAddedProcessEnabled = true;

            if (strncmp(processInfoList[i].name[0], "engine-service",
                        strlen("engine-service")) == 0) {
                mIzat_process_conf.engineServiceEnabled = true;

                if (processInfoList[i].args[1]!= nullptr) {
                    // check if this is DRE-INT engine
                    if (strncmp(processInfoList[i].args[1], "DRE-INT",
                                sizeof("DRE-INT")) == 0) {
                        mIzat_process_conf.engineServiceInfo.dreIntEnabled = true;
                    } else if (strncmp(processInfoList[i].args[1], "PPE",
                                       sizeof("PPE")) == 0) {
                        mIzat_process_conf.engineServiceInfo.ppeEnabled = true;
                    } else if (strncmp(processInfoList[i].args[1], "PPE-INT",
                                       sizeof("PPE-INT")) == 0) {
                        mIzat_process_conf.engineServiceInfo.ppeIntEnabled = true;
                        mIzat_process_conf.engineServiceInfo.ppeEnabled = true;
                    }
                }
            } else if (strncmp(processInfoList[i].name[0], "xtwifi-client",
                               strlen("xtwifi-client")) == 0) {
                mIzat_process_conf.gtpDaemonEnabled = true;
            } else if (strncmp(processInfoList[i].name[0], "slim_daemon",
                               strlen("slim_daemon")) == 0) {
                mIzat_process_conf.slimDaemonEnabled = true;
            } else if (strncmp(processInfoList[i].name[0], "edgnss-daemon",
                               strlen("edgnss-daemon")) == 0) {
                mIzat_process_conf.eDgnssDaemonEnabled = true;
            }
        }
    }

#ifdef _ANDROID_
    // set the property to launch loc_launcher
    // loc_launcher rc file will only launch loc_launcher if
    // property "vendor.qti.izat.value_added_process" is set to "enabled".
    const char* value = "disabled";
    if (mIzat_process_conf.valueAddedProcessEnabled == true) {
        value = "enabled";
    }

    if (0 != property_set("vendor.qti.izat.value_added_process", value)) {
        LOC_LOGe ("failed to set property vendor.qti.izat.value_added_process");
    }
#endif

    if (processInfoList != nullptr) {
        free (processInfoList);
        processInfoList = nullptr;
    }

    LOC_LOGd ("value added process enabled %d, gtp enabled %d, slim daemon enabled %d, "
              "edgnss enabled %d, engine service enabled %d (ppe: %d, ppe-int:%d, dre: %d)",
              mIzat_process_conf.valueAddedProcessEnabled, mIzat_process_conf.gtpDaemonEnabled,
              mIzat_process_conf.slimDaemonEnabled, mIzat_process_conf.eDgnssDaemonEnabled,
              mIzat_process_conf.engineServiceEnabled,
              mIzat_process_conf.engineServiceInfo.ppeEnabled,
              mIzat_process_conf.engineServiceInfo.ppeIntEnabled,
              mIzat_process_conf.engineServiceInfo.dreIntEnabled);
}

uint32_t ContextBase::getCarrierCapabilities() {
    #define carrierMSA (uint32_t)0x2
    #define carrierMSB (uint32_t)0x1
    #define gpsConfMSA (uint32_t)0x4
    #define gpsConfMSB (uint32_t)0x2
    uint32_t capabilities = mGps_conf.CAPABILITIES;
    if ((mGps_conf.SUPL_MODE & carrierMSA) != carrierMSA) {
        capabilities &= ~gpsConfMSA;
    }
    if ((mGps_conf.SUPL_MODE & carrierMSB) != carrierMSB) {
        capabilities &= ~gpsConfMSB;
    }

    LOC_LOGV("getCarrierCapabilities: CAPABILITIES %x, SUPL_MODE %x, carrier capabilities %x",
             mGps_conf.CAPABILITIES, mGps_conf.SUPL_MODE, capabilities);
    return capabilities;
}

LBSProxyBase* ContextBase::getLBSProxy(const char* libName)
{
    LBSProxyBase* proxy = NULL;
    LOC_LOGD("%s:%d]: getLBSProxy libname: %s\n", __func__, __LINE__, libName);
    void* lib = dlopen(libName, RTLD_NOW);

    if ((void*)NULL != lib) {
        getLBSProxy_t* getter = (getLBSProxy_t*)dlsym(lib, "getLBSProxy");
        if (NULL != getter) {
            proxy = (*getter)();
        }
    }
    else
    {
        LOC_LOGW("%s:%d]: FAILED TO LOAD libname: %s\n", __func__, __LINE__, libName);
    }
    if (NULL == proxy) {
        proxy = new LBSProxyBase();
    }
    LOC_LOGD("%s:%d]: Exiting\n", __func__, __LINE__);
    return proxy;
}

LocApiBase* ContextBase::createLocApi(LOC_API_ADAPTER_EVENT_MASK_T exMask)
{
    LocApiBase* locApi = NULL;
    const char* libname = LOC_APIV2_0_LIB_NAME;

    // Check the target
    if (TARGET_NO_GNSS != loc_get_target()){

        if (NULL == (locApi = mLBSProxy->getLocApi(exMask, this))) {
            void *handle = NULL;

            if (QCSR_SS5_ENABLED == mGps_conf.GNSS_DEPLOYMENT) {
                libname = SLL_LOC_API_LIB_NAME;
            }

            if ((handle = dlopen(libname, RTLD_NOW)) != NULL) {
                LOC_LOGD("%s:%d]: %s is present", __func__, __LINE__, libname);
                getLocApi_t* getter = (getLocApi_t*) dlsym(handle, "getLocApi");
                if (getter != NULL) {
                    LOC_LOGD("%s:%d]: getter is not NULL of %s", __func__,
                            __LINE__, libname);
                    locApi = (*getter)(exMask, this);
                }
            }
            // only RPC is the option now
            else {
                LOC_LOGD("%s:%d]: libloc_api_v02.so is NOT present. Trying RPC",
                        __func__, __LINE__);
                handle = dlopen("libloc_api-rpc-qc.so", RTLD_NOW);
                if (NULL != handle) {
                    getLocApi_t* getter = (getLocApi_t*) dlsym(handle, "getLocApi");
                    if (NULL != getter) {
                        LOC_LOGD("%s:%d]: getter is not NULL in RPC", __func__,
                                __LINE__);
                        locApi = (*getter)(exMask, this);
                    }
                }
            }
        }
    }

    // locApi could still be NULL at this time
    // we would then create a dummy one
    if (NULL == locApi) {
        locApi = new LocApiBase(exMask, this);
    }

    return locApi;
}

ContextBase::ContextBase(const MsgTask* msgTask,
                         LOC_API_ADAPTER_EVENT_MASK_T exMask,
                         const char* libName) :
    mLBSProxy(getLBSProxy(libName)),
    mMsgTask(msgTask),
    mLocApi(createLocApi(exMask)),
    mLocApiProxy(mLocApi->getLocApiProxy())
{
}

void ContextBase::setEngineCapabilities(uint64_t supportedMsgMask,
       uint8_t *featureList, bool gnssMeasurementSupported) {

    if (ContextBase::sIsEngineCapabilitiesKnown == false) {
        ContextBase::sSupportedMsgMask = supportedMsgMask;
        ContextBase::sGnssMeasurementSupported = gnssMeasurementSupported;
        if (featureList != NULL) {
            memcpy((void *)ContextBase::sFeaturesSupported,
                    (void *)featureList, sizeof(ContextBase::sFeaturesSupported));
        }
        mGps_conf.AGPS_CONFIG_INJECT &=
                !(isFeatureSupported(LOC_SUPPORTED_FEATURE_DSDA_CONFIGURATION));

        /* */
        if (ContextBase::isFeatureSupported(LOC_SUPPORTED_FEATURE_MEASUREMENTS_CORRECTION)) {
            static uint8_t isSapModeKnown = 0;

            if (!isSapModeKnown) {
                /* Check if SAP is PREMIUM_ENV_AIDING in izat.conf */
                char conf_feature_sap[LOC_MAX_PARAM_STRING];
                loc_param_s_type izat_conf_feature_table[] =
                {
                    { "SAP",           &conf_feature_sap,           &isSapModeKnown, 's' }
                };
                UTIL_READ_CONF(LOC_PATH_IZAT_CONF, izat_conf_feature_table);

                /* Disable this feature if SAP is not PREMIUM_ENV_AIDING in izat.conf */
                if (strcmp(conf_feature_sap, "PREMIUM_ENV_AIDING") != 0) {
                    uint8_t arrayIndex = LOC_SUPPORTED_FEATURE_MEASUREMENTS_CORRECTION >> 3;
                    uint8_t bitPos = LOC_SUPPORTED_FEATURE_MEASUREMENTS_CORRECTION & 7;

                    if (arrayIndex < MAX_FEATURE_LENGTH) {
                        /* To disable the feature we need to reset the bit on the "bitPos"
                           position, so shift a "1" to the left by "bitPos" */
                        ContextBase::sFeaturesSupported[arrayIndex] &= ~(1 << bitPos);
                    }
                }
            }
        }
        ContextBase::sIsEngineCapabilitiesKnown = true;
    }
}


bool ContextBase::isFeatureSupported(uint8_t featureVal)
{
    uint8_t arrayIndex = featureVal >> 3;
    uint8_t bitPos = featureVal & 7;

    if (arrayIndex >= MAX_FEATURE_LENGTH) return false;
    return ((ContextBase::sFeaturesSupported[arrayIndex] >> bitPos ) & 0x1);
}

bool ContextBase::gnssConstellationConfig() {
    return sGnssMeasurementSupported;
}

}
