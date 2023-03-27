//
// Created by tyler on 3/10/23.
//
#include <cstdio>
#include <cstring>
#include "livox_sdk.h"
#include "../BaslerCalibration/basler_config.h"
#include "LidarCapture.h"
#include "../Mapper/projectPoints.h"
#include "../Mapper/loadCalib.h"

typedef enum {
    kDeviceStateDisconnect = 0,
    kDeviceStateConnect = 1,
    kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
    uint8_t handle;
    DeviceState device_state;
    DeviceInfo info;
} DeviceItem;

DeviceItem devices[kMaxLidarCount];
static lidarPoint lidarDepths[2 * BUFFER_SIZE];
static int bufferPosition = 0;
static char bufferInd = 0;
static CalibData *CDATA;

lidarPoint *LidarCapture::get_raw_data() {
    int otherBufferInd = (bufferInd + 1) % 2;
    return &lidarDepths[otherBufferInd * BUFFER_SIZE];
}

sphereCenter LidarCapture::findSphere(float cx, float cy, float r) {
    float r2 = r * r;
    int foundCount = 0;
    lidarPoint foundPoints[2] = {};

    while (true) {
        bool found = false;
        auto buffer = LidarCapture::get_raw_data();
        for (int i = 0; i < BUFFER_SIZE; i++) {
            auto px = buffer[i].px;
            auto px2 = px * px;
            auto py = buffer[i].py;
            auto py2 = py * py;
            if (px2 + py2 < r2) {
                foundPoints[foundCount] = buffer[i];
                foundCount++;
                if (foundCount == 2) {
                    found = true;
                    break;
                }
            }
        }
        // Wait until new buffer
//        if (!found) { while (buffer == LidarCapture::get_raw_data()) {}}
//        else { break; }
        break;
    }
    float px1, px2, py1, py2, pz1, pz2, pd2, x1, x2, y1, y2, z1, z2, d2, scalingRatio, centerX, centerY, centerZ, radius;
    px1 = foundPoints[0].px - cx;
    px2 = foundPoints[1].px - cx;
    py1 = foundPoints[0].py - cy;
    py2 = foundPoints[1].py - cy;
    pz1 = sqrtf(r2 - px1 * px1 - py1 * py1);
    pz2 = sqrtf(r2 - px2 * px2 - py2 * py2);
    pd2 = (px1 - px2) * (px1 - px2) + (py1 - py2) * (py1 - py2) + (pz1 - pz2) * (pz1 - pz2);
    x1 = foundPoints[0].x;
    x2 = foundPoints[1].x;
    y1 = foundPoints[0].y;
    y2 = foundPoints[1].y;
    z1 = foundPoints[0].z;
    z2 = foundPoints[1].z;
    d2 = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2);
    scalingRatio = sqrtf(d2 / pd2);

    centerX = x1 - px1 * scalingRatio;
    centerY = y1 - py1 * scalingRatio;
    centerZ = z1 - pz1 * scalingRatio;
    radius = r * scalingRatio;
    return sphereCenter{
            centerX,
            centerY,
            centerZ,
            radius
    };
}

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
    static uint32_t error_message_count = 0;
    if (message != nullptr) {
        ++error_message_count;
        if (0 == (error_message_count % 100)) {
            printf("handle: %u\n", handle);
            printf("temp_status : %u\n", message->lidar_error_code.temp_status);
            printf("volt_status : %u\n", message->lidar_error_code.volt_status);
            printf("motor_status : %u\n", message->lidar_error_code.motor_status);
            printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
            printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
            printf("pps_status : %u\n", message->lidar_error_code.device_status);
            printf("fan_status : %u\n", message->lidar_error_code.fan_status);
            printf("self_heating : %u\n", message->lidar_error_code.self_heating);
            printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
            printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
            printf("system_status : %u\n", message->lidar_error_code.system_status);
        }
    }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
    if (data && data_num > 0) {
        /** Parsing the timestamp and the point cloud data. */
        if (data->data_type == kCartesian) {
            auto points = (LivoxRawPoint *) data->data;
            Projector::project_points_livox(points, data_num, CDATA->trans, CDATA->rot, CDATA->camMat, CDATA->distCoeffs, lidarDepths, BUFFER_SIZE, bufferInd, bufferPosition);
        } else if (data->data_type == kExtendCartesian) {
            auto points = (LivoxExtendRawPoint *) data->data;
            Projector::project_points_livox(points, data_num, CDATA->trans, CDATA->rot, CDATA->camMat, CDATA->distCoeffs, lidarDepths, BUFFER_SIZE, bufferInd, bufferPosition);
        } else if (data->data_type == kSpherical) {
            return;
        }
    }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
    printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
    if (status == kStatusSuccess) {
        if (response != 0) {
            devices[handle].device_state = kDeviceStateConnect;
        }
    } else if (status == kStatusTimeout) {
        devices[handle].device_state = kDeviceStateConnect;
    }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
    printf("Stopping sampling...");
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
    if (status != kStatusSuccess) {
        printf("Device Query Information Failed %d\n", status);
    }
    if (ack) {
        printf("firm ver: %d.%d.%d.%d\n",
               ack->firmware_version[0],
               ack->firmware_version[1],
               ack->firmware_version[2],
               ack->firmware_version[3]);
    }
}

void LidarConnect(const DeviceInfo *info) {
    uint8_t handle = info->handle;
    QueryDeviceInformation(handle, OnDeviceInformation, nullptr);
    if (devices[handle].device_state == kDeviceStateDisconnect) {
        devices[handle].device_state = kDeviceStateConnect;
        devices[handle].info = *info;
    }
}

void LidarDisConnect(const DeviceInfo *info) {
    uint8_t handle = info->handle;
    devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
    uint8_t handle = info->handle;
    devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
    if (info == nullptr) {
        return;
    }

    uint8_t handle = info->handle;
    if (handle >= kMaxLidarCount) {
        return;
    }
    if (type == kEventConnect) {
        LidarConnect(info);
        printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
    } else if (type == kEventDisconnect) {
        LidarDisConnect(info);
        printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
    } else if (type == kEventStateChange) {
        LidarStateChange(info);
        printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
    }

    if (devices[handle].device_state == kDeviceStateConnect) {
        printf("Device Working State %d\n", devices[handle].info.state);
        if (devices[handle].info.state == kLidarStateInit) {
            printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
        } else {
            printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
        }
        printf("Device feature %d\n", devices[handle].info.feature);
        SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
        if (devices[handle].info.state == kLidarStateNormal) {
            LidarStartSampling(handle, OnSampleCallback, nullptr);
            devices[handle].device_state = kDeviceStateSampling;
        }
    }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
    if (info == nullptr || info->dev_type == kDeviceTypeHub) {
        return;
    }

    printf("Receive Broadcast Code %s\n", info->broadcast_code);

    if (lidar_count > 0) {
        bool found = false;
        int i = 0;
        for (i = 0; i < lidar_count; ++i) {
            if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
                found = true;
                break;
            }
        }
        if (!found) {
            return;
        }
    }

    bool result = false;
    uint8_t handle = 0;
    result = AddLidarToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess) {
        /** Set the point cloud data for a specific Livox LiDAR. */
        SetDataCallback(handle, GetLidarData, nullptr);
        devices[handle].handle = handle;
        devices[handle].device_state = kDeviceStateDisconnect;
    }
}

int LidarCapture::init() {
    cv::Mat camMat, distCoeffs, trans, rot;
    loadCalib(camMat, distCoeffs, trans, rot);
    CalibData cdata {
        camMat, distCoeffs, trans, rot
    };

    return LidarCapture::start(&cdata);
}

int LidarCapture::start(CalibData *cdata) {
    printf("Livox SDK initializing.\n");
/** Initialize Livox-SDK. */
    if (!Init()) {
        return -1;
    }
    printf("Livox SDK has been initialized.\n");

    CDATA = cdata;

    LivoxSdkVersion sdkVersion;
    GetLivoxSdkVersion(&sdkVersion);
    printf("Livox SDK version %d.%d.%d .\n", sdkVersion.major, sdkVersion.minor, sdkVersion.patch);

    memset(devices, 0, sizeof(devices));

/** Set the callback function receiving broadcast message from Livox LiDAR. */
    SetBroadcastCallback(OnDeviceBroadcast);

/** Set the callback function called when device state change,
 * which means connection/disconnection and changing of LiDAR state.
 */
    SetDeviceStateUpdateCallback(OnDeviceInfoChange);

/** Start the device discovering routine. */
    if (!Start()) {
        Uninit();
        return -1;
    }
    printf("Start discovering device.\n");

    return 0;
}

int LidarCapture::stop() {
    int i = 0;
    for (i = 0; i < kMaxLidarCount; ++i) {
        if (devices[i].device_state == kDeviceStateSampling) {
/** Stop the sampling of Livox LiDAR. */
            LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
        }
    }

/** Uninitialized Livox-SDK. */
    Uninit();

    printf("Done!\n");
    return 0;
}