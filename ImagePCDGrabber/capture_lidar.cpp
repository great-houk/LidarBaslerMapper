//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// pylon5The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <cstdio>
#include <unistd.h>
#include <cstring>
#include "livox_sdk.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

const uint16_t SECONDS = 60;
const float DIV = 250.0;

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
pcl::PointCloud<pcl::PointXYZI> cloud;

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
    if (data) {
        /** Parsing the timestamp and the point cloud data. */
//        uint64_t cur_timestamp = *((uint64_t *) (data->timestamp));
        if (data->data_type == kCartesian) {
            auto points = (LivoxRawPoint *) data->data;
            for (int i = 0; i < data_num; i++) {
                auto p = points[i];
                // Filter null points?
                if (p.x == 0 && p.y == 0 && p.z == 0 && p.reflectivity == 0) {
                    return;
                }
                pcl::PointXYZI pt;
                // Idk why, but the calibration likes small scales
                pt.x = float(p.x) / DIV;
                pt.y = float(p.y) / DIV;
                pt.z = float(p.z) / DIV;
                pt.intensity = float(p.reflectivity);
                cloud.push_back(pt);
            }
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

int capture_lidar() {
    printf("Reserving point cloud\n");
    cloud = pcl::PointCloud<pcl::PointXYZI>();
    cloud.reserve(SECONDS * 100000);

    printf("Livox SDK initializing.\n");
/** Initialize Livox-SDK. */
    if (!Init()) {
        return -1;
    }
    printf("Livox SDK has been initialized.\n");

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

    sleep(SECONDS);

    int i = 0;
    for (i = 0; i < kMaxLidarCount; ++i) {
        if (devices[i].device_state == kDeviceStateSampling) {
/** Stop the sampling of Livox LiDAR. */
            LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
        }
    }

/** Uninitialized Livox-SDK. */
    Uninit();

    // Write PCD file
    printf("Starting PCD write...\n");

    pcl::io::savePCDFile(MAIN_DIR"/calib/lidar_calib.pcd", cloud, true);

    printf("Done!\n");
    return 0;
}