/*
 * Copyright (C) 2015 The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "PowerHAL"

#include <hardware/hardware.h>
#include <hardware/power.h>

#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <errno.h>
#include <sys/poll.h>
#include <pthread.h>
#include <linux/netlink.h>
#include <stdlib.h>
#include <stdbool.h>

#include <utils/Log.h>

#include "power.h"

#define STATE_ON "state=1"
#define STATE_OFF "state=0"

#define MAX_LENGTH         50
#define BOOST_SOCKET       "/dev/socket/pb"

#define POWERSAVE_MIN_FREQ 384000
#define POWERSAVE_MAX_FREQ 1026000
#define BIAS_PERF_MIN_FREQ 1134000
#define NORMAL_MAX_FREQ 1512000

#define MAX_FREQ_LIMIT_PATH "/sys/kernel/cpufreq_limit/limited_max_freq"
#define MIN_FREQ_LIMIT_PATH "/sys/kernel/cpufreq_limit/limited_min_freq"

static int client_sockfd;
static struct sockaddr_un client_addr;
static int last_state = -1;

static pthread_mutex_t profile_lock = PTHREAD_MUTEX_INITIALIZER;

enum {
    PROFILE_POWER_SAVE = 0,
    PROFILE_BALANCED,
    PROFILE_HIGH_PERFORMANCE,
    PROFILE_BIAS_POWER,
    PROFILE_BIAS_PERFORMANCE,
    PROFILE_MAX
};

static int current_power_profile = PROFILE_BALANCED;

#define CPUFREQ_LIMIT_PATH "/sys/kernel/cpufreq_limit/"
#define INTERACTIVE_PATH "/sys/devices/system/cpu/cpufreq/interactive/"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static int boostpulse_fd = -1;

static int current_power_profile = -1;
static int requested_power_profile = -1;

static int sysfs_write_str(char *path, char *s)
{
    char buf[80];
    int len;
    int ret = 0;
    int fd;

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        strerror_r(errno, buf, sizeof(buf));
        ALOGE("Error opening %s: %s\n", path, buf);
        return -1 ;
    }

    len = write(fd, s, strlen(s));
    if (len < 0) {
        strerror_r(errno, buf, sizeof(buf));
        ALOGE("Error writing to %s: %s\n", path, buf);
        ret = -1;
    }

    close(fd);

    return ret;
}

static int sysfs_write_int(char *path, int value)
{
    char buf[80];
    snprintf(buf, 80, "%d", value);
    return sysfs_write(path, buf);
}

static void power_init(__attribute__((unused)) struct power_module *module)
{
    ALOGI("%s", __func__);
    socket_init();
}

static int boostpulse_open()
{
    pthread_mutex_lock(&lock);
    if (boostpulse_fd < 0) {
        boostpulse_fd = open(INTERACTIVE_PATH "boostpulse", O_WRONLY);
    }
    pthread_mutex_unlock(&lock);

    return boostpulse_fd;
}

static void power_set_interactive(__attribute__((unused)) struct power_module *module, int on)
{
    if (!is_profile_valid(current_power_profile)) {
        ALOGD("%s: no power profile selected yet", __func__);
        return;
    }

    // break out early if governor is not interactive
    if (!check_governor()) return;

    if (on) {
        sysfs_write_int(INTERACTIVE_PATH "hispeed_freq",
                        profiles[current_power_profile].hispeed_freq);
        sysfs_write_int(INTERACTIVE_PATH "go_hispeed_load",
                        profiles[current_power_profile].go_hispeed_load);
        sysfs_write_int(INTERACTIVE_PATH "timer_rate",
                        profiles[current_power_profile].timer_rate);
        sysfs_write_str(INTERACTIVE_PATH "target_loads",
                        profiles[current_power_profile].target_loads);
    } else {
        sysfs_write_int(INTERACTIVE_PATH "hispeed_freq",
                        profiles[current_power_profile].hispeed_freq_off);
        sysfs_write_int(INTERACTIVE_PATH "timer_rate",
                        profiles[current_power_profile].timer_rate_off);
        sysfs_write_int(INTERACTIVE_PATH "go_hispeed_load",
                        profiles[current_power_profile].go_hispeed_load_off);
        sysfs_write_str(INTERACTIVE_PATH "target_loads",
                        profiles[current_power_profile].target_loads_off);
    }
}

static void set_power_profile(int profile)
{
    if (!is_profile_valid(profile)) {
        ALOGE("%s: unknown profile: %d", __func__, profile);
        return;
    }

    if (!metadata)
        return;

    if (!strncmp(metadata, STATE_ON, sizeof(STATE_ON))) {
        /* Video encode started */
        sync_thread(1);
        enc_boost(1);
    } else if (!strncmp(metadata, STATE_OFF, sizeof(STATE_OFF))) {
        /* Video encode stopped */
        sync_thread(0);
        enc_boost(0);
    }
}

static void touch_boost()
{
    int on;

    if (!metadata)
        return;

    /* Break out early if governor is not interactive */
    if (!check_governor())
        return;

    on = !strncmp(metadata, STATE_ON, sizeof(STATE_ON));

static void power_set_interactive(__attribute__((unused)) struct power_module *module, int on)
{
    if (last_state == on)
        return;

    last_state = on;

    sysfs_write_int(INTERACTIVE_PATH "io_is_busy", on ?
            VID_ENC_IO_IS_BUSY :
            profiles[current_power_profile].io_is_busy);
}

static void set_power_profile(int profile) {
    int min_freq = POWERSAVE_MIN_FREQ;
    int max_freq = NORMAL_MAX_FREQ;

    ALOGV("%s: profile=%d", __func__, profile);

    switch (profile) {
    case PROFILE_HIGH_PERFORMANCE:
        min_freq = NORMAL_MAX_FREQ;
        max_freq = NORMAL_MAX_FREQ;
        break;
    case PROFILE_BIAS_PERFORMANCE:
        min_freq = BIAS_PERF_MIN_FREQ;
        max_freq = NORMAL_MAX_FREQ;
        break;
    case PROFILE_BIAS_POWER:
        min_freq = POWERSAVE_MIN_FREQ;
        max_freq = POWERSAVE_MAX_FREQ;
        break;
    case PROFILE_POWER_SAVE:
        min_freq = POWERSAVE_MIN_FREQ;
        max_freq = POWERSAVE_MAX_FREQ;
        break;
    default:
        break;
    }

    sysfs_write_int(MIN_FREQ_LIMIT_PATH, min_freq);
    sysfs_write_int(MAX_FREQ_LIMIT_PATH, max_freq);

    current_power_profile = profile;

    ALOGD("%s: set power profile mode: %d", __func__, current_power_profile);
}

static void power_hint( __attribute__((unused)) struct power_module *module,
                      power_hint_t hint, void *data)
{
    if (hint == POWER_HINT_SET_PROFILE) {
        pthread_mutex_lock(&profile_lock);
        set_power_profile(*(int32_t *)data);
        pthread_mutex_unlock(&profile_lock);
        return;
    }

    // Skip other hints in powersave mode
    if (current_power_profile == PROFILE_POWER_SAVE)
        return;

    switch (hint) {
        case POWER_HINT_LAUNCH:
        case POWER_HINT_CPU_BOOST:
            ALOGV("POWER_HINT_INTERACTION");
            touch_boost();
            break;
        case POWER_HINT_VIDEO_ENCODE:
            process_video_encode_hint(data);
            break;
        default:
            break;
    }
}

static struct hw_module_methods_t power_module_methods = {
    .open = NULL,
};

static int get_feature(__attribute__((unused)) struct power_module *module,
                       feature_t feature)
{
    if (feature == POWER_FEATURE_SUPPORTED_PROFILES)
        return PROFILE_MAX;

    return -1;
}

struct power_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = POWER_MODULE_API_VERSION_0_2,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = POWER_HARDWARE_MODULE_ID,
        .name = "msm8960 Power HAL",
        .author = "Gabriele M",
        .methods = &power_module_methods,
    },

    .init = power_init,
    .setInteractive = power_set_interactive,
    .powerHint = power_hint,
    .getFeature = get_feature
};
