#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <string.h>
#include "data.h"
#include "log_format.h"

#ifndef CONFIG_BOARD_NATIVE_SIM
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#endif

LOG_MODULE_REGISTER(logger_thread, LOG_LEVEL_INF);

#define LOGGER_THREAD_STACK_SIZE 2048
#define LOGGER_THREAD_PRIORITY 7
#define LOGGER_THREAD_PERIOD_MS 50
#define LOGGER_SYNC_PERIOD_MS 500

#ifdef CONFIG_BOARD_NATIVE_SIM
// Use standard POSIX file I/O for native_sim
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>

#define MOUNT_POINT "/tmp/zephyr_logs"
static FILE *log_file_ptr = NULL;

#else
// Use Zephyr FS API for real hardware
#define SDMMC_NODE DT_ALIAS(sdmmc1)
#define DISK_DRIVE_NAME DT_PROP(SDMMC_NODE, disk_name)
#define MOUNT_POINT "/" DISK_DRIVE_NAME ":"

static FATFS fat_fs;
static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS, .mnt_point = MOUNT_POINT, .fs_data = &fat_fs};

static struct fs_file_t log_file;
#endif

K_THREAD_STACK_DEFINE(logger_stack, LOGGER_THREAD_STACK_SIZE);
static struct k_thread logger_thread;

static char log_file_name[128];

static int mount_filesystem(void)
{
#ifdef CONFIG_BOARD_NATIVE_SIM
    // Create directory if it doesn't exist
    struct stat st = {0};
    if (stat(MOUNT_POINT, &st) == -1) {
        mkdir(MOUNT_POINT, 0755);
    }
    LOG_INF("Using POSIX filesystem at %s", MOUNT_POINT);
    return 0;
#else
    int ret;

    // Initialize the SDMMC disk
    ret = disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL);
    if (ret < 0) {
        LOG_ERR("Disk access initialization failed: %d", ret);
        return ret;
    }

    // Mount the FATFS file system
    ret = fs_mount(&fatfs_mnt);
    if (ret == 0) {
        LOG_INF("File system mounted at %s", MOUNT_POINT);
    } else {
        LOG_ERR("File system mount failed: %d", ret);
    }
    return ret;
#endif
}

static int write_csv_header(void)
{
    const char *header = "Log_Timestamp(ms),"
                         "IMU_Timestamp(ms),Accel_X(m/s^2),Accel_Y(m/s^2),Accel_Z(m/s^2),"
                         "Gyro_X(rad/s),Gyro_Y(rad/s),Gyro_Z(rad/s),"
                         "Baro_Timestamp(ms),"
                         "Baro0_Pressure(Pa),Baro0_Temperature(C),Baro0_Altitude(m),Baro0_NIS,"
                         "Baro0_Faults,Baro0_Healthy,"
                         "Baro1_Pressure(Pa),Baro1_Temperature(C),Baro1_Altitude(m),Baro1_NIS,"
                         "Baro1_Faults,Baro1_Healthy,"
                         "KF_Altitude(m),KF_AltVar,KF_Velocity(m/s),KF_VelVar,"
                         "State,State_Ground_Altitude(m),State_Timestamp(ms),"
                         "Pyro_Status,Pyro_Timestamp(ms),"
                         "Drogue_Fired,Main_Fired,Drogue_Fail,Main_Fail,"
                         "Drogue_Cont_OK,Main_Cont_OK,Drogue_Fire_ACK,Main_Fire_ACK,"
                         "Drogue_Fire_Requested,Main_Fire_Requested\n";
    int ret;

#ifdef CONFIG_BOARD_NATIVE_SIM
    size_t written = fwrite(header, 1, strlen(header), log_file_ptr);
    if (written != strlen(header)) {
        LOG_ERR("Failed to write header");
        return -1;
    }
    fflush(log_file_ptr);
    return 0;
#else
    int ret = fs_write(&log_file, header, strlen(header));
    if (ret < 0) {
        LOG_ERR("Failed to write header to log file: %d", ret);
        return ret;
    }

    ret = fs_sync(&log_file);
    if (ret < 0) {
        LOG_ERR("Failed to sync log file after writing header: %d", ret);
        return ret;
    }
    return 0;
#endif
}

static int create_new_log_file(void)
{
    int file_count = 0;

#ifdef CONFIG_BOARD_NATIVE_SIM
    DIR *dir = opendir(MOUNT_POINT);
    if (dir) {
        struct dirent *entry;
        struct stat st;
        char filepath[256];

        while ((entry = readdir(dir)) != NULL) {
            // Skip "." and ".." entries
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }

            // Build full path and check if it's a regular file
            snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, entry->d_name);
            if (stat(filepath, &st) == 0 && S_ISREG(st.st_mode)) {
                file_count++;
            }
        }
        closedir(dir);
    }

    snprintf(log_file_name, sizeof(log_file_name), "%s/log_%d.csv", MOUNT_POINT, file_count);

    log_file_ptr = fopen(log_file_name, "w");
    if (!log_file_ptr) {
        LOG_ERR("Failed to open log file: %s", log_file_name);
        return -1;
    }

    LOG_INF("Log file created: %s", log_file_name);
#else
    int ret;
    struct fs_dir_t dir;
    struct fs_dirent entry;

    fs_dir_t_init(&dir);
    ret = fs_opendir(&dir, MOUNT_POINT);
    if (ret < 0) {
        LOG_ERR("Failed to open directory: %d", ret);
        return ret;
    }

    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            file_count++;
        }
    }
    fs_closedir(&dir);

    snprintf(log_file_name, sizeof(log_file_name), MOUNT_POINT "/log_%d.csv", file_count);

    fs_file_t_init(&log_file);
    ret = fs_open(&log_file, log_file_name, FS_O_CREATE | FS_O_APPEND | FS_O_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        return ret;
    }

    LOG_INF("Log file created: %s", log_file_name);
#endif

    return write_csv_header();
}

static int format_log_entry(const struct log_frame *frame, char *buffer, size_t buffer_size)
{
    return snprintf(
        buffer, buffer_size,
        "%lld,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lld,"
        "%.3f,%.3f,%.3f,%.3f,%u,%d,"
        "%.3f,%.3f,%.3f,%.3f,%u,%d,"
        "%.3f,%.3f,%.3f,%.3f,%d,%.3f,%lld,"
        "%u,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        frame->log_timestamp,
        frame->imu.timestamp,
        (double)frame->imu.accel[0], (double)frame->imu.accel[1], (double)frame->imu.accel[2],
        (double)frame->imu.gyro[0], (double)frame->imu.gyro[1], (double)frame->imu.gyro[2],
        frame->baro.timestamp,
        (double)frame->baro.baro0.pressure, (double)frame->baro.baro0.temperature,
        (double)frame->baro.baro0.altitude, (double)frame->baro.baro0.nis,
        (unsigned int)frame->baro.baro0.faults, frame->baro.baro0.healthy ? 1 : 0,
        (double)frame->baro.baro1.pressure, (double)frame->baro.baro1.temperature,
        (double)frame->baro.baro1.altitude, (double)frame->baro.baro1.nis,
        (unsigned int)frame->baro.baro1.faults, frame->baro.baro1.healthy ? 1 : 0,
        (double)frame->baro.altitude, (double)frame->baro.alt_variance,
        (double)frame->baro.velocity, (double)frame->baro.vel_variance, (int)frame->state.state,
        (double)frame->state.ground_altitude, frame->state.timestamp,
        (unsigned int)frame->pyro.status_byte, frame->pyro.timestamp,
        frame->pyro.drogue_fired ? 1 : 0, frame->pyro.main_fired ? 1 : 0,
        frame->pyro.drogue_fail ? 1 : 0, frame->pyro.main_fail ? 1 : 0,
        frame->pyro.drogue_cont_ok ? 1 : 0, frame->pyro.main_cont_ok ? 1 : 0,
        frame->pyro.drogue_fire_ack ? 1 : 0, frame->pyro.main_fire_ack ? 1 : 0,
        frame->pyro.drogue_fire_requested ? 1 : 0, frame->pyro.main_fire_requested ? 1 : 0);
}

static void write_log_frame_to_file(const struct log_frame *frame)
{
    char log_entry[512];
    int len = format_log_entry(frame, log_entry, sizeof(log_entry));

    if (len >= sizeof(log_entry)) {
        LOG_ERR("Log buffer size: %zu, Required size: %d", sizeof(log_entry), len);
        return;
    } else if (len < 0) {
        LOG_ERR("Failed to format log entry: %d", len);
        return;
    }

#ifdef CONFIG_BOARD_NATIVE_SIM
    size_t written = fwrite(log_entry, 1, len, log_file_ptr);
    if (written != (size_t)len) {
        LOG_ERR("Failed to write to log file");
    }
#else
    int ret = fs_write(&log_file, log_entry, len);
    if (ret < 0) {
        LOG_ERR("Failed to write to log file: %d", ret);
    }
#endif
}

static void logger_thread_fn(void *p1, void *p2, void *p3)
{
    struct log_frame frame;

    if (mount_filesystem() < 0) {
        return;
    }

    if (create_new_log_file() < 0) {
        return;
    }

    int64_t last_sync_ms = k_uptime_get();

    while (1) {
        frame.log_timestamp = k_uptime_get();
        get_imu_data(&frame.imu);
        get_baro_data(&frame.baro);
        get_state_data(&frame.state);
        get_pyro_data(&frame.pyro);

        write_log_frame_to_file(&frame);

        if ((frame.log_timestamp - last_sync_ms) >= LOGGER_SYNC_PERIOD_MS) {
#ifdef CONFIG_BOARD_NATIVE_SIM
            fflush(log_file_ptr);
#else
            int ret = fs_sync(&log_file);
            if (ret < 0) {
                LOG_ERR("Failed to sync log file: %d", ret);
            }
#endif
            last_sync_ms = frame.log_timestamp;
        }

        k_sleep(K_MSEC(LOGGER_THREAD_PERIOD_MS));
    }
}

void start_logger_thread()
{
    k_thread_create(&logger_thread, logger_stack, K_THREAD_STACK_SIZEOF(logger_stack),
                    logger_thread_fn, NULL, NULL, NULL, LOGGER_THREAD_PRIORITY, 0, K_NO_WAIT);
}
