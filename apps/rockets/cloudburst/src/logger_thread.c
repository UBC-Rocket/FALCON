#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include "data.h"
#include "log_format.h"

LOG_MODULE_REGISTER(logger_thread, LOG_LEVEL_INF);

#define LOGGER_THREAD_STACK_SIZE 2048
#define LOGGER_THREAD_PRIORITY   7
#define LOGGER_THREAD_PERIOD_MS  50
#define LOGGER_SYNC_PERIOD_MS    500

#define SDMMC_NODE DT_NODELABEL(sdmmc1)
#define DISK_DRIVE_NAME DT_PROP(SDMMC_NODE, disk_name)
#define MOUNT_POINT "/" DISK_DRIVE_NAME ":"

static FATFS fat_fs;
static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS,
    .mnt_point = MOUNT_POINT,
    .fs_data = &fat_fs
};

K_THREAD_STACK_DEFINE(logger_stack, LOGGER_THREAD_STACK_SIZE);
static struct k_thread logger_thread;

static struct fs_file_t log_file;
static char log_file_name[32];

static int mount_sd_card(void) {
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
        LOG_INF("File system mounted at %s", fatfs_mnt.mnt_point);
    } else {
        LOG_ERR("File system mount failed: %d", ret);
    }

    return ret;
}

static int write_csv_header(void) {
    const char *header = "Log_Timestamp(ms),"
                         "IMU_Timestamp(ms),Accel_X(m/s^2),Accel_Y(m/s^2),Accel_Z(m/s^2),"
                         "Gyro_X(rad/s),Gyro_Y(rad/s),Gyro_Z(rad/s),"
                         "Baro_Timestamp(ms),"
                         "Baro0_Pressure(Pa),Baro0_Temperature(C),Baro0_Altitude(m),Baro0_NIS,Baro0_Faults,Baro0_Healthy,"
                         "Baro1_Pressure(Pa),Baro1_Temperature(C),Baro1_Altitude(m),Baro1_NIS,Baro1_Faults,Baro1_Healthy,"
                         "KF_Altitude(m),KF_AltVar,KF_Velocity(m/s),KF_VelVar\n";
    int ret;

    // Write the header row to the log file
    ret = fs_write(&log_file, header, strlen(header));
    if (ret < 0) {
        LOG_ERR("Failed to write header to log file: %d", ret);
        return ret;
    }

    // Flush the header to the SD card
    ret = fs_sync(&log_file);
    if (ret < 0) {
        LOG_ERR("Failed to sync log file after writing header: %d", ret);
        return ret;
    }

    return 0;
}

static int create_new_log_file(void) {
    int ret;
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int file_count = 0;

    // Open the directory
    fs_dir_t_init(&dir);
    ret = fs_opendir(&dir, MOUNT_POINT);
    if (ret < 0) {
        LOG_ERR("Failed to open directory: %d", ret);
        return ret;
    }

    // Count the number of files in the directory
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            file_count++;
        }
    }

    fs_closedir(&dir);

    // Generate a new log file name
    snprintf(log_file_name, sizeof(log_file_name), MOUNT_POINT "/log_%d.csv", file_count);

    // Initialize the file
    fs_file_t_init(&log_file);

    // Open the file for appending (create if it doesn't exist)
    ret = fs_open(&log_file, log_file_name, FS_O_CREATE | FS_O_APPEND | FS_O_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        return ret;
    }

    LOG_INF("Log file created: %s", log_file_name);

    // Attempt to write the header row to the log file
    ret = write_csv_header();
    if (ret < 0) {
        LOG_ERR("Failed to write header to log file. Continuing without header.");
        // Do not close the file; allow further writes to continue
    }

    return 0;
}

static int format_log_entry(const struct log_frame *frame, char *buffer, size_t buffer_size) {
    return snprintf(buffer, buffer_size,
                    "%lld,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lld,"
                    "%.3f,%.3f,%.3f,%.3f,%u,%d,"
                    "%.3f,%.3f,%.3f,%.3f,%u,%d,"
                    "%.3f,%.3f,%.3f,%.3f\n",
                    frame->log_timestamp,
                    frame->imu.timestamp, // IMU timestamp
                    (double)frame->imu.accel[0],
                    (double)frame->imu.accel[1],
                    (double)frame->imu.accel[2],
                    (double)frame->imu.gyro[0],
                    (double)frame->imu.gyro[1],
                    (double)frame->imu.gyro[2],
                    frame->baro.timestamp, // Barometer timestamp
                    (double)frame->baro.baro0.pressure,
                    (double)frame->baro.baro0.temperature,
                    (double)frame->baro.baro0.altitude,
                    (double)frame->baro.baro0.nis,
                    (unsigned int)frame->baro.baro0.faults,
                    frame->baro.baro0.healthy ? 1 : 0,
                    (double)frame->baro.baro1.pressure,
                    (double)frame->baro.baro1.temperature,
                    (double)frame->baro.baro1.altitude,
                    (double)frame->baro.baro1.nis,
                    (unsigned int)frame->baro.baro1.faults,
                    frame->baro.baro1.healthy ? 1 : 0,
                    (double)frame->baro.altitude,
                    (double)frame->baro.alt_variance,
                    (double)frame->baro.velocity,
                    (double)frame->baro.vel_variance);
}

static void write_log_frame_to_file(const struct log_frame *frame) {
    char log_entry[512];
    int len;

    // Format the log entry as CSV
    len = format_log_entry(frame, log_entry, sizeof(log_entry));

    // Check if the formatted string was truncated
    if (len >= sizeof(log_entry)) {
        LOG_ERR("Log buffer size: %zu, Required size: %d", sizeof(log_entry), len);
        len = sizeof(log_entry) - 1; // Write only up to the buffer size (excluding null terminator)
        return; // Abort to avoid writing incomplete log entry
    } else if (len < 0) {
        LOG_ERR("Failed to format log entry: %d", len);
        return; // Abort if formatting failed
    }

    // Write the log entry to the file
    int ret = fs_write(&log_file, log_entry, len);
    if (ret < 0) {
        LOG_ERR("Failed to write to log file: %d", ret);
        return;
    }
}

static void logger_thread_fn(void *p1, void *p2, void *p3) {
    struct log_frame frame;

    if (mount_sd_card() < 0) {
        return;
    }

    if (create_new_log_file() < 0) {
        return;
    }

    int64_t last_sync_ms = k_uptime_get();

    while (1) {
        // Collect data
        frame.log_timestamp = k_uptime_get();
        get_imu_data(&frame.imu);
        get_baro_data(&frame.baro);

        // Write the data to the log file
        write_log_frame_to_file(&frame);

        if ((frame.log_timestamp - last_sync_ms) >= LOGGER_SYNC_PERIOD_MS) {
            int ret = fs_sync(&log_file);
            if (ret < 0) {
                LOG_ERR("Failed to sync log file: %d", ret);
            } else {
                last_sync_ms = frame.log_timestamp;
            }
        }

        k_sleep(K_MSEC(LOGGER_THREAD_PERIOD_MS));
    }
}

void start_logger_thread() {
    k_thread_create(
        &logger_thread,
        logger_stack,
        K_THREAD_STACK_SIZEOF(logger_stack),
        logger_thread_fn,
        NULL, NULL, NULL,
        LOGGER_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
}
