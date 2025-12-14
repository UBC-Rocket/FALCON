#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Mount point for the SD card
#define MOUNT_POINT "/SDCARD"

// FATFS mount structure
static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS,
    .mnt_point = MOUNT_POINT,
};

void write_test_log(void) {
    struct fs_file_t file;
    int ret;

    // Open or create a file for writing
    ret = fs_open(&file, MOUNT_POINT "/test_log.txt", FS_O_CREATE | FS_O_WRITE);
    if (ret) {
        LOG_ERR("Failed to open file: %d", ret);
        return;
    }

    // Write some test logs to the file
    const char *log_data = "This is a test log written to the SD card.\n";
    ret = fs_write(&file, log_data, strlen(log_data));
    if (ret < 0) {
        LOG_ERR("Failed to write to file: %d", ret);
    } else {
        LOG_INF("Wrote %d bytes to file", ret);
    }

    // Close the file
    fs_close(&file);
}

void mount_sd_card(void) {
    int ret;

    // Initialize the SDMMC disk
    ret = disk_access_init("SDMMC");
    if (ret) {
        LOG_ERR("Disk access initialization failed: %d", ret);
        return;
    }

    // Mount the FATFS file system
    ret = fs_mount(&fatfs_mnt);
    if (ret == 0) {
        LOG_INF("File system mounted at %s", fatfs_mnt.mnt_point);
    } else {
        LOG_ERR("File system mount failed: %d", ret);
    }
}

int main(void) {
    LOG_INF("SD Test Application Started");

    // Mount the SD card
    mount_sd_card();

    // Write a test log to the SD card
    write_test_log();

    return 0;
}