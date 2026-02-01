#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <ff.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SDMMC_NODE DT_NODELABEL(sdmmc1)
#define DISK_DRIVE_NAME DT_PROP(SDMMC_NODE, disk_name)
#define MOUNT_POINT "/" DISK_DRIVE_NAME ":"

static FATFS fat_fs;

static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS, .mnt_point = MOUNT_POINT, .fs_data = &fat_fs};

void write_test_log(void)
{
    struct fs_file_t file;
    int ret;
    fs_file_t_init(&file);

    // Open or create a file for writing
    ret = fs_open(&file, MOUNT_POINT "/log.txt", FS_O_CREATE | FS_O_WRITE);
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

void mount_sd_card(void)
{
    int ret;

    // Initialize the SDMMC disk
    // ret = disk_access_init("SDMMC");
    ret = disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL);
    if (ret < 0) {
        LOG_ERR("Disk access initialization failed: %d", ret);
        return;
    }
    LOG_INF("init return code: %d", ret);

    // Mount the FATFS file system
    ret = fs_mount(&fatfs_mnt);
    if (ret == 0) {
        LOG_INF("File system mounted at %s", fatfs_mnt.mnt_point);
    } else {
        LOG_ERR("File system mount failed: %d", ret);
    }
}

int main(void)
{
    LOG_INF("SD Test Application Started");

    // Mount the SD card
    mount_sd_card();

    // Write a test log to the SD card
    write_test_log();

    int res = fs_unmount(&fatfs_mnt);
    if (res != 0) {
        printk("Error unmounting disk\n");
        return res;
    }

    return 0;
}
