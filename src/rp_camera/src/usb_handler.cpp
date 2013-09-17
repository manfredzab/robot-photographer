/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <ros/ros.h>

// C includes
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

// Class header include
#include "../include/usb_handler.hpp"

bool UsbHandler::resetDevice(const std::string& device)
{
    int device_file_descriptor = open(device.c_str(), O_WRONLY);
    if (device_file_descriptor < 0)
    {
        return false;
    }

    int return_code = ioctl(device_file_descriptor, USBDEVFS_RESET, 0);
    if (return_code < 0)
    {
        return false;
    }

    close(device_file_descriptor);

    return true;
}
