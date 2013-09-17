/**
 * @class     UsbHandler
 *
 * @brief     Helper class for basic USB operations (like resetting the device).
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef USB_HANDLER_HPP_
#define USB_HANDLER_HPP_

class UsbHandler
{
    public:
        /**
         * Sends the USBDEVFS_RESET command to the given USB device.
         * @param device USB device's path.
         * @returns Operation's success status.
         */
        bool resetDevice(const std::string& device);
};


#endif /* USB_HANDLER_HPP_ */
