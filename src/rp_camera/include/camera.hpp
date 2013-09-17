/**
 * @class     RPCameraNode
 *
 * @brief     Robot photographer's node which takes the pictures using the photographic camera
 *            via the gphoto2 library.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef CAMERA_HPP_
#define CAMERA_HPP_

// ROS includes
#include <ros/ros.h>
#include <rp_camera/PhotoService.h>

// STL includes
#include <string>

// Project includes
#include "gphoto2_handler.hpp"
#include "usb_handler.hpp"

class RPCameraNode
{
    private:
        ros::NodeHandle& node;              /**< Node's handle.                              */

        ros::ServiceServer photo_service;   /**< Advertised photo service.                   */

        GPhoto2Handler gphoto2_handler;     /**< GPhoto2 handler.                            */
        UsbHandler usb_handler;             /**< USB reset handler.                          */

        bool FLASH_AUTOMATIC;               /**< Automatic flash overridable parameter (OP). */
        bool FLASH_ENABLED;                 /**< Flash enabled/disabled flag OP.             */
        std::string USB_DEVICE;             /**< USB device path OP.                         */
        bool USB_RESET_ENABLED;             /**< USB device reset enabled/disabled flag OP.  */

        /**
         * Callback for the "photograph taking" ROS service request.
         * @param request  Empty ROS service request.
         * @param response ROS photo service response, containing the file name of the taken
         *                 picture.
         * @returns Operation's success status.
         */
        bool takePhoto(rp_camera::PhotoService::Request &request, rp_camera::PhotoService::Response &response);

        /**
         * Gets overridable parameters from the parameter server.
         */
        void getOverridableParameters();
    public:
        /**
         * Default photographic camera node's constructor.
         * @param node Handle to ROS node.
         */
        RPCameraNode(ros::NodeHandle& node);
};

#endif /* CAMERA_HPP_ */
