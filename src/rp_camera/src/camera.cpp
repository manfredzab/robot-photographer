/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <camera.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_camera");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPCameraNode workerNode(node);
}


RPCameraNode::RPCameraNode(ros::NodeHandle& node) :
    node(node),
    FLASH_AUTOMATIC(false),
    FLASH_ENABLED(true),
    USB_RESET_ENABLED(false)
{
    // Get overridable parameters from the parameter server
    getOverridableParameters();

    // Initialize the flash
    bool flash_mode_updated = gphoto2_handler.setFlashMode(FLASH_AUTOMATIC, FLASH_ENABLED);
    ROS_WARN_COND(!flash_mode_updated, "Cannot update the flash mode.");

    // Start the photo service
    photo_service = node.advertiseService("/rp/camera/photo", &RPCameraNode::takePhoto, this);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
};


void RPCameraNode::getOverridableParameters()
{
    node.getParamCached("/rp/camera_node/flash_automatic", FLASH_AUTOMATIC);
    node.getParamCached("/rp/camera_node/flash_enabled", FLASH_ENABLED);
    node.getParamCached("/rp/camera_node/usb_device", USB_DEVICE);
    node.getParamCached("/rp/camera_node/usb_reset_enabled", USB_RESET_ENABLED);
}


bool RPCameraNode::takePhoto(rp_camera::PhotoService::Request &request, rp_camera::PhotoService::Response &response)
{
    bool previous_flash_automatic = FLASH_AUTOMATIC;
    bool previous_flash_enabled = FLASH_ENABLED;

    getOverridableParameters(); // Load the overridable parameters from the parameter server

    if (USB_RESET_ENABLED)
    {
        usb_handler.resetDevice(USB_DEVICE);
    }

    if (previous_flash_automatic != FLASH_AUTOMATIC || previous_flash_enabled != FLASH_ENABLED)
    {
        bool flash_mode_updated = gphoto2_handler.setFlashMode(FLASH_AUTOMATIC, FLASH_ENABLED);
        ROS_WARN_COND(!flash_mode_updated, "Cannot update the flash mode.");
    }

    response.picture_file_name = gphoto2_handler.takePicture();

    return !response.picture_file_name.empty();
}
