/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <ros/ros.h>

// C++ includes
#include <cstdlib>
#include <cstdio>

// C includes
#include <fcntl.h>

// Class header include
#include "../include/gphoto2_handler.hpp"


void errorCallback(GPContext *context, const char *format, va_list args, void *data);
void messageCallback(GPContext *context, const char *format, va_list args, void *data);


GPhoto2Handler::GPhoto2Handler()
{
    gp_camera_new(&camera);
    context = gp_context_new();

    gp_context_set_error_func(context, errorCallback, NULL);
    gp_context_set_message_func(context, messageCallback, NULL);

    int return_code = gp_camera_init(camera, context);
    if (return_code < GP_OK)
    {
        ROS_ERROR("%s", "No camera detected.");
        gp_camera_free(camera);
    }
}

GPhoto2Handler::~GPhoto2Handler()
{
    gp_camera_unref(camera);
    gp_context_unref(context);
}


std::string GPhoto2Handler::takePicture()
{
    std::string empty_filename;

    CameraFilePath camera_file_path;
    int return_code = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);

    if (return_code)
    {
        ROS_ERROR("%s", "Could not capture the image from the camera.");
        return empty_filename;
    }

    int file_descriptor = open(camera_file_path.name, O_CREAT | O_WRONLY, 0644);

    CameraFile *file;
    return_code = gp_file_new_from_fd(&file, file_descriptor);

    if (return_code)
    {
        ROS_ERROR("%s", "Could not create a new file to store the image from the camera.");
        return empty_filename;
    }

    return_code = gp_camera_file_get(camera,
                                     camera_file_path.folder,
                                     camera_file_path.name,
                                     GP_FILE_TYPE_NORMAL,
                                     file,
                                     context);

    if (return_code)
    {
        ROS_ERROR("%s", "Could not download the captured image from the camera.");
        return empty_filename;
    }

    return_code = gp_camera_file_delete(camera,
                                        camera_file_path.folder,
                                        camera_file_path.name,
                                        context);

    if (return_code)
    {
        ROS_WARN("%s", "Could not delete the captured image from the camera.");
    }

    gp_file_free(file);

    std::string picture_filename(camera_file_path.name);
    return picture_filename;
}


bool GPhoto2Handler::setFlashMode(bool flash_automatic, bool flash_enabled)
{
    bool success = true;

    // Nikon Coolpix S3100 specific
    const char* flash_mode = flash_automatic ? "Automatic Flash" :
                             flash_enabled   ? "Fill flash" :
                                               "Flash off";

    CameraWidget *widget = NULL;
    int return_code = gp_camera_get_config(camera, &widget, context);

    if (return_code < GP_OK)
    {
        ROS_ERROR("%s", "Could not get the configuration widget for the camera.");
        return false;
    }

    CameraWidget *child = NULL;
    return_code = gp_widget_get_child_by_name(widget, "flashmode", &child);

    if (return_code < GP_OK)
    {
        ROS_ERROR("%s", "Could not get the flash mode widget for the camera.");
        goto fail;
    }

    return_code = gp_widget_set_value(child, flash_mode);
    if (return_code < GP_OK)
    {
        ROS_ERROR("%s", "Could not set the flash mode for the camera.");
        goto fail;
    }

    return_code = gp_camera_set_config(camera, widget, context);
    if (return_code < GP_OK)
    {
        ROS_ERROR("%s", "Could not set the configuration widget for the camera.");
        goto fail;
    }

    success = true;

fail:
    gp_widget_free(widget);

    return success;
}


void errorCallback(GPContext *context, const char *format, va_list args, void *data)
{
    ROS_ERROR(format, args);
}


void messageCallback(GPContext *context, const char *format, va_list args, void *data)
{
    ROS_INFO(format, args);
}
