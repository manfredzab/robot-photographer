/**
 * @class     GPhoto2Handler
 *
 * @brief     Gphoto2 library wrapper for ROS framework.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef GPHOTO2_HANDLER_HPP_
#define GPHOTO2_HANDLER_HPP_

#include <string>
#include <gphoto2/gphoto2-camera.h>

class GPhoto2Handler
{
    private:
        Camera *camera;     /**< Gphoto2 camera object handle.  */
        GPContext *context; /**< Gphoto2 context object handle. */

    public:
        GPhoto2Handler();
        virtual ~GPhoto2Handler();

        /**
         * Sets the camera's flash mode.
         * @param flash_automatic Automatic flash flag (true - enabled/false - disabled).
         * @param flash_enabled   Desired flash mode (if the automatic flash is disabled).
         * @returns Operation's success status.
         */
        bool setFlashMode(bool flash_automatic, bool flash_enabled);

        /**
         * Takes the picture using the photographic camera and returns the string file
         * path to where the taken picture is saved.
         * @returns Path to taken picture (or empty string, if the picture cannot be taken).
         */
        std::string takePicture();
};

#endif /* GPHOTO2_HANDLER_HPP_ */
