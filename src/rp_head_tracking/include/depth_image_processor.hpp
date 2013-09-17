/**
 * @class      RPDepthImageProcessor
 *
 * @brief      Depth image processor class, which provides implementations for simple depth
 *             image manipulation tasks.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DEPTH_IMAGE_PROCESSOR_HPP_
#define DEPTH_IMAGE_PROCESSOR_HPP_

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <constants.hpp>

class RPDepthImageProcessor
{
    private:
        float cumulative_row_sum[FRAME_HEIGHT + 1][FRAME_WIDTH + 1]; /**< Row storage for Viola-Jones
                                                                          integral image smoothing. */
        float integral_image[FRAME_HEIGHT + 1][FRAME_WIDTH + 1];     /**< Integral image.           */

        /**
         * Converts a HSV pixel to RGB vector.
         * @param h  Pixel's hue.
         * @param s  Pixel's saturation.
         * @param v  Pixel's value.
         * @returns RGB vector representation of a given pixel.
         */
        cv::Vec3b hsvToRgb(float h, float s, float v);

    public:
        /**
         * Smooths a given depth image (in place) using Viola-Jones integral image.
         * @param depth_image  Depth image.
         * @param radius       Smoothing radius.
         */
        void smoothDepthImage(const cv_bridge::CvImage::Ptr& depth_image, const int radius);

        /**
         * Filters depth shadows from a given depth image (in place).
         * @param depth_image  Depth image.
         */
        void filterDepthShadow(const cv_bridge::CvImage::Ptr& cv_image);

        /**
         * Filters missing depth data from a given depth image (in place).
         * @param depth_image  Depth image.
         */
        void filterMissingData(const cv_bridge::CvImage::Ptr& depth_image);

        /**
         * Converts depth image to a render image.
         * @param depth_image   Depth image.
         * @param render_image  Output render image.
         */
        void convertDepthImageToRenderImage(const cv_bridge::CvImage::ConstPtr& depth_image, cv::Mat& render_image);
};

#endif /* DEPTH_IMAGE_PROCESSOR_HPP_ */
