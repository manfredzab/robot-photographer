/**
 * @class      RPUtils
 *
 * @brief      Utilities class for the head detection/tracking node.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <constants.hpp>

class RPUtils
{
    public:
        /**
         * Clamps a given rectangle to the given frame.
         * @param rectangle    Input/output rectangle.
         * @param frame_width  Frame width.
         * @param frame_height Frame height.
         */
        static void clampRectangleToFrame(cv::Rect* rectangle, int frame_width = FRAME_WIDTH, int frame_height = FRAME_HEIGHT)
        {
            cv::Rect frame_rectangle(0, 0, frame_width, frame_height);
            *rectangle = frame_rectangle & (*rectangle);
        }

        /**
         * Scales a given rectangle by a fixed percentage.
         * @param rectangle        Input rectangle.
         * @param size_percentage  Percentage by which to scale a given rectangle.
         * @returns Scaled rectangle.
         */
        static cv::Rect resizeRectangle(const cv::Rect& rectangle, double size_percentage)
        {
            double center_x = rectangle.x + rectangle.width * 0.5;
            double center_y = rectangle.y + rectangle.height * 0.5;

            double new_width = rectangle.width * size_percentage;
            double new_height = rectangle.height * size_percentage;

            return cv::Rect(center_x - new_width * 0.5, center_y - new_height * 0.5, new_width, new_height);
        }


        /**
         * Copies the input sensor image to an OpenCV image.
         * @param input_image Input sensor image.
         * @returns Copied OpenCV image.
         */
        static cv_bridge::CvImage::Ptr copyToCvImage(const sensor_msgs::Image::ConstPtr& input_image)
        {
            cv_bridge::CvImage::Ptr result;
            try
            {
                result = cv_bridge::toCvCopy(input_image, input_image->encoding);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return result;
            }

            return result;
        }

        /**
         * Converts the input sensor image to an OpenCV image (without copying the data).
         * @param input_image Input sensor image.
         * @returns Converted OpenCV image.
         */
        static cv_bridge::CvImage::ConstPtr shareToCvImage(const sensor_msgs::Image::ConstPtr& input_image)
        {
            cv_bridge::CvImage::ConstPtr result;
            try
            {
                result = cv_bridge::toCvShare(input_image, input_image->encoding);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return result;
            }

            return result;
        }

        /**
         * Checks if the saturation is valid (i.e. is pixel not saturated too low).
         * @param saturation Input saturation
         * @returns True if the pixel's saturation is valid, and false otherwise.
         */
        static bool isSaturationValid(uint8_t saturation)
        {
            return (saturation > HEAD_MINIMUM_SATURATION);
        }

        /**
         * Checks if the pixel's value/brightness (in HSV color space) is valid.
         * @param value Input pixel's value.
         * @returns True if the pixel's value is valid, and false otherwise.
         */
        static bool isValueValid(uint8_t value)
        {
            return (value > HEAD_MINIMUM_BRIGHTNESS) && (value < HEAD_MAXIMUM_BRIGHTNESS);
        }

        /**
         * Measures rectangle similarity.
         * @param rectangle_a First rectangle.
         * @param rectangle_b Second rectangle.
         * @returns true if rectangles are sufficiently similar, and false otherwise.
         */
        static bool areRectanglesSufficientlySimilar(const cv::Rect& rectangle_a, const cv::Rect& rectangle_b)
        {
            cv::Rect intersection_rectangle = rectangle_a & rectangle_b;
            return (intersection_rectangle.area() > 0);

            // // Jaccard's coefficient
            // double overlap_coefficient = (double)intersection_rectangle.area() / (double)(rectangle_a.area() + rectangle_b.area() - intersection_rectangle.area());
            // return (overlap_coefficient > 0.1);
        }

};

#endif /* UTILS_HPP_ */
