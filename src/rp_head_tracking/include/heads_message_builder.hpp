/**
 * @class      RPHeadsMessageBuilder
 *
 * @brief      Detected/tracked head message builder class.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef HEADS_MESSAGE_BUILDER_HPP_
#define HEADS_MESSAGE_BUILDER_HPP_

// STL includes
#include <vector>
#include <string>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking message includes
#include <rp_head_tracking/Heads.h>

/**
 * @struct  Head
 * @brief   Detected/tracked head structure.
 */
struct Head
{
    cv::Rect rectangle;         /**< Head's rectangle.         */
    int last_detected_history;  /**< Head's detection history. */
};

class RPHeadsMessageBuilder
{
    public:
        /**
         * Builds a detected/tracked head message.
         * @param frame_id      Frame ID.
         * @param frame_number  Current frame number.
         * @param time_stamp    Current frame time stamp.
         * @param heads         Detected heads.
         * @param depth_image   Depth image.
         * @param heads_message Built heads message.
         */
        static void buildHeadsMessage(const std::string& frame_id,
                                      const unsigned int frame_number,
                                      const ros::Time& time_stamp,
                                      const std::vector<Head>& heads,
                                      const cv_bridge::CvImage::ConstPtr& depth_image,
                                      rp_head_tracking::Heads& heads_message);
};

#endif /* HEADS_MESSAGE_BUILDER_HPP_ */
