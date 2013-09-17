/**
 * @class      RPMockHeadTrackingNode
 *
 * @brief      Mock head tracking node that publishes randomly generated head detections, which
 *             evolve using a random walk over the scene. Useful for framing/composition algorithm
 *             testing.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef MOCK_HEAD_TRACKING_HPP_
#define MOCK_HEAD_TRACKING_HPP_

// STL includes
#include <vector>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <heads_message_builder.hpp>

#define HEAD_COUNT_DEFAULT 3

class RPMockHeadTrackingNode
{
    private:
        ros::NodeHandle& node;                  /**< ROS node's handle.                          */

        unsigned int frame_count;               /**< Frame count.                                */

        int HEAD_COUNT;                         /**< Simulated head count overridable parameter. */
        std::vector<Head> heads;                /**< Simulated heads.                            */
        ros::Publisher heads_publisher;         /**< Simulated heads publisher.                  */

        cv_bridge::CvImage::Ptr depth_image;    /**< Simulated depth image.                      */

    public:
        /**
         * Default mock head tracking node constructor.
         * @param node  Node's handle.
         */
        RPMockHeadTrackingNode(ros::NodeHandle& node);

        /**
         * Gets overridable parameters from the parameter server.
         */
        void getOverridableParameters();

        /**
         * Generates and publishes mock heads.
         */
        void generateAndPublishMockHeads();

        /**
         * Generates a single mock head.
         * @param generated_head  Resulting generated mock head rectangle.
         */
        void generateMockHead(cv::Rect& generated_head);

        /**
         * Publishes simulated heads.
         */
        void publishHeads();

        /**
         * Initializes a random depth image.
         * @param depth_image  Generated random depth image.
         */
        void initializeRandomDepthImage(const cv_bridge::CvImage::Ptr& depth_image);
};

#endif /* MOCK_HEAD_TRACKING_HPP_ */
