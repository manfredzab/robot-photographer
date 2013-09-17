/**
 * @class      RPDepthHeadTracker
 *
 * @brief      Head tracker in depth images.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DEPTH_HEAD_TRACKER_HPP_
#define DEPTH_HEAD_TRACKER_HPP_

// STL includes
#include <vector>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <depth_head_detector.hpp>
#include <heads_message_builder.hpp>

// Constants
#define CAMSHIFT_CONVERGENCE_THRESHOLD_PERCENTAGE 0.95
#define CAMSHIFT_ITERATION_THRESHOLD 2
#define MEANSHIFT_ITERATION_THRESHOLD 2
#define SEARCH_WINDOW_SCALE 2.0

class RPDepthHeadTracker
{
    public:
        /**
         * Tracks heads from a given depth image.
         * @param depth_image     Depth image.
         * @param depth_detector  Depth head detector.
         * @param heads           Tracked heads.
         */
        void trackHeads(const cv_bridge::CvImage::ConstPtr& depth_image, RPDepthHeadDetector& depth_detector, std::vector<Head>& heads);
};

#endif /* DEPTH_HEAD_TRACKER_HPP_ */
