/**
 * @class      RPHeadTrackingNode
 *
 * @brief      Robot photographer's head tracking node, which uses the colour and depth inputs
 *             from Kinect to detect and track humans in Luke's vicinity.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef HEAD_TRACKING_HPP_
#define HEAD_TRACKING_HPP_

// STL includes
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <head_tracking_state.h>
#include <color_image_processor.hpp>
#include <depth_image_processor.hpp>
#include <color_face_detector.hpp>
#include <depth_head_detector.hpp>
#include <depth_head_tracker.hpp>
#include <heads_message_builder.hpp>
#include <bayesian_skin_classifier.hpp>
#include <kernel_logistic_regression_classifier.hpp>

// Defaults for overridable parameters
#define GUI_OUTPUT_ENABLED_DEFAULT true

#define SKIN_HEURISTIC_ENABLED_DEFAULT true

#define KLR_CLASSIFIER_ENABLED_DEFAULT false
#define KLR_CLASSIFIER_TRAINING_TIME_LIMIT_DEFAULT 30.0
#define FACE_HUE_SAMPLES_LIMIT_DEFAULT 10

#define NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION_DEFAULT 6
#define NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST_DEFAULT 2

#define DEPTH_SHADOW_FILTER_ENABLED_DEFAULT true
#define SMOOTHING_FILTER_ENABLED_DEFAULT true
#define SMOOTHING_RADIUS_DEFAULT 2

class RPHeadTrackingNode
{
    private:
        const std::string WINDOW_NAME;                            /**< Output window's name (if GUI
                                                                       output is enabled).           */
        bool   GUI_OUTPUT_ENABLED;                                /**< GUI output enabled flag OP.   */

        int    NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION;  /**< Number of frames until head
                                                                       detector is reinitialized OP. */
        int    NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST;     /**< Number of head detector's
                                                                       misses for a given head until
                                                                       head is declared lost (OP).   */

        bool   KLR_CLASSIFIER_ENABLED;                            /**< KLR classifier enabled OP.    */
        double KLR_CLASSIFIER_TRAINING_TIME_LIMIT;                /**< KLR classifier training time
                                                                       limit OP.                     */
        int    FACE_HUE_SAMPLES_LIMIT;                            /**< Face hue sample limit OP for
                                                                       KLR classifier training.      */

        double SKIN_LIKELIHOOD_RATIO_MINIMUM;                     /**< Minimum skin likelihood ratio
                                                                       for Bayesian skin classifier. */

        bool   SKIN_HEURISTIC_ENABLED;                            /**< Skin heuristic enabled flag
                                                                       OP.                           */
        bool   DEPTH_SHADOW_FILTER_ENABLED;                       /**< Depth shadow filter enabled
                                                                       flag OP.                      */
        bool   SMOOTHING_FILTER_ENABLED;                          /**< Depth smoothing filter
                                                                       enabled flag OP.              */
        int    SMOOTHING_RADIUS;                                  /**< Depth smoothing radius OP.    */


        volatile bool debug_view_enabled;                         /**< Debug view enabled flag.       */
        volatile bool color_view_enabled;                         /**< Color view enabled flag.       */
        volatile bool backprojection_view_enabled;                /**< Skin probability backprojection
                                                                       view enabled flag.             */
        ros::NodeHandle& node;                                    /**< Node's handle.                 */
        HeadTrackingState state;                                  /**< Node's state.                  */

        uint32_t current_tracking_frame_number;                   /**< Current frame number.          */
        std::string frame_id;                                     /**< Frame ID (same as Kinect's RGB
                                                                       camera ID).                    */
        std::vector<Head> heads;                                  /**< Detected/tracked head
                                                                       rectangles.                    */
        cv::Mat render_image;                                     /**< Image for output rendering.    */

        RPDepthHeadDetector depth_head_detector;                  /**< Head detector from depth data. */
        RPDepthHeadTracker depth_head_tracker;                    /**< Head tracker using depth data. */
        RPDepthImageProcessor depth_image_processor;              /**< Depth image processor.         */
        RPColorImageProcessor color_image_processor;              /**< Color image processor.         */
        RPColorFaceDetector color_face_detector;                  /**< Face detector from color data. */
        RPKernelLogisticRegressionClassifier face_hue_classifier; /**< Kernel logistic regression
                                                                       classifier for face/non-face
                                                                       hue classification.            */
        int face_hue_sample_count;                                /**< Gathered face hue sample count.*/
        RPBayesianSkinClassifier skin_classifier;                 /**< Bayesian skin color classifier.*/

        ros::Publisher heads_publisher;                           /**< Detected heads publisher.      */
        ros::Publisher state_publisher;                           /**< Head tracking state publisher. */

        /**
         * Callback for the depth and color inputs from RGB-D sensor.
         * @param depth_input_image  Input depth image.
         * @param color_input_image  Input color image.
         */
        void sensorInputCallback(const sensor_msgs::Image::ConstPtr& depth_input_image, const sensor_msgs::Image::ConstPtr& color_input_image);

        /**
         * Detects heads in a given depth image (potentially using color cues for detected head
         * verification).
         * @param depth_image  Input depth image.
         * @param color_image  Input color image.
         */
        void detectHeads(const cv_bridge::CvImage::Ptr& depth_image, const cv_bridge::CvImage::Ptr& color_image);

        /**
         * Detects faces in a given color image.
         * @param color_image  Input color image.
         */
        void detectFaces(const cv_bridge::CvImage::ConstPtr& color_image);

        /**
         * Tracks detected heads in a given depth image.
         * @param depth_image  Input depth image.
         */
        void trackHeads(const cv_bridge::CvImage::Ptr& depth_image);

        /**
         * Publishes detected/tracked heads in a current depth image.
         * @param depth_image  Current depth image.
         */
        void publishHeads(const cv_bridge::CvImage::ConstPtr& depth_image);

        /**
         * Updates the node's state.
         * @param state Desired new state.
         */
        void updateState(const HeadTrackingState state);

        /**
         * Gets static overridable parameters from the parameter server.
         */
        void getStaticParameters();

        /**
         * Gets dynamic head detection and tracking overridable parameters from the parameter server.
         */
        void getDynamicHeadDetectionAndTrackingParameters();

        /**
         * Processes key presses for the GUI output window.
         */
        void processOutputWindowKeyPress();

    public:
        /**
         * Default head detection and tracking node's constructor.
         * @param node Handle to ROS node.
         */
        RPHeadTrackingNode(ros::NodeHandle& node);
};

#endif /* HEAD_TRACKING_HPP_ */
