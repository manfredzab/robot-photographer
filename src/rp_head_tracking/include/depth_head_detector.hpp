/**
 * @class      RPDepthHeadDetector
 *
 * @brief      Head detector from depth images.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DEPTH_HEAD_DETECTOR_HPP_
#define DEPTH_HEAD_DETECTOR_HPP_

// STL includes
#include <vector>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <constants.hpp>
#include <bayesian_skin_classifier.hpp>
#include <color_image_processor.hpp>
#include <color_face_detector.hpp>
#include <kernel_logistic_regression_classifier.hpp>

// Overridable parameter defaults
#define SKIN_HEURISTIC_ENABLED_DEFAULT true

#define KLR_CLASSIFIER_ENABLED_DEFAULT false

#define SKIN_AREA_MINIMUM_DEFAULT 0.01
#define SKIN_LIKELIHOOD_RATIO_MINIMUM_DEFAULT 0.1
#define FACE_HUE_PROBABILITY_MINIMUM_DEFAULT 0.33

#define IGNORE_LOW_CANDIDATE_HEADS_DEFAULT true
#define HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND_DEFAULT 1.4f

// Constants
#define SENSOR_VERTICAL_DISTANCE_FROM_GROUND 0.62f
#define SIN_SENSOR_ANGLE 0.13917310096f
#define COS_SENSOR_ANGLE 0.99026806874f

#define HEAD_INNER_BOUND_M 0.1f
#define HEAD_INNER_BOUND_DEPTH_TOLERANCE_M 0.1f

#define HEAD_OUTER_BOUND_M 0.3f
#define HEAD_OUTER_BOUND_DEPTH_TOLERANCE_M 0.2f

#define HEAD_AXIS_CONTINUATION_TOLERANCE_M 0.05f

#define HEAD_AXIS_MINIMUM_LENGTH_M 0.20f
#define HEAD_AXIS_MAXIMUM_LENGTH_M 0.30f
#define HEAD_AXIS_MAXIMUM_ROTATION_ANGLE_DEG 35.0f
#define HEAD_AXIS_MAXIMUM_DISTANCE_M 5.00f

class RPDepthHeadDetector
{
    private:
        /**
         * Detected head structure.
         */
        struct RPDetectedHead
        {
            public:
                float average_head_distance;           /**< Average head distance from Kinect sensor. */
                float average_head_center;             /**< Average horizontal head center pixel.     */

                std::vector<int> vertical_head_axis_x; /**< Vertical head axis point X coordinates.   */
                std::vector<int> vertical_head_axis_y; /**< Vertical head axis point Y coordinates.   */

                int skin_pixel_count;                  /**< Skin pixel count.                         */
                int total_pixels_within_bounds;        /**< Total number of pixels assigned to this
                                                            head.                                     */

                std::vector<double>head_hue_histogram; /**< Hue histogram.                            */
                double head_hue_valid_sample_count;    /**< Valid hue pixel count.                    */

                /**
                 * Default detected head constructor.
                 */
                RPDetectedHead()
                {
                    vertical_head_axis_x.reserve(FRAME_HEIGHT / 3);
                    vertical_head_axis_y.reserve(FRAME_HEIGHT / 3);

                    average_head_center = 0.0f;
                    average_head_distance = 0.0f;

                    skin_pixel_count = 0;
                    total_pixels_within_bounds = 0;

                    head_hue_histogram = std::vector<double>(MAX_HUE + 1, 0.0);
                    head_hue_histogram[0] = 1.0;

                    head_hue_valid_sample_count = 0.0;
                }
        };

        //
        const ros::NodeHandle& node;                     /**< Node's handle.                          */

        bool   SKIN_HEURISTIC_ENABLED;                   /**< Skin heuristic overridable parameter.   */

        double SKIN_AREA_MINIMUM;                        /**< Minimum skin area OP.                   */
        double SKIN_LIKELIHOOD_RATIO_MINIMUM;            /**< Minimum skin likelihood ratio OP.       */
        double FACE_HUE_PROBABILITY_MINIMUM;             /**< Minimum face hue probability OP.        */
        bool   KLR_CLASSIFIER_ENABLED;                   /**< KLR skin classifier enabled flag OP.    */

        bool   IGNORE_LOW_CANDIDATE_HEADS;               /**< OP to ignore low candidate heads.       */
        double HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND;   /**< OP for minimum head's vertical distance
                                                              from the ground not to be ignored.      */

        /**
         * Checks whether a given pixel is a skin pixel.
         * @param skin_classifier  Skin hue histogram-based skin classifier.
         * @param pixel            Pixel to classify.
         * @returns True, if a given pixel is a skin pixel, and false otherwise.
         */
        bool isSkinPixel(RPBayesianSkinClassifier& skin_classifier, cv::Vec3b& pixel);

        /**
         * Renders the pixels that satisfy head bounds.
         * @param render_image  Image in which to render the pixels satisfying the head bounds.
         */
        void renderPixelsSatisfyingHeadBounds(cv::Mat& render_image);

        /**
         * Gets static overridable parameters from the parameter server.
         */
        void getStaticParameters();

        /**
         * Gets dynamic overridable parameters from the parameter server.
         */
        void getDynamicParameters();

    public:
        bool is_pixel_satisfying_head_size_bound[FRAME_HEIGHT][FRAME_WIDTH]; /**< Flag array indicating
                                                                                  whether a given pixel
                                                                                  satisfies a head bound. */
        /**
         * Default constructor of head detector in depth images.
         * @param node  ROS node's handle.
         */
        RPDepthHeadDetector(const ros::NodeHandle& node);

        /**
         * Detects the heads in a given depth image (potentially using heuristics from the color image).
         * @param depth_image         Depth image.
         * @param color_image         Color image.
         * @param skin_classifier     Bayesian skin classifier.
         * @param klr_hue_classifier  KLR skin hue classifier.
         * @param heads               Detected heads.
         * @param render_image        Image to which the GUI information should be rendered.
         * @param gui_output_enabled  GUI output enabled/disabled flag.
         */
        void detectHeads(const cv_bridge::CvImage::ConstPtr& depth_image,
                         const cv_bridge::CvImage::Ptr& color_image,
                         RPBayesianSkinClassifier& skin_classifier,
                         RPKernelLogisticRegressionClassifier& klr_hue_classifier,
                         std::vector<cv::Rect>& heads,
                         cv::Mat& render_image,
                         bool gui_output_enabled);

        /**
         * Gets pixels satisfying the head bounds (result is stored in
         * "is_pixel_satisfying_head_size_bound" array).
         * @param depth_image  Input depth image.
         */
        void getPixelsSatisfyingHeadBounds(const cv_bridge::CvImage::ConstPtr& depth_image);

};

#endif /* DEPTH_HEAD_DETECTOR_HPP_ */
