/**
 * @class      RPColorImageProcessor
 *
 * @brief      Color image processor class, which provides implementations for simple color
 *             image manipulation tasks.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef COLOR_IMAGE_PROCESSOR_HPP_
#define COLOR_IMAGE_PROCESSOR_HPP_

// ROS includes
#include <bayesian_skin_classifier.hpp>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Constants
#define MAX_HUE 180
#define FACE_REGION_EXPANSION_FACTOR 1.5

class RPColorImageProcessor
{
    public:
        /**
         * Copies the color RGB image provided by cv_bridge into an OpenCV matrix.
         * @param color_image   Input color RGB image from cv_bridge.
         * @param render_image  Same RGB image copied to an OpenCV matrix.
         */
        void convertColorImageToRenderImage(const cv_bridge::CvImage::ConstPtr& color_image, cv::Mat& render_image);

        /**
         * Converts the color RGB image provided by cv_bridge into a backpropagation image, stored
         * in an OpenCV matrix.
         * @param color_image                    Input color RGB image from cv_bridge.
         * @param render_image                   Backpropagation image stored in an OpenCV matrix.
         * @param skin_classifier                Bayesian skin classifier.
         * @param skin_likelihood_ratio_minimum  Minimum skin likelihood ratio (used as a
         *                                       threshold).
         */
        void convertColorImageToBackpropagationImage(const cv_bridge::CvImage::ConstPtr& color_image, cv::Mat& render_image, RPBayesianSkinClassifier& skin_classifier, const int skin_likelihood_ratio_minimum);

        /**
         * Creates a skin hue histogram from a given detected face rectangle in a given color
         * image.
         * @param face_rectangle  Input detected face rectangle.
         * @param color_image     Input color image.
         * @param hue_histogram   Output skin hue histogram.
         */
        void createSkinHueTrainingExample(cv::Rect& face_rectangle, const cv_bridge::CvImage::ConstPtr& color_image, std::vector<double>& hue_histogram);

        /**
         * Creates a non-skin hue histogram from a given detected face rectangle in a given
         * color image.
         * @param face_rectangle  Input detected face rectangle.
         * @param color_image     Input color image.
         * @param hue_histogram   Output non-skin hue histogram.
         */
        void createNonSkinHueTrainingExample(cv::Rect& face_rectangle, const cv_bridge::CvImage::ConstPtr& color_image, std::vector<double>& hue_histogram);

        /**
         * Creates a histogram from a given region in an image.
         * @param histogram_region  Histogram region in an image.
         * @param histogram         Resultant histogram.
         */
        void createHistogram(const cv::Mat& histogram_region, std::vector<double>& histogram);
};

#endif /* COLOR_IMAGE_PROCESSOR_HPP_ */
