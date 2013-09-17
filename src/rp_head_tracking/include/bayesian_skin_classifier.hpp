/**
 * @class      RPBayesianSkinClassifier
 *
 * @brief      Hue-histogram based Bayesian skin classifier.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef BAYESIAN_SKIN_CLASSIFIER_HPP_
#define BAYESIAN_SKIN_CLASSIFIER_HPP_

// STL includes
#include <string>

// ROS includes
#include <ros/ros.h>

// Overridable parameter defaults
#define BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH_DEFAULT      "/home/turtlebot/catkin_ws/src/rp_head_tracking/data/skin_histogram.txt"
#define BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH_DEFAULT  "/home/turtlebot/catkin_ws/src/rp_head_tracking/data/non_skin_histogram.txt"

// Constants
#define HISTOGRAM_SUBSAMPLING  8
#define HISTOGRAM_BUCKET_COUNT 32

class RPBayesianSkinClassifier
{
    private:
        double skin_histogram[HISTOGRAM_BUCKET_COUNT][HISTOGRAM_BUCKET_COUNT][HISTOGRAM_BUCKET_COUNT];     /**< Skin pixel RGB histogram. */
        double non_skin_histogram[HISTOGRAM_BUCKET_COUNT][HISTOGRAM_BUCKET_COUNT][HISTOGRAM_BUCKET_COUNT]; /**< Non-skin pixel RGB histogram. */
        
        std::string BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH;      /**< Skin pixel RGB histogram file path.     */
        std::string BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH;  /**< Non-skin pixel RGB histogram file path. */

        /**
         * Initializes Bayesian hue classifier using skin/non-skin histograms.
         * @param skin_histogram_file_name      Skin histogram file path.
         * @param non_skin_histogram_file_name  Non-skin histogram file path.
         */
        void initialize(std::string skin_histogram_file_name, std::string non_skin_histogram_file_name);

        /**
         * Gets the overridable parameters from the parameter server.
         * @param node Handle to ROS node.
         */
        void getOverridableParameters(const ros::NodeHandle& node);

    public:
        /**
         * Default Bayesian skin classifier constructor.
         * @param node Handle to ROS node.
         */
        RPBayesianSkinClassifier(const ros::NodeHandle& node);
        
        /**
         * Gets the probability that a given color was generated from a skin image.
         * @param r Red component of the RGB pixel.
         * @param g Green component of the RGB pixel.
         * @param b Blue component of the RGB pixel.
         * @returns Probability that a given pixel was generated from a skin image.
         */
        double skinProbability(int r, int g, int b)
        {
            return skin_histogram[r / HISTOGRAM_SUBSAMPLING][g / HISTOGRAM_SUBSAMPLING][b] / HISTOGRAM_SUBSAMPLING;
        }
        
        /**
         * Gets the probability that a given color was generated from a non-skin image.
         * @param r Red component of the RGB pixel.
         * @param g Green component of the RGB pixel.
         * @param b Blue component of the RGB pixel.
         * @returns Probability that a given pixel was generated from a non-skin image.
         */
        double nonSkinProbability(int r, int g, int b)
        {
            return non_skin_histogram[r / HISTOGRAM_SUBSAMPLING][g / HISTOGRAM_SUBSAMPLING][b / HISTOGRAM_SUBSAMPLING];
        }
};

#endif /* BAYESIAN_SKIN_CLASSIFIER_HPP_ */
