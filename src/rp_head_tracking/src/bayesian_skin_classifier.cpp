/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <bayesian_skin_classifier.hpp>

// ROS includes
#include <ros/ros.h>

// C++ includes
#include <fstream>

RPBayesianSkinClassifier::RPBayesianSkinClassifier(const ros::NodeHandle& node) :
    BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH(BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH_DEFAULT),
    BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH(BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH_DEFAULT)
{
    // Get parameters from the parameter server
    getOverridableParameters(node);

    initialize(BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH, BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH);
}


void RPBayesianSkinClassifier::getOverridableParameters(const ros::NodeHandle& node)
{
    node.getParamCached("/rp/head_tracking_node/bayesian_classifier_skin_histogram_path", BAYESIAN_CLASSIFIER_SKIN_HISTOGRAM_PATH);
    node.getParamCached("/rp/head_tracking_node/bayesian_classifier_non_skin_histogram_path", BAYESIAN_CLASSIFIER_NON_SKIN_HISTOGRAM_PATH);
}


void RPBayesianSkinClassifier::initialize(std::string skin_histogram_file_name, std::string non_skin_histogram_file_name)
{
    std::ifstream skin_histogram_file_stream, non_skin_histogram_file_stream;
    
    // Set up exception handling
    skin_histogram_file_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    non_skin_histogram_file_stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
        skin_histogram_file_stream.open(skin_histogram_file_name.c_str());
        non_skin_histogram_file_stream.open(non_skin_histogram_file_name.c_str());

        for (int r = 0; r < HISTOGRAM_BUCKET_COUNT; r++)
        {
            for (int g = 0; g < HISTOGRAM_BUCKET_COUNT; g++)
            {
                for (int b = 0; b < HISTOGRAM_BUCKET_COUNT; b++)
                {
                    skin_histogram_file_stream >> skin_histogram[r][g][b];
                    non_skin_histogram_file_stream >> non_skin_histogram[r][g][b];
                }
            }
        }

        skin_histogram_file_stream.close();
        non_skin_histogram_file_stream.close();
    }
    catch (std::ifstream::failure e)
    {
        ROS_ERROR("Cannot access skin histogram files %s and %s.", skin_histogram_file_name.c_str(), non_skin_histogram_file_name.c_str());
    }
}
