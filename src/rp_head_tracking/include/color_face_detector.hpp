/**
 * @class      RPColorFaceDetector
 *
 * @brief      Face detector in colour images, based on Viola and Jones (2001) face detector
 *             implementation in OpenCV.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef COLOR_FACE_DETECTOR_HPP_
#define COLOR_FACE_DETECTOR_HPP_

// STL includes
#include <vector>
#include <string>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>

// RPHeadTracking includes
#include <constants.hpp>

// Constants
#define FACE_DETECTOR_FRONTAL_CASCADE_PATH_DEFAULT "/home/turtlebot/catkin_ws/src/rp_head_tracking/data/haarcascade_frontalface_default.xml"
#define FACE_DETECTOR_PROFILE_CASCADE_PATH_DEFAULT "/home/turtlebot/catkin_ws/src/rp_head_tracking/data/haarcascade_profileface.xml"
#define VIOLA_JONES_NEIGHBOR_COUNT_DEFAULT 3

class RPColorFaceDetector
{
    private:
        // Viola-Jones face detector cascade
        cv::CascadeClassifier frontal_cascade;          /**< Frontal face detector cascade.           */
        cv::CascadeClassifier profile_cascade;          /**< Profile face detector cascade.           */

        // Overridable parameters
        std::string FACE_DETECTOR_FRONTAL_CASCADE_PATH; /**< Frontal face detector cascade file path. */
        std::string FACE_DETECTOR_PROFILE_CASCADE_PATH; /**< Profile face detector cascade file path. */
        int         VIOLA_JONES_NEIGHBOR_COUNT;         /**< Minimal neighbor count required for
                                                             Viola-Jones face detectors to register
                                                             a face detection.                        */
        /**
         * Gets the overridable parameters from the parameter server.
         * @param node Handle to ROS node.
         */
        void getOverridableParameters(const ros::NodeHandle& node);

    public:
        /**
         * Default autonomous photography node's constructor.
         * @param node Handle to ROS node.
         */
        RPColorFaceDetector(const ros::NodeHandle& node);

        /**
         * Detects faces in a given RGB image.
         * @param input_image  Input RGB image.
         * @param faces        Deteted faces.
         */
        void detectFaces(const cv_bridge::CvImage::ConstPtr& input_image, std::vector<cv::Rect>& faces);

        /**
         * Detects faces in a given OpenCV matrix representation of an image (in CV_8UC3 format, i.e.
         * using 8 unsigned bits to represent each of the R, G and B color channels).
         * @param input_image  Input image matrix in CV_8UC3 format.
         * @param faces        Deteted faces.
         */
        void detectFaces(const cv::Mat& input_image, std::vector<cv::Rect>& faces);
};

#endif /* COLOR_FACE_DETECTOR_HPP_ */
