/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <color_face_detector.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


RPColorFaceDetector::RPColorFaceDetector(const ros::NodeHandle& node) :
    FACE_DETECTOR_FRONTAL_CASCADE_PATH(FACE_DETECTOR_FRONTAL_CASCADE_PATH_DEFAULT),
    FACE_DETECTOR_PROFILE_CASCADE_PATH(FACE_DETECTOR_PROFILE_CASCADE_PATH_DEFAULT),
    VIOLA_JONES_NEIGHBOR_COUNT(VIOLA_JONES_NEIGHBOR_COUNT_DEFAULT)
{
    getOverridableParameters(node);

    // Initialize the face detector cascades
    if (!frontal_cascade.load(FACE_DETECTOR_FRONTAL_CASCADE_PATH))
    {
        ROS_ERROR("Cannot load classifier cascade for the face detector from \"%s\".", FACE_DETECTOR_FRONTAL_CASCADE_PATH.c_str());
    }

    if (!profile_cascade.load(FACE_DETECTOR_PROFILE_CASCADE_PATH))
    {
        ROS_ERROR("Cannot load classifier cascade for the face detector from \"%s\".", FACE_DETECTOR_PROFILE_CASCADE_PATH.c_str());
    }
}


void RPColorFaceDetector::getOverridableParameters(const ros::NodeHandle& node)
{
    node.getParamCached("/rp/head_tracking_node/face_detector_frontal_cascade_path", FACE_DETECTOR_FRONTAL_CASCADE_PATH);
    node.getParamCached("/rp/head_tracking_node/face_detector_profile_cascade_path", FACE_DETECTOR_PROFILE_CASCADE_PATH);
    node.getParamCached("/rp/head_tracking_node/viola_jones_neighbor_count", VIOLA_JONES_NEIGHBOR_COUNT);
}


void RPColorFaceDetector::detectFaces(const cv::Mat& input_image, std::vector<cv::Rect>& faces)
{
    // Convert input image to grayscale
    cv::Mat grayscale_image = cv::Mat::zeros(input_image.rows, input_image.cols, CV_8UC3);

    cv::cvtColor(input_image, grayscale_image, CV_BGR2GRAY);
    cv::equalizeHist(grayscale_image, grayscale_image);

//    frontal_cascade.detectMultiScale(grayscale_image, faces, 1.1, VIOLA_JONES_NEIGHBOR_COUNT);

    std::vector<cv::Rect> frontal_faces;
    frontal_cascade.detectMultiScale(grayscale_image, frontal_faces, 1.2, VIOLA_JONES_NEIGHBOR_COUNT);

    std::vector<cv::Rect> profile_faces;
    profile_cascade.detectMultiScale(grayscale_image, profile_faces, 1.2, VIOLA_JONES_NEIGHBOR_COUNT);

    faces.insert(faces.end(), frontal_faces.begin(), frontal_faces.end());
    faces.insert(faces.end(), profile_faces.begin(), profile_faces.end());
}



void RPColorFaceDetector::detectFaces(const cv_bridge::CvImage::ConstPtr& input_image, std::vector<cv::Rect>& faces)
{
    detectFaces(input_image->image, faces);
}
