/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <color_image_processor.hpp>

// ROS includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// RPHeadTracking includes
#include <utils.hpp>

void RPColorImageProcessor::convertColorImageToRenderImage(const cv_bridge::CvImage::ConstPtr& color_image, cv::Mat& render_image)
{
    color_image->image.copyTo(render_image);
}


void RPColorImageProcessor::convertColorImageToBackpropagationImage(const cv_bridge::CvImage::ConstPtr& color_image, cv::Mat& render_image, RPBayesianSkinClassifier& skin_classifier, const int skin_likelihood_ratio_minimum)
{
    for (int y = 0; y < color_image->image.rows; y++)
    {
        for (int x = 0; x < color_image->image.cols; x++)
        {
            const cv::Vec3b& current_pixel = color_image->image.at<cv::Vec3b>(y, x);

            double skin_probability     = skin_classifier.skinProbability(current_pixel[2], current_pixel[1], current_pixel[0]);
            double non_skin_probability = skin_classifier.nonSkinProbability(current_pixel[2], current_pixel[1], current_pixel[0]);
            double skin_likelihood_ratio = (non_skin_probability > 0.0) ? (skin_probability / non_skin_probability) : 1.0;

            render_image.at<cv::Vec3b>(y, x) = (skin_likelihood_ratio > skin_likelihood_ratio_minimum) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
        }
    }
}


void RPColorImageProcessor::createHistogram(const cv::Mat& histogram_region, std::vector<double>& histogram)
{
    // Build the hue histogram as a training point
    histogram.resize(MAX_HUE + 1, 0.0);
    // Initialize the bias feature
    histogram[0] = 1.0;

    // Build the actual histogram
    double valid_hue_pixels = 0.0;
    for (int y = 0; y < histogram_region.rows; y++)
    {
        for (int x = 0; x < histogram_region.cols; x++)
        {
            cv::Vec3b pixel_hsv = histogram_region.at<cv::Vec3b>(y, x);
            if (RPUtils::isSaturationValid(pixel_hsv[1]) && RPUtils::isValueValid(pixel_hsv[2]))
            {
                int hue = pixel_hsv[0];
                histogram[hue + 1]++;
                valid_hue_pixels++;
            }
        }
    }

    // Normalize the histogram
    for (int i = 1; i <= MAX_HUE; i++)
    {
        histogram[i] /= valid_hue_pixels;
    }
}


void RPColorImageProcessor::createSkinHueTrainingExample(cv::Rect& face_rectangle, const cv_bridge::CvImage::ConstPtr& color_image, std::vector<double>& hue_histogram)
{
    // Slightly shrink the face rectangle for the positive training example (to 95% of the size)
    cv::Rect shrunk_face_rectangle = RPUtils::resizeRectangle(face_rectangle, 0.95);
    RPUtils::clampRectangleToFrame(&shrunk_face_rectangle);

    // Resize the face rectangle to width/height ratio of 2/3
    double new_width = (double)shrunk_face_rectangle.height * 2.0 / 3.0;
    double width_delta = (double)shrunk_face_rectangle.width - new_width;
    shrunk_face_rectangle.x += width_delta / 2.0;
    shrunk_face_rectangle.width = new_width;

    // Create a face ROI
    cv::Mat face_region(color_image->image, shrunk_face_rectangle);

    // Create an elliptical mask
    cv::Mat elliptical_mask = cv::Mat::zeros(shrunk_face_rectangle.height, shrunk_face_rectangle.width, CV_8UC3);
    int shrunk_face_rectangle_horizontal_axis = shrunk_face_rectangle.width / 2;
    int shrunk_face_rectangle_vertical_axis = shrunk_face_rectangle.height / 2;
    cv::ellipse(elliptical_mask, cv::Point(shrunk_face_rectangle_horizontal_axis, shrunk_face_rectangle_vertical_axis), cv::Size(shrunk_face_rectangle_horizontal_axis, shrunk_face_rectangle_vertical_axis), 0.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

    // Convert the face region to HSV
    cv::Mat face_region_hsv;
    cv::cvtColor(face_region, face_region_hsv, CV_BGR2HSV);

    // Apply the elliptical mask to HSV face region
    cv::bitwise_and(face_region_hsv, elliptical_mask, face_region_hsv);
    createHistogram(face_region_hsv, hue_histogram);
}


void RPColorImageProcessor::createNonSkinHueTrainingExample(cv::Rect& face_rectangle, const cv_bridge::CvImage::ConstPtr& color_image, std::vector<double>& hue_histogram)
{
    // Expand the face rectangle for negative training example
    cv::Rect outer_face_rectangle = RPUtils::resizeRectangle(face_rectangle, FACE_REGION_EXPANSION_FACTOR * FACE_REGION_EXPANSION_FACTOR);
    cv::Rect inner_face_rectangle = RPUtils::resizeRectangle(face_rectangle, FACE_REGION_EXPANSION_FACTOR);
    RPUtils::clampRectangleToFrame(&outer_face_rectangle);
    RPUtils::clampRectangleToFrame(&inner_face_rectangle);

    cv::Mat expanded_face_region(color_image->image, outer_face_rectangle);

    // Create a rectangular mask
    cv::Mat rectangular_mask = cv::Mat::zeros(outer_face_rectangle.height, outer_face_rectangle.width, CV_8UC3);
    cv::Point rectangular_mask_tl = inner_face_rectangle.tl() - outer_face_rectangle.tl();
    cv::rectangle(rectangular_mask, cv::Rect(rectangular_mask_tl, inner_face_rectangle.size()), cv::Scalar(255, 255, 255), -1);

    // Convert the face region to HSV
    cv::Mat face_region_hsv;
    cv::cvtColor(expanded_face_region, face_region_hsv, CV_BGR2HSV);

    // Invert and apply the expanded mask
    cv::bitwise_not(rectangular_mask, rectangular_mask);
    cv::bitwise_and(face_region_hsv, rectangular_mask, face_region_hsv);
    createHistogram(face_region_hsv, hue_histogram);
}
