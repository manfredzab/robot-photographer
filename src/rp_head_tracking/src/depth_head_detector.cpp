/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <depth_head_detector.hpp>

// Macros
#define SQUARE(X) ((X) * (X))

// C++ includes
#include <cmath>
#include <cstdio>

// STL includes
#include <list>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

// RPHeadTracking includes
#include <distance_converter.hpp>
#include <utils.hpp>


RPDepthHeadDetector::RPDepthHeadDetector(const ros::NodeHandle& node) :
    node(node),
    SKIN_HEURISTIC_ENABLED(SKIN_HEURISTIC_ENABLED_DEFAULT),
    SKIN_AREA_MINIMUM(SKIN_AREA_MINIMUM_DEFAULT),
    SKIN_LIKELIHOOD_RATIO_MINIMUM(SKIN_LIKELIHOOD_RATIO_MINIMUM_DEFAULT),
    FACE_HUE_PROBABILITY_MINIMUM(FACE_HUE_PROBABILITY_MINIMUM_DEFAULT),
    KLR_CLASSIFIER_ENABLED(KLR_CLASSIFIER_ENABLED_DEFAULT),
    IGNORE_LOW_CANDIDATE_HEADS(IGNORE_LOW_CANDIDATE_HEADS_DEFAULT),
    HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND(HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND_DEFAULT)
{
    // Get static parameters from the parameter server
    getStaticParameters();
}


void RPDepthHeadDetector::getStaticParameters()
{
    node.getParamCached("/rp/head_tracking_node/klr_classifier_enabled", KLR_CLASSIFIER_ENABLED);
}


void RPDepthHeadDetector::getDynamicParameters()
{
    node.getParamCached("/rp/head_tracking_node/skin_likelihood_ratio_minimum", SKIN_LIKELIHOOD_RATIO_MINIMUM);
    node.getParamCached("/rp/head_tracking_node/skin_area_minimum", SKIN_AREA_MINIMUM);
    node.getParamCached("/rp/head_tracking_node/ignore_low_candidate_heads", IGNORE_LOW_CANDIDATE_HEADS);
    node.getParamCached("/rp/head_tracking_node/head_min_vertical_distance_from_ground", HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND);
    node.getParamCached("/rp/head_tracking_node/face_hue_probability_minimum", FACE_HUE_PROBABILITY_MINIMUM);
    node.getParamCached("/rp/head_tracking_node/skin_heuristic_enabled", SKIN_HEURISTIC_ENABLED);
}


void RPDepthHeadDetector::detectHeads(const cv_bridge::CvImage::ConstPtr& depth_image,
                                      const cv_bridge::CvImage::Ptr& color_image,
                                      RPBayesianSkinClassifier& skin_classifier,
                                      RPKernelLogisticRegressionClassifier& klr_hue_classifier,
                                      std::vector<cv::Rect>& heads,
                                      cv::Mat& render_image,
                                      bool gui_output_enabled = false)
{
    // Get overridable parameters from the parameter server
    getDynamicParameters();

    // Convert the color image to HSV if needed
    if (SKIN_HEURISTIC_ENABLED && KLR_CLASSIFIER_ENABLED)
    {
        cv::cvtColor(color_image->image, color_image->image, CV_BGR2HSV);
    }

    // Storage for head vertical axes
    std::list<RPDetectedHead*> head_axes;

    // Get pixels satisfying basic head bounds
    getPixelsSatisfyingHeadBounds(depth_image);
    if (false)//gui_output_enabled)
    {
        renderPixelsSatisfyingHeadBounds(render_image);
    }

    // Build the head axes
    for (int y = 0; y < FRAME_HEIGHT; y++)
    {
        for (int x = 0; x < FRAME_WIDTH; x++)
        {
            if (!is_pixel_satisfying_head_size_bound[y][x])
            {
                continue;
            }

            // Get the lateral gradients
            float current_pixel_distance_m = depth_image->image.at<float>(y, x);
            int outer_bound_pixel_distance = 0.5f * RPDistanceConverter::horizontalWidthToPixels(HEAD_OUTER_BOUND_M, current_pixel_distance_m);

            // TODO: unroll loops
            int left_lateral_gradient_x = -1, right_lateral_gradient_x = -1;
            for (int i = 1; i <= outer_bound_pixel_distance; i++)
            {
                if (left_lateral_gradient_x < 0)
                {
                    if ((x - i >= 0) && (depth_image->image.at<float>(y, x - i) - current_pixel_distance_m > HEAD_OUTER_BOUND_DEPTH_TOLERANCE_M))
                    {
                        left_lateral_gradient_x = x - i;
                    }
                }

                if (right_lateral_gradient_x < 0)
                {
                    if ((x + i < depth_image->image.cols) && (depth_image->image.at<float>(y, x + i) - current_pixel_distance_m > HEAD_OUTER_BOUND_DEPTH_TOLERANCE_M))
                    {
                        right_lateral_gradient_x = x + i;
                    }
                }
            }

            if (gui_output_enabled)
            {
                render_image.at<cv::Vec3b>(y, left_lateral_gradient_x) = cv::Vec3b(255, 0, 0);
                render_image.at<cv::Vec3b>(y, right_lateral_gradient_x) = cv::Vec3b(255, 0, 0);
            }

            // Get the current point on the horizontal head axis
            int head_axis_x = (left_lateral_gradient_x + right_lateral_gradient_x) / 2;
            head_axis_x = std::max(0, std::min(FRAME_WIDTH - 1, head_axis_x)); // Clamp the axis to the image limits

            // Find the closest head
            bool head_axis_found = false;
            for (std::list<RPDetectedHead*>::iterator head_iterator = head_axes.begin(); head_iterator != head_axes.end(); ++head_iterator)
            {
                RPDetectedHead& tracked_head_candidate = **head_iterator;

                int previous_head_axis_x = tracked_head_candidate.vertical_head_axis_x.back();
                int previous_head_axis_y = tracked_head_candidate.vertical_head_axis_y.back();

                float current_head_axis_distance = depth_image->image.at<float>(y, head_axis_x);

                float width_difference = RPDistanceConverter::pixelsToWidth(previous_head_axis_x, tracked_head_candidate.average_head_distance) - RPDistanceConverter::pixelsToWidth(head_axis_x, current_head_axis_distance);
                float height_difference = RPDistanceConverter::pixelsToHeight(previous_head_axis_y, tracked_head_candidate.average_head_distance) - RPDistanceConverter::pixelsToHeight(y, current_head_axis_distance);
                float depth_difference = tracked_head_candidate.average_head_distance - current_head_axis_distance;

                if (sqrt(SQUARE(width_difference) + SQUARE(height_difference) + SQUARE(depth_difference)) < HEAD_AXIS_CONTINUATION_TOLERANCE_M)
                {
                    // Continue building the previous vertical head axis

                    // Update the averages
                    tracked_head_candidate.average_head_distance = ((tracked_head_candidate.average_head_distance * (float)tracked_head_candidate.vertical_head_axis_x.size()) + current_head_axis_distance) / (float)(tracked_head_candidate.vertical_head_axis_x.size() + 1);
                    tracked_head_candidate.average_head_center = ((tracked_head_candidate.average_head_center * (float)tracked_head_candidate.vertical_head_axis_x.size()) + (float)head_axis_x) / (float)(tracked_head_candidate.vertical_head_axis_x.size() + 1);

                    // Add the current point averaging the current ones if they already exist
                    if (y == tracked_head_candidate.vertical_head_axis_y.back())
                    {
                        *(tracked_head_candidate.vertical_head_axis_x.end() - 1) = (head_axis_x + tracked_head_candidate.vertical_head_axis_x.back()) / 2;
                    }
                    else
                    {
                        tracked_head_candidate.vertical_head_axis_x.push_back(head_axis_x);
                        tracked_head_candidate.vertical_head_axis_y.push_back(y);
                    }

                    if (SKIN_HEURISTIC_ENABLED)
                    {
                        if (KLR_CLASSIFIER_ENABLED)
                        {
                            // Update the hue histogram for the head using the hue of the original point
                            int candidate_pixel_hue = color_image->image.at<cv::Vec3b>(y, x)[0];

                            tracked_head_candidate.head_hue_histogram[candidate_pixel_hue + 1]++;
                            tracked_head_candidate.head_hue_valid_sample_count++;
                        }
                        else
                        {
                            // Check whether the original point belongs to skin
                            if (isSkinPixel(skin_classifier, color_image->image.at<cv::Vec3b>(y, x)))
                            {
                                tracked_head_candidate.skin_pixel_count++;
                            }
                            tracked_head_candidate.total_pixels_within_bounds++;
                        }
                    }

                    // Mark the vertical head axis as found
                    head_axis_found = true;
                    break;
                }
            }

            if (!head_axis_found)
            {
                // Start a new run
                RPDetectedHead* head = new RPDetectedHead();
                head->vertical_head_axis_x.push_back(head_axis_x);
                head->vertical_head_axis_y.push_back(y);

                head->average_head_distance = depth_image->image.at<float>(y, head_axis_x);
                head->average_head_center = head_axis_x;

                head_axes.push_back(head);
            }
        }
    }

    // Check the sizes and hues of the detected faces
    std::vector<cv::Rect> detected_faces;
    for (std::list<RPDetectedHead*>::iterator head_iterator = head_axes.begin(); head_iterator != head_axes.end(); ++head_iterator)
    {
        RPDetectedHead& current_head = **head_iterator;

        // Check the head's size
        int head_axis_height_px = current_head.vertical_head_axis_y.back() - current_head.vertical_head_axis_y.front();
        int head_axis_width_px = current_head.vertical_head_axis_x.back() - current_head.vertical_head_axis_x.front();

        float head_axis_height_m = RPDistanceConverter::pixelsToHeight(head_axis_height_px, current_head.average_head_distance);
        float head_axis_width_m = RPDistanceConverter::pixelsToWidth(head_axis_width_px, current_head.average_head_distance);

        float head_angle_deg = (atan(head_axis_height_m / head_axis_width_m) * 180.0f / M_PI);
        float head_axis_length_m = sqrt(head_axis_height_m*head_axis_height_m + head_axis_width_m*head_axis_width_m);

        if ((current_head.average_head_distance > HEAD_AXIS_MAXIMUM_DISTANCE_M) ||
            (head_axis_length_m < HEAD_AXIS_MINIMUM_LENGTH_M) ||
            (head_axis_length_m > HEAD_AXIS_MAXIMUM_LENGTH_M) ||
            (fabs(head_angle_deg) < 90.0f - HEAD_AXIS_MAXIMUM_ROTATION_ANGLE_DEG))
        {
            continue;
        }

        if (SKIN_HEURISTIC_ENABLED)
        {
            if (KLR_CLASSIFIER_ENABLED)
            {
                // Check the head's hue
                // Start by normalizing the head hue histogram
                for (int i = 1; i <= MAX_HUE; i++)
                {
                    current_head.head_hue_histogram[i] /= current_head.head_hue_valid_sample_count;
                }

                double face_hue_probability = klr_hue_classifier.probability(current_head.head_hue_histogram);
                if (face_hue_probability < 0.5)
                {
                    ROS_INFO("Rejecting detected face since the hue probability is %f.", face_hue_probability);
                    continue;
                }
            }
            else
            {
                double skin_area = (double)current_head.skin_pixel_count / (double)current_head.total_pixels_within_bounds;
                if (skin_area < SKIN_AREA_MINIMUM)
                {
                    ROS_INFO("Rejecting detected head based on skin heuristic: skin area %f, minimum %f.", skin_area, SKIN_AREA_MINIMUM);
                    continue;
                }
            }
        }

        // Build the result rectangle
        int head_rectangle_width = head_axis_height_px * 2.0 / 3.0;
        int head_rectangle_x = (int)(current_head.average_head_center - 0.5 * head_rectangle_width);
        int head_rectangle_y = current_head.vertical_head_axis_y.front();

        cv::Rect head_rectangle(cv::Point(head_rectangle_x, head_rectangle_y), cv::Size(head_rectangle_width, head_axis_height_px));
        RPUtils::clampRectangleToFrame(&head_rectangle);

        if (gui_output_enabled)
        {
            head_angle_deg = (head_angle_deg < 0.0f) ? (90.0f + head_angle_deg) : -(90.0f - head_angle_deg);
            cv::RotatedRect rotated_rect(cv::Point(head_rectangle.x + head_rectangle.width / 2, head_rectangle.y + head_rectangle.height / 2), head_rectangle.size(), head_angle_deg);
            cv::Point2f rect_points[4];
            rotated_rect.points(rect_points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(render_image, rect_points[i], rect_points[(i+1)%4], cv::Scalar(0, 255 ,0));
            }
        }

        heads.push_back(head_rectangle);

        // Output debug information
        if (gui_output_enabled)
        {
            cv::Vec3b head_color(255, 255, 255);
            for (unsigned i = 0; i < current_head.vertical_head_axis_x.size(); i++)
            {
                render_image.at<cv::Vec3b>(current_head.vertical_head_axis_y[i], current_head.vertical_head_axis_x[i]) = head_color;

                float current_pixel_distance_m = depth_image->image.at<float>(current_head.vertical_head_axis_y[i], current_head.vertical_head_axis_x[i]);
                int inner_bound_pixel_distance = 0.5f * RPDistanceConverter::horizontalWidthToPixels(HEAD_INNER_BOUND_M, current_pixel_distance_m);
                render_image.at<cv::Vec3b>(current_head.vertical_head_axis_y[i], current_head.vertical_head_axis_x[i] - inner_bound_pixel_distance) = cv::Vec3b(0, 0, 255);
                render_image.at<cv::Vec3b>(current_head.vertical_head_axis_y[i], current_head.vertical_head_axis_x[i] + inner_bound_pixel_distance) = cv::Vec3b(0, 0, 255);
            }

            char head_debug_string[256];
            sprintf(head_debug_string, "%4.2f m, %4.2f deg", head_axis_length_m, head_angle_deg);
            cv::putText(render_image, head_debug_string, head_rectangle.tl() + cv::Point(1, 1), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0));
            cv::putText(render_image, head_debug_string, head_rectangle.tl(), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0));
        }
    }

    // Release the memory for the heads
    for (std::list<RPDetectedHead*>::iterator head_iterator = head_axes.begin(); head_iterator != head_axes.end(); ++head_iterator)
    {
        RPDetectedHead* current_head = *head_iterator;
        delete current_head;
    }
}


void RPDepthHeadDetector::getPixelsSatisfyingHeadBounds(const cv_bridge::CvImage::ConstPtr& depth_image)
{
    for (int y = 0; y < FRAME_HEIGHT; y++)
    {
        for (int x = 0; x < FRAME_WIDTH; x++)
        {
            is_pixel_satisfying_head_size_bound[y][x] = false;
        }
    }

    float left_pixel_distance_m, current_pixel_distance_m, right_pixel_distance_m;
    for (int y = 0; y < FRAME_HEIGHT; y++)
    {
        for (int x = 1; x < FRAME_WIDTH - 1; x++)
        {
            left_pixel_distance_m = depth_image->image.at<float>(y, x - 1);
            current_pixel_distance_m = depth_image->image.at<float>(y, x);
            right_pixel_distance_m = depth_image->image.at<float>(y, x + 1);

            // Ignore head candidates below a certain vertical threshold
            int current_pixel_vertical_distance_from_frame_center = (FRAME_HEIGHT / 2) - y;
            float head_vertical_distance_from_sensor = RPDistanceConverter::pixelsToHeight(current_pixel_vertical_distance_from_frame_center, current_pixel_distance_m);
            if (IGNORE_LOW_CANDIDATE_HEADS)
            {
                // Calculate head's vertical distance from the floor plane
                float head_vertical_distance_from_ground = SENSOR_VERTICAL_DISTANCE_FROM_GROUND + (current_pixel_distance_m * SIN_SENSOR_ANGLE) + (head_vertical_distance_from_sensor * COS_SENSOR_ANGLE);
                if (head_vertical_distance_from_ground < HEAD_MIN_VERTICAL_DISTANCE_FROM_GROUND)
                {
                    continue;
                }
            }

            if ((left_pixel_distance_m > current_pixel_distance_m) || (current_pixel_distance_m < right_pixel_distance_m))
            {
                // Skip non-minimum points
                continue;
            }

            // Current point is a local minimum, check the inner bounds
            int inner_bound_pixel_distance = 0.5f * RPDistanceConverter::horizontalWidthToPixels(HEAD_INNER_BOUND_M, current_pixel_distance_m);
            if ((x - inner_bound_pixel_distance < 0) || (x + inner_bound_pixel_distance >= depth_image->image.cols))
            {
                continue;
            }

            bool correct_inner_bounds = true;
            for (int i = 1; i <= inner_bound_pixel_distance; i++)
            {
                if ((depth_image->image.at<float>(y, x + i) - current_pixel_distance_m > HEAD_INNER_BOUND_DEPTH_TOLERANCE_M) ||
                    (depth_image->image.at<float>(y, x - i) - current_pixel_distance_m > HEAD_INNER_BOUND_DEPTH_TOLERANCE_M))
                {
                    correct_inner_bounds = false;
                    break;
                }
            }

           if (!correct_inner_bounds)
           {
               continue;
           }

            // Check the outer bounds
            int outer_bound_pixel_distance = 0.5f * RPDistanceConverter::horizontalWidthToPixels(HEAD_OUTER_BOUND_M, current_pixel_distance_m);
            if ((x - outer_bound_pixel_distance < 0) || (x + outer_bound_pixel_distance >= depth_image->image.cols))
            {
                continue;
            }

            if ((depth_image->image.at<float>(y, x + outer_bound_pixel_distance) - current_pixel_distance_m < HEAD_OUTER_BOUND_DEPTH_TOLERANCE_M) ||
                (depth_image->image.at<float>(y, x - outer_bound_pixel_distance) - current_pixel_distance_m < HEAD_OUTER_BOUND_DEPTH_TOLERANCE_M))
            {
                continue;
            }

            // Outer bound condition satisfied
            is_pixel_satisfying_head_size_bound[y][x] = true;
        }
    }
}


void RPDepthHeadDetector::renderPixelsSatisfyingHeadBounds(cv::Mat& render_image)
{
    for (int y = 0; y < FRAME_HEIGHT; y++)
    {
        for (int x = 0; x < FRAME_WIDTH; x++)
        {
            if (is_pixel_satisfying_head_size_bound[y][x])
            {
                render_image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            }
        }
    }
}


inline bool RPDepthHeadDetector::isSkinPixel(RPBayesianSkinClassifier& skin_classifier, cv::Vec3b& pixel)
{
    double skin_probability     = skin_classifier.skinProbability(pixel[2], pixel[1], pixel[0]);
    double non_skin_probability = skin_classifier.nonSkinProbability(pixel[2], pixel[1], pixel[0]);
    double skin_likelihood_ratio = (non_skin_probability > 0.0) ? (skin_probability / non_skin_probability) : 1.0;

    return (skin_likelihood_ratio > SKIN_LIKELIHOOD_RATIO_MINIMUM);
}
