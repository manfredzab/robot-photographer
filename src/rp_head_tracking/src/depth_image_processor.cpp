/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <depth_image_processor.hpp>

void RPDepthImageProcessor::smoothDepthImage(const cv_bridge::CvImage::Ptr& depth_image, const int radius)
{
    // Fill the first row and first column
    for (int x = 0; x < depth_image->image.cols + 1; x++)
    {
        cumulative_row_sum[0][x] = 0.0f;
        integral_image[0][x] = 0.0f;
    }

    for (int y = 0; y < depth_image->image.rows + 1; y++)
    {
        cumulative_row_sum[y][0] = 0.0f;
        integral_image[y][0] = 0.0f;
    }

    // Fill the rest of the integral image
    for (int y = 1; y < depth_image->image.rows + 1; y++)
    {
        for (int x = 1; x < depth_image->image.cols + 1; x++)
        {
            cumulative_row_sum[y][x] = cumulative_row_sum[y - 1][x] + depth_image->image.at<float>(y - 1, x - 1);
            integral_image[y][x] = integral_image[y][x - 1] + cumulative_row_sum[y][x];
        }
    }

    // Smooth the input image
    for (int y = radius; y < depth_image->image.rows - radius; y++)
    {
        for (int x = radius; x < depth_image->image.cols - radius; x++)
        {
            float top_left_integral = integral_image[y - radius + 1][x - radius + 1];
            float top_right_integral = integral_image[y - radius + 1][x + radius + 1];
            float bottom_left_integral = integral_image[y + radius + 1][x - radius + 1];
            float bottom_right_integral = integral_image[y + radius + 1][x + radius + 1];

            depth_image->image.at<float>(y, x) = (bottom_right_integral - bottom_left_integral - top_right_integral + top_left_integral) / (float)(4 * radius * radius);
        }
    }
}


void RPDepthImageProcessor::filterDepthShadow(const cv_bridge::CvImage::Ptr& cv_image)
{
    float previous_depth;
    for (int y = 0; y < cv_image->image.rows; y++)
    {
        previous_depth = 0.0f;
        for (int x = 0; x < cv_image->image.cols; x++)
        {
            float current_depth = cv_image->image.at<float>(y, x);
            if (current_depth == current_depth) // Not a NaN
            {
                previous_depth = current_depth;
            }
            else
            {
                cv_image->image.at<float>(y, x) = previous_depth;
            }
        }
    }
}


void RPDepthImageProcessor::filterMissingData(const cv_bridge::CvImage::Ptr& depth_image)
{
    const float MISSING_FAR_DEPTH = 8.0f;

    for (int y = 0; y < depth_image->image.rows; y++)
    {
        for (int x = 0; x < depth_image->image.cols; x++)
        {
            float current_depth = depth_image->image.at<float>(y, x);
            if (current_depth != current_depth) // NaN
            {
                depth_image->image.at<float>(y, x) = MISSING_FAR_DEPTH;
            }
        }
    }
}


void RPDepthImageProcessor::convertDepthImageToRenderImage(const cv_bridge::CvImage::ConstPtr& depth_image, cv::Mat& render_image)
{
    const float MIN_RANGE = 0.4f; // In meters
    const float MAX_RANGE = 8.0f;

    for (int y = 0; y < depth_image->image.rows; y++)
    {
        for (int x = 0; x < depth_image->image.cols; x++)
        {
            float current_depth = depth_image->image.at<float>(y, x);
            if (current_depth == current_depth) // Not a NaN
            {
                float relative_distance = (current_depth - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
                render_image.at<cv::Vec3b>(y, x) = hsvToRgb(360.0f * relative_distance, 1.0f, 1.0f);
            }
            else
            {
                render_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }
}


inline cv::Vec3b RPDepthImageProcessor::hsvToRgb(float h, float s, float v)
{
    cv::Vec3b result(0, 0, 0);

    if (h != h)
    {
        return result;
    }

    int i;
    float f, p, q, t;
    if (s == 0)
    {
        return result;
    }

    h /= 60;
    i = floor(h);
    f = h - i;
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));
    switch (i)
    {
        case 0:
            result[2] = 255 * v;
            result[1] = 255 * t;
            result[0] = 255 * p;
            break;
        case 1:
            result[2] = 255 * q;
            result[1] = 255 * v;
            result[0] = 255 * p;
            break;
        case 2:
            result[2] = 255 * p;
            result[1] = 255 * v;
            result[0] = 255 * t;
            break;
        case 3:
            result[2] = 255 * p;
            result[1] = 255 * q;
            result[0] = 255 * v;
            break;
        case 4:
            result[2] = 255 * t;
            result[1] = 255 * p;
            result[0] = 255 * v;
            break;
        default:
            result[2] = 255 * v;
            result[1] = 255 * p;
            result[0] = 255 * q;
            break;
    }

    return result;
}
