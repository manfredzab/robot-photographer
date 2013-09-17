/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <depth_head_tracker.hpp>

// RPHeadTracking includes
#include <utils.hpp>

void RPDepthHeadTracker::trackHeads(const cv_bridge::CvImage::ConstPtr& depth_image, RPDepthHeadDetector& depth_detector, std::vector<Head>& heads)
{
    // Get pixels satisfying basic head bounds
    depth_detector.getPixelsSatisfyingHeadBounds(depth_image);

    // Perform CAMShift on detected heads
    for (unsigned i = 0; i < heads.size(); i++)
    {
        cv::Rect current_head_rectangle(heads[i].rectangle);

        int current_camshift_iteration = 0;
        bool camshift_converged = false;
        while ((!camshift_converged) && (current_camshift_iteration < CAMSHIFT_ITERATION_THRESHOLD))
        {
            double zeroth_moment, first_moment_x, first_moment_y;
            for (int t = 0; t < MEANSHIFT_ITERATION_THRESHOLD; t++)
            {
                // Find the moments
                zeroth_moment = first_moment_x = first_moment_y = 0.0;
                for (int y = current_head_rectangle.y; y < current_head_rectangle.y + current_head_rectangle.height; y++)
                {
                    for (int x = current_head_rectangle.x; x < current_head_rectangle.x + current_head_rectangle.width; x++)
                    {
                        if (depth_detector.is_pixel_satisfying_head_size_bound[y][x])
                        {
                            zeroth_moment++;
                            first_moment_x += x;
                            first_moment_y += y;
                        }
                    }
                }

                if (0 == zeroth_moment)
                {
                    break;
                }

                // Update the location of the current head
                double new_centroid_x = first_moment_x / zeroth_moment;
                double new_centroid_y = first_moment_y / zeroth_moment;

                double previous_centroid_x = (double)current_head_rectangle.x + ((double)current_head_rectangle.width / 2.0);
                double previous_centroid_y = (double)current_head_rectangle.y + ((double)current_head_rectangle.height / 2.0);

                current_head_rectangle += cv::Point(new_centroid_x - previous_centroid_x, new_centroid_y - previous_centroid_y);
                RPUtils::clampRectangleToFrame(&current_head_rectangle);
            }

            double new_width = SEARCH_WINDOW_SCALE * sqrt(zeroth_moment);
            double new_height = 1.5 * new_width;

            // Check for convergence
            current_head_rectangle.width = (int)new_width;
            current_head_rectangle.height = (int)new_height;

            cv::Rect rectangle_intersection = current_head_rectangle & heads[i].rectangle;
            cv::Rect rectangle_union = current_head_rectangle | heads[i].rectangle;

            // Jaccard coefficient
            camshift_converged = ((double)rectangle_intersection.area() / (double)rectangle_union.area()) > CAMSHIFT_CONVERGENCE_THRESHOLD_PERCENTAGE;
            current_camshift_iteration++;

            // Update the current head rectangle
            RPUtils::clampRectangleToFrame(&current_head_rectangle);
            heads[i].rectangle = current_head_rectangle;
        }
    }

    // Check for heads that should be removed
    if (heads.size() > 0)
    {
        for (int i = heads.size() - 1; i >= 0; i--)
        {
            if (heads[i].rectangle.area() == 0)
            {
                heads.erase(heads.begin() + i);
            }
        }
    }
}

