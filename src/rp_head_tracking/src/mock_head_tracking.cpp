/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <mock_head_tracking.hpp>

// C++ includes
#include <cstdlib>

// RPHeadTracking includes
#include <heads_message_builder.hpp>
#include <constants.hpp>
#include <utils.hpp>

// RPHeadTracking message includes
#include <rp_head_tracking/Heads.h>

#define FRAME_ID "rp_mock_head_tracker"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_head_tracking");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPMockHeadTrackingNode worker_node(node);
}


RPMockHeadTrackingNode::RPMockHeadTrackingNode(ros::NodeHandle& node) :
    HEAD_COUNT(HEAD_COUNT_DEFAULT),
    node(node),
    frame_count(0),
    depth_image(new cv_bridge::CvImage())
{
    depth_image->image = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_32F);
    initializeRandomDepthImage(depth_image);

    heads_publisher = node.advertise<rp_head_tracking::Heads>("/rp/head_tracking/heads", 1);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        generateAndPublishMockHeads();

        ros::spinOnce();
        rate.sleep();
    }
}


void RPMockHeadTrackingNode::initializeRandomDepthImage(const cv_bridge::CvImage::Ptr& depth_image)
{
    const float max_dimension = FRAME_HEIGHT + FRAME_WIDTH;
    for (int y = 0; y < depth_image->image.rows; y++)
    {
        for (int x = 0; x < depth_image->image.cols; x++)
        {
            // 0.4 <= depth < 8.0
            depth_image->image.at<float>(y, x) = 0.4f + 7.6f * ((float)(x + y) / max_dimension);

//            // 1.0 <= random_depth < 4.0
//            float random_depth = (3.0f * ((float)rand() / (float)RAND_MAX)) + 1.0f;
//            depth_image->image.at<float>(y, x) = random_depth;
        }
    }
}


void RPMockHeadTrackingNode::generateAndPublishMockHeads()
{
    frame_count++;

    // Get overridable parameters
    getOverridableParameters();

    // Move the heads (clipping to frame)
    for (unsigned i = 0; i < heads.size(); i++)
    {
        int x_offset = (rand() % 3) - 1;
        int y_offset = (rand() % 3) - 1;

        cv::Point offset(x_offset, y_offset);

        heads[i].rectangle += offset;

        // Clamp to frame
        RPUtils::clampRectangleToFrame(&heads[i].rectangle);

        // We killed the head - regenerate a new one
        if (heads[i].rectangle.area() == 0)
        {
            generateMockHead(heads[i].rectangle);
        }
    }

    while (heads.size() < HEAD_COUNT)
    {
        // Keep adding heads
        cv::Rect generated_head;
        generateMockHead(generated_head);

        Head head = { generated_head, 0 };
        heads.push_back(head);
    }

    while (heads.size() > HEAD_COUNT)
    {
        // Choose one head and remove it
        int victim_index = rand() % (int)heads.size();
        heads.erase(heads.begin() + victim_index);
    }

    publishHeads();
}


void RPMockHeadTrackingNode::publishHeads()
{
    rp_head_tracking::Heads heads_message;
    RPHeadsMessageBuilder::buildHeadsMessage(FRAME_ID,
                                             frame_count,
                                             ros::Time::now(),
                                             heads,
                                             depth_image,
                                             heads_message);

    heads_publisher.publish(heads_message);
}


void RPMockHeadTrackingNode::generateMockHead(cv::Rect& generated_head)
{
    // Generate head's height (in pixels)
    // 20 <= head_height <= 100
    int head_height = (rand() % 80) + 20;
    int head_width = head_height * 2 / 3;

    // Generate head's location
    int head_left = rand() % (FRAME_WIDTH - head_width);
    int head_top = rand() % (FRAME_HEIGHT - head_height);

    generated_head.x = head_left;
    generated_head.y = head_top;
    generated_head.width = head_width;
    generated_head.height = head_height;
}


void RPMockHeadTrackingNode::getOverridableParameters()
{
    node.getParamCached("/rp/mock_head_tracking_node/head_count", HEAD_COUNT);
}
