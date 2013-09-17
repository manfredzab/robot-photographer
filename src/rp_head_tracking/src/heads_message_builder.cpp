/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <heads_message_builder.hpp>

void RPHeadsMessageBuilder::buildHeadsMessage(const std::string& frame_id,
                                              const unsigned int frame_number,
                                              const ros::Time& time_stamp,
                                              const std::vector<Head>& heads,
                                              const cv_bridge::CvImage::ConstPtr& depth_image,
                                              rp_head_tracking::Heads& heads_message)
{
    heads_message.header.frame_id = frame_id;
    heads_message.header.seq = frame_number;
    heads_message.header.stamp = time_stamp;

    int head_count = heads.size();
    heads_message.x.resize(head_count);
    heads_message.y.resize(head_count);
    heads_message.width.resize(head_count);
    heads_message.height.resize(head_count);
    heads_message.depth.resize(head_count);

    for (int i = 0; i < head_count; i++)
    {
        cv::Rect current_head_rectangle = heads[i].rectangle;

        heads_message.x[i] = current_head_rectangle.x;
        heads_message.y[i] = current_head_rectangle.y;
        heads_message.width[i] = current_head_rectangle.width;
        heads_message.height[i] = current_head_rectangle.height;
        heads_message.depth[i] = depth_image->image.at<float>(current_head_rectangle.y + (current_head_rectangle.height / 2),
                                                              current_head_rectangle.x + (current_head_rectangle.width / 2));
    }
}
