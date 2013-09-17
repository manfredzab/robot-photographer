/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <head_tracking.hpp>

// ROS includes
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt8.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// RPHeadTracking includes
#include <utils.hpp>
#include <constants.hpp>

// RPHeadTracking message includes
#include <rp_head_tracking/Heads.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_head_tracking");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPHeadTrackingNode worker_node(node);
}


RPHeadTrackingNode::RPHeadTrackingNode(ros::NodeHandle& node) :
    // Private fields
    node(node),
    color_face_detector(node),
    skin_classifier(node),
    depth_head_detector(node),
    debug_view_enabled(false),
    color_view_enabled(true),
    backprojection_view_enabled(false),
    current_tracking_frame_number(0),
    face_hue_sample_count(0),
    state(INVALID),

    // Constants
    WINDOW_NAME("RP Head Tracker Output"),

    // Overridable parameters
    GUI_OUTPUT_ENABLED(GUI_OUTPUT_ENABLED_DEFAULT),
    NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION(NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION_DEFAULT),
    NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST(NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST_DEFAULT),
    DEPTH_SHADOW_FILTER_ENABLED(DEPTH_SHADOW_FILTER_ENABLED_DEFAULT),
    SMOOTHING_FILTER_ENABLED(SMOOTHING_FILTER_ENABLED_DEFAULT),
    SMOOTHING_RADIUS(SMOOTHING_RADIUS_DEFAULT),
    SKIN_HEURISTIC_ENABLED(SKIN_HEURISTIC_ENABLED_DEFAULT),
    SKIN_LIKELIHOOD_RATIO_MINIMUM(SKIN_LIKELIHOOD_RATIO_MINIMUM_DEFAULT),
    FACE_HUE_SAMPLES_LIMIT(FACE_HUE_SAMPLES_LIMIT_DEFAULT),
    KLR_CLASSIFIER_TRAINING_TIME_LIMIT(KLR_CLASSIFIER_TRAINING_TIME_LIMIT_DEFAULT),
    KLR_CLASSIFIER_ENABLED(KLR_CLASSIFIER_ENABLED_DEFAULT)
{
    // Initialize publishers
    heads_publisher = node.advertise<rp_head_tracking::Heads>("/rp/head_tracking/heads", 1);
    state_publisher = node.advertise<std_msgs::UInt8>("/rp/head_tracking/state", 1);

    // Get static parameters from the parameter server
    getStaticParameters();

    // If GUI is enabled, construct the output window
    if (GUI_OUTPUT_ENABLED)
    {
        cv::namedWindow(WINDOW_NAME, 0);
        cv::resizeWindow(WINDOW_NAME, 640, 480);
        render_image = cv::Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
    }

    // Create subscribers
    message_filters::Subscriber<sensor_msgs::Image> depth_data_subscriber(node, "/camera/depth_registered/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> color_data_subscriber(node, "/camera/rgb/image_color", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SynchronizationPolicy;

    message_filters::Synchronizer<SynchronizationPolicy> sync(SynchronizationPolicy(10), depth_data_subscriber, color_data_subscriber);
    sync.registerCallback(boost::bind(&RPHeadTrackingNode::sensorInputCallback, this, _1, _2));

    // Set the initial state
    updateState(KLR_CLASSIFIER_ENABLED ? GATHERING_FACE_HUE_DATA : DETECTING_HEADS);
    
    // Spin at 10 Hz
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}


void RPHeadTrackingNode::getStaticParameters()
{
    node.getParamCached("/rp/head_tracking_node/gui_output_enabled", GUI_OUTPUT_ENABLED);
    node.getParamCached("/rp/head_tracking_node/number_of_frames_until_detector_reinitialization", NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION);
    node.getParamCached("/rp/head_tracking_node/number_of_detection_misses_until_head_is_lost", NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST);
    node.getParamCached("/rp/head_tracking_node/face_hue_samples_limit", FACE_HUE_SAMPLES_LIMIT);
    node.getParamCached("/rp/head_tracking_node/klr_classifier_enabled", KLR_CLASSIFIER_ENABLED);
    node.getParamCached("/rp/head_tracking_node/klr_classifier_training_time_limit", KLR_CLASSIFIER_TRAINING_TIME_LIMIT);
}


void RPHeadTrackingNode::getDynamicHeadDetectionAndTrackingParameters()
{
    node.getParamCached("/rp/head_tracking_node/skin_heuristic_enabled", SKIN_HEURISTIC_ENABLED);
    node.getParamCached("/rp/head_tracking_node/skin_likelihood_ratio_minimum", SKIN_LIKELIHOOD_RATIO_MINIMUM);
    node.getParamCached("/rp/head_tracking_node/depth_shadow_filter_enabled", DEPTH_SHADOW_FILTER_ENABLED);
    node.getParamCached("/rp/head_tracking_node/smoothing_filter_enabled", SMOOTHING_FILTER_ENABLED);
    node.getParamCached("/rp/head_tracking_node/smoothing_radius", SMOOTHING_RADIUS);
}


void RPHeadTrackingNode::sensorInputCallback(const sensor_msgs::Image::ConstPtr& depth_input_image, const sensor_msgs::Image::ConstPtr& color_input_image)
{
    bool enabled = false;
    node.getParamCached("/rp/head_tracking_node/enabled", enabled);

    if (!enabled)
    {
        return;
    }

    if (frame_id.empty())
    {
        frame_id = color_input_image->header.frame_id;
    }
    getDynamicHeadDetectionAndTrackingParameters();

    switch (state)
    {
        case GATHERING_FACE_HUE_DATA:
        {
            color_view_enabled = true;

            cv_bridge::CvImage::ConstPtr color_image = RPUtils::shareToCvImage(color_input_image);

            detectFaces(color_image);

            ROS_INFO("Gathering face hue data: %d samples collected.", face_hue_sample_count);

            if (face_hue_sample_count > FACE_HUE_SAMPLES_LIMIT)
            {
                face_hue_classifier.train(KLR_CLASSIFIER_TRAINING_TIME_LIMIT);

                face_hue_sample_count = 0;

                updateState(DETECTING_HEADS);
            }

            break;
        }
        case DETECTING_HEADS:
        {
            color_view_enabled = false;

            cv_bridge::CvImage::Ptr color_image = RPUtils::copyToCvImage(color_input_image);
            cv_bridge::CvImage::Ptr depth_image = RPUtils::copyToCvImage(depth_input_image);

            detectHeads(depth_image, color_image);

            publishHeads(depth_image);

            //ROS_INFO("Detecting heads from depth: %d heads detected.", heads.size());

            if (heads.size() > 0)
            {
                updateState(TRACKING_HEADS);

                current_tracking_frame_number = 0;
            }

            break;
        }
        case TRACKING_HEADS:
        {
            // Update frame count
            current_tracking_frame_number++;

            color_view_enabled = false;

            cv_bridge::CvImage::Ptr depth_image = RPUtils::copyToCvImage(depth_input_image);

            trackHeads(depth_image);

            publishHeads(depth_image);

            //ROS_INFO("Tracking heads from depth: %d heads tracked.", heads.size());

            if ((heads.size() == 0) || (0 == current_tracking_frame_number % NUMBER_OF_FRAMES_UNTIL_DETECTOR_REINITIALIZATION))
            {
                updateState(DETECTING_HEADS);
            }

            break;
        }
    }

    if (GUI_OUTPUT_ENABLED)
    {
        cv::imshow(WINDOW_NAME, render_image);

        processOutputWindowKeyPress();
    }
}


void RPHeadTrackingNode::updateState(const HeadTrackingState input_state)
{
    state = input_state;

    std_msgs::UInt8 state_message;
    state_message.data = static_cast<uint8_t>(state);

    state_publisher.publish(state_message);
}


void RPHeadTrackingNode::publishHeads(const cv_bridge::CvImage::ConstPtr& depth_image)
{
    rp_head_tracking::Heads heads_message;

    RPHeadsMessageBuilder::buildHeadsMessage(frame_id,
                                             current_tracking_frame_number,
                                             ros::Time::now(),
                                             heads,
                                             depth_image,
                                             heads_message);

    heads_publisher.publish(heads_message);
}


void RPHeadTrackingNode::detectHeads(const cv_bridge::CvImage::Ptr& depth_image, const cv_bridge::CvImage::Ptr& color_image)
{
    if (DEPTH_SHADOW_FILTER_ENABLED)
    {
        depth_image_processor.filterDepthShadow(depth_image);
    }
    else
    {
        depth_image_processor.filterMissingData(depth_image);
    }

    if (SMOOTHING_FILTER_ENABLED)
    {
        depth_image_processor.smoothDepthImage(depth_image, SMOOTHING_RADIUS);
    }

    if (GUI_OUTPUT_ENABLED)
    {
		if (color_view_enabled)
		{
		    if (!backprojection_view_enabled)
		    {
		        color_image_processor.convertColorImageToRenderImage(color_image, render_image);
		    }
		    else
			{
			    color_image_processor.convertColorImageToBackpropagationImage(color_image, render_image, skin_classifier, SKIN_LIKELIHOOD_RATIO_MINIMUM);
			}
		}
		else
		{
			depth_image_processor.convertDepthImageToRenderImage(depth_image, render_image);
		}
    }

    std::vector<cv::Rect> new_head_rectangles;
    depth_head_detector.detectHeads(depth_image, color_image, skin_classifier, face_hue_classifier, new_head_rectangles, render_image, false);

    // Increment the counters on the old heads which were not currently detected
    for (unsigned i = 0; i < heads.size(); i++)
    {
        bool corresponding_new_head_found = false;
        for (unsigned j = 0; j < new_head_rectangles.size(); j++)
        {
            if (RPUtils::areRectanglesSufficientlySimilar(heads[i].rectangle, new_head_rectangles[j]))
            {
                corresponding_new_head_found = true;
                break;
            }
        }

        if (!corresponding_new_head_found)
        {
            heads[i].last_detected_history++;
        }
    }

    // Add/update newly found heads
    for (unsigned i = 0; i < new_head_rectangles.size(); i++)
    {
        cv::Rect& new_head_rectangle = new_head_rectangles[i];

        bool old_head_found = false;
        for (unsigned j = 0; j < heads.size(); j++)
        {
            if (RPUtils::areRectanglesSufficientlySimilar(new_head_rectangle, heads[j].rectangle))
            {
                heads[j].last_detected_history = 0;
                heads[j].rectangle = new_head_rectangle;

                old_head_found = true;
                break;
            }
        }

        // Check if the detected head is new
        if (!old_head_found)
        {
            Head new_head = { new_head_rectangle, 0 };
            heads.push_back(new_head);
        }
    }

    // Check for heads that should be removed
    if (heads.size() > 0)
    {
        for (int i = heads.size() - 1; i >= 0; i--)
        {
            if (heads[i].last_detected_history > NUMBER_OF_DETECTION_MISSES_UNTIL_HEAD_IS_LOST)
            {
                heads.erase(heads.begin() + i);
            }
        }
    }

    if (GUI_OUTPUT_ENABLED)
    {
        for (unsigned i = 0; i < heads.size(); i++)
        {
            cv::rectangle(render_image, heads[i].rectangle, cv::Scalar(255, 0, 0));
        }
    }
}


void RPHeadTrackingNode::detectFaces(const cv_bridge::CvImage::ConstPtr& color_image)
{
    if (GUI_OUTPUT_ENABLED && color_view_enabled)
    {
        color_image_processor.convertColorImageToRenderImage(color_image, render_image);
    }

    std::vector<cv::Rect> faces;
    color_face_detector.detectFaces(color_image, faces);

    face_hue_sample_count += faces.size();

    for (unsigned i = 0; i < faces.size(); i++)
    {
        std::vector<double> positive_hue_histogram;
        color_image_processor.createSkinHueTrainingExample(faces[i], color_image, positive_hue_histogram);
        face_hue_classifier.addTrainingSample(positive_hue_histogram, true);

        std::vector<double> negative_hue_histogram;
        color_image_processor.createNonSkinHueTrainingExample(faces[i], color_image, negative_hue_histogram);
        face_hue_classifier.addTrainingSample(negative_hue_histogram, false);
    }

    cv::Scalar face_color(0, 255, 255);
    if (GUI_OUTPUT_ENABLED && color_view_enabled)
    {
        for (unsigned i = 0; i < faces.size(); i++)
        {
            cv::rectangle(render_image, faces[i], face_color);
            char head_count_string[256];
            sprintf(head_count_string, "%d", face_hue_sample_count - faces.size() + i + 1);
            cv::putText(render_image, head_count_string, faces[i].tl(), cv::FONT_HERSHEY_PLAIN, 1.0, face_color);
        }
    }
}


void RPHeadTrackingNode::trackHeads(const cv_bridge::CvImage::Ptr& depth_image)
{
    if (DEPTH_SHADOW_FILTER_ENABLED)
    {
        depth_image_processor.filterDepthShadow(depth_image);
    }
    else
    {
        depth_image_processor.filterMissingData(depth_image);
    }

    if (SMOOTHING_FILTER_ENABLED)
    {
        depth_image_processor.smoothDepthImage(depth_image, SMOOTHING_RADIUS);
    }

    if (GUI_OUTPUT_ENABLED && !color_view_enabled)
    {
        depth_image_processor.convertDepthImageToRenderImage(depth_image, render_image);
    }

    depth_head_tracker.trackHeads(depth_image, depth_head_detector, heads);

    if (GUI_OUTPUT_ENABLED && !color_view_enabled)
    {
        for (unsigned i = 0; i < heads.size(); i++)
        {
            cv::rectangle(render_image, heads[i].rectangle, cv::Scalar(0, 255, 255));
        }
    }
}

void RPHeadTrackingNode::processOutputWindowKeyPress()
{
    char key = cv::waitKey(30);
    switch (key)
    {
        case 'y':
        {
            GUI_OUTPUT_ENABLED = !GUI_OUTPUT_ENABLED;
            node.setParam("/rp/head_tracking_node/gui_output_enabled", GUI_OUTPUT_ENABLED);

            break;
        }
        case 'g':
        {
            SKIN_LIKELIHOOD_RATIO_MINIMUM += 0.01;
            node.setParam("/rp/head_tracking_node/skin_likelihood_ratio_minimum", SKIN_LIKELIHOOD_RATIO_MINIMUM);

            ROS_INFO("Skin likelihood ratio: %f", SKIN_LIKELIHOOD_RATIO_MINIMUM);

            break;
        }
        case 'b':
        {
            SKIN_LIKELIHOOD_RATIO_MINIMUM -= 0.01;
            if (SKIN_LIKELIHOOD_RATIO_MINIMUM < 0.0)
            {
                SKIN_LIKELIHOOD_RATIO_MINIMUM = 0.0;
            }
            node.setParam("/rp/head_tracking_node/skin_likelihood_ratio_minimum", SKIN_LIKELIHOOD_RATIO_MINIMUM);

            ROS_INFO("Skin likelihood ratio: %f", SKIN_LIKELIHOOD_RATIO_MINIMUM);

            break;
        }
        case 't':
        {
            backprojection_view_enabled = !backprojection_view_enabled;
            break;
        }
        case 'r':
        {
            color_view_enabled = !color_view_enabled;
            break;
        }
        case 'e':
        {
            debug_view_enabled = !debug_view_enabled;
            break;
        }
        case 'q':
        {
            bool current_depth_shadow_filter_enabled;
            node.getParamCached("/rp/head_tracking_node/depth_shadow_filter_enabled", current_depth_shadow_filter_enabled);

            current_depth_shadow_filter_enabled = !current_depth_shadow_filter_enabled;

            node.setParam("/rp/head_tracking_node/depth_shadow_filter_enabled", current_depth_shadow_filter_enabled);
            break;
        }
        case 'w':
        {
            bool current_smoothing_filter_enabled;
            node.getParamCached("/rp/head_tracking_node/smoothing_filter_enabled", current_smoothing_filter_enabled);

            current_smoothing_filter_enabled = !current_smoothing_filter_enabled;

            node.setParam("/rp/head_tracking_node/smoothing_filter_enabled", current_smoothing_filter_enabled);
            break;
        }
        case 's':
        {
            int current_smoothing_radius;
            node.getParamCached("/rp/head_tracking_node/smoothing_radius", current_smoothing_radius);

            current_smoothing_radius++;

            node.setParam("/rp/head_tracking_node/smoothing_radius", current_smoothing_radius);
            break;
        }
        case 'x':
        {
            int current_smoothing_radius;
            node.getParamCached("/rp/head_tracking_node/smoothing_radius", current_smoothing_radius);

            if (current_smoothing_radius > 1)
            {
                current_smoothing_radius--;
                node.setParam("/rp/head_tracking_node/smoothing_radius", current_smoothing_radius);
            }

            break;
        }
    }
}
