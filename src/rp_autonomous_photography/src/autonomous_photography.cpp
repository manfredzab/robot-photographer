/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <autonomous_photography.hpp>

// ROS includes
#include <std_msgs/String.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// RPFraming includes
#include <framing_status.h>

// RPNavigation includes
#include <direction_source.h>

// RPCamera includes
#include <rp_camera/PhotoService.h>

// RPUploader includes
#include <rp_uploader/UploaderService.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_autonomous_photography");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPAutonomousPhotographyNode worker_node(node);
}


RPAutonomousPhotographyNode::RPAutonomousPhotographyNode(ros::NodeHandle& node) :
    node(node),
    is_taking_picture(false),
    waiting_for_camera(false),
    state(AVOIDING_OBSTACLES),
    framing_state(NO_FRAME),
    TIME_BETWEEN_PICTURES(TIME_BETWEEN_PICTURES_DEFAULT)
{
    // Get overridable parameters from the parameter server
    getOverridableParameters();

    // Create publishers/subscribers/service clients/timers
    state_publisher = node.advertise<std_msgs::UInt8>("/rp/autonomous_photography/state", 1);
    direction_source_publisher = node.advertise<std_msgs::UInt8>("/rp/autonomous_photography/direction_source", 1);

    framing_status_subscriber = node.subscribe("/rp/framing/frame", 1, &RPAutonomousPhotographyNode::framingMessageCallback, this);

    camera_client = node.serviceClient<rp_camera::PhotoService>("/rp/camera/photo");
    uploader_client = node.serviceClient<rp_uploader::UploaderService>("/rp/uploader/upload");

    framing_timer = node.createWallTimer(ros::WallDuration(TIME_BETWEEN_PICTURES), &RPAutonomousPhotographyNode::framingTimerCallback, this, true, true);
    framing_timer_start_time = ros::WallTime::now();

    camera_timer = node.createWallTimer(ros::WallDuration(CAMERA_DELAY_S), &RPAutonomousPhotographyNode::cameraTimerCallback, this, true, false);

    // Sleep for 3 seconds to ensure that all connections are established
    ros::Duration stabilize_duration(3.0);
    stabilize_duration.sleep();

    // Enable the locomotion node
    setLocomotionEnabled(true);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
};


void RPAutonomousPhotographyNode::setLocomotionEnabled(const bool enabled)
{
    node.setParam("/rp/locomotion_node/enabled", enabled);
}


void RPAutonomousPhotographyNode::setFramingEnabled(const bool enabled)
{
    node.setParam("/rp/framing_node/enabled", enabled);
}


void RPAutonomousPhotographyNode::setHeadTrackingEnabled(const bool enabled)
{
    node.setParam("/rp/head_tracking_node/enabled", enabled);
}


void RPAutonomousPhotographyNode::framingMessageCallback(const rp_framing::Frame& frame_message)
{
    // Check if we are not in an obstacle avoidance state.
    AutonomousPhotographyState current_state;
    state_mutex.lock();
    {
        current_state = state;
    }
    state_mutex.unlock();

    if (AVOIDING_OBSTACLES == current_state)
    {
        return;
    }

    // Check what state was reported by the framing node. If good framing is not reported, ignore it.
    FramingStatus current_framing_state = static_cast<FramingStatus>(frame_message.framing_status);
    framing_state_mutex.lock();
    {
        framing_state = current_framing_state;
    }
    framing_state_mutex.unlock();

    if (FRAMED == current_framing_state)
    {
        if (!is_taking_picture)
        {
            is_taking_picture = true;
            picture_thread = boost::thread(&RPAutonomousPhotographyNode::takeAndUploadPicture, this);

            waiting_for_camera = true;
            camera_timer.setPeriod(ros::WallDuration(CAMERA_DELAY_S));
            camera_timer.start();
        }

        if (waiting_for_camera)
        {
            frame_mutex.lock();
            {
                parseFramingMessage(frame_message);
            }
            frame_mutex.unlock();
        }
    }
}


void RPAutonomousPhotographyNode::parseFramingMessage(const rp_framing::Frame& frame_message)
{
    ROS_INFO("-------------- FRAME UPDATE BEGIN ------------------");
    frame = cv::Rect(frame_message.frame_x, frame_message.frame_y, frame_message.frame_width, frame_message.frame_height);

    ROS_INFO("Frame: (%d, %d) %d x %d", frame.x, frame.y, frame.width, frame.height);

    heads.clear();
    for (unsigned i = 0; i < frame_message.heads_x.size(); i++)
    {
        heads.push_back(cv::Rect(frame_message.heads_x[i], frame_message.heads_y[i], frame_message.heads_width[i], frame_message.heads_height[i]));
        ROS_INFO("Head: (%d, %d) %d x %d", heads.back().x, heads.back().y, heads.back().width, heads.back().height);
    }
    ROS_INFO("-------------- FRAME UPDATE END ------------------\n");
}


void RPAutonomousPhotographyNode::takeAndUploadPicture()
{
    setState(TAKING_PICTURE);

    // Stop the robot from moving
    publishDirectionSource(NONE);

    // Take the picture
    rp_camera::PhotoService photo_service_message;
    if (camera_client.call(photo_service_message))
    {
        std::string picture_file_name = photo_service_message.response.picture_file_name;
        ROS_INFO("Received the following filename: %s", picture_file_name.c_str());

        // Load the picture
        cv::Mat picture = cv::imread(picture_file_name, CV_LOAD_IMAGE_COLOR);

        // Set up the file name strings for framed/annotated pictures
        std::string cropped_picture_file_name = FRAMED_PICTURE_PREFIX + picture_file_name;
        std::string annotated_picture_file_name = ANNOTATED_PICTURE_PREFIX + picture_file_name;

        // Crop out the frame and mark it in the original picture
        cv::Mat frame_region;
        frame_mutex.lock();
        {
            frame_region = cv::Mat(picture, frame);

            // Save the cropped image
            cv::imwrite(cropped_picture_file_name, frame_region);

            // Annotate the original picture
            cv::rectangle(picture, frame, cv::Scalar(255, 255, 255), 5);
            for (unsigned i = 0; i < heads.size(); i++)
            {
                cv::rectangle(picture, heads[i], cv::Scalar(0, 0, 255), 5);
            }


        }
        frame_mutex.unlock();

        // Save the annotated original image
        cv::imwrite(annotated_picture_file_name, picture);

        // Upload to image to Flickr
        rp_uploader::UploaderService uploader_service_message;
        uploader_service_message.request.picture_file_name = cropped_picture_file_name;

        if (!uploader_client.call(uploader_service_message) || !uploader_service_message.response.success)
        {
            ROS_WARN("Could not upload the picture %s.", cropped_picture_file_name.c_str());
        }
    }

    is_taking_picture = false;

    // Switch back to obstacle avoidance mode
    setState(AVOIDING_OBSTACLES);

    // Disable framing and head-tracking nodes until the next framing timer callback
    setFramingEnabled(false);
    setHeadTrackingEnabled(false);


    framing_timer.stop();
    framing_timer.setPeriod(ros::WallDuration(TIME_BETWEEN_PICTURES));
    framing_timer.start();
    framing_timer_start_time = ros::WallTime::now();
}


void RPAutonomousPhotographyNode::publishDirectionSource(DirectionSource direction_source)
{
    std_msgs::UInt8 direction_source_message;
    direction_source_message.data = static_cast<uint8_t>(direction_source);

    direction_source_publisher.publish(direction_source_message);
}


void RPAutonomousPhotographyNode::cameraTimerCallback(const ros::WallTimerEvent& timer_event)
{
    ROS_INFO("Camera timer called back.");

    waiting_for_camera = false;
    camera_timer.stop();

    setState(UPLOADING_PICTURE);
    publishDirectionSource(OBSTACLE_AVOIDANCE_NODE);
}


void RPAutonomousPhotographyNode::framingTimerCallback(const ros::WallTimerEvent& timer_event)
{
    ROS_INFO("Framing timer callback...");

    setState(FRAMING_PICTURE);

    // Enable head-tracking and framing nodes
    setHeadTrackingEnabled(true);
    setFramingEnabled(true);

    publishDirectionSource(FRAMING_NODE);
}


void RPAutonomousPhotographyNode::setState(AutonomousPhotographyState input_state)
{
    // Publish the state
    std_msgs::UInt8 state_message;
    state_message.data = static_cast<uint8_t>(input_state);

    state_publisher.publish(state_message);

    // Save the state
    state_mutex.lock();
    {
        state = input_state;
    }
    state_mutex.unlock();
}


void RPAutonomousPhotographyNode::getOverridableParameters()
{
    node.getParamCached("/rp/autonomous_photography_node/time_between_pictures", TIME_BETWEEN_PICTURES);
}
