/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <framing.hpp>

// C++ includes
#include <cmath>
#include <limits>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

// RPHeadTracking includes
#include <distance_converter.hpp>
#include <utils.hpp>

// RPFraming message includes
#include <rp_framing/Frame.h>

// RPCamera photo service includes
#include <rp_camera/PhotoService.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_framing");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPFramingNode worker_node(node);
}


RPFramingNode::RPFramingNode(ros::NodeHandle& node) :
    node(node),
    node_enabled(false),
    framing_status(NO_FRAME),
    WINDOW_NAME("RP Framing Output"),
    CAMERA_FRAME_RECTANGLE(cv::Rect(0, 0, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT)),

    CAMERA_TRANSLATION_X(CAMERA_TRANSLATION_X_DEFAULT),
    CAMERA_TRANSLATION_Y(CAMERA_TRANSLATION_Y_DEFAULT),
    CAMERA_TRANSLATION_Z(CAMERA_TRANSLATION_Z_DEFAULT),

    CAMERA_ROTATION_X(CAMERA_ROTATION_X_DEFAULT),
    CAMERA_ROTATION_Y(CAMERA_ROTATION_Y_DEFAULT),
    CAMERA_ROTATION_Z(CAMERA_ROTATION_Z_DEFAULT),

    GUI_OUTPUT_ENABLED(GUI_OUTPUT_ENABLED_DEFAULT)
{
    // Retrieve overridable parameters from the parameter server
    getOverridableParameters();

    // Initialize frame and driving direction publishers
    frame_publisher = node.advertise<rp_framing::Frame>("/rp/framing/frame", 1);
    driving_direction_publisher = node.advertise<std_msgs::UInt8>("/rp/framing/driving_direction", 1);

    // Initialize subscribers and clients (depending on the debug mode)
    bool debug_mode_enabled = false;
    node.getParamCached("/rp/framing_node/debug_mode_enabled", debug_mode_enabled);

    if (debug_mode_enabled)
    {
        depth_subscriber = node.subscribe<sensor_msgs::Image>("/camera/depth_registered/image", 1, &RPFramingNode::depthImageCallback, this);
        camera_client = node.serviceClient<rp_camera::PhotoService>("/rp/camera/photo");
    }
    else
    {
        head_tracker_subscriber = node.subscribe("/rp/head_tracking/heads", 1, &RPFramingNode::headTrackerMessageCallback, this);
    }

    if (GUI_OUTPUT_ENABLED)
    {
        cv::namedWindow(WINDOW_NAME, 0);
        cv::resizeWindow(WINDOW_NAME, 400, 300);
    }

    // TODO: Read camera matrix from a file
    double f_x = 3.36844496231691e+003;
    double f_y = 3.35219250970755e+003;
    double c_x = 2.03462408202436e+003;
    double c_y = 1.65514654107310e+003;

    double k1 = -3.4919904628252375e-002;
    double k2 = 1.5841322842869990e-001;
    double k3 = -1.5475861168401756e-001;

    double p1 = -1.1552648177682796e-003;
    double p2 = -1.2701354733792702e-002;

    camera_matrix = (cv::Mat_<double>(3, 3) <<
            f_x, 0.0, c_x,
            0.0, f_y, c_y,
            0.0, 0.0, 1.0);

    distortion_coefficients = (cv::Mat_<double>(5, 1) <<
            k1, k2, p1, p2, k3);

    initializeTranslationVector();
    initializeRotationVector();

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
};


void RPFramingNode::initializeRotationVector()
{
    double angle_x_rad = CAMERA_ROTATION_X * M_PI / 180.0;
    double angle_y_rad = CAMERA_ROTATION_Y * M_PI / 180.0;
    double angle_z_rad = CAMERA_ROTATION_Z * M_PI / 180.0;

    cv::Mat rotation_x = (cv::Mat_<double>(3, 3) <<
            1.0,               0.0,               0.0,
            0.0,               cos(angle_x_rad),  -sin(angle_x_rad),
            0.0,               sin(angle_x_rad),  cos(angle_x_rad));

    cv::Mat rotation_y = (cv::Mat_<double>(3, 3) <<
            cos(angle_y_rad),  0.0,               sin(angle_y_rad),
            0.0,               1.0,               0.0,
            -sin(angle_y_rad), 0.0,               cos(angle_y_rad));

    cv::Mat rotation_z = (cv::Mat_<double>(3, 3) <<
            cos(angle_z_rad),  -sin(angle_z_rad), 0.0,
            sin(angle_z_rad),  cos(angle_z_rad),  0.0,
            0.0,               0.0,               1.0);

    cv::Mat combined_rotation = rotation_z * rotation_y * rotation_x;

    // Convert to Rodrigues representation (axis/angle, where angle is the magnitude of the vector)
    rotation_vector = cv::Mat_<double>(3, 1);
    cv::Rodrigues(combined_rotation, rotation_vector);
}


void RPFramingNode::initializeTranslationVector()
{
    translation_vector = (cv::Mat_<double>(3, 1) <<
        CAMERA_TRANSLATION_X, CAMERA_TRANSLATION_Y, CAMERA_TRANSLATION_Z);
}


void RPFramingNode::getOverridableParameters()
{
    node.getParamCached("/rp/framing_node/camera_translation_x", CAMERA_TRANSLATION_X);
    node.getParamCached("/rp/framing_node/camera_translation_y", CAMERA_TRANSLATION_Y);
    node.getParamCached("/rp/framing_node/camera_translation_z", CAMERA_TRANSLATION_Z);

    node.getParamCached("/rp/framing_node/camera_rotation_x",    CAMERA_ROTATION_X);
    node.getParamCached("/rp/framing_node/camera_rotation_y",    CAMERA_ROTATION_Y);
    node.getParamCached("/rp/framing_node/camera_rotation_z",    CAMERA_ROTATION_Z);

    node.getParamCached("/rp/framing_node/gui_output_enabled",   GUI_OUTPUT_ENABLED);
}


void RPFramingNode::depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_input_image)
{
    getOverridableParameters();
    initializeRotationVector();
    initializeTranslationVector();

    cv::Mat render_image = cv::Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);

    cv_bridge::CvImage::Ptr depth_image = RPUtils::copyToCvImage(depth_input_image);
    //depth_processor.filterMissingData(depth_image);
    depth_processor.convertDepthImageToRenderImage(depth_image, render_image);

    cv::imshow("[DEBUG]", render_image);
    char key = cv::waitKey();

    if (key == 'p')
    {
        rp_camera::PhotoService photo_service_message;
        if (camera_client.call(photo_service_message))
        {
            std::string picture_file_name = photo_service_message.response.picture_file_name;
            ROS_INFO("Received the following filename: %s", picture_file_name.c_str());
        }
    }
    else if (key == 'q')
    {
        std::vector<cv::Vec3b> object_colors;
        std::vector<cv::Point3f> object_points;
        for (int y = 0; y < depth_image->image.rows; y++)
        {
            for (int x = 0; x < depth_image->image.cols; x++)
            {
                float depth_value = depth_image->image.at<float>(y, x);
                if (depth_value == depth_value) // Not a NaN
                {
                    int distance_from_center_x = x - (FRAME_WIDTH / 2);
                    int distance_from_center_y = y - (FRAME_HEIGHT / 2);

                    float left_m = RPDistanceConverter::pixelsToWidth(distance_from_center_x, depth_value);
                    float top_m = RPDistanceConverter::pixelsToHeight(distance_from_center_y, depth_value);

                    object_points.push_back(cv::Point3f(left_m, top_m, depth_value));
                    object_colors.push_back(render_image.at<cv::Vec3b>(y, x));
                }
            }
        }

        std::vector<cv::Point2f> projected_image_points;
        cv::projectPoints(object_points,
                          rotation_vector,
                          translation_vector,
                          camera_matrix,
                          distortion_coefficients,
                          projected_image_points);

        cv::Mat projected_image = cv::Mat::zeros(CAMERA_FRAME_HEIGHT, CAMERA_FRAME_WIDTH, CV_8UC3);
        for (unsigned i = 0; i < projected_image_points.size(); i++)
        {
            cv::Point2f projection = projected_image_points[i];

            if ((projection.x > 0) && (projection.x < CAMERA_FRAME_WIDTH - 1) && (projection.y > 0) && (projection.y < CAMERA_FRAME_HEIGHT - 1))
            {
                cv::rectangle(projected_image, cv::Point(projection.x - 1, projection.y - 1), cv::Point(projection.x + 1, projection.y + 1), cv::Scalar(object_colors[i]), -1);
            }
        }

        cv::imwrite("P_out.jpg", projected_image);
    }
}


void RPFramingNode::refreshNodeEnabledStatus()
{
    // Check whether the node is enabled in parameter server
    bool enabled = false;
    node.getParamCached("/rp/framing_node/enabled", enabled);

    if (!enabled)
    {
        node_enabled = false;
    }
    else if (!node_enabled)
    {
        node_enabled = true;

        // Initial enable: mark the time
        enable_time = ros::WallTime::now();
    }
}


void RPFramingNode::headTrackerMessageCallback(const rp_head_tracking::Heads& tracked_heads)
{
    ros::Time processing_start = ros::Time::now();

    refreshNodeEnabledStatus();
    if (!node_enabled)
    {
        return;
    }

    // Convert tracked heads to top left/bottom right head rectangle points in the Kinect
    // camera coordinates
    std::vector<cv::Point3f> head_points;
    convertToKinectCameraCoordinates(tracked_heads, head_points);

    if (head_points.empty())
    {
        // ROS_INFO("No heads to frame.");

        framing_status = NO_FRAME;
        driving_direction = FORWARD;

        publishFramingStatus(NULL);
        publishDrivingDirection();

        return;
    }

    // Create an empty image for rendering if GUI output is enabled
    cv::Mat render_image;
    if (GUI_OUTPUT_ENABLED)
    {
        render_image = cv::Mat::zeros(CAMERA_FRAME_HEIGHT, CAMERA_FRAME_WIDTH, CV_8UC3);
    }

    // Convert head rectangle points in the Kinect camera coordinates to photographic camera
    // image plane coordinates
    std::vector<cv::Rect> projected_head_rectangles;
    convertToCameraImageCoordinates(head_points, projected_head_rectangles);

    // Find the head rectangle closest to the center of the camera image plane
    int closest_rectangle_index = centermostHeadRectangleIndex(projected_head_rectangles);

    // Try framing the person closest to the picture center
    frameSinglePerson(projected_head_rectangles[closest_rectangle_index], true, frame);

    // Check for additional candidates in the frame
    std::set<cv::Rect, RPRectangleComparator> candidates;
    getCandidatesInFrame(projected_head_rectangles, frame, candidates);

    // Check whether framing a single person slightly off to the right yields better results
    if (candidates.size() > 1)
    {
        cv::Rect frame_right;
        frameSinglePerson(projected_head_rectangles[closest_rectangle_index], false, frame_right);

        std::set<cv::Rect, RPRectangleComparator> candidates_to_include_right;
        getCandidatesInFrame(projected_head_rectangles, frame_right, candidates_to_include_right);

        if (candidates_to_include_right.size() < candidates.size())
        {
            candidates = candidates_to_include_right;
            frame = frame_right;
        }
    }

    // If we there are more people in our single-person frame, iteratively update the framing
    bool framed = (candidates.size() == 1);
    while (!framed)
    {
        // Find the bounding rectangle for all candidates
        cv::Rect bounding_rectangle = projected_head_rectangles[closest_rectangle_index];
        for (std::set<cv::Rect, RPRectangleComparator>::iterator it = candidates.begin(); it != candidates.end(); ++it)
        {
            bounding_rectangle |= *it;
        }

        if (GUI_OUTPUT_ENABLED)
        {
            cv::rectangle(render_image, bounding_rectangle, cv::Scalar(255, 255, 255), 1);
        }

        // Check whether the bounding rectangle is "wide" or "narrow"
        if ((double)bounding_rectangle.width > 1.6 * (double)bounding_rectangle.height)
        {
            frameWideGroup(bounding_rectangle, frame);
        }
        else
        {
            frameNarrowGroup(bounding_rectangle, frame);
        }

        // Check whether the new frame contains any new candidates
        std::set<cv::Rect, RPRectangleComparator> new_candidates;
        getCandidatesInFrame(projected_head_rectangles, frame, new_candidates);

        if (candidates == new_candidates)
        {
            framed = true;
        }
        else
        {
            candidates = new_candidates;
        }
    }

    if (GUI_OUTPUT_ENABLED)
    {
        // Render detected head rectangles
        for (unsigned i = 0; i < projected_head_rectangles.size(); i++)
        {
            cv::rectangle(render_image, projected_head_rectangles[i], cv::Scalar(0, 0, 255), 3);
        }

        // Render the centermost head rectangle
        cv::rectangle(render_image, projected_head_rectangles[closest_rectangle_index], cv::Scalar(0, 255, 0), 3);

        // Render the calculated frame
        cv::rectangle(render_image, frame, cv::Scalar(255, 255, 0), 3);

        // Show the render image
        cv::imshow(WINDOW_NAME, render_image);
        cv::waitKey(30);
    }

    // Get the threshold for the framing quality based on the elapsed time
    // since the framing node has been enabled.
    double elapsed_seconds_since_enable = (ros::WallTime::now() - enable_time).toSec();

    double current_max_deviation_from_correct_framing = MAX_DEVIATION_FROM_CORRECT_FRAMING_PERCENT * elapsed_seconds_since_enable / MAX_TIME_FOR_FRAMING_S;
    if (current_max_deviation_from_correct_framing > MAX_DEVIATION_FROM_CORRECT_FRAMING_PERCENT)
    {
        current_max_deviation_from_correct_framing = MAX_DEVIATION_FROM_CORRECT_FRAMING_PERCENT;
    }

    // Calculate the minimum and the current framing overlap percentage
    double min_allowed_framing_overlap = 100.0 - current_max_deviation_from_correct_framing;
    double current_framing_overlap = 100.0 * (double)(frame & CAMERA_FRAME_RECTANGLE).area() / (double)frame.area();

    //ROS_INFO("Current overlap: %f, allowed overlap: %f", current_framing_overlap, min_allowed_framing_overlap);

    // Clip the frame to the picture rectangle and ensure that the frame is at least of the minimal size
    frame &= CAMERA_FRAME_RECTANGLE;
    bool sufficient_frame_size = (frame.width >= MINIMAL_PICTURE_FRAME_WIDTH) && (frame.height >= MINIMAL_PICTURE_FRAME_HEIGHT);
    if (sufficient_frame_size && (current_framing_overlap > min_allowed_framing_overlap))
    {
        framing_status = FRAMED;
        driving_direction = STOP;
    }
    else
    {
        if (!sufficient_frame_size)
        {
            framing_status = FRAME_TOO_SMALL;
            driving_direction = FORWARD;
        }
        else
        {
            // Enclosing frame is out of the image plane. Try to center the composition.
            if ((frame.y < 0) || (frame.y + frame.height > CAMERA_FRAME_HEIGHT))
            {
                //ROS_INFO("Ideal frame is out of vertical bounds.");

                framing_status = FRAME_OUT_OF_BOUNDS_VERTICALLY;
            }
            else // Enclosing frame is out of the image plane width-wise
            {
                framing_status = FRAME_OUT_OF_BOUNDS_HORIZONTALLY;
            }

            int frame_center_x = frame.x + (frame.width / 2);
            driving_direction = (frame_center_x > (CAMERA_FRAME_WIDTH / 2)) ? RIGHT : LEFT;

            //ROS_INFO("Framing turn: %s", (driving_direction == RIGHT) ? "RIGHT" : "LEFT");
        }
    }

    publishFramingStatus(&projected_head_rectangles);
    publishDrivingDirection();
}


void RPFramingNode::publishDrivingDirection()
{
    std_msgs::UInt8 converted_direction;
    converted_direction.data = drivingDirectionToUint8(driving_direction);

    driving_direction_publisher.publish(converted_direction);
}


void RPFramingNode::publishFramingStatus(const std::vector<cv::Rect>* head_rectangles)
{
    rp_framing::Frame frame_message;

    frame_message.framing_status = static_cast<uint8_t>(framing_status);
    frame_message.frame_x = frame.x;
    frame_message.frame_y = frame.y;
    frame_message.frame_width = frame.width;
    frame_message.frame_height = frame.height;

    if (NULL != head_rectangles)
    {
        for (std::vector<cv::Rect>::const_iterator it = head_rectangles->begin(); it != head_rectangles->end(); ++it)
        {
            const cv::Rect& current_head_rectangle = *it;
            frame_message.heads_x.push_back(current_head_rectangle.x);
            frame_message.heads_y.push_back(current_head_rectangle.y);
            frame_message.heads_width.push_back(current_head_rectangle.width);
            frame_message.heads_height.push_back(current_head_rectangle.height);
        }
    }

    frame_publisher.publish(frame_message);
}


void RPFramingNode::getCandidatesInFrame(const std::vector<cv::Rect>& candidate_head_rectangles, const cv::Rect& frame, std::set<cv::Rect, RPRectangleComparator>& candidates_in_frame)
{
    for (unsigned i = 0; i < candidate_head_rectangles.size(); i++)
    {
        if ((frame & candidate_head_rectangles[i]).area() > 0)
        {
            candidates_in_frame.insert(candidate_head_rectangles[i]);
        }
    }
}


void RPFramingNode::frameWideGroup(const cv::Rect& bounding_rectangle, cv::Rect& frame)
{
    double s = 0.375 * (double)bounding_rectangle.width;

    double bounding_rectangle_center_x = bounding_rectangle.x + 0.5 * bounding_rectangle.width;
    double bounding_rectangle_center_y = bounding_rectangle.y + 0.5 * bounding_rectangle.height;

    frame.x = bounding_rectangle_center_x - 2.0 * s;
    frame.y = bounding_rectangle_center_y - 1.2 * s;

    frame.width = 4.0 * s;
    frame.height = 3.0 * s;
}



void RPFramingNode::frameNarrowGroup(const cv::Rect& bounding_rectangle, cv::Rect& frame)
{
    double s = 0.5 * (double)bounding_rectangle.height;

    double bounding_rectangle_center_x = bounding_rectangle.x + 0.5 * bounding_rectangle.width;
    double bounding_rectangle_center_y = bounding_rectangle.y + 0.5 * bounding_rectangle.height;

    frame.x = bounding_rectangle_center_x - 2.0 * s;
    frame.y = bounding_rectangle_center_y - 1.2 * s;

    frame.width = 4.0 * s;
    frame.height = 3.0 * s;
}


void RPFramingNode::frameSinglePerson(const cv::Rect& head_rectangle, bool off_center_left, cv::Rect& frame)
{
    double s = 1.5 * (double)head_rectangle.height;

    double head_center_x = head_rectangle.x + 0.5 * head_rectangle.width;
    double head_center_y = head_rectangle.y + 0.5 * head_rectangle.height;

    frame.x = head_center_x - 2.0 * s;
    frame.y = head_center_y - 1.2 * s;

    frame.width = 4.0 * s;
    frame.height = 3.0 * s;

    // Put the face slightly off-center
    double horizontal_offset = (off_center_left ? 0.2 : -0.2) * (double)head_rectangle.width;
    frame.x += horizontal_offset;
}


int RPFramingNode::centermostHeadRectangleIndex(const std::vector<cv::Rect>& head_rectangles)
{
    int centermost_rectangle_index = 0;
    double min_distance_to_center = std::numeric_limits<double>::max();

    cv::Vec2d frame_center(CAMERA_FRAME_WIDTH / 2, CAMERA_FRAME_HEIGHT / 2);
    for (unsigned i = 0; i < head_rectangles.size(); i++)
    {
        cv::Vec2d head_rectangle_center(head_rectangles[i].x + (head_rectangles[i].width / 2),
                                        head_rectangles[i].y + (head_rectangles[i].height / 2));

        double current_distance_to_center = cv::norm(head_rectangle_center - frame_center);

        if (current_distance_to_center < min_distance_to_center)
        {
            min_distance_to_center = current_distance_to_center;
            centermost_rectangle_index = i;
        }
    }

    return centermost_rectangle_index;
}


void RPFramingNode::convertToCameraImageCoordinates(const std::vector<cv::Point3f>& head_points, std::vector<cv::Rect>& projected_head_rectangles)
{
    // Project the points onto the photographic camera plane
    std::vector<cv::Point2f> projected_image_points;
    cv::projectPoints(head_points,
                      rotation_vector,
                      translation_vector,
                      camera_matrix,
                      distortion_coefficients,
                      projected_image_points);

    // Build the projected head rectangles
    for (unsigned i = 0; i < projected_image_points.size(); i += 2)
    {
        cv::Rect projected_rectangle(projected_image_points[i], projected_image_points[i + 1]);
        RPUtils::clampRectangleToFrame(&projected_rectangle, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);

        projected_head_rectangles.push_back(projected_rectangle);
    }
}


void RPFramingNode::convertToKinectCameraCoordinates(const rp_head_tracking::Heads& tracked_heads, std::vector<cv::Point3f>& projected_points)
{
    for (int i = 0; i < tracked_heads.x.size(); i++)
    {
        float depth_m = tracked_heads.depth[i];

        float head_width_m = RPDistanceConverter::pixelsToWidth(tracked_heads.width[i], depth_m);
        float head_height_m = RPDistanceConverter::pixelsToHeight(tracked_heads.height[i], depth_m);

        int distance_from_center_x = tracked_heads.x[i] - (FRAME_WIDTH / 2);
        int distance_from_center_y = tracked_heads.y[i] - (FRAME_HEIGHT / 2);

        float head_left_m = RPDistanceConverter::pixelsToWidth(distance_from_center_x, depth_m);
        float head_top_m = RPDistanceConverter::pixelsToHeight(distance_from_center_y, depth_m);

        cv::Point3f head_tl(head_left_m, head_top_m, depth_m);
        cv::Point3f head_br(head_left_m + head_width_m, head_top_m + head_height_m, depth_m);

        projected_points.push_back(head_tl);
        projected_points.push_back(head_br);
    }
}
