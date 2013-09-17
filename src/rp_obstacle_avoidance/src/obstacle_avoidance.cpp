/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <obstacle_avoidance.hpp>

// STL includes
#include <iostream>
#include <cmath>

// ROS includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

// PCL includes
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// Macro to check whether a given raw depth value is invalid (Kinect/GFreenect specific)
#define INVALID_DEPTH_VALUE(ITERATOR) ((*((ITERATOR) + 0) == 0) && \
                                       (*((ITERATOR) + 1) == 0) && \
                                       (*((ITERATOR) + 2) == 192) && \
                                       (*((ITERATOR) + 3) == 127))

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_obstacle_avoidance");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPObstacleAvoidanceNode worker_node(node);
}


RPObstacleAvoidanceNode::RPObstacleAvoidanceNode(ros::NodeHandle& node) :
    node(node),
    sensor_angle(0.0f),
    current_direction(STOP),

    // Initialize overridable parameters
    FOCUS_FIELD_WIDTH(FOCUS_FIELD_WIDTH_DEFAULT),
    FOCUS_FIELD_HEIGHT(FOCUS_FIELD_HEIGHT_DEFAULT),
    FOCUS_FIELD_DEPTH(FOCUS_FIELD_DEPTH_DEFAULT),
    CLOUD_FILTER_SIZE(CLOUD_FILTER_SIZE_DEFAULT),
    MAX_INVALID_DEPTH_DATA(MAX_INVALID_DEPTH_DATA_DEFAULT),
    SMOOTHING_FRAME_LIMIT(SMOOTHING_FRAME_LIMIT_DEFAULT),
    VERBOSE_OUTPUT_ENABLED(VERBOSE_OUTPUT_ENABLED_DEFAULT)
{
    // Get parameters from the parameter server
    getOverridableParameters();

    // Initialize ROS publisher for the driving direction
    const int DRIVING_DIRECTION_BUFFER_SIZE = 1;
    driving_direction_publisher = node.advertise<std_msgs::UInt8>(DRIVING_DIRECTION_TOPIC, DRIVING_DIRECTION_BUFFER_SIZE);

    // Initialize ROS subscriber for the Kinect sensor angle
    const int SENSOR_TILT_ANGLE_BUFFER_SIZE = 1;
    node.subscribe(SENSOR_TILT_ANGLE_TOPIC, SENSOR_TILT_ANGLE_BUFFER_SIZE, &RPObstacleAvoidanceNode::sensorAngleCallback, this);

    // Initialize synchronized ROS subscribers for the Kinect's point cloud input and depth images
    const int SYNCHRONIZED_SUBSCRIBERS_BUFFER_SIZE = 5;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > point_cloud_subscriber(node, SENSOR_POINT_CLOUD_TOPIC, SYNCHRONIZED_SUBSCRIBERS_BUFFER_SIZE);

    message_filters::Subscriber<sensor_msgs::Image> depth_data_subscriber(node, SENSOR_DEPTH_IMAGE_TOPIC, SYNCHRONIZED_SUBSCRIBERS_BUFFER_SIZE);

    // Create a synchronized subscriber for Kinect's point cloud and depth image inputs
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> SynchronizationPolicy;

    const int SYNCHRONIZATION_WINDOW_SIZE = 10;
    message_filters::Synchronizer<SynchronizationPolicy> synchronized_subscriber(SynchronizationPolicy(SYNCHRONIZATION_WINDOW_SIZE), point_cloud_subscriber, depth_data_subscriber);

    // Register the callback for the synchronized subscriber
    synchronized_subscriber.registerCallback(boost::bind(&RPObstacleAvoidanceNode::kinectInputCallback, this, _1, _2));

    // Spin the node at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
};


void RPObstacleAvoidanceNode::kinectInputCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud, const sensor_msgs::Image::ConstPtr& depth_image)
{
    // Check the proportion of valid depth readings; if the depth image is nearly empty, we must be
    // standing in front of a large object.
    DrivingDirection proposed_direction;
    if (getInvalidDepthDataProportion(depth_image) > MAX_INVALID_DEPTH_DATA)
    {
        proposed_direction = (current_direction != FORWARD) ? current_direction : RIGHT;
    }
    else
    {
        // Create an empty point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Reduce the point cloud density
        reducePointCloudDensity(point_cloud, cloud_filtered, CLOUD_FILTER_SIZE);

        // Level it to be parallel to the ground
        levelPointCloud(cloud_filtered, cloud_filtered);

        // Crop it to the area in front of the robot
        cropPointCloud(cloud_filtered,
                       cloud_filtered,
                      -FOCUS_FIELD_WIDTH / 2,
                       FOCUS_FIELD_WIDTH / 2,
                      -FOCUS_FIELD_HEIGHT,
                       0.0,
                       FOCUS_FIELD_DEPTH,
                       0.0);

        // Get the driving direction from the contents of the point cloud ahead of the robot
        proposed_direction = drivingDirectionFromFrontalPointCloud(cloud_filtered);
    }

    // Take care to ensure that the current turn direction is maintained (to avoid oscillation)
    if (current_direction == FORWARD || proposed_direction == FORWARD)
    {
        if (proposed_direction != current_direction)
        {
            publishDrivingDirection(proposed_direction);
            current_direction = proposed_direction;
        }
    }

    // Produce output if necessary
    if (VERBOSE_OUTPUT_ENABLED)
    {
        ROS_INFO("Current direction: %s", ((current_direction == FORWARD) ? "FORWARD" :
                                           (current_direction == LEFT) ? "LEFT" : "RIGHT"));
    }
};


void RPObstacleAvoidanceNode::levelPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& levelled_cloud)
{
    // Get the last known sensor's tilt angle
    float angle_rad;
    sensor_angle_mutex.lock();
    {
        angle_rad = M_PI * sensor_angle / 180.0f;
    }
    sensor_angle_mutex.unlock();

    // Create the appropriate rotation matrix
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix <<
        1.0f,     0.0f,           0.0f,            0.0f,
        0.0f,     cos(angle_rad), -sin(angle_rad), -SENSOR_DISTANCE_FROM_GROUND,
        0.0f,     sin(angle_rad), cos(angle_rad),  0.0f,
        0.0f,     0.0f,           0.0f,            1.0f;

    // Rotate back the point cloud according to input from Kinect's accelerometer
    pcl::transformPointCloud(*in_cloud, *levelled_cloud, rotation_matrix);
}


DrivingDirection RPObstacleAvoidanceNode::drivingDirectionFromFrontalPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& frontal_point_cloud)
{
    // Do not use smoothing while turning
    if (current_direction != FORWARD)
    {
        point_counts_ahead.clear();
    }

    // Add the sample of the number of points ahead, capping the sample count
    point_counts_ahead.push_front(frontal_point_cloud->size());
    while (point_counts_ahead.size() > SMOOTHING_FRAME_LIMIT)
    {
        point_counts_ahead.pop_back();
    }

    // Get the smoothed number of points ahead
    int smoothed_point_count_ahead = 0;
    for (unsigned i = 0; i < point_counts_ahead.size(); i++)
    {
        smoothed_point_count_ahead += point_counts_ahead[i];
    }
    smoothed_point_count_ahead /= point_counts_ahead.size();

    if (smoothed_point_count_ahead == 0)
    {
        // Coast is clear
        return FORWARD;
    }
    else
    {
        // Find out on which side of the robot (left/right) lies the centroid of an obstacle and turn
        // in the opposite direction,
        float centroid_x = 0.0f;
        for (unsigned i = 0; i < frontal_point_cloud->size(); i++)
        {
            centroid_x += frontal_point_cloud->points[i].x;
        }
        centroid_x /= frontal_point_cloud->size();

        return (centroid_x < 0.0) ? RIGHT : LEFT;
    }
}


void RPObstacleAvoidanceNode::reducePointCloudDensity(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_to_filter, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, double voxel_size)
{
    // Create the appropriate voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_size_filter;
    voxel_size_filter.setInputCloud(cloud_to_filter);
    voxel_size_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    // Subsample the input point cloud
    voxel_size_filter.filter(*filtered_cloud);
};


void RPObstacleAvoidanceNode::cropPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cropped_cloud,
                                             double x_limit_left,
                                             double x_limit_right,
                                             double y_limit_above,
                                             double y_limit_below,
                                             double z_limit_ahead,
                                             double z_limit_behind)
{
    // Create the appropriate pass-through filter
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
    pass_through_filter.setInputCloud(in_cloud);

    // Crop horizontally
    pass_through_filter.setFilterFieldName("x");
    pass_through_filter.setFilterLimits(x_limit_left, x_limit_right);
    pass_through_filter.filter(*cropped_cloud);

    // Crop vertically
    pass_through_filter.setFilterFieldName("y");
    pass_through_filter.setFilterLimits(y_limit_above, y_limit_below);
    pass_through_filter.filter(*cropped_cloud);

    // Crop depth-wise
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(z_limit_behind, z_limit_ahead);
    pass_through_filter.filter(*cropped_cloud);
};


double RPObstacleAvoidanceNode::getInvalidDepthDataProportion(
    const sensor_msgs::Image::ConstPtr& depth_image)
{
    int invalid_depth_pixels = 0;

    // Count the invalid depth pixels directly in raw depth data (in 32FC1 encoding)
    for (std::vector<unsigned char>::const_iterator iterator = depth_image->data.begin(),
         iterator_end = depth_image->data.end();
         iterator != iterator_end;
         std::advance(iterator, 4))
    {
        invalid_depth_pixels += INVALID_DEPTH_VALUE(iterator);
    }

    return 4.0 * (double)invalid_depth_pixels / (double)depth_image->data.size();
}


void RPObstacleAvoidanceNode::publishDrivingDirection(const DrivingDirection driving_direction)
{
    // Convert and publish the direction
    std_msgs::UInt8 converted_direction;
    converted_direction.data = drivingDirectionToUint8(driving_direction);

    driving_direction_publisher.publish(converted_direction);
}


void RPObstacleAvoidanceNode::sensorAngleCallback(const std_msgs::Float64& angle)
{
    // Save sensor's angle
    sensor_angle_mutex.lock();
    {
        sensor_angle = angle.data;
    }
    sensor_angle_mutex.unlock();
}


void RPObstacleAvoidanceNode::getOverridableParameters()
{
    // Get overridable parameters from the parameter server
    node.getParamCached("/rp/obstacle_avoidance_node/focus_field_width",      FOCUS_FIELD_WIDTH);
    node.getParamCached("/rp/obstacle_avoidance_node/focus_field_height",     FOCUS_FIELD_HEIGHT);
    node.getParamCached("/rp/obstacle_avoidance_node/focus_field_depth",      FOCUS_FIELD_DEPTH);
    node.getParamCached("/rp/obstacle_avoidance_node/cloud_filter_size",      CLOUD_FILTER_SIZE);
    node.getParamCached("/rp/obstacle_avoidance_node/smoothing_frame_limit",  SMOOTHING_FRAME_LIMIT);
    node.getParamCached("/rp/obstacle_avoidance_node/max_invalid_depth_data", MAX_INVALID_DEPTH_DATA);
    node.getParamCached("/rp/obstacle_avoidance_node/verbose_output_enabled", VERBOSE_OUTPUT_ENABLED);
}
