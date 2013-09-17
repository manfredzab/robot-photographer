/**
 * @class      RPObstacleAvoidanceNode
 *
 * @brief      Robot photographer's obstacle avoidance ROS node, which uses point cloud and depth
 *             image inputs to detect obstacles in front of the robot, and generates the driving
 *             directions accordingly.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

// STL includes
#include <deque>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

// PCL includes
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Boost includes
#include <boost/thread/mutex.hpp>

// RPLocomotion includes
#include <driving_direction.h>

// Constants
#define DRIVING_DIRECTION_TOPIC  "/rp/obstacle_avoidance/driving_direction"
#define SENSOR_TILT_ANGLE_TOPIC  "/cur_tilt_angle"
#define SENSOR_POINT_CLOUD_TOPIC "/camera/depth_registered/points"
#define SENSOR_DEPTH_IMAGE_TOPIC "/camera/depth_registered/image"

#define SENSOR_DISTANCE_FROM_GROUND   0.61f

// Overridable parameter defaults
#define FOCUS_FIELD_WIDTH_DEFAULT      0.4
#define FOCUS_FIELD_HEIGHT_DEFAULT     1.4
#define FOCUS_FIELD_DEPTH_DEFAULT      0.5
#define CLOUD_FILTER_SIZE_DEFAULT      0.03
#define MAX_INVALID_DEPTH_DATA_DEFAULT 0.4
#define SMOOTHING_FRAME_LIMIT_DEFAULT  2
#define VERBOSE_OUTPUT_ENABLED_DEFAULT false


class RPObstacleAvoidanceNode
{
    private:
        ros::NodeHandle& node;                      /**< Node's handle.                          */

        DrivingDirection current_direction;         /**< Current driving direction of the robot. */

        std::deque<unsigned> point_counts_ahead;    /**< History of points ahead of the robot.   */

        float sensor_angle;                         /**< Sensor's angle.                         */
        boost::mutex sensor_angle_mutex;            /**< Sensor's angle lock.                    */

        ros::Publisher driving_direction_publisher; /**< Driving direction publisher.            */

        double FOCUS_FIELD_WIDTH;                   /**< ROI width overridable parameter (OP).   */
        double FOCUS_FIELD_HEIGHT;                  /**< ROI height OP.                          */
        double FOCUS_FIELD_DEPTH;                   /**< ROI depth OP.                           */
        double CLOUD_FILTER_SIZE;                   /**< Voxel filter grid size OP.              */
        double MAX_INVALID_DEPTH_DATA;              /**< Maximum allowed percentage OP.          */
        int    SMOOTHING_FRAME_LIMIT;               /**< Smoothing frame limit OP.               */
        bool   VERBOSE_OUTPUT_ENABLED;              /**< Verbose output OP.                      */

        /**
         * Reduces the number of points in the cloud using voxel grid filter.
         * @param cloud_to_filter Point cloud to be filtered.
         * @param filtered_cloud  Resulting filtered point cloud.
         * @param voxel_size      Voxel grid filter size.
         */
        void reducePointCloudDensity(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_to_filter, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, double voxel_size);

        /**
         * Levels the point cloud w.r.t. the ground (using Kinect's accelerometer).
         * @param cloud_to_level Point cloud to be levelled.
         * @param leveled_cloud  Resulting levelled point cloud.
         */
        void levelPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_to_level, pcl::PointCloud<pcl::PointXYZ>::Ptr& levelled_cloud);

        /**
         * Reduces the point cloud to an area of interest (ROI) in which robot looks for obstacles.
         * @param cloud_to_crop  Point cloud to be cropped.
         * @param cropped_cloud  Resulting cropped point cloud.
         * @param x_limit_left   X axis left crop limit.
         * @param x_limit_right  X axis right crop limit.
         * @param y_limit_above  Y axis top crop limit.
         * @param y_limit_below  Y axis bottom crop limit.
         * @param z_limit_ahead  Z axis front crop limit.
         * @param z_limit_behind Z axis back crop limit.
         */
        void cropPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_to_crop,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& cropped_cloud,
                            double x_limit_left,
                            double x_limit_right,
                            double y_limit_above,
                            double y_limit_below,
                            double z_limit_ahead,
                            double z_limit_behind);

        /**
         * Gets the proportion of invalid data (missing depth readings) in a given depth image.
         * @param depth_image Input depth image.
         * @returns Proportion of invalid data in the given depth image (between 0.0 and 1.0).
         */
        double getInvalidDepthDataProportion(const sensor_msgs::Image::ConstPtr& depth_image);

        /**
         * Computes the driving direction from the levelled point cloud in front of the robot.
         * @param frontal_point_cloud Levelled point cloud in front of the robot.
         * @returns Computed driving direction.
         */
        DrivingDirection drivingDirectionFromFrontalPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& frontal_point_cloud);

        /**
         * Publishes the given driving direction.
         * @param driving_direction Driving direction to publish.
         */
        void publishDrivingDirection(const DrivingDirection driving_direction);

        /**
         * Gets the overridable parameters from the parameter server.
         */
        void getOverridableParameters();

        /**
         * Callback for the point cloud and depth image inputs from the Kinect.
         * @param point_cloud Input point cloud from Kinect.
         * @param depth_image Input depth image from Kinect.
         */
        void kinectInputCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud, const sensor_msgs::Image::ConstPtr& depth_image);

        /**
         * Callback for the accelerometer's inputs from the Kinect.
         * @param angle Kinect's tilt angle.
         */
        void sensorAngleCallback(const std_msgs::Float64& angle);

    public:
        /**
         * Default obstacle avoidance node's constructor.
         * @param node Handle to ROS node.
         */
        RPObstacleAvoidanceNode(ros::NodeHandle& node);
};

#endif /* OBSTACLE_AVOIDANCE_HPP_ */
