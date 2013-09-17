/**
 * @class      RPNavigationNode
 *
 * @brief      Robot photographer's navigation node, which multiplexes between the competing
 *             driving directions proposed by obstacle avoidance and photographic composition
 *             (framing) nodes.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

// ROS includes
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

// Boost includes
#include <boost/thread/mutex.hpp>

// RPLocomotion includes
#include <driving_direction.h>

// RPNavigation includes
#include <direction_source.h>

class RPNavigationNode
{
    private:
        ros::NodeHandle& node;                         /**< Node's handle.                          */

        ros::Publisher driving_direction_publisher;    /**< Driving direction publisher.            */

        ros::Subscriber obstacle_avoidance_subscriber; /**< Driving direction subscriber from
                                                            obstacle avoidance node.                */

        ros::Subscriber framing_subscriber;            /**< Driving direction subscriber from
                                                            framing node.                           */
        ros::Subscriber auton_photography_subscriber;  /**< Autonomous photography node subscriber
                                                            which determines the source of driving
                                                            direction.                              */

        DrivingDirection obstacle_avoidance_driving_direction;   /**< Driving direction proposed by
                                                                      obstacle avoidance node.      */
        boost::mutex obstacle_avoidance_driving_direction_mutex; /**< Obstacle avoidance driving
                                                                      direction's mutex.            */

        DrivingDirection framing_driving_direction;              /**< Driving direction proposed by
                                                                      framing node.                 */
        boost::mutex framing_driving_direction_mutex;            /**< Framing driving direction's
                                                                      mutex.                        */

        DirectionSource driving_direction_source;                /**< Driving direction source
                                                                      proposed by autonomous
                                                                      photography node.             */
        boost::mutex driving_direction_source_mutex;             /**< Driving direction source
                                                                      mutex.                        */

        /**
         * Callback for the driving direction message from the obstacle avoidance node.
         * @param direction_code Received obstacle avoidance direction.
         */
        void obstacleAvoidanceDirectionMessageCallback(const std_msgs::UInt8& direction_code);

        /**
         * Callback for the driving direction message from the framing node.
         * @param direction_code Received framing direction.
         */
        void framingDirectionMessageCallback(const std_msgs::UInt8& direction_code);

        /**
         * Callback for the driving direction source message from the autonomous photography node.
         * @param direction_source_code Received driving direction source.
         */
        void autonomousPhotographyDirectionSourceMessageCallback(const std_msgs::UInt8& direction_source_code);

        /**
         * Obtains the driving direction based on the driving direction/direction source inputs
         * from obstacle avoidance, framing and autonomous photography nodes.
         * @param driving_direction  Produced driving direction.
         * @param direction_source   Produced driving direction source.
         */
        void getDrivingDirectionAndSource(DrivingDirection* driving_direction, DirectionSource* direction_source);

        /**
         * Clears the stored driving directions.
         */
        void clearPublishedDrivingDirections();

    public:
        /**
         * Default navigation node's constructor.
         * @param node Handle to ROS node.
         */
        RPNavigationNode(ros::NodeHandle& node);
};

#endif /* NAVIGATION_HPP_ */
