/**
 * @class      RPLocomotionNode
 *
 * @brief      Robot photographer's locomotion node, which converts driving direction messages
 *             and bumper press events into linear/angular velocity messages.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef LOCOMOTION_HPP_
#define LOCOMOTION_HPP_

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

// Boost includes
#include <boost/thread/mutex.hpp>

// Kobuki includes
#include <kobuki_msgs/BumperEvent.h>

// RPNavigation includes
#include <direction_source.h>

// Constants
#define FRAMING_LINEAR_VELOCITY_DEFAULT  0.1
#define FRAMING_ANGULAR_VELOCITY_DEFAULT 0.33
#define OBSTACLE_AVOIDANCE_LINEAR_VELOCITY_DEFAULT  0.15
#define OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY_DEFAULT 0.5

class RPLocomotionNode
{
    private:
        ros::NodeHandle& node;                      /**< Node's handle.                         */

        geometry_msgs::TwistPtr locomotion_message; /**< Pointer to locomotion message.         */
        boost::mutex locomotion_message_mutex;      /**< Locomotion message's mutex.            */

        ros::Publisher locomotion_publisher;        /**< Locomotion publisher.                  */
        ros::Publisher state_publisher;             /**< Locomotion state publisher.            */
        ros::Publisher motor_power_publisher;       /**< Motor power publisher.                 */

        ros::Subscriber navigation_subscriber;      /**< Navigation input subscriber.           */
        ros::Subscriber bumper_subscriber;          /**< Bumper events subscriber.              */
        ros::Subscriber cnc_subscriber;             /**< C&C input subscriber which determines
                                                         the source of driving direction.       */

        ros::WallTimer bumper_timer;                /**< Timer for a turn-around motion when
                                                         the bumper event is triggered.         */

        volatile bool processing_bumper_event;      /**< Bumper event flag.                     */

        double FRAMING_LINEAR_VELOCITY;             /**< Framing linear velocity OP.            */
        double FRAMING_ANGULAR_VELOCITY;            /**< Framing angular velocity OP.           */

        double OBSTACLE_AVOIDANCE_LINEAR_VELOCITY;  /**< Obstacle avoidance linear velocity OP. */
        double OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY; /**< Obstacle avoidance angular velocity OP.*/

        /**
         * Resets the locomotion message to stop the robot.
         * @param frame_message Input framing status message.
         */
        void clearLocomotionMessage();

        /**
         * Sets the motor enabled state.
         * @param enabled Desired motors state (enabled/disabled).
         */
        void motorsEnabled(bool enabled);

        /**
         * Sets/clears the "processing bumper event" flag.
         * @param processing Desired "processing bumper event" flag.
         */
        void setProcessingBumperEvent(bool processing);

        /**
         * Callback for the received navigation message.
         * @param direction_and_source_code Driving direction and direction source message.
         */
        void navigationMessageCallback(const std_msgs::UInt16& direction_and_source_code);

        /**
         * Callback for the received bumper event.
         * @param timer_event Pointer to the bumper event object.
         */
        void bumperEventCallback(const kobuki_msgs::BumperEventConstPtr bumper_event);

        /**
         * Callback for the bumper timer event.
         * @param timer_event Timer callback object.
         */
        void bumperTimerCallback(const ros::WallTimerEvent& timer_event);

        /**
         * Gets the overridable parameters from the parameter server.
         */
        void getOverridableParameters();

    public:
        /**
         * Default locomotion node's constructor.
         * @param node Handle to ROS node.
         */
        RPLocomotionNode(ros::NodeHandle& node);
};

#endif /* LOCOMOTION_HPP_ */
