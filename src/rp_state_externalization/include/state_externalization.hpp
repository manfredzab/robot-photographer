/**
 * @class      RPStateExternalizationNode
 *
 * @brief      Robot photographer's node responsible for generating vocal/visual status messages
 *             about the internal state of the robot.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef STATE_EXTERNALIZATION_HPP_
#define STATE_EXTERNALIZATION_HPP_

// STL includes
#include <string>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// Boost includes
#include <boost/thread/mutex.hpp>

// RPLocomotion includes
#include <locomotion_state.h>

// RPHeadTracking includes
#include <head_tracking_state.h>

// RPFraming includes
#include <framing_status.h>

// RPFraming message includes
#include <rp_framing/Frame.h>

// RPAutonomousPhotography includes
#include <autonomous_photography_state.h>


class RPStateExternalizationNode
{
    private:
        ros::NodeHandle& node;                                   /**< Node's handle.                 */

        ros::Subscriber locomotion_state_subscriber;             /**< Locomotion state subscriber.   */
        ros::Subscriber head_tracking_state_subscriber;          /**< Head tracker state subscriber. */
        ros::Subscriber framing_status_subscriber;               /**< Framing status subscriber.     */
        ros::Subscriber autonomous_photography_state_subscriber; /**< Autonomous photography state
                                                                      subscriber.                    */
        ros::Publisher text_message_publisher;                   /**< Text message publisher.        */
        ros::Publisher vocal_message_publisher;                  /**< Vocal message publisher.       */

        LocomotionState locomotion_state;                        /**< Locomotion state.              */
        boost::mutex locomotion_state_mutex;                     /**< Locomotion state's mutex.      */

        HeadTrackingState head_tracking_state;                   /**< Head-tracking state.           */
        boost::mutex head_tracking_state_mutex;                  /**< Head-tracking state's mutex.   */

        FramingStatus framing_state;                             /**< Framing status.                */
        boost::mutex framing_state_mutex;                        /**< Framing status' mutex.         */

        AutonomousPhotographyState autonomous_photography_state; /**< Autonomous photography status. */
        boost::mutex autonomous_photography_state_mutex;         /**< Autonomous photography status'
                                                                      mutex.                         */

        std::string text_message;                                /**< Text message to publish.       */
        std::string vocal_message;                               /**< Vocal message to publish.      */

        std::string published_message_type;                      /**< Last published message type.   */
        ros::Time last_publish_time;                             /**< Last published message time.   */

        /**
         * Callback for locomotion state message.
         * @param locomotion_state_message Locomotion state message.
         */
        void locomotionStateCallback(const std_msgs::UInt8& locomotion_state_message);

        /**
         * Callback for head tracker's state message.
         * @param head_tracking_state_message  Head tracker's state message.
         */
        void headTrackingStateCallback(const std_msgs::UInt8& head_tracking_state_message);

        /**
         * Callback for framing state message.
         * @param frame_message Framing state message.
         */
        void framingMessageCallback(const rp_framing::Frame& frame_message);

        /**
         * Callback for autonomous photography state message.
         * @param autonomous_photography_state_message Autonomous photography state message.
         */
        void autonomousPhotographyStateCallback(const std_msgs::UInt8& autonomous_photography_state_message);

        /**
         * Chooses two strings uniformly at random from a given string array.
         * @param in_strings        Input string array.
         * @param in_strings_size   Input string array size.
         * @param out_first_string  First chosen string.
         * @param out_second_string Second chosen string.
         */
        void chooseTwoMessages(std::string* in_strings, int in_strings_size, std::string* out_first_string, std::string* out_second_string);

        /**
         * Publishes the stored text status message.
         */
        void publishTextMessage();

        /**
         * Publishes the stored vocal status message.
         */
        void publishVocalMessage();

        /**
         * Returns a snapshot of the cumulative robot's state.
         * @param out_locomotion_state              Output locomotion state.
         * @param out_head_tracking_state           Output head tracking state.
         * @param out_framing_state                 Output framing state.
         * @param out_autonomous_photography_state  Output autonomous photography state.
         */
        void getCurrentState(LocomotionState* out_locomotion_state, HeadTrackingState* out_head_tracking_state, FramingStatus* out_framing_state, AutonomousPhotographyState* out_autonomous_photography_state);

        /**
         * Updates the status messages (text and vocal) based on the cumulative robot's state
         * and returns the message type string.
         * @param in_locomotion_state              Input locomotion state.
         * @param in_head_tracking_state           Input head tracking state.
         * @param in_framing_state                 Input framing state.
         * @param in_autonomous_photography_state  Input autonomous photography state.
         * @returns Current message type string.
         */
        std::string updateStateMessages(LocomotionState in_locomotion_state, HeadTrackingState in_head_tracking_state, FramingStatus in_framing_state, AutonomousPhotographyState in_autonomous_photography_state);

    public:

        /**
         * Default state externalization node's constructor.
         * @param node Handle to ROS node.
         */
        RPStateExternalizationNode(ros::NodeHandle& node);
};

#endif /* STATE_EXTERNALIZATION_HPP_ */
