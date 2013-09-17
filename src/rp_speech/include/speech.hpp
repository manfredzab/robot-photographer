/**
 * @class      RPSpeechNode
 *
 * @brief      Robot photographer's text-to-speech synthesis node, which vocalizes the input
 *             status messages using the Espeak library.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef SPEECH_HPP_
#define SPEECH_HPP_

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// Boost includes
#include <boost/thread/mutex.hpp>

class RPSpeechNode
{
    private:
        ros::NodeHandle& node;                      /**< Node handle.                            */

        ros::Subscriber message_subscriber;         /**< Message subscriber.                     */

        std::string message;                        /**< Message to be said.                     */
        boost::mutex message_mutex;                 /**< Message's mutex.                        */

        volatile bool has_message;                  /**< Flag whether the message is received
                                                         and not yet said.                       */

        /**
         * Callback for the message to vocalize.
         * @param message Input message.
         */
        void messageCallback(const std_msgs::String::ConstPtr& message);

        /**
         * Vocalizes the current message.
         */
        void sayCurrentMessage();

    public:
        /**
         * Default speech node's constructor.
         * @param node Handle to ROS node.
         */
        RPSpeechNode(ros::NodeHandle& node);
};

#endif /* SPEECH_HPP_ */
