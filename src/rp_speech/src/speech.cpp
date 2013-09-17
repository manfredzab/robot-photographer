/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <speech.hpp>
#include "espeak/speak_lib.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_speech");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPSpeechNode worker_node(node);
}


RPSpeechNode::RPSpeechNode(ros::NodeHandle& node) :
    node(node),
    has_message(false)
{
    // Initialize display message subscriber
    message_subscriber = node.subscribe("/rp/state_externalization/vocal_message", 1, &RPSpeechNode::messageCallback, this);

    // Initialize the text-to-speech engine using Klatt's synthesizer
    char voice[] = { "klatt" };
    espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 500, NULL, 0);
    espeak_SetVoiceByName(voice);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        sayCurrentMessage();

        ros::spinOnce();
        rate.sleep();
    }
};


void RPSpeechNode::messageCallback(const std_msgs::String::ConstPtr& input_message)
{
    message_mutex.lock();
    {
        message = input_message->data;
    }
    message_mutex.unlock();

    has_message = true;
}


void RPSpeechNode::sayCurrentMessage()
{
    if (has_message)
    {
        has_message = false;

        message_mutex.lock();
        {
            espeak_Synth(message.c_str(), message.length() + 1, 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL);
            espeak_Synchronize();
        }
        message_mutex.unlock();
    }
}
