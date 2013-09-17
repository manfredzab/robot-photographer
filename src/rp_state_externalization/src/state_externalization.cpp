/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
// C++ includes
#include <cstdlib>
#include <ctime>

// RPStateExternalization includes
#include <state_externalization.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_state_externalization");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPStateExternalizationNode worker_node(node);
}


RPStateExternalizationNode::RPStateExternalizationNode(ros::NodeHandle& node) :
    node(node),
    locomotion_state(NORMAL),
    head_tracking_state(DETECTING_HEADS),
    framing_state(NO_FRAME),
    autonomous_photography_state(AVOIDING_OBSTACLES),
    published_message_type("NO_MESSAGE"),
    last_publish_time(ros::Time::now()),
    text_message("State externalization via display initialized."),
    vocal_message("State externalization via vocalization initialized.")
{
    // Initialize the random number generator
    srand(time(NULL));

    // Initialize publishers/subscribers
    text_message_publisher  = node.advertise<std_msgs::String>("/rp/state_externalization/text_message", 1);
    vocal_message_publisher = node.advertise<std_msgs::String>("/rp/state_externalization/vocal_message", 1);

    locomotion_state_subscriber             = node.subscribe("/rp/locomotion/state",             1, &RPStateExternalizationNode::locomotionStateCallback,            this);
    head_tracking_state_subscriber          = node.subscribe("/rp/head_tracking/state",          1, &RPStateExternalizationNode::headTrackingStateCallback,          this);
    framing_status_subscriber               = node.subscribe("/rp/framing/frame",                1, &RPStateExternalizationNode::framingMessageCallback,             this);
    autonomous_photography_state_subscriber = node.subscribe("/rp/autonomous_photography/state", 1, &RPStateExternalizationNode::autonomousPhotographyStateCallback, this);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        LocomotionState            current_locomotion_state;
        HeadTrackingState          current_head_tracking_state;
        FramingStatus              current_framing_state;
        AutonomousPhotographyState current_autonomous_photography_state;

        getCurrentState(&current_locomotion_state, &current_head_tracking_state, &current_framing_state, &current_autonomous_photography_state);
        std::string current_message_type = updateStateMessages(current_locomotion_state, current_head_tracking_state, current_framing_state, current_autonomous_photography_state);

        if ((0 != current_message_type.compare(published_message_type)) && ((ros::Time::now() - last_publish_time).sec > 3.0))
        {
            publishTextMessage();
            publishVocalMessage();

            published_message_type = current_message_type;
            last_publish_time = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }
};



void RPStateExternalizationNode::publishTextMessage()
{
    // Publish the message
    std_msgs::String message_to_publish;
    message_to_publish.data = text_message;

    text_message_publisher.publish(message_to_publish);
}


void RPStateExternalizationNode::publishVocalMessage()
{
    // Publish the message
    std_msgs::String message_to_publish;
    message_to_publish.data = text_message;

    vocal_message_publisher.publish(message_to_publish);
}


void RPStateExternalizationNode::getCurrentState(LocomotionState* out_locomotion_state, HeadTrackingState* out_head_tracking_state, FramingStatus* out_framing_state, AutonomousPhotographyState* out_autonomous_photography_state)
{
    // Get individual node states
    locomotion_state_mutex.lock();
    {
        *out_locomotion_state = locomotion_state;
    }
    locomotion_state_mutex.unlock();

    head_tracking_state_mutex.lock();
    {
        *out_head_tracking_state = head_tracking_state;
    }
    head_tracking_state_mutex.unlock();

    autonomous_photography_state_mutex.lock();
    {
        *out_autonomous_photography_state = autonomous_photography_state;
    }
    autonomous_photography_state_mutex.unlock();

    framing_state_mutex.lock();
    {
        *out_framing_state = framing_state;
    }
    framing_state_mutex.unlock();
}


std::string RPStateExternalizationNode::updateStateMessages(LocomotionState in_locomotion_state, HeadTrackingState in_head_tracking_state, FramingStatus in_framing_state, AutonomousPhotographyState in_autonomous_photography_state)
{
    std::string current_message_type;

    if (PROCESSING_BUMPER_EVENT == in_locomotion_state)
    {
        current_message_type = "BUMPER_MESSAGE";

        std::string messages[] = { "Sorry!", "My bad!", "Apologies!", "I am sorry!", "My bad!", "Excuse me!" };
        chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
    }
    else
    {
        if (GATHERING_FACE_HUE_DATA == in_head_tracking_state)
        {
            current_message_type = "FACE_HUE_MESSAGE";

            std::string messages[] = { "I am learning the room's lighting conditions.", "I am looking for the light sources in the room.", "I am checking the lighting sources nearby." };
            chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
        }
        else
        {
            switch (in_autonomous_photography_state)
            {
                case AVOIDING_OBSTACLES:
                {
                    current_message_type = "OBSTACLE_AVOIDANCE_MESSAGE";

                    std::string messages[] = { "I will go and take some pictures from another angle...", "I will take a look from over there...", "I will try from another point..." };
                    chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                    break;
                }
                case TAKING_PICTURE:
                {
                    current_message_type = "TAKING_PICTURE_MESSAGE";

                    std::string messages[] = { "Smile, I am taking a picture!", "Look at the camera, I am taking a picture!", "Look up, I am taking the picture!", "I am taking the picture, say cheese!", "Stand still, I am taking the picture! Smile!" };
                    chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                    break;
                }
                case UPLOADING_PICTURE:
                {
                    current_message_type = "UPLOADING_PICTURE_MESSAGE";

                    //new_display_message = "http://flickr.com/photos/robot-photographer I am uploading picture to Flickr...";
                    std::string messages[] = { "Thank you! I am saving the picture now...", "Thanks, the photo looks very good!", "Thanks, the photograph looks perfect! I am saving it...", "Thank you, that looks really good! Let me save it..." };
                    chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                    break;
                }
                case FRAMING_PICTURE:
                {
                    switch (in_framing_state)
                    {
                        case FRAME_OUT_OF_BOUNDS_HORIZONTALLY:
                        {
                            current_message_type = "HORIZONTAL_OUT_OF_BOUNDS_MESSAGE";

                            std::string messages[] = { "The picture is off to one side... I will try to center it!", "Let me try to center the picture a bit...", "The picture is a bit off to a side. Hold on." };
                            chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                            break;
                        }
                        case FRAME_OUT_OF_BOUNDS_VERTICALLY:
                        {
                            current_message_type = "VERTICAL_OUT_OF_BOUNDS_MESSAGE";

                            std::string messages[] = { "I am trying to get a better photo composition. Hold on a second...", "Let me take a look from a different angle... Hold on.", "Something is a bit off... I will take a look from there." };
                            chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                            break;
                        }
                        case FRAME_TOO_SMALL:
                        {
                            current_message_type = "FRAME_TOO_SMALL_MESSAGE";

                            std::string messages[] = { "You are too far, I am coming closer...", "Hold on, I will come a bit closer.", "You are too far. Hold on a second.", "I need to come a bit closer..." };
                            chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                            break;
                        }
                        case NO_FRAME:
                        {
                            current_message_type = "NO_FRAME_MESSAGE";

                            std::string messages[] = { "I cannot see anything. I will try from over there...", "I'm looking for something to photograph.", "Hmmm... Nothing to photograph here." };
                            chooseTwoMessages(messages, sizeof(messages) / sizeof(std::string), &text_message, &vocal_message);
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }

    return current_message_type;
}

void RPStateExternalizationNode::chooseTwoMessages(std::string* in_strings, int in_strings_size, std::string* out_first_string, std::string* out_second_string)
{
    if (in_strings_size < 1)
    {
        ROS_ERROR("Invalid message array passed to the state externalization: received array with size %d, expected >= 1.", in_strings_size);
        return;
    }

    if (in_strings_size == 1)
    {
        *out_first_string = *out_second_string = in_strings[0];
        return;
    }

    int first_string_index = rand() % in_strings_size;

    int second_string_index = rand() % (in_strings_size - 1);
    if (second_string_index >= first_string_index)
    {
        second_string_index++;
    }

    int string_order = rand() % 2;
    *out_first_string = in_strings[(string_order == 0) ? first_string_index : second_string_index];
    *out_second_string = in_strings[(string_order == 0) ? second_string_index : first_string_index];
}


void RPStateExternalizationNode::locomotionStateCallback(const std_msgs::UInt8& locomotion_state_message)
{
    locomotion_state_mutex.lock();
    {
        locomotion_state = static_cast<LocomotionState>(locomotion_state_message.data);
    }
    locomotion_state_mutex.unlock();
}


void RPStateExternalizationNode::headTrackingStateCallback(const std_msgs::UInt8& head_tracking_state_message)
{
    head_tracking_state_mutex.lock();
    {
        head_tracking_state = static_cast<HeadTrackingState>(head_tracking_state_message.data);
    }
    head_tracking_state_mutex.unlock();
}


void RPStateExternalizationNode::framingMessageCallback(const rp_framing::Frame& frame_message)
{
    // Check if we are not in an obstacle avoidance state.
    FramingStatus current_framing_state = static_cast<FramingStatus>(frame_message.framing_status);
    framing_state_mutex.lock();
    {
        framing_state = current_framing_state;
    }
    framing_state_mutex.unlock();
}


void RPStateExternalizationNode::autonomousPhotographyStateCallback(const std_msgs::UInt8& autonomous_photography_state_message)
{
    autonomous_photography_state_mutex.lock();
    {
        autonomous_photography_state = static_cast<AutonomousPhotographyState>(autonomous_photography_state_message.data);
    }
    autonomous_photography_state_mutex.unlock();
}
