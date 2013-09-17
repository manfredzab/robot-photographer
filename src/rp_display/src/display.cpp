/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <display.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_display");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPDisplayNode worker_node(node);
}


RPDisplayNode::RPDisplayNode(ros::NodeHandle& node) :
    node(node),
    has_display_message(false),
    DISPLAY_DEVICE_PORT(DISPLAY_DEVICE_PORT_DEFAULT)
{
    // Shorten Boost's TCP namespace
    using boost::asio::ip::tcp;

    // Get the overridable parameters
    getOverridableParameters();

    // Initialize display message subscriber
    display_message_subscriber = node.subscribe("/rp/state_externalization/text_message", 1, &RPDisplayNode::displayMessageCallback, this);

    // Open the TCP connection
    ROS_INFO("Waiting for the display device connection to the port %d...", DISPLAY_DEVICE_PORT);

    endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), DISPLAY_DEVICE_PORT));
    acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(io_service, *endpoint));

    acceptor->accept(*tcp_stream.rdbuf());

    ROS_INFO("Display device connected!");

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        publishCurrentDisplayMessage();

        ros::spinOnce();
        rate.sleep();
    }
};


RPDisplayNode::~RPDisplayNode()
{
    if (tcp_stream.good())
    {
        tcp_stream.close();
    }
}


void RPDisplayNode::getOverridableParameters()
{
    node.getParamCached("/rp/display_node/display_device_port", DISPLAY_DEVICE_PORT);
}


void RPDisplayNode::displayMessageCallback(const std_msgs::String::ConstPtr& input_display_message)
{
    display_message_mutex.lock();
    {
        display_message = input_display_message->data;
    }
    display_message_mutex.unlock();

    has_display_message = true;
}


void RPDisplayNode::publishCurrentDisplayMessage()
{
    if (has_display_message)
    {
        has_display_message = false;

        display_message_mutex.lock();
        {
            if (tcp_stream.good())
            {
                tcp_stream << display_message;
                tcp_stream.flush();
            }
            else
            {
                ROS_ERROR("Bad TCP stream to the display device. Tried publishing the message \"%s\".", display_message.c_str());

                tcp_stream.close();
                acceptor->accept(*tcp_stream.rdbuf());
            }
        }
        display_message_mutex.unlock();
    }
}
