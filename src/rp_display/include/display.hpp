/**
 * @class      RPDisplayNode
 *
 * @brief      Robot photographer's display node, which sends the status messages/hyperlinks
 *             via TCP to a corresponding client application that shows these messages.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DISPLAY_HPP_
#define DISPLAY_HPP_

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// Boost includes
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

#define DISPLAY_DEVICE_PORT_DEFAULT 8711                /**< TCP server's default port.           */

class RPDisplayNode
{
    private:
        ros::NodeHandle& node;                          /**< Node's handle.                       */

        ros::Subscriber display_message_subscriber;     /**< Display message subscriber           */

        std::string display_message;                    /**< Display message to show.             */
        boost::mutex display_message_mutex;             /**< Display message's mutex.             */

        volatile bool has_display_message;              /**< Flag whether the display message is
                                                             received and not yet displayed.      */

        boost::asio::io_service io_service;                         /**< TCP service.             */
        boost::shared_ptr<boost::asio::ip::tcp::endpoint> endpoint; /**< TCP connection endpoint. */
        boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor; /**< TCP connection acceptor. */
        boost::asio::ip::tcp::iostream tcp_stream;                  /**< TCP stream to the display
                                                                         device.                  */

        int DISPLAY_DEVICE_PORT;                        /**< TCP server's port overridable
                                                             parameter.                           */

        /**
         * Callback for display message.
         * @param display_message Callback object, containing a pointer to display message string.
         */
        void displayMessageCallback(const std_msgs::String::ConstPtr& display_message);

        /**
         * Publishes the current display message over TCP.
         */
        void publishCurrentDisplayMessage();

        /**
         * Gets overridable parameters from the parameter server.
         */
        void getOverridableParameters();

    public:
        /**
         * Default display node's constructor.
         * @param node Handle to ROS node.
         */
        RPDisplayNode(ros::NodeHandle& node);

        /**
         * Default display node's destructor, which shuts down the TCP connection.
         */
        virtual ~RPDisplayNode();
};

#endif /* DISPLAY_HPP_ */
