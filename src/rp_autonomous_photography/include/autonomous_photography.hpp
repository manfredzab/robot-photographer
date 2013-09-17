/**
 * @class      RPAutonomousPhotographyNode
 *
 * @brief      Robot photographer's picture-taking process coordinator node, which issues commands
 *             for taking and uploading the pictures.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef AUTONOMOUS_PHOTOGRAPHY_HPP_
#define AUTONOMOUS_PHOTOGRAPHY_HPP_

// STL includes
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// Boost includes
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

// RPFraming includes
#include <framing_status.h>

// RPLocomotion includes
#include <driving_direction.h>
#include <locomotion_state.h>

// RPNavigation includes
#include <direction_source.h>

// RPFraming message includes
#include <rp_framing/Frame.h>

// RPAutonomousPhotography includes
#include <autonomous_photography_state.h>

// Constants
#define FRAMED_PICTURE_PREFIX "P_"
#define ANNOTATED_PICTURE_PREFIX "A_"

#define CAMERA_DELAY_S 6.0

// Defaults for the overridable parameters
#define TIME_BETWEEN_PICTURES_DEFAULT 45.0

class RPAutonomousPhotographyNode
{
    private:
        ros::NodeHandle& node;                      /**< Node's handle.                         */

        AutonomousPhotographyState state;           /**< Node's state.                          */
        boost::mutex state_mutex;                   /**< Node's state mutex.                    */

        ros::Publisher state_publisher;             /**< Node's state publisher.                */
        ros::Publisher direction_source_publisher;  /**< Direction source publisher.            */

        ros::WallTimer framing_timer;               /**< Timer for framing.                     */
        ros::WallTimer camera_timer;                /**< Timer for camera (between the photo
                                                         command issue and the picture taking). */

        ros::WallTime framing_timer_start_time;     /**< Last time when framing timer was
                                                         enabled.                               */

        ros::ServiceClient camera_client;           /**< Camera client.                         */
        ros::ServiceClient uploader_client;         /**< Uploader client.                       */

        ros::Subscriber framing_status_subscriber;  /**< Framing status subscriber.             */
        FramingStatus framing_state;                /**< Framing state.                         */
        boost::mutex framing_state_mutex;           /**< Framing state mutex.                   */

        cv::Rect frame;                             /**< Received picture frame.                */
        std::vector<cv::Rect> heads;                /**< Received detected head positions.      */
        boost::mutex frame_mutex;                   /**< Picture frame mutex.                   */

        volatile bool is_taking_picture;            /**< Flag indicating whether a picture is
                                                         being taken.                           */
        volatile bool waiting_for_camera;           /**< Flag indicating whether the node is
                                                         waiting for the camera.                */
        boost::thread picture_thread;               /**< Asynchronous picture taking thread.    */

        double TIME_BETWEEN_PICTURES;               /**< Overridable parameter for the time
                                                         between pictures.                      */

        /**
         * Callback for framing timer.
         * @param timer_event Timer callback object.
         */
        void framingTimerCallback(const ros::WallTimerEvent& timer_event);

        /**
         * Callback for camera delay timer.
         * @param timer_event Timer callback object.
         */
        void cameraTimerCallback(const ros::WallTimerEvent& timer_event);

        /**
         * Callback for framing status message.
         * @param frame_message Framing status message (containing the ideal framing rectangle).
         */
        void framingMessageCallback(const rp_framing::Frame& frame_message);

        /**
         * Parses the frame message into frame/head rectangle vector.
         * @param frame_message Input framing status message.
         */
        void parseFramingMessage(const rp_framing::Frame& frame_message);

        /**
         * Enables/disables head tracking node.
         * @param enabled Desired head tracking node status.
         */
        void setHeadTrackingEnabled(const bool enabled);

        /**
         * Enables/disables framing node.
         * @param enabled Desired framing node status.
         */
        void setFramingEnabled(const bool enabled);

        /**
         * Enables/disables locomotion node.
         * @param enabled Desired locomotion node status.
         */
        void setLocomotionEnabled(const bool enabled);

        /**
         * Publishes a given direction source.
         * @param direction_source Direction source to publish.
         */
        void publishDirectionSource(DirectionSource direction_source);

        /**
         * Takes a picture using RPCamera service and uploads it to Flickr using RPUploader
         * service.
         */
        void takeAndUploadPicture();

        /**
         * Sets the node's state.
         * @param input_state Input node's state.
         */
        void setState(AutonomousPhotographyState input_state);

        /**
         * Gets the overridable parameters from the parameter server.
         */
        void getOverridableParameters();

    public:
        /**
         * Default autonomous photography node's constructor.
         * @param node Handle to ROS node.
         */
        RPAutonomousPhotographyNode(ros::NodeHandle& node);
};

#endif /* AUTONOMOUS_PHOTOGRAPHY_HPP_ */

