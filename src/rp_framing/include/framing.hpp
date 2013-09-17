/**
 * @class      RPFramingNode
 *
 * @brief      Robot photographer's photographic composition node, which uses the human head
 *             locations provided by the head tracking node to calculate the most aesthetically
 *             pleasing framing for the picture.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef FRAMING_HPP_
#define FRAMING_HPP_

// STL includes
#include <string>
#include <vector>
#include <set>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// RPFraming includes
#include <framing_status.h>

// RPHeadTracking includes
#include <depth_image_processor.hpp>
#include <rp_head_tracking/Heads.h>

// RPLocomotion includes
#include <driving_direction.h>

// Constants
#define CAMERA_FRAME_WIDTH 4320
#define CAMERA_FRAME_HEIGHT 3240

// 6 MP
//#define MINIMAL_PICTURE_FRAME_WIDTH 2880
//#define MINIMAL_PICTURE_FRAME_HEIGHT 2160

// 4 MP
//#define MINIMAL_PICTURE_FRAME_WIDTH 2312
//#define MINIMAL_PICTURE_FRAME_HEIGHT 1734

// 3.5 MP
#define MINIMAL_PICTURE_FRAME_WIDTH 2160
#define MINIMAL_PICTURE_FRAME_HEIGHT 1620

// 1.5 MP
//#define MINIMAL_PICTURE_FRAME_WIDTH 1536
//#define MINIMAL_PICTURE_FRAME_HEIGHT 1152

#define MAX_TIME_FOR_FRAMING_S                     60.0
#define MAX_DEVIATION_FROM_CORRECT_FRAMING_PERCENT 50.0

// Defaults for overridable parameters
#define CAMERA_TRANSLATION_X_DEFAULT 0.0
#define CAMERA_TRANSLATION_Y_DEFAULT 0.65
#define CAMERA_TRANSLATION_Z_DEFAULT -0.01

#define CAMERA_ROTATION_X_DEFAULT 10.0
#define CAMERA_ROTATION_Y_DEFAULT -0.3
#define CAMERA_ROTATION_Z_DEFAULT 1.4

#define GUI_OUTPUT_ENABLED_DEFAULT false

class RPFramingNode
{
    private:
        /**
         * Functor for strict weak ordering of rectangles (required for STL sets).
         */
        struct RPRectangleComparator
        {
            bool operator() (const cv::Rect& lhs, const cv::Rect& rhs) const
            {
                return (lhs.x < rhs.x) ||
                       ((lhs.x == rhs.x) && (lhs.y < rhs.y)) ||
                       ((lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.width < rhs.width)) ||
                       ((lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.width == rhs.width) && (lhs.height < rhs.height));
            }
        };

        ros::NodeHandle& node;                      /**< Node's handle.                          */
        bool node_enabled;                          /**< Node's enabled/disabled flag.           */

        ros::WallTime enable_time;                  /**< Node's last enable time.                */

        FramingStatus framing_status;               /**< Framing status.                         */

        cv::Rect frame;                             /**< Produced ideal frame.                   */
        DrivingDirection driving_direction;         /**< Produced driving direction suggestion.  */

        ros::Publisher driving_direction_publisher; /**< Driving direction publisher.            */
        ros::Publisher frame_publisher;             /**< Produced ideal frame publisher.         */

        ros::Subscriber head_tracker_subscriber;    /**< Head-tracking input subscriber.         */

        const cv::Rect CAMERA_FRAME_RECTANGLE;      /**< Camera's frame rectangle.               */

        // Debugging objects
        ros::Subscriber depth_subscriber;           /**< Kinect depth input subscriber (for
                                                         debugging).                             */

        RPDepthImageProcessor depth_processor;      /**< Kinect depth image processor (for
                                                         debugging).                             */

        ros::ServiceClient camera_client;           /**< Camera client (for debugging).          */

        cv::Mat camera_matrix;                      /**< Camera intrinsics matrix (for
                                                         debugging).                             */
        cv::Mat distortion_coefficients;            /**< Camera distortion coefficients (for
                                                         debugging).                             */
        cv::Mat rotation_vector;                    /**< Camera rotation vector (w.r.t. robot's
                                                         base; for debugging).                   */
        cv::Mat translation_vector;                 /**< Camera translation vector (w.r.t.
                                                         robot's base; for debugging).           */

        const std::string WINDOW_NAME;              /**< Output window name (if GUI output is
                                                         enabled).                               */

        double CAMERA_TRANSLATION_X;                /**< Camera translation in X direction OP.   */
        double CAMERA_TRANSLATION_Y;                /**< Camera translation in Y direction OP.   */
        double CAMERA_TRANSLATION_Z;                /**< Camera translation in Z direction OP.   */

        double CAMERA_ROTATION_X;                   /**< Camera rotation around X axis OP.       */
        double CAMERA_ROTATION_Y;                   /**< Camera rotation around Y axis OP.       */
        double CAMERA_ROTATION_Z;                   /**< Camera rotation around Z axis OP.       */

        bool GUI_OUTPUT_ENABLED;                    /**< GUI output enabled flag OP.             */

        /**
         * Head-tracker callback.
         * @param tracked_heads Message containing detected/tracked heads.
         */
        void headTrackerMessageCallback(const rp_head_tracking::Heads& tracked_heads);

        /**
         * Depth image callback.
         * @param depth_input_image Pointer to depth input image.
         */
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_input_image);

        /**
         * Initializes rotation vector as set by the overridable parameters (in Rodrigues
         * representation).
         */
        void initializeRotationVector();

        /**
         * Initializes translation vector as set by the overridable parameters.
         */
        void initializeTranslationVector();

        /**
         * Converts tracked heads to top-left/bottom-right head rectangle points in the Kinect's
         * camera coordinates.
         * @param tracked_heads              Tracked head rectangles in Kinect's image plane.
         * @param projected_points           Projected top-left/bottom-right head rectangle points
         *                                   in Kinect's world coordinate system.
         */
        void convertToKinectCameraCoordinates(const rp_head_tracking::Heads& tracked_heads, std::vector<cv::Point3f>& projected_points);

        /**
         * Converts top-left/bottom-right head rectangle points in the Kinect camera coordinates
         * to head rectangles in the photographic camera's image plane.
         * @param head_points                3D top-left/bottom-right head rectangle points in
         *                                   Kinect's coordinate system.
         * @param projected_head_rectangles  Projected head rectangles in the photographic camera's
         *                                   image plane.
         */
        void convertToCameraImageCoordinates(const std::vector<cv::Point3f>& head_points, std::vector<cv::Rect>& projected_head_rectangles);

        /**
         * Finds the index of the rectangle closest to the camera frame center.
         * @param head_rectangles  Detected head rectangles.
         */
        int centermostHeadRectangleIndex(const std::vector<cv::Rect>& head_rectangles);

        /**
         * Finds the candidates whose head rectangles at least partially intersect with a given
         * frame.
         * @param candidate_head_rectangles  Detected head rectangles.
         * @param frame                      Current ideal frame.
         * @param candidates_in_frame        Resulting candidates in the frame.
         */
        void getCandidatesInFrame(const std::vector<cv::Rect>& candidate_head_rectangles, const cv::Rect& frame, std::set<cv::Rect, RPRectangleComparator>& candidates_in_frame);

        /**
         * Calculates the best frame for a single person.
         * @param head_rectangle  Head rectangle for a candidate head.
         * @param off_center_left Flag describing whether the face should be slightly off-centered
         *                        to the left or to the right.
         * @param frame           Resulting frame.
         */
        void frameSinglePerson(const cv::Rect& head_rectangle, bool off_center_left, cv::Rect& frame);

        /**
         * Calculates the best frame for a wide group of heads.
         * @param bounding_rectangle Bounding rectangle for a candidate set of heads.
         * @param frame              Resulting frame.
         */
        void frameWideGroup(const cv::Rect& bounding_rectangle, cv::Rect& frame);

        /**
         * Calculates the best frame for a narrow group of heads.
         * @param bounding_rectangle Bounding rectangle for a candidate set of heads.
         * @param frame              Resulting frame.
         */
        void frameNarrowGroup(const cv::Rect& bounding_rectangle, cv::Rect& frame);

        /**
         * Publishes the framing status including the ideal frame and the detected head rectangles
         * in the photographic camera's image plane.
         * @param head_rectangles Detected head rectangles reprojected into the photographic
         *                        camera's image plane.
         */
        void publishFramingStatus(const std::vector<cv::Rect>* head_rectangles);

        /**
         * Publishes the stored driving direction.
         */
        void publishDrivingDirection();

        /**
         * Retrieves the "node enabled" status from the parameter server and marks the time
         * of the "enable the node" command issue.
         */
        void refreshNodeEnabledStatus();

        /**
         * Gets the overridable parameters from the parameter server.
         */
        void getOverridableParameters();

    public:
        /**
          * Default composition and framing node's constructor.
          * @param node Handle to ROS node.
          */
        RPFramingNode(ros::NodeHandle& node);
};

#endif /* FRAMING_HPP_ */
