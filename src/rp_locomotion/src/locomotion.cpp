/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <locomotion.hpp>

// STL includes
#include <cmath>

// Kobuki includes
#include <kobuki_msgs/MotorPower.h>

// RPLocomotion includes
#include <driving_direction.h>
#include <locomotion_state.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_locomotion");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPLocomotionNode worker_node(node);
}


RPLocomotionNode::RPLocomotionNode(ros::NodeHandle& node) :
    node(node),
    processing_bumper_event(false),
    locomotion_message(new geometry_msgs::Twist()),
    FRAMING_LINEAR_VELOCITY(FRAMING_LINEAR_VELOCITY_DEFAULT),
    FRAMING_ANGULAR_VELOCITY(FRAMING_ANGULAR_VELOCITY_DEFAULT),
    OBSTACLE_AVOIDANCE_LINEAR_VELOCITY(OBSTACLE_AVOIDANCE_LINEAR_VELOCITY_DEFAULT),
    OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY(OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY_DEFAULT)
{
    // Initialize publishers/subscribers/timers
    locomotion_publisher = node.advertise<geometry_msgs::Twist>("/rp/locomotion/velocity", 1);
    state_publisher = node.advertise<std_msgs::UInt8>("/rp/locomotion/state", 1);
    motor_power_publisher = node.advertise<kobuki_msgs::MotorPower>("/rp/locomotion/motor_power", 1);

    navigation_subscriber = node.subscribe("/rp/navigation/driving_direction_and_source", 1, &RPLocomotionNode::navigationMessageCallback, this);
    bumper_subscriber = node.subscribe("/mobile_base/events/bumper", 1, &RPLocomotionNode::bumperEventCallback, this);

    double approximate_sleep_time = 2.0 * M_PI / OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;
    bumper_timer = node.createWallTimer(ros::WallDuration(approximate_sleep_time), &RPLocomotionNode::bumperTimerCallback, this, true, false);

    setProcessingBumperEvent(false);
    clearLocomotionMessage();

    motorsEnabled(true);

    // Spin at 5 Hz
    ros::Rate rate(5);
    while (ros::ok())
    {
        locomotion_message_mutex.lock();
        {
            locomotion_publisher.publish(locomotion_message);
        }
        locomotion_message_mutex.unlock();

        ros::spinOnce();
        rate.sleep();
    }
};


void RPLocomotionNode::getOverridableParameters()
{
    node.getParamCached("/rp/locomotion_node/framing_linear_velocity", FRAMING_LINEAR_VELOCITY);
    node.getParamCached("/rp/locomotion_node/framing_angular_velocity", FRAMING_ANGULAR_VELOCITY);
    node.getParamCached("/rp/locomotion_node/obstacle_avoidance_linear_velocity", OBSTACLE_AVOIDANCE_LINEAR_VELOCITY);
    node.getParamCached("/rp/locomotion_node/obstacle_avoidance_angular_velocity", OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY);
}


void RPLocomotionNode::setProcessingBumperEvent(bool processing)
{
    processing_bumper_event = processing;

    std_msgs::UInt8 bumper_event_message;
    bumper_event_message.data = static_cast<uint8_t>(processing ? PROCESSING_BUMPER_EVENT : NORMAL);

    state_publisher.publish(bumper_event_message);
}


void RPLocomotionNode::bumperTimerCallback(const ros::WallTimerEvent& timer_event)
{
    setProcessingBumperEvent(false);

    ROS_INFO("Bumper timer called.");
}


void RPLocomotionNode::bumperEventCallback(const kobuki_msgs::BumperEventConstPtr bumper_event)
{
    if ((bumper_event->state == kobuki_msgs::BumperEvent::PRESSED) && !processing_bumper_event)
    {
        setProcessingBumperEvent(true);

        locomotion_message_mutex.lock();
        {
            // For direction of turn maintenance
            double old_angular_velocity = locomotion_message->angular.z;
            clearLocomotionMessage();

            switch (bumper_event->bumper)
            {
                case kobuki_msgs::BumperEvent::CENTER:
                {
                    locomotion_message->angular.z = (old_angular_velocity < 0.0) ? OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY : -OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;
                    break;
                }
                case kobuki_msgs::BumperEvent::LEFT:
                {
                    locomotion_message->angular.z = -OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;
                    break;
                }
                case kobuki_msgs::BumperEvent::RIGHT:
                {
                    locomotion_message->angular.z = OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;
                    break;
                }
            }
        }
        locomotion_message_mutex.unlock();

        // Prepare the timer for turn-around motion if a bumper is pressed: calculate the approximate
        // sleep time given that the angular velocity is specified in radians per second.
        double approximate_sleep_time = 2.0 * M_PI / OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;
        bumper_timer.stop();
        bumper_timer.setPeriod(ros::WallDuration(approximate_sleep_time));
        bumper_timer.start();

        ROS_INFO("Started bumper timer.");
    }
}


void RPLocomotionNode::navigationMessageCallback(const std_msgs::UInt16& direction_and_source_code)
{
    if (processing_bumper_event)
    {
        return;
    }

    // Check whether the node is enabled in parameter server
    bool enabled = false;
    node.getParamCached("/rp/locomotion_node/enabled", enabled);

    if (!enabled)
    {
        return;
    }

    // Get the parameters from the parameter server
    getOverridableParameters();

    // Convert direction and source code to 8 bit values
    uint8_t direction_source_code = direction_and_source_code.data & 0xFF;
    uint8_t driving_direction_code = (direction_and_source_code.data & 0xFF00) >> 8;

    // Get the driving direction source
    DirectionSource direction_source = uint8ToDirectionSource(direction_source_code);

    // Get the driving direction
    DrivingDirection direction = uint8ToDrivingDirection(driving_direction_code);

    double linear_velocity = (direction_source == FRAMING_NODE) ? FRAMING_LINEAR_VELOCITY : OBSTACLE_AVOIDANCE_LINEAR_VELOCITY;
    double angular_velocity = (direction_source == FRAMING_NODE) ? FRAMING_ANGULAR_VELOCITY : OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY;

    locomotion_message_mutex.lock();
    {
        clearLocomotionMessage();
        switch (direction)
        {
            case FORWARD:
            {
                locomotion_message->linear.x = linear_velocity;
                break;
            }
            case BACKWARD:
            {
                locomotion_message->linear.x = -linear_velocity;
                break;
            }
            case LEFT:
            {
                locomotion_message->angular.z = angular_velocity;
                break;
            }
            case RIGHT:
            {
                locomotion_message->angular.z = -angular_velocity;
                break;
            }
            case STOP:
            default:
            {
                // Do nothing since locomotion message is already cleared.
                break;
            }
        }
    }
    locomotion_message_mutex.unlock();
}


void RPLocomotionNode::clearLocomotionMessage()
{
    locomotion_message->linear.x = 0.0;
    locomotion_message->linear.y = 0.0;
    locomotion_message->linear.z = 0.0;
    locomotion_message->angular.x = 0.0;
    locomotion_message->angular.y = 0.0;
    locomotion_message->angular.z = 0.0;
}


void RPLocomotionNode::motorsEnabled(bool enabled)
{
    kobuki_msgs::MotorPower power_message;
    power_message.state = enabled ? (uint8_t)kobuki_msgs::MotorPower::ON :
                                    (uint8_t)kobuki_msgs::MotorPower::OFF;

    motor_power_publisher.publish(power_message);
}
