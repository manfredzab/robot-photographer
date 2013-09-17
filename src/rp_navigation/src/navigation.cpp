/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <navigation.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rp_navigation");

    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPNavigationNode worker_node(node);
}


RPNavigationNode::RPNavigationNode(ros::NodeHandle& node) :
    node(node),
    obstacle_avoidance_driving_direction(STOP),
    framing_driving_direction(STOP),
    driving_direction_source(OBSTACLE_AVOIDANCE_NODE)
{
    // Initialize driving direction publisher
    driving_direction_publisher = node.advertise<std_msgs::UInt16>("/rp/navigation/driving_direction_and_source", 1);

    // Initialize driving direction subscribers
    obstacle_avoidance_subscriber = node.subscribe("/rp/obstacle_avoidance/driving_direction", 1, &RPNavigationNode::obstacleAvoidanceDirectionMessageCallback, this);
    framing_subscriber = node.subscribe("/rp/framing/driving_direction", 1, &RPNavigationNode::framingDirectionMessageCallback, this);

    // Initialize Command & Control subscriber
    auton_photography_subscriber = node.subscribe("/rp/autonomous_photography/direction_source", 1, &RPNavigationNode::autonomousPhotographyDirectionSourceMessageCallback, this);

    // Spin at 5 Hz
    DrivingDirection driving_direction;
    DirectionSource direction_source;

    ros::Rate rate(5);
    while (ros::ok())
    {
        getDrivingDirectionAndSource(&driving_direction, &direction_source);

        // Publish the direction
        ROS_INFO("Navigation direction: %s", (driving_direction == LEFT) ? "LEFT" :
                                             (driving_direction == RIGHT) ? "RIGHT" :
                                             (driving_direction == STOP) ? "STOP" :
                                             (driving_direction == FORWARD) ? "FORWARD" :  "NONE");

        std_msgs::UInt16 converted_direction_and_source;
        converted_direction_and_source.data = (drivingDirectionToUint8(driving_direction) << 8) | (directionSourceToUint8(direction_source));

        driving_direction_publisher.publish(converted_direction_and_source);

        ros::spinOnce();
        rate.sleep();
    }
};


void RPNavigationNode::getDrivingDirectionAndSource(DrivingDirection* driving_direction, DirectionSource* direction_source)
{
    // Store a copy of the driving direction source
    driving_direction_source_mutex.lock();
    {
        *direction_source = driving_direction_source;
    }
    driving_direction_source_mutex.unlock();

    ROS_INFO("Navigation direction source: %s", (*direction_source == OBSTACLE_AVOIDANCE_NODE) ? "OBSTACLE_AVOIDANCE_NODE" :
                                                (*direction_source == FRAMING_NODE) ? "FRAMING_NODE" : "NONE");

    if (NONE == driving_direction_source)
    {
        *driving_direction = STOP;
        return;
    }

    // Store a copy of the obstacle avoidance driving direction
    obstacle_avoidance_driving_direction_mutex.lock();
    {
        *driving_direction = obstacle_avoidance_driving_direction;
    }
    obstacle_avoidance_driving_direction_mutex.unlock();

    switch (*direction_source)
    {
        case FRAMING_NODE:
        {
            // Allow overriding of the driving direction by the framing node if there are no obstacles
            // ahead or if the framing node would prefer the robot to spin in place or stop moving.
            framing_driving_direction_mutex.lock();
            {
                if ((FORWARD == *driving_direction) ||
                    (LEFT == framing_driving_direction) ||
                    (RIGHT == framing_driving_direction) ||
                    (STOP == framing_driving_direction))
                {
                    *driving_direction = framing_driving_direction;
                }
            }
            framing_driving_direction_mutex.unlock();

            break;
        }

        case OBSTACLE_AVOIDANCE_NODE:
        case NONE:
        default:
        {
            // Do nothing, we already have the required direction
            break;
        }
    }
}


void RPNavigationNode::clearPublishedDrivingDirections()
{
    obstacle_avoidance_driving_direction_mutex.lock();
    {
        obstacle_avoidance_driving_direction = STOP;
    }
    obstacle_avoidance_driving_direction_mutex.unlock();

    framing_driving_direction_mutex.lock();
    {
        framing_driving_direction = STOP;
    }
    framing_driving_direction_mutex.unlock();
}


void RPNavigationNode::obstacleAvoidanceDirectionMessageCallback(const std_msgs::UInt8& direction_code)
{
    obstacle_avoidance_driving_direction_mutex.lock();
    {
        obstacle_avoidance_driving_direction = uint8ToDrivingDirection(direction_code.data);
    }
    obstacle_avoidance_driving_direction_mutex.unlock();
}


void RPNavigationNode::framingDirectionMessageCallback(const std_msgs::UInt8& direction_code)
{
    framing_driving_direction_mutex.lock();
    {
        framing_driving_direction = uint8ToDrivingDirection(direction_code.data);
    }
    framing_driving_direction_mutex.unlock();
}


void RPNavigationNode::autonomousPhotographyDirectionSourceMessageCallback(const std_msgs::UInt8& direction_source_code)
{
    driving_direction_source_mutex.lock();
    {
        driving_direction_source = static_cast<DirectionSource>(direction_source_code.data);
    }
    driving_direction_source_mutex.unlock();
}
