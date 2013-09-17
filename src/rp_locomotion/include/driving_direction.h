/**
 * @enum      DrivingDirection
 *
 * @brief     Robot's driving direction enumeration.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef __DRIVING_DIRECTION_H
#define __DRIVING_DIRECTION_H

#include <std_msgs/UInt8.h>

enum DrivingDirection
{
    FORWARD = 0,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

/**
 * Converts a given driving direction to unsigned eight bit representation.
 * @param driving_direction Input driving direction.
 * @returns Unsigned eight bit representation of the given driving direction.
 */
uint8_t drivingDirectionToUint8(DrivingDirection driving_direction)
{
    return static_cast<uint8_t>(driving_direction);
}

/**
 * Converts an unsigned eight bit driving direction representation to enumeration.
 * @param driving_direction Input eight bit representation of the driving direction.
 * @returns Converted driving direction enumeration.
 */
DrivingDirection uint8ToDrivingDirection(uint8_t driving_direction)
{
    return static_cast<DrivingDirection>(driving_direction);
}

#endif
