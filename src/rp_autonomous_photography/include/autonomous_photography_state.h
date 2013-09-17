/**
 * @enum      AutonomousPhotographyState
 *
 * @brief     Autonomous photography node state.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef AUTONOMOUS_PHOTOGRAPHY_STATE_H_
#define AUTONOMOUS_PHOTOGRAPHY_STATE_H_

#include <std_msgs/UInt8.h>

enum AutonomousPhotographyState
{
    AVOIDING_OBSTACLES,
    TAKING_PICTURE,
    UPLOADING_PICTURE,
    FRAMING_PICTURE
};

#endif /* AUTONOMOUS_PHOTOGRAPHY_STATE_H_ */
