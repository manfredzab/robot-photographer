/**
 * @enum      FramingStatus
 *
 * @brief     Framing and composition node's state.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef FRAMING_STATUS_H_
#define FRAMING_STATUS_H_

#include <std_msgs/UInt8.h>

enum FramingStatus
{
    NO_FRAME = 0,
    FRAME_OUT_OF_BOUNDS_HORIZONTALLY,
    FRAME_OUT_OF_BOUNDS_VERTICALLY,
    FRAME_TOO_SMALL,
    FRAMED
};

#endif /* FRAMING_STATUS_H_ */
