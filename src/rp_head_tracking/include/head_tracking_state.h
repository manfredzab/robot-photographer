/**
 * @enum      HeadTrackingState
 *
 * @brief     Head tracking node's state.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef HEAD_TRACKING_STATE_H_
#define HEAD_TRACKING_STATE_H_

enum HeadTrackingState
{
    GATHERING_FACE_HUE_DATA,
    DETECTING_HEADS,
    TRACKING_HEADS,
    INVALID
};

#endif /* HEAD_TRACKING_STATE_H_ */
