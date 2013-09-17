/**
 * @enum      DirectionSource
 *
 * @brief     Robot's driving direction source enumeration.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DIRECTION_SOURCE_H_
#define DIRECTION_SOURCE_H_

enum DirectionSource
{
    NONE = 0,
    OBSTACLE_AVOIDANCE_NODE,
    FRAMING_NODE
};

/**
 * Converts a given driving direction source to unsigned eight bit representation.
 * @param direction_source Input driving direction source.
 * @returns Unsigned eight bit representation of the given driving direction source.
 */
uint8_t directionSourceToUint8(DirectionSource direction_source)
{
    return static_cast<uint8_t>(direction_source);
}

/**
 * Converts an unsigned eight bit driving direction source representation to enumeration.
 * @param direction_source Input eight bit representation of the driving direction source.
 * @returns Converted driving direction source enumeration.
 */
DirectionSource uint8ToDirectionSource(uint8_t direction_source)
{
    return static_cast<DirectionSource>(direction_source);
}

#endif /* DIRECTION_SOURCE_H_ */
