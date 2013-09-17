/**
 * @class      RPDistanceConverter
 *
 * @brief      Distance converter class, which converts between pixel and metric representations
 *             of given depth images.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef DISTANCE_CONVERTER_HPP_
#define DISTANCE_CONVERTER_HPP_

// RPHeadTracking includes
#include <constants.hpp>

#define VERTICAL_FOV_COEFFICIENT 0.92f
#define HORIZONTAL_FOV_COEFFICIENT 1.23f

namespace RPDistanceConverter
{

/**
 * Converts horizontal metric widths to pixels.
 * @param width    Width (in meters) to convert.
 * @param distance Distance (in meters) at which this width is observed.
 * @returns Corresponding width in pixels.
 */
inline float horizontalWidthToPixels(float width, float distance)
{
    return (FRAME_WIDTH * width) / (HORIZONTAL_FOV_COEFFICIENT * distance);
}

/**
 * Converts vertical metric heights to pixels.
 * @param width    Height (in meters) to convert.
 * @param distance Distance (in meters) at which this height is observed.
 * @returns Corresponding height in pixels.
 */
inline float verticalHeightToPixels(float height, float distance)
{
    return (FRAME_HEIGHT * height) / (VERTICAL_FOV_COEFFICIENT * distance);
}

/**
 * Converts pixels to metric widths.
 * @param pixels   Width (in pixels).
 * @param distance Distance (in meters) at which this width is observed.
 * @returns Corresponding width in meters.
 */
inline float pixelsToWidth(int pixels, float distance)
{
    return (HORIZONTAL_FOV_COEFFICIENT * distance * (float)pixels) / (float)FRAME_WIDTH;
}

/**
 * Converts pixels to metric heights.
 * @param pixels   Height (in pixels).
 * @param distance Distance (in meters) at which this height is observed.
 * @returns Corresponding height in meters.
 */
inline float pixelsToHeight(int pixels, float distance)
{
    // 48 fov
    return (VERTICAL_FOV_COEFFICIENT * distance * (float)pixels) / (float)FRAME_HEIGHT;
}

} // namespace RPDistanceConverter

#endif /* DISTANCE_CONVERTER_HPP_ */
