#ifndef ATAN2_H
#define ATAN2_H

namespace ORB_Standalone
{

/**
 * @brief Fast atan2 implementation returning angle in degrees [0, 360.0f).
 *
 * Efficient polynomial minimax approximation of atan2.
 *
 * @param y Vertical Cartesian coordinate (moment m01)
 * @param x Horizontal Cartesian coordinate (moment m10)
 * @return Angle in degrees in range [0, 360.0f)
 */
float FastAtan2(float y, float x);

} // namespace ORB_Standalone

#endif // ATAN2_H
