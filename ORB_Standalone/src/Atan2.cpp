#include "Atan2.h"
#include <cmath>

namespace ORB_Standalone
{

static const float PI = 3.14159265358979323846f;
static const float RAD_TO_DEG = 180.0f / PI;

// Minimax polynomial coefficients for fast atan2
static const float p1 = 0.9997878412794807f * RAD_TO_DEG;
static const float p3 = -0.3258083974640975f * RAD_TO_DEG;
static const float p5 = 0.1555786518463281f * RAD_TO_DEG;
static const float p7 = -0.04432655554792128f * RAD_TO_DEG;

float FastAtan2(float y, float x)
{
    float ax = std::abs(x);
    float ay = std::abs(y);

    if (ax == 0.0f && ay == 0.0f)
        return 0.0f;

    float angle;
    if (ax >= ay)
    {
        float c = ay / (ax + 1e-10f);
        float c2 = c * c;
        angle = (((p7 * c2 + p5) * c2 + p3) * c2 + p1) * c;
    }
    else
    {
        float c = ax / (ay + 1e-10f);
        float c2 = c * c;
        angle = 90.0f - (((p7 * c2 + p5) * c2 + p3) * c2 + p1) * c;
    }

    if (x < 0.0f)
        angle = 180.0f - angle;
    if (y < 0.0f)
        angle = 360.0f - angle;

    if (angle >= 360.0f)
        angle -= 360.0f;
    if (angle < 0.0f)
        angle += 360.0f;

    return angle;
}

} // namespace ORB_Standalone
