#include <math.h>

#include "MathUtil.h"
#include "Vector3.h"
extern float wrapPi(float theta)
{
	theta += kPi;
	theta -= floor(theta / k2Pi)*k2Pi;
	theta -= kPi;
	return theta;
}

extern float safeAcos(float x)
{
	if (x < -1.0)
		return kPi;
	if (x > 1.0)
		return 0;
	return acos(x);
}

