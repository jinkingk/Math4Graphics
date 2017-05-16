#include "Quaternion.h"
void Quaternion::setToRotateAboutX(float theta)
{

}

void Quaternion::setToRotateAboutY(float theta)
{

}

void Quaternion::setToRotateAboutZ(float theta)
{

}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta)
{

}

void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation)
{

}

void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation)
{

}

void Quaternion::normalize()
{

}

float Quaternion::getRotationAngle() const
{

}

Vector3 Quaternion::getRotationAxis() const
{

}

Quaternion Quaternion::operator*=(const Quaternion &a)
{

}

Quaternion Quaternion::operator*(const Quaternion &a) const
{

}

extern float dotProduct(const Quaternion &a, const Quaternion &b)
{

}

extern Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
{

}

extern Quaternion conjugate(const Quaternion&q)
{

}

extern Quaternion pow(const Quaternion&q, float exponent)
{

}
