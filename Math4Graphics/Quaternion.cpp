#include <assert.h>
#include<math.h>

#include "Quaternion.h"
#include "MathUtil.h"
#include "Vector3.h"
#include"EulerAngles.h"

////////////////////////////////////////////////////////////
//ȫ������
////////////////////////////////////////////////////////////
//ȫ�ֵ�λ��Ԫ��
const Quaternion kQuaternionIdentity = {
	1.0f,0.0f,0.0f,0.0f
};

////////////////////////////////////////////////////////////
//���Ա
////////////////////////////////////////////////////////////

//������ָ������ת����Ԫ��
void Quaternion::setToRotateAboutX(float theta)
{
	//������
	float thetaOver2 = theta * .5f;

	//��ֵ
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
	//������
	float thetaOver2 = theta * .5f;

	//��ֵ
	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	//������
	float thetaOver2 = theta * .5f;

	//��ֵ
	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta)
{
	//��ת������׼��
	assert(fabs(vectroMag(axis) - 1.0f) < .01f);

	//�����Ǻ�sinֵ
	float thetaOver2 = theta * .5f;
	float sinThetaOver2 = sin(thetaOver2);

	//��ֵ
	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}

//---------------------------------------------------------------------------------
//EulerAngles::setToRotateObjectToInertial
//��������-������ת����Ԫ��

void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation)
{
	//�����ǵ�sin �� cosֵ
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	//������
	w = ch*cp*cb + sh*sp*sb;
	x = ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;
}
//�������-������ת��Ԫ��
void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation)
{
	//�����ǵ�sin �� cosֵ
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	//������
	w = ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y = ch*sp*sb - sh*cp*cb;
	z = sh*sp*cb - ch*cp*sb;
}
//������Ԫ��
void Quaternion::normalize()
{
	//ģ
	float mag = (float)sqrt(w*w + x*x + y*y + z*z);

	//����Ƿ�Ϊ0
	if (mag > 0.0f)
	{
		//����
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else {
		//���鷳��
		assert(false);
	}
}
//������ת��
float Quaternion::getRotationAngle() const
{
	//������ w = cos(theta/2)
	float thetaOver2 = safeAcos(w);

	//������ת��
	return thetaOver2 * 2.0f;
}

Vector3 Quaternion::getRotationAxis() const
{
	//���� sin^2(theta/2) ,  w = cos(theta/2), sin^2(x) + cos^2(x) =1
	float sinThetaOver2Sq = 1.0f - w*w;

	//��֤����
	if (sinThetaOver2Sq <= 0.0f)
	{
		//��λ��Ԫ���򲻾�ȷ����ֵ  ֻҪ������Ч����
		return Vector3(1.0f, 0.0f, 0.0f);
	}

	//����1/sin(theta/2)
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);

	//������ת��
	return Vector3(
		x*oneOverSinThetaOver2,
		y*oneOverSinThetaOver2,
		z*oneOverSinThetaOver2
	);
}

Quaternion& Quaternion::operator*=(const Quaternion &a)
{
	//�˲���ֵ
	*this = *this * a;

	return *this;
}

Quaternion Quaternion::operator*(const Quaternion &a) const
{
	Quaternion result;

	result.w = w*a.w - x*a.x - y*a.y - z*a.z;
	result.x = w*a.x + x*a.w + z*a.y - y*a.z;
	result.y = w*a.y + y*a.w + x*a.z - z*a.x;
	result.w = w*a.z + z*a.w + y*a.x - x*a.y;

	return result;
}

//////////////////////////////////////////////////////////////////////
//�ǳ�Ա����
/////////////////////////////////////////////////////////////////////

//��Ԫ�����
extern float dotProduct(const Quaternion &a, const Quaternion &b)
{
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}
//�������Բ�ֵ
extern Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
{
	if (t <= 0.0f) return a;
	if (t >= 1.0f)return b;
	//
	//δ��
	//
}
//��Ԫ������
extern Quaternion conjugate(const Quaternion&q)
{
	Quaternion result;
	//��ת����ͬ
	result.w = q.w;

	//��ת���෴
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	return result;
}
//��Ԫ����
extern Quaternion pow(const Quaternion&q, float exponent)
{
	//��鵥λ��Ԫ��,��ֹ����
	if (fabs(q.w) > .9999f) {
		return q;
	}

	//��ȡ���alpha(alpha=theta/2)
	float alpha = acos(q.w);

	//������alpha
	float newAlpha = alpha * exponent;

	//������w
	Quaternion result;
	result.w = cos(newAlpha);

	//������x,y,z
	float mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z *mult;

	return result;
}
