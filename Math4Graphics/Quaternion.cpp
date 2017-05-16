#include <assert.h>
#include<math.h>

#include "Quaternion.h"
#include "MathUtil.h"
#include "Vector3.h"
#include"EulerAngles.h"

////////////////////////////////////////////////////////////
//全局数据
////////////////////////////////////////////////////////////
//全局单位四元数
const Quaternion kQuaternionIdentity = {
	1.0f,0.0f,0.0f,0.0f
};

////////////////////////////////////////////////////////////
//类成员
////////////////////////////////////////////////////////////

//构造绕指定轴旋转的四元数
void Quaternion::setToRotateAboutX(float theta)
{
	//计算半角
	float thetaOver2 = theta * .5f;

	//赋值
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
	//计算半角
	float thetaOver2 = theta * .5f;

	//赋值
	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	//计算半角
	float thetaOver2 = theta * .5f;

	//赋值
	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta)
{
	//旋转轴必须标准化
	assert(fabs(vectroMag(axis) - 1.0f) < .01f);

	//计算半角和sin值
	float thetaOver2 = theta * .5f;
	float sinThetaOver2 = sin(thetaOver2);

	//赋值
	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}

//---------------------------------------------------------------------------------
//EulerAngles::setToRotateObjectToInertial
//构造物体-惯性旋转的四元数

void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation)
{
	//计算半角的sin 和 cos值
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	//计算结果
	w = ch*cp*cb + sh*sp*sb;
	x = ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;
}
//构造惯性-物体旋转四元数
void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation)
{
	//计算半角的sin 和 cos值
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	//计算结果
	w = ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y = ch*sp*sb - sh*cp*cb;
	z = sh*sp*cb - ch*cp*sb;
}
//正则化四元数
void Quaternion::normalize()
{
	//模
	float mag = (float)sqrt(w*w + x*x + y*y + z*z);

	//检测是否为0
	if (mag > 0.0f)
	{
		//正则化
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else {
		//有麻烦了
		assert(false);
	}
}
//返回旋转角
float Quaternion::getRotationAngle() const
{
	//计算半角 w = cos(theta/2)
	float thetaOver2 = safeAcos(w);

	//返回旋转角
	return thetaOver2 * 2.0f;
}

Vector3 Quaternion::getRotationAxis() const
{
	//计算 sin^2(theta/2) ,  w = cos(theta/2), sin^2(x) + cos^2(x) =1
	float sinThetaOver2Sq = 1.0f - w*w;

	//保证精度
	if (sinThetaOver2Sq <= 0.0f)
	{
		//单位四元数或不精确的数值  只要返回有效向量
		return Vector3(1.0f, 0.0f, 0.0f);
	}

	//计算1/sin(theta/2)
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);

	//返回旋转轴
	return Vector3(
		x*oneOverSinThetaOver2,
		y*oneOverSinThetaOver2,
		z*oneOverSinThetaOver2
	);
}

Quaternion& Quaternion::operator*=(const Quaternion &a)
{
	//乘并赋值
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
//非成员函数
/////////////////////////////////////////////////////////////////////

//四元数点乘
extern float dotProduct(const Quaternion &a, const Quaternion &b)
{
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}
//球面线性差值
extern Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
{
	if (t <= 0.0f) return a;
	if (t >= 1.0f)return b;
	//
	//未完
	//
}
//四元数共轭
extern Quaternion conjugate(const Quaternion&q)
{
	Quaternion result;
	//旋转量相同
	result.w = q.w;

	//旋转轴相反
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	return result;
}
//四元数幂
extern Quaternion pow(const Quaternion&q, float exponent)
{
	//检查单位四元数,防止除零
	if (fabs(q.w) > .9999f) {
		return q;
	}

	//提取半角alpha(alpha=theta/2)
	float alpha = acos(q.w);

	//计算新alpha
	float newAlpha = alpha * exponent;

	//计算新w
	Quaternion result;
	result.w = cos(newAlpha);

	//计算新x,y,z
	float mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z *mult;

	return result;
}
