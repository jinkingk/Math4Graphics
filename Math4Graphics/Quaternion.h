///////////////////////////////////////////////////////////
//Quaternion.h
///////////////////////////////////////////////////////////
#pragma once
class Vector3;
class EulerAngles;

//---------------------------------------------------------
//Quaternion类
//实现在3D中表现角位移的四元数

class Quaternion
{
public:
	//公共数据
	float w, x, y, z;

	//公共操作
	//置为单位四元数
	void indetity() { w = x = y = z = 0; }

	//构造执行旋转的四元数
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3 &axis,float theta);

	//构造执行物体-惯性旋转的四元数
	void setToRotateObjectToInertial(const EulerAngles &orientation);
	void setToRotateInertialToObject(const EulerAngles &orientation);

	//叉乘
	Quaternion operator * (const Quaternion &a) const;

	//赋值乘法
	Quaternion& operator *= (const Quaternion &a);

	//正则化
	void normalize();

	//提取旋转角和旋转轴
	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
};

//全局“单位”四元数
extern const Quaternion kQuaternionIdentity;

//四元数点乘
extern float dotProduct(const Quaternion &a, const Quaternion &b);

//球面线性差值
extern Quaternion slerp(const Quaternion &a, const Quaternion &b, float t);

//四元数共轭
extern Quaternion conjugate(const Quaternion&q);

//四元数幂
extern Quaternion pow(const Quaternion&q, float exponent);
