//////////////////////////////////////////////
//EulerAngles.cpp
//////////////////////////////////////////////

#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

//全局单位欧拉常量
const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

///////////////////////////////////
//class EulerAngles Implementation
///////////////////////////////////

//--------------------------------------------------
//EulerAngles::canonize
//将欧拉角转换到限制集中

void EulerAngles::canonize() {
	//-pi - pi
	pitch = wrapPi(pitch);
	//-pi/2 - pi/2
	if (pitch < -kPiOver2) {
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}else if (pitch > kPiOver2) {
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}

	//检查万向索
	if (fabs(pitch)>kPiOver2-1e-4)
	{
		//在万向锁中将所有绕垂直轴的旋转赋给heading
		heading += bank;
		bank = 0.0f;
	}else {
		//非万向锁
		bank = wrapPi(bank);
	}

	heading = wrapPi(heading);
}
//从物体-惯性四元数到欧拉角
void EulerAngles::fromObjectToIntertialQuaternion(const Quaternion &q)
{
	//计算sin(pitch)
	float sp = -2.0f * (q.y*q.z - q.w*q.x);

	//检查万向索
	if (fabs(sp) > 0.9999f){
		//从正上方或正下方看
		pitch = kPiOver2 * sp;
		//bank 置零 计算heading
		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}else{
		//计算角度
		pitch = asin(sp);
		heading = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//从惯性-物体四元数到欧拉角
void EulerAngles::fromIntertialToObjectQuaternion(const Quaternion &q)
{
	//计算sin(pitch)
	float sp = -2.0f * (q.y*q.z + q.w*q.x);

	//检查万向索
	if (fabs(sp) > 0.9999f) {
		//从正上方或正下方看
		pitch = kPiOver2 * sp;
		//bank 置零 计算heading
		heading = atan2(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}
	else {
		//计算角度
		pitch = asin(sp);
		heading = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//从物体世界坐标系变换矩阵到欧拉角
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m)
{
	//通过m32计算 sin(pitch)
	float sp = -m.m32;
	//检查万向锁
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank 置零 计算heading
		heading = atan2(-m.m23, m.m11);
		bank = 0.0f;
	}else{
		//计算角度
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}
//从世界-物体坐标系变换矩阵到欧拉角
void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m)
{
	//通过m23计算 sin(pitch)
	float sp = -m.m23;
	//检查万向锁
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank 置零 计算heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		//计算角度
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}
//根据旋转矩阵构造欧拉角
void EulerAngles::fromRotationMatrix(const RotationMatrix &m)
{
	//通过m23计算 sin(pitch)
	float sp = -m.m23;
	//检查万向锁
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank 置零 计算heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		//计算角度
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}
