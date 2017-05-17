//////////////////////////////////////////////////////////////////
//Matrix4x3.cpp
//////////////////////////////////////////////////////////////////

#include <assert.h>
#include <math.h>

#include "Vector3.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "RotationMatrix.h"
#include "Matrix4x3.h"
#include "MathUtil.h"

//置为单位矩阵
void Matrix4x3::identity()
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m21 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = 0.0f; ty = 0.0f; tz = 1.0f;
}
//将平移部分的第四行置为零
void Matrix4x3::zeroTranslation()
{
	tx = ty = tz = 0.0f;
}
//平移部分赋值
void Matrix4x3::setTranslation(const Vector3&d)
{
	tx = d.x; ty = d.y; tz = d.z;
}
//平移部分赋值
void Matrix4x3::setupTrainslation(const Vector3&d)
{
	//线性变化部分为单位矩阵
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m21 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

	tx = d.x; ty = d.y; tz = d.z;
}

void Matrix4x3::setupLocalToParent(const Vector3 &pos, const EulerAngles&orient)
{
	//创建一个旋转矩阵
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);

	//构造4X3矩阵
	setupLocalToParent(pos, orientMatrix);
}

void Matrix4x3::setupLocalToParent(const Vector3 &pos, const RotationMatrix&orient)
{
	//复制矩阵的旋转部分
	//旋转矩阵一般是惯性-物体 即 父-局部
	//因此要做转职
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;

	//设置平移部分
	tx = pos.x; ty = pos.y; tz = pos.z;
}

void Matrix4x3::setupParentToLocal(const Vector3 &pos, const EulerAngles&orient)
{
	//创建一个旋转矩阵
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);

	//构造4X3矩阵
	setupParentToLocal(pos, orientMatrix);
}

void Matrix4x3::setupParentToLocal(const Vector3 &pos, const RotationMatrix&orient)
{
	//复制矩阵的旋转部分
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;

	//设置平移部分
	//世界-惯性只需平移坐负的向量
	//但旋转“先发生”，所以要旋转平一部分
	tx = -(pos.x*m11 + pos.y*m21 + pos.z*m31);
	ty = -(pos.x*m12 + pos.y*m22 + pos.z*m32);
	tz = -(pos.x*m13 + pos.y*m23 + pos.z*m33);
}
//构造绕坐标轴旋转的矩阵
//旋转轴由从1开始的索引制定
//
//1-绕x轴旋转
//2-绕y轴旋转
//3-绕z轴旋转
//theta是旋转量 用弧度表示
void Matrix4x3::setupRotate(int axis, float theta)
{
	//取得旋转角sin cos
	float s, c;
	sinCos(&s, &c, theta);
	//判断旋转轴
	switch (axis)
	{
	case 1://x
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = c; m23 = s;
		m31 = 0.0f; m32 = -s; m33 = c;
		break;
	case 2://y
		m11 = c; m12 = 0.0f; m13 = -s;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s; m32 = 0.0f; m33 = c;
		break;
	case 3://z
		m11 = c; m12 = s; m13 = 0.0f;
		m21 = -s; m22 = c; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	default:
		assert(false);
	}
	tx = ty = tz = 0.0f;
}
//绕任意轴旋转
void Matrix4x3::setupRotate(const Vector3&axis, float theta)
{
	//检查是否为单位向量
	assert(fabs(axis*axis - 1.0f) < .01f);
	//取得旋转角sin cos
	float s, c;
	sinCos(&s, &c, theta);
	//计算1-cos(theta)和一些公用的子表达式
	float a = 1.0f - c;
	float ax = a*axis.x;
	float ay = a*axis.y;
	float az = a*axis.z;
	//矩阵的赋值
	m11 = ax*axis.x + c;
	m12 = ax*axis.y + axis.z*s;
	m13 = ax*axis.z + axis.y*s;

	m21 = ay*axis.x - axis.z*s;
	m22 = ay*axis.y + c;
	m23 = ay*axis.z + axis.x*s;

	m31 = az*axis.x + axis.y*s;
	m32 = az*axis.y - axis.x*s;
	m33 = az*axis.z + c;

	//平移部分置零
	tx = ty = tz = 0;
}
//从四元数到矩阵
void Matrix4x3::fromQuaternion(const Quaternion&q)
{
	//计算一些公用的子表达式
	float ww = 2.0*q.w;
	float xx = 2.0*q.x;
	float yy = 2.0*q.y;
	float zz = 2.0*q.z;

	//矩阵的赋值
	m11 = 1.0f - yy*q.y - zz*q.z;
	m12 = xx*q.y + ww*q.z;
	m13 = xx*q.z - ww*q.x;

	m21 = xx*q.y - ww*q.z;
	m22 = 1.0f - xx*q.x - zz*q.z;
	m23 = yy*q.z + ww*q.x;

	m31 = xx*q.z + ww*q.y;
	m32 = yy*q.z - ww*q.x;
	m33 = 1.0f - xx*q.x - yy*q.y;

	tx = ty = tz = 0.0f;
}
//沿坐标轴缩放
void Matrix4x3::setupScale(const Vector3&s)
{
	m11 = s.x; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = s.y; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = s.z;

	tx = ty = tz = 0.0f;
}
//沿任意轴缩放 旋转轴为单位向量
void Matrix4x3::setupScaleAlongAxis(const Vector3&axis, float k)
{
	assert(fabs(axis*axis - 1.0f) < 0.1f);
	//计算k-1和常用的子表达式
	float a = k - 1.0f;
	float ax = a*axis.x;
	float ay = a*axis.y;
	float az = a*axis.z;

	//矩阵元素赋值
	m11 = ax*axis.x + 1.0f;
	m22 = ay*axis.y + 1.0f;
	m33 = az*axis.z + 1.0f;

	m12 = m21 = ax*axis.y;
	m13 = m31 = ax*axis.z;
	m23 = m32 = ay*axis.z;

	tx = ty = tz = 0.0f;
}
//构造切变矩阵
void Matrix4x3::setupSphear(int axis, float s, float t)
{

}
//构造投影矩阵，投影平面过原点，且垂直于单位向量n
void Matrix4x3::setupProject(const Vector3&n)
{
	assert(fabs(n*n - 1.0f) < .01f);

	m11 = 1.0f - n.x*n.x;
	m22 = 1.0f - n.y*n.y;
	m33 = 1.0f - n.z*n.z;

	m12 = m21 = -n.x*n.y;
	m13 = m31 = -n.x*n.z;
	m23 = m32 = -n.y*n.z;

	tx = ty = tz = 0.0f;
}
//构造反射矩阵
//1 沿x=k平面反射
//2 沿yk
//3 沿z=k
void Matrix4x3::setupReflect(int axis, float k /*= 0.0f*/)
{
	switch (axis)
	{
	case 1://x
		m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 2.0f*k;
		ty = 0.0f;
		tz = 0.0f;

		break;
	case 2://y
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 0.0f;
		ty = 2.0f*k;
		tz = 0.0f;

		break;
	case 3://z
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;

		tx = 0.0f;
		ty = 0.0f;
		tz = 2.0f*k;

		break;
	default:
		assert(false);
	}
}
// 构造反射矩阵
void Matrix4x3::setupReflect(const Vector3&n)
{
	assert(fabs(n*n - 1.0f) < .01f);

	float ax = -2.0f*n.x;
	float ay = -2.0f*n.y;
	float az = -2.0f*n.z;

	m11 = 1.0f + ax*n.x;
	m22 = 1.0f + ay*n.y;
	m33 = 1.0f + az*n.z;

	m12 = m21 = ax*n.y;
	m13 = m31 = ax*n.z;
	m23 = m32 = ay*n.z;

	tx = ty = tz = 0.0f;
}

Vector3 operator*(const Vector3&p, const Matrix4x3 &m)
{
	return Vector3(
		p.x*m.m11 + p.y*m.m21 + p.z*m.m31 + m.tx,
		p.x*m.m12 + p.y*m.m22 + p.z*m.m32 + m.ty,
		p.x*m.m13 + p.y*m.m23 + p.z*m.m33 + m.tz
		);
}

Matrix4x3 operator*(const Matrix4x3&a, const Matrix4x3&b)
{
	Matrix4x3 r;
	r.m11 = a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31;
	r.m12 = a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m33;
	r.m13 = a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33;

	r.m21 = a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31;
	r.m22 = a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32;
	r.m23 = a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33;

	r.m31 = a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31;
	r.m32 = a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32;
	r.m33 = a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33;

	//计算平移部分
	r.tx = a.tx*b.m11 + a.ty*b.m21 + a.tz*b.m31 + b.tx;
	r.ty = a.tx*b.m12 + a.ty*b.m22 + a.tz*b.m32 + b.ty;
	r.tz = a.tx*b.m13 + a.ty*b.m23 + a.tz*b.m33 + b.tz;

	return r;
}

Vector3 &operator*=(Vector3&p, const Matrix4x3 &m)
{
	p = p*m;
	return p;
}

Matrix4x3 &operator*=(Matrix4x3&a, const Matrix4x3&m)
{
	a = a*m;
	return a;
}

float determinant(const Matrix4x3&m)
{
	return
		m.m11*(m.m22*m.m33 - m.m23*m.m32)
		+ m.m12*(m.m23*m.m31 - m.m21*m.m33)
		+ m.m13*(m.m21*m.m32 - m.m22*m.m31);
}

Matrix4x3 inverse(const Matrix4x3&m)
{
	//计算行列式
	float det = determinant(m);
	//如果是奇异的 行列式为零 没有逆矩阵
	assert(fabs(det) > 0.000001f);
	//计算1/行列式
	float oneOverDet = 1.0f / det;
	//计算3x3部分的逆 用伴随矩阵除以行列式
	Matrix4x3 r;
	r.m11 = (m.m22*m.m33 - m.m23*m.m32)*oneOverDet;
	r.m12 = (m.m13*m.m32 - m.m12*m.m33)*oneOverDet;
	r.m13 = (m.m12*m.m23 - m.m13*m.m22)*oneOverDet;

	r.m21 = (m.m23*m.m31 - m.m21*m.m33)*oneOverDet;
	r.m22 = (m.m11*m.m33 - m.m13*m.m31)*oneOverDet;
	r.m23 = (m.m13*m.m21 - m.m11*m.m23)*oneOverDet;

	r.m31 = (m.m21*m.m32 - m.m22*m.m31)*oneOverDet;
	r.m32 = (m.m12*m.m31 - m.m11*m.m32)*oneOverDet;
	r.m33 = (m.m11*m.m22 - m.m12*m.m21)*oneOverDet;

	//计算平移部分的逆
	r.tx = -(m.tx*m.m11 + m.ty*m.m21 + m.tz*m.m31);
	r.ty = -(m.tx*m.m12 + m.ty*m.m22 + m.tz*m.m32);
	r.tz = -(m.tx*m.m13 + m.ty*m.m23 + m.tz*m.m33);

	return r;
}

Vector3 getTranslation(const Matrix4x3&m)
{
	return Vector3(m.tx, m.ty, m.tz);
}

Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3&m)
{
	return Vector3(
		-(m.tx*m.m11 + m.ty*m.m12 + m.tz*m.m13),
		-(m.tx*m.m21 + m.ty*m.m22 + m.tz*m.m23),
		-(m.tx*m.m31 + m.ty*m.m32 + m.tz*m.m33)
	);
}

Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3&m)
{
	return Vector3(m.tx, m.ty, m.tz);
}
