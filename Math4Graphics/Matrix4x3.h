//////////////////////////////////////////////////////////////
//Matrix4x3.h
//////////////////////////////////////////////////////////////
#pragma once

class Vector3;
class EulerAngles;
class Quaternion;
class RotationMatrix;

class Matrix4x3
{
public:
	//公共数据

	//矩阵的值
	//3x3线性变换 最后一行平移
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	float tx, ty, tz;

	//公共操作
	//置为单位矩阵
	void identity();

	//直接访问平移部分

	void zeroTranslation();
	void setTranslation(const Vector3&d);
	void setupTrainslation(const Vector3&d);

	//构造父空间-局部空间变换的矩阵

	void setupLocalToParent(const Vector3 &pos, const EulerAngles&orient);
	void setupLocalToParent(const Vector3 &pos, const RotationMatrix&orient);
	void setupParentToLocal(const Vector3 &pos, const EulerAngles&orient);
	void setupParentToLocal(const Vector3 &pos, const RotationMatrix&orient);

	//构造绕坐标轴旋转的矩阵
	void setupRotate(int axis, float theta);

	//构造然任意轴旋转的矩阵
	void setupRotate(const Vector3&axis, float theta);

	//构造旋转矩阵，角位移由四元数给出
	void fromQuaternion(const Quaternion&q);

	//构造沿坐标轴缩放的矩阵
	void setupScale(const Vector3&s);

	//构造沿任意轴缩放的矩阵
	void setupScaleAlongAxis(const Vector3&axis, float k);

	//构造切变矩阵
	void setupSphear(int axis, float s, float t);

	//构造投影矩阵，投影过平面原点
	void setupProject(const Vector3&n);

	//构造反射矩阵
	void setupReflect(int axis, float k = 0.0f);

	//构造沿任意平面反射的矩阵
	void setupReflect(const Vector3&n);

};

//运算符*用来变换点或连接矩阵

Vector3 operator*(const Vector3&p, const Matrix4x3 &m);
Matrix4x3 operator*(const Matrix4x3&a, const Matrix4x3&b);

//运算符*=
Vector3 &operator*=(Vector3&p, const Matrix4x3 &m);
Matrix4x3 &operator*=(Matrix4x3&a, const Matrix4x3&m);

//计算3x3部分的行列式值
float determinant(const Matrix4x3&m);

//计算矩阵的逆
Matrix4x3 inverse(const Matrix4x3&m);

//提取矩阵的平移部分
Vector3 getTranslation(const Matrix4x3&m);

//取位置/方位
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3&m);
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3&m);