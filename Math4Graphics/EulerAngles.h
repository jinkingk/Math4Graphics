//////////////////////////////////////////////////
//EulerAngles.h
//////////////////////////////////////////////////
#pragma once
//预声明
class Quaternion;
class Matrix4x3;
class RotationMatrix;

//------------------------------------------------
//class EulerAngles
//
//该类用于表达heading-pitch-bank欧拉角系统
class EulerAngles
{
public:
	//公共数据
	//直接的表示方式
	//用弧度保存三个角度
	float heading;
	float pitch;
	float bank;
	//公共操作
	//缺省构造函数
	EulerAngles() {}
	//接受三个参数的构造函数
	EulerAngles(float h, float p, float b) :heading(h), pitch(p), bank(b) {}
	//置零
	void indetity() { heading = pitch = bank = 0; }
	//变为限制级欧拉角
	void canonize();
	//从四元数到欧拉角
	void fromObjectToIntertialQuaternion(const Quaternion &q);
	void fromIntertialToObjectQuaternion(const Quaternion &q);

	//从矩阵转换到欧拉角
	//平移部分被省略，并假设矩阵是正交的
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	//从旋转矩阵转到欧拉角
	void fromRotationMatrix(const RotationMatrix &m);
};
//全局的“单位”欧拉角
extern const EulerAngles kEulerAnglesIdentity;
////////////////////////////////////////////////////////////////