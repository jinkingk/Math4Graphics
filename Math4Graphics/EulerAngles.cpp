//////////////////////////////////////////////
//EulerAngles.cpp
//////////////////////////////////////////////

#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

//ȫ�ֵ�λŷ������
const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

///////////////////////////////////
//class EulerAngles Implementation
///////////////////////////////////

//--------------------------------------------------
//EulerAngles::canonize
//��ŷ����ת�������Ƽ���

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

	//���������
	if (fabs(pitch)>kPiOver2-1e-4)
	{
		//���������н������ƴ�ֱ�����ת����heading
		heading += bank;
		bank = 0.0f;
	}else {
		//��������
		bank = wrapPi(bank);
	}

	heading = wrapPi(heading);
}
//������-������Ԫ����ŷ����
void EulerAngles::fromObjectToIntertialQuaternion(const Quaternion &q)
{
	//����sin(pitch)
	float sp = -2.0f * (q.y*q.z - q.w*q.x);

	//���������
	if (fabs(sp) > 0.9999f){
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;
		//bank ���� ����heading
		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}else{
		//����Ƕ�
		pitch = asin(sp);
		heading = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//�ӹ���-������Ԫ����ŷ����
void EulerAngles::fromIntertialToObjectQuaternion(const Quaternion &q)
{
	//����sin(pitch)
	float sp = -2.0f * (q.y*q.z + q.w*q.x);

	//���������
	if (fabs(sp) > 0.9999f) {
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;
		//bank ���� ����heading
		heading = atan2(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}
	else {
		//����Ƕ�
		pitch = asin(sp);
		heading = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//��������������ϵ�任����ŷ����
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m)
{
	//ͨ��m32���� sin(pitch)
	float sp = -m.m32;
	//���������
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank ���� ����heading
		heading = atan2(-m.m23, m.m11);
		bank = 0.0f;
	}else{
		//����Ƕ�
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}
//������-��������ϵ�任����ŷ����
void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m)
{
	//ͨ��m23���� sin(pitch)
	float sp = -m.m23;
	//���������
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank ���� ����heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		//����Ƕ�
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}
//������ת������ŷ����
void EulerAngles::fromRotationMatrix(const RotationMatrix &m)
{
	//ͨ��m23���� sin(pitch)
	float sp = -m.m23;
	//���������
	if (fabs(sp) > 9.99999f) {
		pitch = kPiOver2 *sp;
		//bank ���� ����heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		//����Ƕ�
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}
