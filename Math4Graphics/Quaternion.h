///////////////////////////////////////////////////////////
//Quaternion.h
///////////////////////////////////////////////////////////
#pragma once
class Vector3;
class EulerAngles;

//---------------------------------------------------------
//Quaternion��
//ʵ����3D�б��ֽ�λ�Ƶ���Ԫ��

class Quaternion
{
public:
	//��������
	float w, x, y, z;

	//��������
	//��Ϊ��λ��Ԫ��
	void indetity() { w = x = y = z = 0; }

	//����ִ����ת����Ԫ��
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3 &axis,float theta);

	//����ִ������-������ת����Ԫ��
	void setToRotateObjectToInertial(const EulerAngles &orientation);
	void setToRotateInertialToObject(const EulerAngles &orientation);

	//���
	Quaternion operator * (const Quaternion &a) const;

	//��ֵ�˷�
	Quaternion& operator *= (const Quaternion &a);

	//����
	void normalize();

	//��ȡ��ת�Ǻ���ת��
	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
};

//ȫ�֡���λ����Ԫ��
extern const Quaternion kQuaternionIdentity;

//��Ԫ�����
extern float dotProduct(const Quaternion &a, const Quaternion &b);

//�������Բ�ֵ
extern Quaternion slerp(const Quaternion &a, const Quaternion &b, float t);

//��Ԫ������
extern Quaternion conjugate(const Quaternion&q);

//��Ԫ����
extern Quaternion pow(const Quaternion&q, float exponent);
