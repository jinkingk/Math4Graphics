//////////////////////////////////////////////////////
//RotationMatirx.h
//////////////////////////////////////////////////////
#pragma once
class Vector3;
class EulerAngles;
class Quaternion;

//--------------------------------------------------------------
//RotationMatrix��


class RotationMatrix
{
public:
	//public data
	//����9ֵ
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;

	//��������

	//��Ϊ��λ����
	void identity();

	//����ָ���ķ�λ�������
	void setup(const EulerAngles &orientation);

	//������Ԫ���������,�������Ԫ����������ָ������ı仯
	void fromInertialToObjectQuaternion(const Quaternion &q);

	void fromObjectToInertialQuaternion(const Quaternion &q);

	//ִ����ת
	Vector3 inertialToObject(const Vector3 &v) const;
	Vector3 objectToInertial(const Vector3 &v) const;
};
///////////////////////////////////////////////////////////////////