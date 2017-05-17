//////////////////////////////////////////////////
//EulerAngles.h
//////////////////////////////////////////////////
#pragma once
//Ԥ����
class Quaternion;
class Matrix4x3;
class RotationMatrix;

//------------------------------------------------
//class EulerAngles
//
//�������ڱ��heading-pitch-bankŷ����ϵͳ
class EulerAngles
{
public:
	//��������
	//ֱ�ӵı�ʾ��ʽ
	//�û��ȱ��������Ƕ�
	float heading;
	float pitch;
	float bank;
	//��������
	//ȱʡ���캯��
	EulerAngles() {}
	//�������������Ĺ��캯��
	EulerAngles(float h, float p, float b) :heading(h), pitch(p), bank(b) {}
	//����
	void indetity() { heading = pitch = bank = 0; }
	//��Ϊ���Ƽ�ŷ����
	void canonize();
	//����Ԫ����ŷ����
	void fromObjectToIntertialQuaternion(const Quaternion &q);
	void fromIntertialToObjectQuaternion(const Quaternion &q);

	//�Ӿ���ת����ŷ����
	//ƽ�Ʋ��ֱ�ʡ�ԣ������������������
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	//����ת����ת��ŷ����
	void fromRotationMatrix(const RotationMatrix &m);
};
//ȫ�ֵġ���λ��ŷ����
extern const EulerAngles kEulerAnglesIdentity;
////////////////////////////////////////////////////////////////