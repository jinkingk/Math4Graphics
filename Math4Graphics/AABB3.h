////////////////////////////////////////////////////
//AABB3.h
////////////////////////////////////////////////////
#pragma once

#include "Vector3.h"
class Matrix4x3;
//------------------------------------------------------------
//ʵ��3D�������α߽����
class AABB3
{
public:
	//��������
	//��С�������
	Vector3 min;
	Vector3 max;
	//��ѯ���ֲ���
	Vector3 size() { return max - min; }
	float xSize() { return max.x - min.x; }
	float ySize() { return max.y - min.y; }
	float zSize() {return max.z-min.z;}
	Vector3 center() { return (min + max)*.5f; };
	//��ȡ8�������е�һ��
	Vector3 corner(int i) const;
	//���α߽��
	//���
	void empty();
	//��ӵ�
	void add(const Vector3&p);
	//���AABB
	void add(const AABB3 &box);
	//�任���α߽�� ������AABB
	void setToTransformedBox(const AABB3 &box, const Matrix4x3 &m);
	//�����ཻ�Բ���
	//���α߽��Ϊ��
	bool isEmpty() const;
	//����ĳ��
	bool contains(const Vector3 &p) const;
	//���ر߽���ϵ������
	Vector3 closestPointTo(const Vector3 &p) const;
	//�����ཻ
	bool intersectsSphere(const Vector3&center, float radius) const;
	//�Ͳ��������ཻ
	float rayIntersect(const Vector3&rayOrq, const Vector3 &rayDelta,
		Vector3 *returnNormal = 0) const;
	//�߽����ƽ�����һ��
	int classifyPlane(const Vector3 &n, float d) const;
	//��ƽ��Ķ�̬�ཻ�Բ���
	float intersectPlane(const Vector3 &n, float planeD, const Vector3&dir) const;
};
//�������AABB���ཻ��
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2,AABB3 *boxIntersect=0);
//�����˶�AABB�;�ֹAABB�ཻʱ�Ĳ�����
float intersectMovingAABB(
	const AABB3 &stationaryBox,
	const AABB3 &movingBox,
	const Vector3 &d
);