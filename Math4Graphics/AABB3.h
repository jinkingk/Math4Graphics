////////////////////////////////////////////////////
//AABB3.h
////////////////////////////////////////////////////
#pragma once

#include "Vector3.h"
class Matrix4x3;
//------------------------------------------------------------
//实现3D轴对齐矩形边界框类
class AABB3
{
public:
	//公共数据
	//最小点和最大点
	Vector3 min;
	Vector3 max;
	//查询各种参数
	Vector3 size() { return max - min; }
	float xSize() { return max.x - min.x; }
	float ySize() { return max.y - min.y; }
	float zSize() {return max.z-min.z;}
	Vector3 center() { return (min + max)*.5f; };
	//提取8各顶点中的一个
	Vector3 corner(int i) const;
	//矩形边界框
	//清空
	void empty();
	//添加点
	void add(const Vector3&p);
	//添加AABB
	void add(const AABB3 &box);
	//变换矩形边界框 计算新AABB
	void setToTransformedBox(const AABB3 &box, const Matrix4x3 &m);
	//包含相交性测试
	//矩形边界框为空
	bool isEmpty() const;
	//包含某点
	bool contains(const Vector3 &p) const;
	//返回边界框上的最近点
	Vector3 closestPointTo(const Vector3 &p) const;
	//和球相交
	bool intersectsSphere(const Vector3&center, float radius) const;
	//和参数射线相交
	float rayIntersect(const Vector3&rayOrq, const Vector3 &rayDelta,
		Vector3 *returnNormal = 0) const;
	//边界框在平面的哪一面
	int classifyPlane(const Vector3 &n, float d) const;
	//和平面的动态相交性测试
	float intersectPlane(const Vector3 &n, float planeD, const Vector3&dir) const;
};
//检测两个AABB的相交性
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2,AABB3 *boxIntersect=0);
//返回运动AABB和静止AABB相交时的参数点
float intersectMovingAABB(
	const AABB3 &stationaryBox,
	const AABB3 &movingBox,
	const Vector3 &d
);