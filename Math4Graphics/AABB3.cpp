//////////////////////////////////////////////////////
//AABB3.cpp
//////////////////////////////////////////////////////
#include <assert.h>
#include <stdlib.h>

#include "AABB3.h"
#include "Matrix4x3.h"
#include "CommonStuff.h"

Vector3 AABB3::corner(int i) const
{
	assert(i >= 0);
	assert(i <= 7);
	return Vector3(
		(i & 1) ? max.x : min.x,
		(i & 2) ? max.y : min.y,
		(i & 4) ? max.z : min.z
	);
}
//将值赋为极大值/极小值以清空矩形边框
void AABB3::empty()
{
	const float kBigNumber = 1e37f;
	min.x = min.y = min.z = kBigNumber;
	max.x = max.y = max.z = -kBigNumber;
}
//向矩形边界框中加一个点
void AABB3::add(const Vector3&p)
{
	//必要的时候扩张矩形边界框以包含这个点
	if (p.x < min.x) min.x = p.x;
	if (p.x > max.x) max.x = p.x;
	if (p.y < min.y) min.y = p.y;
	if (p.y > max.y) max.y = p.y;
	if (p.z < min.z) min.z = p.z;
	if (p.z > max.z) max.z = p.z;
}
//向矩形边界框中添加AABB
void AABB3::add(const AABB3 &box)
{
	if (box.min.x < min.x) min.x = box.min.x;
	if (box.max.x > max.x) max.x = box.max.x;
	if (box.min.y < min.y) min.y = box.min.y;
	if (box.max.y > max.y) max.y = box.max.y;
	if (box.min.z < min.z) min.z = box.min.z;
	if (box.max.z > max.z) max.z = box.max.z;
}
//变换矩阵边界框并计算新的AABB
void AABB3::setToTransformedBox(const AABB3 &box, const Matrix4x3 &m)
{
	if (box.isEmpty())
	{
		empty();
		return;
	}
	//从平移部分开始
	min = max = getTranslation(m);
	//以此检查矩阵的9个元素,计算新的AABB
	if (m.m11>0.0f){
		min.x += m.m11 * box.min.x; max.x += m.m11*box.max.x;
	}
	else{
		min.x += m.m11 * box.max.x; max.x += m.m11*box.min.x;
	}
	if (m.m12 > 0.0f) {
		min.y += m.m12 * box.min.x; max.y += m.m12*box.max.x;
	}
	else {
		min.y += m.m12 * box.max.x; max.y += m.m12*box.min.x;
	}
	if (m.m13 > 0.0f) {
		min.z += m.m13 * box.min.x; max.z += m.m13*box.max.x;
	}
	else {
		min.z += m.m13 * box.max.x; max.z += m.m13*box.min.x;
	}

	if (m.m21 > 0.0f) {
		min.x += m.m21 * box.min.y; max.x += m.m21*box.max.y;
	}
	else {
		min.x += m.m21 * box.max.y; max.x += m.m21*box.min.y;
	}
	if (m.m22 > 0.0f) {
		min.y += m.m22 * box.min.y; max.y += m.m22*box.max.y;
	}
	else {
		min.y += m.m22 * box.max.y; max.y += m.m22*box.min.y;
	}
	if (m.m23 > 0.0f) {
		min.z += m.m23 * box.min.y; max.z += m.m23*box.max.y;
	}
	else {
		min.z += m.m23 * box.max.y; max.z += m.m23*box.min.y;
	}

	if (m.m31 > 0.0f) {
		min.x += m.m31 * box.min.z; max.x += m.m31*box.max.z;
	}
	else {
		min.x += m.m31 * box.max.z; max.x += m.m31*box.min.z;
	}
	if (m.m32 > 0.0f) {
		min.y += m.m32 * box.min.z; max.y += m.m32*box.max.z;
	}
	else {
		min.y += m.m32 * box.max.z; max.y += m.m32*box.min.z;
	}
	if (m.m33 > 0.0f) {
		min.z += m.m33 * box.min.z; max.z += m.m33*box.max.z;
	}
	else {
		min.z += m.m33 * box.max.z; max.z += m.m33*box.min.z;
	}
}

bool AABB3::isEmpty() const
{
	//检查是否在某个轴上反过来了
	return (min.x > max.x) || (min.y > max.y) || (min.z > max.z);
}

bool AABB3::contains(const Vector3 &p) const
{
	return
		(p.x >= min.x) && (p.x <= max.x) &&
		(p.y >= min.y) && (p.y <= max.y) &&
		(p.z >= min.z) && (p.z <= max.z);
}
//返回距离AABB上最近的点
Vector3 AABB3::closestPointTo(const Vector3 &p) const
{
	//在每一维上将p"推向"矩形边界框
	Vector3 r;
	if (p.x < min.x) {
		r.x = min.x;
	}else if(p.x>max.x){
		r.x = max.x;
	}else{
		r.x = p.x;
	}

	if (p.y < min.y) {
		r.y = min.y;
	}
	else if (p.y > max.y) {
		r.y = max.y;
	}
	else {
		r.y = p.y;
	}

	if (p.z < min.z) {
		r.z = min.z;
	}
	else if (p.z > max.z) {
		r.z = max.z;
	}
	else {
		r.z = p.z;
	}

	return r;
}

bool AABB3::intersectsSphere(const Vector3&center, float radius) const
{
	//找到矩形边界框上距球心最近的点
	Vector3 closestPoint = closestPointTo(center);
	//检查最近点和球心的距离是否小于半径
	return distanceSquared(center, closestPoint) < radius*radius;
}

float AABB3::rayIntersect(const Vector3&rayOrq, const Vector3 &rayDelta, Vector3 *returnNormal /*= 0*/) const
{
	//todo
}
//静止的AABB与平面相交检测
int AABB3::classifyPlane(const Vector3 &n, float d) const
{
	//检查向量法 计算最小和最大的D 即距离
	float minD, maxD;
	if (n.x > 0.0f) {
		minD = n.x*min.x; maxD = n.x*max.x;
	}else{
		minD = n.x*max.x; maxD = n.x*min.x;
	}
	if (n.y > 0.0f) {
		minD += n.y*min.y; maxD += n.y*max.y;
	}
	else {
		minD += n.y*max.y; maxD += n.y*min.y;
	}
	if (n.z > 0.0f) {
		minD += n.z*min.z; maxD += n.z*max.z;
	}
	else {
		minD += n.z*max.z; maxD += n.z*min.z;
	}
	//前面
	if (minD >= d)
		return 1;
	//背面
	if (maxD <= d)
		return -1;
	//横跨平面
	return 0;
}

float AABB3::intersectPlane(const Vector3 &n, float planeD, const Vector3&dir) const
{
	//TODO
}

//////////////////////////////////////////////////////
//非成员函数
//////////////////////////////////////////////////////

//检测两个AABB相交
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2, AABB3 *boxIntersect/*=0*/)
{
	//判断是否有重叠
	if (box1.min.x > box2.max.x) return false;
	if (box1.max.x < box2.min.x) return false;
	if (box1.min.y > box2.max.y) return false;
	if (box1.max.y < box2.min.y) return false;
	if (box1.min.z > box2.max.z) return false;
	if (box1.max.z < box2.min.z) return false;
	//有重叠，计算重叠部分的AABB
	if (boxIntersect != NULL) {
		boxIntersect->min.x = max(box1.min.x, box2.min.x);
		boxIntersect->max.x = min(box1.max.x, box2.max.x);
		boxIntersect->min.y = max(box1.min.y, box2.min.y);
		boxIntersect->max.y = min(box1.max.y, box2.max.y);
		boxIntersect->min.z = max(box1.min.z, box2.min.z);
		boxIntersect->max.z = min(box1.max.z, box2.max.z);
	}
	return true;
}

float intersectMovingAABB(const AABB3 &stationaryBox, const AABB3 &movingBox, const Vector3 &d)
{
	//TODO
}
