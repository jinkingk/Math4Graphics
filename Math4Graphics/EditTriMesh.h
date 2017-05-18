#pragma once
#include "Vector3.h"
class Matrix4x3;

class EditTriMesh {
public:
	//局部类型
	//Vertex 类用于保存顶点信息
	class Vectex {
	public:
		Vectex() { setDefaults(); }
		void setDefaults();
		//3D坐标
		Vector3 p;
		//纹理坐标
		float u, v;
		//顶点级表面法向
		Vector3 normal;
		//工具变量
		int mark;
	};
	//类Tri用于保存三角形信息
	class Tri {
		Tri() { setDefaults(); }
		void setDefaults();
		//面顶点
		struct Vert {
			int index;//顶点列表的索引
			float u, v;//纹理坐标
		};
		Vert v[3];
		//表面法向量
		Vector3 normal;
		//属于网格的哪部分
		int part;
		//材质列表索引
		int material;
		//工具变量
		int mark;
		//判断是否为退化三角形 
		bool isDegenerate() const;
		//返回顶点索引
		int findVertex(int vertexIndex) const;
	};
	//材质
	class Material {
	public:
		Material() { setDefaults(); }
		void setDefaults();
		char diffuseTextureName[256];
		//工具变量
		int mark;
	};
	//控制优化的选项
	class OptimationParameters {
	public:
		OptimationParameters(){ setDefaults();}
		void setDefaults();

		float coincidentVertexTolerance;
		float cosOfEdgeAngleTolerance;
		void setEdgeAngleToleranceInDegrees(float degrees);
	};
	//标准类操作
	EditTriMesh();
	EditTriMesh(const EditTriMesh&x);
	~EditTriMesh();

private:
	//网格列表
	int vAlloc;
	int vCount;
	Vectex *vList;
	int tAlloc;
	int tCount;
	Tri *tList;
	int mCount;
	Material *mList;
	//实现细节
	void construct();
};