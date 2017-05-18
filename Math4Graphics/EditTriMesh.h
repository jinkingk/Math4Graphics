#pragma once
#include "Vector3.h"
class Matrix4x3;

class EditTriMesh {
public:
	//�ֲ�����
	//Vertex �����ڱ��涥����Ϣ
	class Vectex {
	public:
		Vectex() { setDefaults(); }
		void setDefaults();
		//3D����
		Vector3 p;
		//��������
		float u, v;
		//���㼶���淨��
		Vector3 normal;
		//���߱���
		int mark;
	};
	//��Tri���ڱ�����������Ϣ
	class Tri {
		Tri() { setDefaults(); }
		void setDefaults();
		//�涥��
		struct Vert {
			int index;//�����б������
			float u, v;//��������
		};
		Vert v[3];
		//���淨����
		Vector3 normal;
		//����������Ĳ���
		int part;
		//�����б�����
		int material;
		//���߱���
		int mark;
		//�ж��Ƿ�Ϊ�˻������� 
		bool isDegenerate() const;
		//���ض�������
		int findVertex(int vertexIndex) const;
	};
	//����
	class Material {
	public:
		Material() { setDefaults(); }
		void setDefaults();
		char diffuseTextureName[256];
		//���߱���
		int mark;
	};
	//�����Ż���ѡ��
	class OptimationParameters {
	public:
		OptimationParameters(){ setDefaults();}
		void setDefaults();

		float coincidentVertexTolerance;
		float cosOfEdgeAngleTolerance;
		void setEdgeAngleToleranceInDegrees(float degrees);
	};
	//��׼�����
	EditTriMesh();
	EditTriMesh(const EditTriMesh&x);
	~EditTriMesh();

private:
	//�����б�
	int vAlloc;
	int vCount;
	Vectex *vList;
	int tAlloc;
	int tCount;
	Tri *tList;
	int mCount;
	Material *mList;
	//ʵ��ϸ��
	void construct();
};