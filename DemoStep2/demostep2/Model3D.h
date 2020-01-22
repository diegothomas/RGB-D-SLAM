#ifndef __MODEL3D_H
#define __MODEL3D_H

#pragma once

#include "Primitive.h"

struct Group {
	vector<int> _indices;
	// Normal vector
	float _nmle [3];
	// Distance to origine
	float _dist;

	float _colorGroup[3];
};

class Model3D {
private:
	/* The list of all primitives */
	vector<PlanarPatch *> _AllPrim;
		
	/* The list of current primitives to be rendered*/
	vector<PlanarPatch *> _ViewedPrim; /// ??
	
	/* The list of working primitives*/
	vector<PlanarPatch *> _WorkingPrim; /// ??
	
	/* The list of old primitives*/
	vector<Primitive *> _OldPrim; /// ??

	/* The list of merged primitives*/
	vector<PlanarPatch *> _MergedPrim; /// ??
	
	/* The list of keyframes*/
	vector<KeyFrame *> _Keyframes;

	/* The list of Groups*/
	vector<Group *> _Groups;

	/* The optimizable graph*/
	SparseOptimizer _optimizer;

	char _path[100];
	int _indx;
	Eigen::MatrixXf _pose;
	bool _Merged;

public:
	
	/* Constructor */
	Model3D() {
		_Merged = false;
		size_t available, total;
		cudaMemGetInfo(&available, &total);
		cout << "Available memory: " << available << "Total Memory: " << total << endl;
	};
	
	Model3D(char *path) {
		_Merged = false;
		sprintf(_path, "%s", path);
		_indx = 0;
		_pose = Eigen::Matrix4f::Identity();

		SlamLinearSolver* linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
		OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
		_optimizer.setAlgorithm(solver);
		
		size_t available, total;
		cudaMemGetInfo(&available, &total);
		cout << "Available memory: " << available << "Total Memory: " << total << endl;
	};
	
	/* Empty destructor */
	~Model3D() {
		for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++)
			delete (*it);
		_AllPrim.clear();

		_ViewedPrim.clear();
		_WorkingPrim.clear();
		_OldPrim.clear();
		
		_optimizer.clear();
	};

	void AddKeyFrame(KeyFrame *Key_in);

	void AddPrim(PlanarPatch *Prim);

	void AddGeoCnstr();

	void AddIdCnstr(vector <PlanarPatch *> Set1, vector <PlanarPatch *> Set2);

	void Draw(VECTOR3D objectLightPosition, bool light, bool color) {
		if (!_MergedPrim.empty()) {
			for (vector<PlanarPatch *>::iterator it = _MergedPrim.begin(); it != _MergedPrim.end(); it++)
				(*it)->DrawMeshBump(objectLightPosition, light, color, light);
		} else {
			for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++)
				(*it)->DrawMeshBump(objectLightPosition, light, color, false);
		}
	}

	void DrawIdx() {
		int i = 0;
		for (vector<PlanarPatch *>::iterator it = _MergedPrim.begin(); it != _MergedPrim.end(); it++) {
			(*it)->DrawIdx(i);
			i++;
		}
	}

	void DrawPrim(VECTOR3D objectLightPosition, bool light, bool color, int indx){
		_MergedPrim[indx]->DrawMeshBump(objectLightPosition, light, color, true);
	}
	
	void DrawPrimFlat(int indx){
		_MergedPrim[indx]->DrawMeshFlat();
	}

	void DrawGraph(bool keyf);

	void LoadModel3D(char *filename);

	void UpdateWorking(float x, float y, float z, float lx, float ly, float lz, char *filename);

	void UpdateVisible(float x, float y, float z, float lx, float ly, float lz);
	
	void Update();

	void FragmentRegistration();

	void UpdateGroups();

	void Merge();

	void OptimizeGraph(vector <int> Set1, vector <PlanarPatch *> Set2, KeyFrame *MatchKey, KeyFrame *RefKey);

	/////////////// Accessors ////////////////////////////////////////////////////////////////////////
	inline int Size() {return _MergedPrim.size();};
	inline void setInvisible(int indx){_MergedPrim[indx]->SetView(false);}
	inline void ReInit(){
		for (vector<PlanarPatch *>::iterator it = _MergedPrim.begin(); it != _MergedPrim.end(); it++) {
			(*it)->SetView(true);
		}
	}
	inline void RubbPrim(int indx, int pix_i, int pix_j, int s = 2) {_MergedPrim[indx]->Rubb(pix_i, pix_j, s);}
	inline void DrawPrim(int indx, int pix_i, int pix_j, int s = 2) {_MergedPrim[indx]->Draw(pix_i, pix_j, s);}
	inline void ExtrudePrim(int indx, int pix_i, int pix_j, int s = 2) {_MergedPrim[indx]->Extrude(pix_i, pix_j, s);}
	inline void DigPrim(int indx, int pix_i, int pix_j, int s = 2) {_MergedPrim[indx]->Dig(pix_i, pix_j, s);}
	inline void ComputeTriangulation(int indx){_MergedPrim[indx]->ComputeTriangulation();}
	inline void UpdateTextures(int indx){_MergedPrim[indx]->UpdateTextures();}

};


#endif