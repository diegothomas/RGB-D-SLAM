/* Author: Diego Thomas
 * Date Created: 3/11/2013
 * Last Modification: 3/11/2013
 * All classes needed for the reconstruction using graph of primitives
 */

#ifndef __GRAPH_H
#define __GRAPH_H

#include "Primitive.h"

using namespace std;

/*
The class Graph defines the topology graph structure which is our global model
*/
class Graph {
private:
	unsigned short **_data_Bump_dev;
	unsigned char **_data_RGB_dev;
	unsigned char **_data_Mask_dev;

public:
	// The set of nodes
	vector<Primitive *> _nodes;
		
	int _SizeVBUFF [2];

	int _curr_ind;

	Eigen::MatrixXf _pose;
	
	// Total number of primitives
	int _Size;

	unsigned short **_BumpTab;
	unsigned char **_RGBTab;
	unsigned char **_MaskTab;
	float **_ParamTab;
	
	// constructor empty graph
	Graph(): _Size(0) {

		_SizeVBUFF [0] = 0;
		_SizeVBUFF [1] = 0;

		_curr_ind = 0;
		_pose = Eigen::Matrix4f::Identity();

		_nodes.clear();
	};

	// constructor
	Graph(int frag_n, int frag_m): _Size(0) {

		_SizeVBUFF [0] = frag_n;
		_SizeVBUFF [1] = frag_m;

		_curr_ind = 0;
		_pose = Eigen::Matrix4f::Identity();

		_nodes.clear();

		_data_Bump_dev = (unsigned short **) malloc(50*sizeof(unsigned short *));
		_data_RGB_dev = (unsigned char **) malloc(50*sizeof(unsigned char *));
		_data_Mask_dev = (unsigned char **) malloc(50*sizeof(unsigned char *));

		for (int i = 0; i < 50; i++) {
			checkCudaErrors( cudaMalloc((void**)&_data_Bump_dev[i], 3*frag_m*frag_n * sizeof(unsigned short)) );	
			checkCudaErrors( cudaMalloc((void**)&_data_RGB_dev[i], 3*frag_m*frag_n * sizeof(unsigned char)) );	
			checkCudaErrors( cudaMalloc((void**)&_data_Mask_dev[i], frag_m*frag_n * sizeof(unsigned char)) );		
		}

		size_t available, total;
		cudaMemGetInfo(&available, &total);
		cout << "Available memory: " << available << "Total Memory: " << total << endl;

	};
	
	// destructor
	~Graph(){	

		for (vector<Primitive *>::iterator it =_nodes.begin(); it != _nodes.end(); it++)
			delete (*it);

		_nodes.clear();
		_Size = 0;

		for (int i = 0; i < 50; i++) {
			checkCudaErrors( cudaFree(_data_Bump_dev[i]) );
			checkCudaErrors( cudaFree(_data_RGB_dev[i]) );
			checkCudaErrors( cudaFree(_data_Mask_dev[i]) );
		}

		free(_data_Bump_dev);
		free(_data_RGB_dev);
		free(_data_Mask_dev);
	};

	inline bool AddPrimitive(Primitive *Prim) {
		int idx = _nodes.size();

		if (idx > 49) {
			cout << "Error  not enough place to add the primitive" << endl;
			_nodes.push_back(NULL);
			return false;
		}

		Prim->_Bump_dev = _data_Bump_dev[idx];
		Prim->_RGB_dev = _data_RGB_dev[idx];
		Prim->_Mask_dev = _data_Mask_dev[idx];
		checkCudaErrors( cudaMemset(Prim->_Mask_dev, 0, _SizeVBUFF [0] * _SizeVBUFF [1] * sizeof(unsigned char)) );

		_nodes.push_back(Prim);
		return true;
	};

	void Update(PredictedFrame *frame, float *pose);

	void save(char *filename, int indx);
	
	bool exist(Primitive *Prim_in, float *nmle, float dist, float *center);

	int included(Primitive *Prim_in, float thresh, int *nbPts, bool GPU = true);

	void Clean();
	
};


#endif