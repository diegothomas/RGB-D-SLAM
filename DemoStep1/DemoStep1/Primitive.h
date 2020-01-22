/* Author: Diego Thomas
 * Date Created: 3/11/2013
 * Last Modification: 5/9/2013
 * All classes needed for the reconstruction using graph of primitives
 */

#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include "Frame.h"

#define RES_PLANE 0.004

/*
The class Primitive defines the primitive respresentation
*/
class Primitive {
public:
	// The bump map
	unsigned short *_Bump;
	// The RGB map
	unsigned char *_RGB;
	unsigned char *_RGB_buff;
	// The Mask (mask == 10 means 0 confidence) !!!
	unsigned short *_Mask;
	unsigned char *_Mask_char;
	bool *_Mask_grad;
	float *_VMap;
	float *_NMap;

	//********** Parameters of the primitive ******************/
	float *_param_dev;

	float _anchor[3];

	// Normal vector
	float _nmle [3];
	// Basis vector 1
	float _e1 [3];
	// Basis vector 2
	float _e2 [3];
	// Distance to origine
	float _dist;
	// Scaling factors
	float _scale[2];
	// Size	
	int _Size[2];
	int _prev_Size[2];
	// Center point shift relative to the origine projection
	float _Shift[2];
		
	int _lvl;
	int _lvl_prec;
	int _idx;
	int _count;

	std::vector<cv::KeyPoint> _keypoints;
	cv::Mat _descriptors;

	// OpenGl data for rendering
	GLuint _vertex_buf;
	GLuint _index_buf;
	cudaGraphicsResource_t _Resources_t[2];
	size_t _buf_size;
	float *_VMap_VBO;
	float *_RGB_VBO;
	float *_Mask_VBO;
	unsigned int *_Index_dev;

	// Pointers to data on device
	unsigned char *_Mask_dev;
	unsigned short *_Bump_dev;
	unsigned char *_RGB_dev;
	
public:	
	int _visibility;

	// Constructor
	Primitive(float *nmle, float *e1, float *e2, float dist, float *scale, int *Size, float *Shift, float *anchor): _lvl(1), _lvl_prec(1), _count(0) {
		_nmle[0] = nmle[0]; _nmle[1] = nmle[1]; _nmle[2] = nmle[2];
		_e1[0] = e1[0]; _e1[1] = e1[1]; _e1[2] = e1[2];
		_e2[0] = e2[0]; _e2[1] = e2[1]; _e2[2] = e2[2];
		_dist = dist;
		_scale[0] = scale[0]; _scale[1] = scale[1];
		_Size[0] = Size[0]; _Size[1] = Size[1];
		_prev_Size[0] = Size[0]; _prev_Size[1] = Size[1];
		_Shift[0] = Shift[0]; _Shift[1] = Shift[1];
		_anchor[0] = anchor[0]; _anchor[1] = anchor[1]; _anchor[2] = anchor[2];

		_idx = 0;
		_visibility = 0;

		_Bump = NULL;
		_RGB = NULL;
		_RGB_buff = NULL;
		_Mask = NULL;
		_Mask_char = NULL;
		_Mask_grad = NULL;
		_VMap = NULL;
		_NMap = NULL;

		_Mask_dev = NULL;
		_Bump_dev = NULL;
		_RGB_dev = NULL;

		_VMap_VBO = 0;
		_RGB_VBO = 0;
		_Mask_VBO = 0;
		_Index_dev = 0;


	};
	// Destructor
	~Primitive() {
		if (_Bump != NULL) {
			_Bump = NULL;
			_RGB = NULL;
			_Mask_char = NULL;
		}
				
		if (_Mask != NULL) {
			_Mask = NULL;
		}
		
		if (_Mask_grad != NULL) {
			free(_Mask_grad);
			_Mask_grad = NULL;
		}

		if (_VMap != NULL) 		{
			free(_VMap);
			_VMap = NULL;
		}

		if (_NMap != NULL)  	{
			free(_NMap);
			_NMap = NULL;
		}	

		_keypoints.clear();
	};

	inline void Unload() {
		_Bump_dev = NULL;
		_RGB_dev = NULL;
		_Mask_dev = NULL;
		checkCudaErrors( cudaFree(_param_dev) );
	}

	inline void LoadOnCPU(unsigned short *bump, unsigned char *rgb, unsigned char *mask) {
		_Bump = bump;
		_RGB = rgb;
		_Mask_char = mask;
		
		checkCudaErrors( cudaMemcpy(_Bump, _Bump_dev, 3*_Size[0]*_Size[1] * sizeof (unsigned short), cudaMemcpyDeviceToHost) );
		checkCudaErrors( cudaMemcpy(_RGB, _RGB_dev, 3*_Size[0]*_Size[1] * sizeof (unsigned char), cudaMemcpyDeviceToHost) );
		checkCudaErrors( cudaMemcpy(_Mask_char, _Mask_dev,_Size[0]* _Size[1] * sizeof (unsigned char), cudaMemcpyDeviceToHost) );
	}
		
	inline void SetCenter(float *center) {_Shift[0] = center[0]; _Shift[1] = center[1];};
	inline void SetSize(int *size) {
		_Size[0] = size[0]; _Size[1] = size[1];	
		_prev_Size[0] = size[0]; _prev_Size[1] = size[1];};

	void Allocate() {
		checkCudaErrors( cudaMalloc((void **) &_param_dev,14*sizeof(float)) );
	}
	
	void FreeCleanMem() {
		if (_Bump != NULL) {
			free(_Bump);
			_Bump = NULL;
			free(_RGB);
			_RGB = NULL;
			free(_Mask_char);
			_Mask_char = NULL;
		}
	}
		
	inline void SetParam() {
		float tmp [14];

		tmp[0] = _nmle[0]; tmp[1] = _nmle[1]; tmp[2] = _nmle[2]; tmp[3] = _dist; 
		tmp[4] = _scale[0]; tmp[5] = _scale[1]; 
		tmp[6] = _Shift[0]; tmp[7] = _Shift[1];
		tmp[8] = _e1[0]; tmp[9] = _e1[1]; tmp[10] = _e1[2];
		tmp[11] = _e2[0]; tmp[12] = _e2[1]; tmp[13] = _e2[2];
    
		checkCudaErrors( cudaMemcpy(_param_dev, tmp, 14 * sizeof(float), cudaMemcpyHostToDevice) );
	}

	inline float *getCenter(float decal) {
		float *res = new float [3];
		res[0] = (_dist+decal)*_nmle[0] + _Shift[0]*_e1[0] + _Shift[1]*_e2[0];
		res[1] = (_dist+decal)*_nmle[1] + _Shift[0]*_e1[1] + _Shift[1]*_e2[1];
		res[2] = (_dist+decal)*_nmle[2] + _Shift[0]*_e1[2] + _Shift[1]*_e2[2];
		return res;
	}

	inline float *getCorners() {
		float *res = new float [12];
		float x, y, d;
		x = _Shift[0];
		y = _Shift[1];

		d = _dist;

		res[0] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[1]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[2]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0] + (float(_Size[0]))*2.0/_scale[0];
		y = _Shift[1];
		res[3] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[4]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[5]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0] + (float(_Size[0]))*2.0/_scale[0];
		y = _Shift[1] + (float(_Size[1]))*2.0/_scale[1];
		res[6] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[7]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[8]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0];//*2.0/_scale[0];
		y = _Shift[1] + (float(_Size[1]))*2.0/_scale[1];
		res[9] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[10]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[11]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];

		return res;
	}		

	bool Clean();	
	void CpyToShortMask();
	void CpyToCharMask();
	void ComputeVMAPNMAP(float *pose);
};


#endif