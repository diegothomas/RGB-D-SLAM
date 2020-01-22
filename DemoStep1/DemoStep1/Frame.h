/* Author: Diego Thomas
 * Date Created: 3/11/2013
 * Last Modification: 5/8/2013
 * Class needed for the frame manager
 */

#ifndef __FRAME_H
#define __FRAME_H

#include "Utilities.h"

#define BUFFER_OFFSET(a) ((char*)NULL + (a))

/*
The class InputFrame defines the input frame from hard drive data to be aligned and integrated.
The data is maintained on the device.
*/
class InputFrame {
public:	
	float * _RGB_dev;
	float * _VMap_dev;
	float * _NMap_dev;
	
	GLuint _frame_buf;
	GLuint _index_buf;
	cudaGraphicsResource_t _Resources_t[2];
	size_t _buf_size;

	/////////////////Parameters////////////////////////////

	int _n, _m; // dimensions of the current frame

	bool _offline;

public:	
	// constructor
	InputFrame(int n, int m): _n(n), _m(m) { 

		if (DISPLAY_FRAME_IN) {
			glGenBuffers(1, &_frame_buf);
			glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);
    
			/* on alloue de l'espace */
			glBufferData(GL_ARRAY_BUFFER,                   /* target */
						  (n*m*3*sizeof(float)) +    /* taille des positions */
						  (n*m*3*sizeof(float)) +  /* taille des normales */
						  (n*m*3*sizeof(float)), 
						 NULL,                              /* ... */
						 GL_DYNAMIC_DRAW);                   /* mode */

			glBindBuffer(GL_ARRAY_BUFFER, 0);

			checkCudaErrors( cudaGraphicsGLRegisterBuffer(&_Resources_t[0], _frame_buf,  cudaGraphicsRegisterFlagsNone) );
			float *vbo = NULL;
			checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );
			checkCudaErrors( cudaGraphicsResourceGetMappedPointer(  (void**) &vbo, &_buf_size, _Resources_t[0]) );

			_VMap_dev = &vbo[0];
			_NMap_dev = &vbo[3*n*m];
			_RGB_dev = &vbo[6*n*m];
			checkCudaErrors( cudaMemset(_VMap_dev,0, 3*n * m * sizeof(float)) );
			checkCudaErrors( cudaMemset(_NMap_dev,0, 3*n * m * sizeof(float)) );
			checkCudaErrors( cudaMemset(_RGB_dev,0, 3*n * m * sizeof(float)) );

			checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );
			vbo = NULL;    
		} else {
			// Allocate memory for the of rgb normal and vertice images on GPU
			checkCudaErrors( cudaMalloc((void**)&_VMap_dev, 3*n * m * sizeof(float)) );
			checkCudaErrors( cudaMemset(_VMap_dev,0, n * m * sizeof(float)) );
			checkCudaErrors( cudaMalloc((void**)&_NMap_dev, 3*n * m * sizeof(float)) );
			checkCudaErrors( cudaMemset(_NMap_dev,0, n * m * sizeof(float)) );
			checkCudaErrors( cudaMalloc((void**)&_RGB_dev, 3*n * m * sizeof(float)) );
			checkCudaErrors( cudaMemset(_RGB_dev,0, 3*n * m * sizeof(float)) );
		}
		
		return;
	};

	// destructor
	virtual ~InputFrame() {};

	inline int getN() {return _n;}
	inline int getM() {return _m;} 
	inline float * getVMap() { return _VMap_dev;}
	inline float * getNMap() { return _NMap_dev;}	
	inline float * getRGB() { return _RGB_dev;}	

	void Draw(bool color = true);
};

/*
The class InputFrame defines the input frame from hard drive data to be aligned and integrated.
The data is maintained on the device.
*/
class OffLineFrame : public InputFrame {
public:
	// The current depth and color images are always kept on the device memory
	cv::gpu::GpuMat _depth_dev_test;
	cv::gpu::GpuMat _color_dev_test;
	
	// constructor
	OffLineFrame(int n, int m): InputFrame(n,m) { 								
		_depth_dev_test.create(n, m, CV_16UC1);		
		_color_dev_test.create(n, m, CV_8UC3);	
		_offline = true;
		return;
	};

	// destructor
	virtual ~OffLineFrame() { 		
		_depth_dev_test.release();
		_color_dev_test.release();

		if (DISPLAY_FRAME_IN) {
			/* suppression de l'objet tampon */
			cudaGraphicsUnregisterResource(_Resources_t[0]);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glDeleteBuffers(1, &_frame_buf);
			_VMap_dev = 0;
			_NMap_dev = 0;
			_RGB_dev = 0;
		} else {
			checkCudaErrors( cudaFree(_VMap_dev) );
			checkCudaErrors( cudaFree(_NMap_dev) );
			checkCudaErrors( cudaFree(_RGB_dev) );
		}

		_n = 0;
		_m = 0;
	};
	
	void LoadFrame(string filename_depth, string filename_color);
};

/*
The class KinectFrame defines the input frame from Kinect device to be aligned and integrated.
The data is maintained on the device.
*/
class KinectFrame : public InputFrame {
public:
	///////////////GPU data/////////////////////////
	BYTE *d_colorFrame;
	USHORT *d_depthFrame;
	LONG *d_colorCoord;
	
	// constructor
	KinectFrame(int n, int m): InputFrame(n,m) { 								
		checkCudaErrors( cudaMalloc((void**)&d_colorFrame, 4*n * m * sizeof(BYTE)) );
		checkCudaErrors( cudaMalloc((void**)&d_depthFrame, n * m * sizeof(USHORT)) );
		checkCudaErrors( cudaMalloc((void**)&d_colorCoord, 2*n * m * sizeof(LONG)) );
		_offline = false;
		return;
	};

	// destructor
	virtual ~KinectFrame() { 
		
		checkCudaErrors( cudaFree(d_colorFrame) );
		checkCudaErrors( cudaFree(d_depthFrame) );
		checkCudaErrors( cudaFree(d_colorCoord) );

		if (DISPLAY_FRAME_IN) {
			/* suppression de l'objet tampon */
			cudaGraphicsUnregisterResource(_Resources_t[0]);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glDeleteBuffers(1, &_frame_buf);
			_VMap_dev = 0;
			_NMap_dev = 0;
			_RGB_dev = 0;
		} else {
			checkCudaErrors( cudaFree(_VMap_dev) );
			checkCudaErrors( cudaFree(_NMap_dev) );
			checkCudaErrors( cudaFree(_RGB_dev) );
		}

		_n = 0;
		_m = 0;
	};
	
	void LoadKinectData(BYTE* h_colorFrame, USHORT* h_depthFrame, LONG* h_colorCoord);
};

/*
The class PredictedFrame defines the current predicted frame.
The data is maintained on the device.
*/
class PredictedFrame {
public:
	///////////////CPU data////////////////////////
	float *_depth;
	float *_rgb;
	
	unsigned char *_Mask_dev;
	unsigned char *_Mask_dev_swap;
	
	unsigned char *_Label_dev;
	unsigned char *_Label_dev_swap;

	float * _RGB_dev;
	float * _VMap_dev;
	float * _NMap_dev;
	unsigned int *_Index_dev;
	
	GLuint _frame_buf;
	GLuint _index_buf;
	cudaGraphicsResource_t _Resources_t[2];
	size_t _buf_size;

	/////////////////Parameters////////////////////////////

	int _n, _m; // dimensions of the current frame
	
	// constructor
	PredictedFrame(int n, int m): _n(n), _m(m) { 
		
		// Allocate memory for the of depth, normal and vertice images on GPU
		checkCudaErrors( cudaMalloc((void**)&_Mask_dev, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMemset(_Mask_dev,0, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMalloc((void**)&_Mask_dev_swap, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMemset(_Mask_dev_swap,0, n * m * sizeof(unsigned char)) );
		
		checkCudaErrors( cudaMalloc((void**)&_Label_dev, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMemset(_Label_dev,0, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMalloc((void**)&_Label_dev_swap, n * m * sizeof(unsigned char)) );
		checkCudaErrors( cudaMemset(_Label_dev_swap,0, n * m * sizeof(unsigned char)) );
						
		// Allocate memory for depth and rgb on CPU
		_depth = (float *) malloc(n * m * sizeof(float));
		_rgb = (float *) malloc(4 * n * m * sizeof(float));
		
		glGenBuffers(1, &_frame_buf);
		glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);
    
		/* on alloue de l'espace */
		glBufferData(GL_ARRAY_BUFFER,                   /* target */
					  (n*m*3*sizeof(float)) +    /* taille des positions */
					  (n*m*3*sizeof(float)) +  /* taille des normales */
					  (n*m*4*sizeof(float)), 
					 NULL,                              /* ... */
					 GL_DYNAMIC_DRAW);                   /* mode */

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		checkCudaErrors( cudaGraphicsGLRegisterBuffer(&_Resources_t[0], _frame_buf,  cudaGraphicsRegisterFlagsNone) );
		float *vbo = NULL;
		checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );
		checkCudaErrors( cudaGraphicsResourceGetMappedPointer(  (void**) &vbo, &_buf_size, _Resources_t[0]) );

		_VMap_dev = &vbo[0];
		_NMap_dev = &vbo[3*n*m];
		_RGB_dev = &vbo[6*n*m];
		checkCudaErrors( cudaMemset(_VMap_dev,0, 3*n * m * sizeof(float)) );
		checkCudaErrors( cudaMemset(_NMap_dev,0, 3*n * m * sizeof(float)) );
		checkCudaErrors( cudaMemset(_RGB_dev,0, 4*n * m * sizeof(float)) );

		checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );
		vbo = NULL;    

		glGenBuffers(1, &_index_buf);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _index_buf);
    
		/* on alloue de l'espace */
		glBufferData(GL_ELEMENT_ARRAY_BUFFER,                   /* target */
						((n-1) * (m-1) *4*sizeof(unsigned int)), 
						NULL,                              /* ... */
						GL_STREAM_DRAW);                   /* mode */

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		checkCudaErrors( cudaGraphicsGLRegisterBuffer(&_Resources_t[1], _index_buf,  cudaGraphicsRegisterFlagsWriteDiscard) );
		checkCudaErrors( cudaDeviceSynchronize() );

		unsigned int *vbo_ind;
		checkCudaErrors( cudaGraphicsMapResources (1, &_Resources_t[1]) );
		checkCudaErrors( cudaGraphicsResourceGetMappedPointer(  (void**) &vbo_ind, &_buf_size, _Resources_t[1]) );
		_Index_dev = vbo_ind;

		checkCudaErrors( cudaGraphicsUnmapResources (1, &_Resources_t[1]) );
		vbo_ind = NULL;

		return;
	};

	// destructor
	~PredictedFrame() { 
		
		checkCudaErrors( cudaFree(_Mask_dev) );
		checkCudaErrors( cudaFree(_Mask_dev_swap) );

		free(_depth);
		free(_rgb);

		/* suppression de l'objet tampon */
		cudaGraphicsUnregisterResource(_Resources_t[0]);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glDeleteBuffers(1, &_frame_buf);
		_VMap_dev = 0;
		_NMap_dev = 0;
		_RGB_dev = 0;

		cudaGraphicsUnregisterResource(_Resources_t[1]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDeleteBuffers(1, &_index_buf);
		_Index_dev = 0;

		_n = 0;
		_m = 0;
	};

	inline int getN() {return _n;}
	inline int getM() {return _m;} 
	inline float * getVMap() { return _VMap_dev;}
	inline float * getNMap() { return _NMap_dev;}	
	inline float * getRGB() { return _RGB_dev;}	
	inline float * getRGBCPU() { return _rgb;}
	inline unsigned char *getMask() {return _Mask_dev;}

	void print(char *filename);
	void save(char *filename, int indx);
	void ReadFrame(cudaGraphicsResource_t *Resources);	
	void Draw(bool color = true, bool quad = false);
	void Transform(float *pose, bool quad = false);
	void Merge(InputFrame *frame);
	void Cpy(InputFrame *frame);
	void InitMask();
	void ProjectMask(float *pose);
	void ProjectLabel(float *pose_loc, float *pose_glob, float *Equations_dev, int nbPlans);
};


#endif