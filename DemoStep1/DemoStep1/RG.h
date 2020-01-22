/* Author: Diego Thomas
 * Date Created: 11/20/2012
 * Last Modification: 05/09/2013
 * All classes needed for the reconstruction using graph of primitives
 */

#ifndef __RG_H
#define __RG_H

#include "Graph.h"

void AllocVariables(int n, int m);

void FreeVariables();

struct ParamThread
{
	Graph *_graph;
	PredictedFrame *_frame;
	char *_filename;
	bool _finished;
	float _pose_loc[16];
	int _indx;

	ParamThread(int n, int m) : _graph(NULL), _finished(true), _indx(0) { 
		_frame = new PredictedFrame(n, m);
		_graph = new Graph();
	};
	ParamThread(Graph *graph, char *filename): _graph(graph), _filename(filename), _finished(true), _indx(0) {};
};

/*
The class RG defines the Reconstruction process
*/
class RG {
public:
	// The current frame
	InputFrame *_frame;
	// The predicted frame
	PredictedFrame *_predicted_frame;
	// The local model
	Graph *_graph;
	Graph *_graph_global;

	unsigned short **_data_Bump;
	unsigned char **_data_RGB;
	unsigned char **_data_Mask;

	int _indx;

	//Class to read from Kinect
	SkeletonTrack* _Ske;

	// Transform local coordinate into global coordinate
	Eigen::Matrix3f _poseR;
	Eigen::Vector3f _poset;	
	Eigen::Matrix3f _poseRPrev;
	Eigen::Vector3f _posetPrev;

	float *_intr_dev;
	float *_Rcurr_dev;
	float *_tcurr_dev;
	float *_Equations_dev;
	float *_Equations;
	float *_Centers;
		
	cv::gpu::GpuMat _img1;
	cv::gpu::GpuMat _Gx;
	cv::gpu::GpuMat _Gy;
	
	// Counter for current frame
	int _count;
	bool _first_out;
	bool _first_in;
	bool _kinect;
	bool _RGBDRep;
	int _nbPlan;

	// Path to the data
	char *_path;
	vector<string> _timestamp_depth;
	vector<string> _timestamp_color;
	vector<double *> _timestamp_pose;
	
	int _Size;
	int _Size_data;
	int _lvl;
	bool _real;
	int _curr_plan_idx;
	bool _Detect;

	float _distThres;
	float _angleThres;
	int _max_iter[3];
	float _thres_cvg_alpha;
	float _thres_cvg_tr;
	float _thres_accept;
	float _thres_accept_d;
	
	// OpenGl data for rendering
	GLuint _frame_buf;
	GLuint _textureId;
	GLuint _DepthRendererId;
	cudaGraphicsResource_t _Resources_t[1];
	size_t _buf_size;
					
	HANDLE hthread[NB_THREADS];	
	HANDLE mutex;

	// Constructor n = height, m = width
	RG(int n, int m, char *path, float *Calib, int lvl, float distThres, float angleThres, int Sizedata, bool real = false, bool Kinect = false, bool RGBDRep = true, float thres_cvg_alpha = 1e-6, float thres_cvg_tr = 1e-6): 
		_count(1), _path(path), _lvl(lvl), _distThres(distThres), _angleThres(angleThres), _thres_cvg_alpha(thres_cvg_alpha), _thres_cvg_tr(thres_cvg_tr),
		_thres_accept(0.2f), _thres_accept_d(0.05f) {
	
		if (Kinect)
			_frame = new KinectFrame(n,m);
		else
			_frame = new OffLineFrame(n,m);

		_predicted_frame = new PredictedFrame(n,m);
		_graph = new Graph(1024, 1024);
		//_graph_global = new Graph(1024, 1024);

		_max_iter[0] = 3;
		_max_iter[1] = 2;
		_max_iter[2] = 1;

		_Ske = NULL;
		for (int i = 0; i < NB_THREADS; i++)
			hthread[i]= 0;
		
		if (Kinect) {
			/* Check for Kinect */
			HRESULT hr;
			_Ske = new SkeletonTrack();

			if ((hr = _Ske->initKinect() != S_OK)) {
				cout << "Error initKinect"  << endl;
			}

			assert(_Ske->getCheight() == height);
			assert(_Ske->getCwidth() == width);

			_Size_data = INT_MAX;
		} else {
			if (!real) {
				if (RGBDRep) {
					LoadTimeStamps( &_timestamp_depth, &_timestamp_color, _path);
					_Size_data = _timestamp_depth.size();
				} else {
					_Size_data = Sizedata; //2999; //5489; //2999;//2699;
				}
			} else {
				_Size_data = Sizedata; //8500; //11500;//8400;//15100;// 15700;//8500;
			}
		}

		_real = real;
		_kinect = Kinect;
		_RGBDRep = RGBDRep;
		_first_out = true;
		_Size = 0;
		_Detect = false;
		_indx = 0;
		_nbPlan = 0;

		_img1.create(n, m, CV_32FC1);		
		_Gx.create(n, m, CV_32FC1);		
		_Gy.create(n, m, CV_32FC1);

		_Equations_dev = NULL;
		_Equations = NULL;
		_Centers = NULL;

		_data_Bump = (unsigned short **) malloc(50*sizeof(unsigned short *));
		_data_RGB = (unsigned char **) malloc(50*sizeof(unsigned char *));
		_data_Mask = (unsigned char **) malloc(50*sizeof(unsigned char *));

		for (int i = 0; i < 50; i++) {
			_data_Bump[i] = (unsigned short *) malloc(3*1024* 1024*sizeof(unsigned short));	
			_data_RGB[i] = (unsigned char *) malloc(3*1024* 1024*sizeof(unsigned char));
			_data_Mask[i] = (unsigned char *) malloc(1024* 1024*sizeof(unsigned char));	
		}

		AllocVariables(n, m);		
		SetCalibrationMatrix(Calib);
		AllocPoseMatrix();
		AllocPosePrimMatrix();
		AllocBuffers(divUp (n, THREAD_SIZE_L_X), divUp (m, THREAD_SIZE_L_Y));
		AllocBufPrim();

		// ICP parameters
		float CalibGL [4];
		_first_in = true;
		CalibGL[0] = Calib[0];
		CalibGL[1] = Calib[1]; 
		CalibGL[2] = Calib[2];
		CalibGL[3] = Calib[3];

		checkCudaErrors( cudaMalloc((void **) &_intr_dev, 4*sizeof(float)) );
		checkCudaErrors( cudaMemcpy(_intr_dev, CalibGL, 4*sizeof(float), cudaMemcpyHostToDevice) );

		checkCudaErrors( cudaMalloc((void **) &_Rcurr_dev, 9*sizeof(float)) );
		checkCudaErrors( cudaMalloc((void **) &_tcurr_dev, 3*sizeof(float)) );

		_poseRPrev = Eigen::MatrixXf::Identity(3,3);
		_posetPrev = Eigen::Vector3f::Zero(3,1);
		_poseR = Eigen::MatrixXf::Identity(3,3);
		_poset = Eigen::Vector3f::Zero(3,1);
		
		// create a texture object
		glGenTextures(1, &_textureId);
		glBindTexture(GL_TEXTURE_2D, _textureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m, n, 0, GL_RGBA, GL_FLOAT, 0);
		glBindTexture(GL_TEXTURE_2D, 0);

		// Create a depth renderer buffer
		glGenTextures(1, &_DepthRendererId);
		glBindTexture(GL_TEXTURE_2D, _DepthRendererId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, m, n, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
						
		glGenFramebuffers(1, &_frame_buf);
		glBindFramebuffer(GL_FRAMEBUFFER, _frame_buf);

		// attach a texture to FBO color attachement point
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _textureId, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _DepthRendererId, 0);
		
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

		if (status != GL_FRAMEBUFFER_COMPLETE) {
			cout << "Error : " << status << endl;
		}
		
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
		checkCudaErrors( cudaGraphicsGLRegisterImage( &_Resources_t[0],	_textureId,	GL_TEXTURE_2D, cudaGraphicsRegisterFlagsNone) );
		
	};

	// Destructor
	~RG() {
		delete _frame;
		delete _predicted_frame;
		//delete _mask_frame;
		delete _graph;
		//delete _graph_global;

		_timestamp_depth.clear();
		_timestamp_color.clear();
		_count = 0;
				
		if (_Equations_dev != NULL) {
			checkCudaErrors( cudaFree(_Equations_dev) );
			_Equations_dev = NULL;
		}

		if (_Equations != NULL) {
			free(_Equations);
			_Equations = NULL;
			free(_Centers);
			_Centers = NULL;
		}

		for (int i = 0; i < 50; i++) {
			free(_data_Bump[i]);
			free(_data_RGB[i]);
			free(_data_Mask[i]);
		}

		free(_data_Bump);
		free(_data_RGB);
		free(_data_Mask);

		FreeVariables();
		FreeBuffers();
		FreeCalibrationMatrix();
		FreePoseMatrix();
		FreePosePrimMatrix();
		FreeBufPrim();

		cudaFree(_intr_dev);
		cudaFree(_Rcurr_dev);
		cudaFree(_tcurr_dev);
				
		glDeleteTextures(1, &_textureId);
		_textureId = 0;
		glDeleteFramebuffers(1, &_frame_buf);
		_frame_buf = 0;
	};

	bool LoadFrame();

	void Finish();
	
	// Detect planes from the current Frame
	void DetectPrimitives(cudaGraphicsResource_t *Resources_predicted_frame, float *pose);
	
	void CreatePrimitives(cudaGraphicsResource_t *Resources_predicted_frame, float *pose);
	
	// Align current frame with predicted frame
	bool AlignFrameGaussNewton(cudaGraphicsResource_t Resources_frame, cudaGraphicsResource_t Resources_predicted_frame);

	void ReorganizeEqua(cudaGraphicsResource_t *Resources_predicted_frame, float *pose);
};


#endif