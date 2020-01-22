/* Author: Diego Thomas
 * Date Created: 10/9/2013 (mnth/ day/ year)
 * Last Modification: 10/9/2013
 * All classes needed for the reconstruction using graph of primitives
 */

#ifndef __KEYFRAME_H
#define __KEYFRAME_H

#include "Utilities.h"

#define RGBDREP false
#define ZHOUREP true
#define KINECTV2 false

class KeyFrame {
private:
	cv::Mat _descriptors;
	vector<float *> _keypoints; // in world coordinate system
	vector<cv::KeyPoint> _keypoints2D; // in world coordinate system

	///////////////CPU data////////////////////////
	float *_depth;
	float *_rgb;
	
	int _n, _m; // dimensions of the current frame
	int _lvl;
	float _ds;
	float _factor;

	int _max_iter [3];
	float _thres_cvg_alpha;
	float _thres_cvg_tr;
	float _distThres;
	float _angleThres;
	int _indx;

	//index in the optimizable graph
	int _indx_g2o;

public:

	//3D position and orientation of the camera
	Eigen::MatrixXf _pose;
	vector<int> _nodes; // in world coordinate system

	//constructor
	KeyFrame() {};
	KeyFrame(char *path, int indx): _indx(indx) {
		//Read camera pose, descriptors and key points 3D position from the temporary file
		char Framename[100];
		char RGBname[100];
		char Depthname[100];
		string line;
		char tmpline[200];
		_ds = 1.0;
		_factor = 5000.0;
		_n = 480;
		_m = 640;
		_lvl = 3;
		_max_iter [0] = 10;
		_max_iter [1] = 10;
		_max_iter [2] = 10;
		_thres_cvg_alpha = 1e-8;
		_thres_cvg_tr = 1e-8;
		_distThres = 0.1;
		_angleThres = sin (40.0 * PI / 180.f);
		_indx_g2o= -1;
			
		_depth = NULL;
		_rgb = NULL;

		ifstream  filestr;
		sprintf(Framename, "%s\\Pose%d.txt", path, indx);
		filestr.open (Framename, fstream::in);
		while (!filestr.is_open()) {
			cout << "Could not open " << Framename << endl;
			return;
		}

		// Read Camera Pose
		_pose = Eigen::Matrix4f::Identity();

		float dataPose [16];
		getline (filestr,line);
		strcpy(tmpline, line.c_str());
		sscanf (tmpline,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &dataPose[0], &dataPose[1], &dataPose[2], &dataPose[3], 
																			&dataPose[4], &dataPose[5], &dataPose[6], &dataPose[7],
																			&dataPose[8], &dataPose[9], &dataPose[10], &dataPose[11], 
																			&dataPose[12], &dataPose[13], &dataPose[14], &dataPose[15]);

		_pose(0,0) = dataPose[0]; _pose(0,1) = dataPose[4]; _pose(0,2) = dataPose[8]; _pose(0,3) = dataPose[12];
		_pose(1,0) = dataPose[1]; _pose(1,1) = dataPose[5]; _pose(1,2) = dataPose[9]; _pose(1,3) = dataPose[13];
		_pose(2,0) = dataPose[2]; _pose(2,1) = dataPose[6]; _pose(2,2) = dataPose[10]; _pose(2,3) = dataPose[14];
		_pose(3,0) = dataPose[3]; _pose(3,1) = dataPose[7]; _pose(3,2) = dataPose[11]; _pose(3,3) = dataPose[15];

		//cout << "Pose Key Frame: " << _pose << endl;

		//Read RGB and Depth images
		sprintf(RGBname, "%s\\RGB%d.tiff", path, indx);
		cv::Mat img_RGB = cv::imread(RGBname, CV_LOAD_IMAGE_UNCHANGED);

		sprintf(Depthname, "%s\\Depth%d.tiff", path, indx);
		cv::Mat img_Depth = cv::imread(Depthname, CV_LOAD_IMAGE_UNCHANGED);

		//-- Step 1: Detect the keypoints using SURF Detector
		int minHessian = 400;

		cv::SurfFeatureDetector detector( minHessian );

		cv::Mat img(height, width, CV_8UC1);
		int i,j,k;
		for (i=0,k=height-1;i<height;i++,k--) {
			for (j=0;j<width;j++) {
				img.at<unsigned char>(k,j) = unsigned char((img_RGB.at<cv::Vec3w>(k,j)[2]/200 + img_RGB.at<cv::Vec3w>(k,j)[1]/200 + img_RGB.at<cv::Vec3w>(k,j)[0]/200)/3.0);

				/*_rgb[3*(_m*k+j)] = float(img_RGB.at<cv::Vec3w>(i,j)[2])/(200.0*255.0);
				_rgb[3*(_m*k+j)+1] = float(img_RGB.at<cv::Vec3w>(i,j)[1])/(200.0*255.0);
				_rgb[3*(_m*k+j)+2] = float(img_RGB.at<cv::Vec3w>(i,j)[0])/(200.0*255.0);
				_depth[_m*k+j] = (_ds/_factor) * float(img_Depth.at<cv::Vec3w>(i,j)[0]);*/
			}
		}

		//std::vector<cv::KeyPoint> keypoints;
		detector.detect( img, _keypoints2D );

		//-- Step 2: Calculate descriptors (feature vectors)
		cv::SurfDescriptorExtractor extractor;
		extractor.compute( img, _keypoints2D, _descriptors );

		////-- Draw keypoints
		//cv::Mat img_keypoints;

		//cv::drawKeypoints( img, _keypoints2D, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

		////-- Show detected (drawn) keypoints
		//int tmprand = rand();
		//sprintf(Framename, "Keypoints %d", tmprand);
		//cv::imshow(Framename, img_keypoints );

		//-- Step 3: Calculate 3D position for all 2D keypoints

		float fx, fy, cx, cy;
		if (RGBDREP) {
			/*** RGBD data **/
			fx = 1.6*640.0/2.0; 
			fy = 1.8*480.0/2.0; 
			cx = 319.5; 
			cy = 239.5;
		} else if(ZHOUREP) {
			/*** ZHOUREP data **/
			fx = 525.0;
			fy = 525.0; 
			cx = 319.5;
			cy = 239.5;
		} else if(KINECTV2) {
			/*** Live Kinect v2 data **/
			fx = 1.2*512.0/2.0;
			fy = 1.6*424.0/2.0;
			cx = 255.5;
			cy = 211.5;
		} else {
			/*** Live Kinect data **/
			fx = 580.8857;
			fy = 583.317;
			cx = 319.5; //317.389;
			cy = 239.5; //238.108;
		}

		for( int i = 0; i < _keypoints2D.size(); i++ )
		{
			//-- Get the keypoints from the good matches
			float ind_i = _keypoints2D[i].pt.y;
			float ind_j = _keypoints2D[i].pt.x;

			int u = static_cast<int>(ind_i);
			int v = static_cast<int>(ind_j);
			unsigned short d = img_Depth.at<unsigned short>(u,v);
			//unsigned short d = img_Depth.at<cv::Vec3w>(u,v)[0];

			float pt [3];

			pt[2] = float(d)/5000.0;
			pt[0] = (ind_j - cx)*pt[2] / fx;
			pt[1] = (height-ind_i - cy)*pt[2] / fy;
			pt[2] = -pt[2];

			
			float *pose3D = new float [3];
			pose3D[0] = pt[0]*_pose(0,0) + pt[1]*_pose(0,1) + pt[2]*_pose(0,2) + _pose(0,3);
			pose3D[1] = pt[0]*_pose(1,0) + pt[1]*_pose(1,1) + pt[2]*_pose(1,2) + _pose(1,3);
			pose3D[2] = pt[0]*_pose(2,0) + pt[1]*_pose(2,1) + pt[2]*_pose(2,2) + _pose(2,3);

			if (d == 0) {
				pose3D[0] = 0.0;
				pose3D[1] = 0.0;
				pose3D[2] = 0.0;
			}

			_keypoints.push_back(pose3D);
		}


		filestr.close();

		img_RGB.~Mat();
		img_Depth.~Mat();
		img.~Mat();
		
		/*remove ( Framename );
		remove ( RGBname );
		remove ( Depthname );*/

	};

	//destructor
	~KeyFrame() {
		_descriptors.release();//_descriptors.deallocate();
		for (vector<float *>::iterator it = _keypoints.begin(); it != _keypoints.end(); it++) 
			delete (*it);

		_keypoints.clear();
		_keypoints2D.clear();

		free(_depth);
		free(_rgb);
	};
	
	inline int getN() {return _n;}
	inline int getM() {return _m;} 
	inline float * getDepth() {return _depth;}
	inline int getIndx() {return _indx_g2o;}
	inline void setIndx(int indx) {_indx_g2o = indx;}

	void Load(char *path) {
		if (_depth != NULL)
			return;

		char RGBname[100];
		char Depthname[100];

		// Allocate memory for depth and rgb on CPU
		_depth = (float *) malloc(_n * _m * sizeof(float));
		_rgb = (float *) malloc(3 * _n * _m * sizeof(float));

		sprintf(RGBname, "%s\\RGB%d.tiff", path, _indx);
		cv::Mat img_RGB = cv::imread(RGBname, CV_LOAD_IMAGE_UNCHANGED);

		sprintf(Depthname, "%s\\Depth%d.tiff", path, _indx);
		cv::Mat img_Depth = cv::imread(Depthname, CV_LOAD_IMAGE_UNCHANGED);

		int i,j,k;
		for (i=0,k=height-1;i<height;i++,k--) {
			for (j=0;j<width;j++) {
				_rgb[3*(_m*k+j)] = float(img_RGB.at<cv::Vec3w>(i,j)[2])/(200.0*255.0);
				_rgb[3*(_m*k+j)+1] = float(img_RGB.at<cv::Vec3w>(i,j)[1])/(200.0*255.0);
				_rgb[3*(_m*k+j)+2] = float(img_RGB.at<cv::Vec3w>(i,j)[0])/(200.0*255.0);
				_depth[_m*k+j] = (_ds/_factor) * float(img_Depth.at<unsigned short>(i,j));
				//_depth[_m*k+j] = (_ds/_factor) * float(img_Depth.at<cv::Vec3w>(i,j)[0]);
			}
		}

		img_RGB.~Mat();
		img_Depth.~Mat();
	}

	void UnLoad() {
		if (_depth == NULL)
			return;

		free(_depth);
		_depth = NULL;
		free(_rgb);
		_rgb = NULL;
	}

	int Align(KeyFrame *frame, float *Transfo);

	double *getQuaternion();

	void SetFromQuaternion(double *v);

	void Transform(Eigen::MatrixXf ThePose);
};

bool IsNeighboor(KeyFrame *F1, KeyFrame *F2);

#endif