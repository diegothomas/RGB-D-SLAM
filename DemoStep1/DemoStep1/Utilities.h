#ifndef __UTILITIES_H
#define __UTILITIES_H

#pragma once

/*** Include files for OpenGL to work ***/
#include <GL\glew.h>
#include <GL\glut.h>

/*** Standard include files for manipulating vectors, files etc... ***/
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <conio.h>
#include <algorithm>
#include <time.h>
#include <numeric>
#include <iterator>

#include <boost/make_shared.hpp>

/*** Include files to manipulate matrices ***/
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

/*** Include files for gpu operations on vectors ***/
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/remove.h>
#include <thrust/generate.h>
#include <thrust/detail/type_traits.h>

/*** Include files for CUDA to work ***/
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h
// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop

/*** Include files for Opencv to work ***/
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/nonfree/features2d.hpp"
//#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>


/*** Include files for the project ***/
#include "KinectCapture.h"
//#include "absorient.h"

/*** Include files for cuda kernels ***/
#include "Primitive.cuh"
#include "Kernel.cuh"
#include "ICP.cuh"
#include "RG.cuh"

/*** Global constant variables ***/
#define VERBOSE false		// Display or not intermediate messages
#define INC_SIZE 25
#define INC_STEP 64
#define NB_THREADS 1
#define NUM_THREADS 10
#define GAP  25             /* gap between subwindows */
#define lighting true
#define TIMING false
#define PI 3.1415926535897932384626433832795
#define DISPLAY_FRAME_IN true
#define MIN_SIZE_PLAN 5000

using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Vector2f;
using Eigen::Matrix2f;

using namespace std;


inline float round(float number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

// Load TimeStamps
void LoadTimeStamps(vector<string> *timestamp_depth, vector<string> *timestamp_color, char *path);

void LoadAssociatedTimeStamps(vector<string> *timestamp_depth, vector<string> *timestamp_color, char *path);

void LoadGroundTruth(vector<string> *timestamp_depth, vector<string> *timestamp_color, vector<double *> *timestamp_pose, char *path);

void PrintImage(void *Image, int *size_I, int format, char *windowname);

void SaveTrajectory(const char *filename_depth, vector<Eigen::Matrix3f> PosesR, vector<Eigen::Vector3f> Posest, vector<string> _timestamp);

int ConnectedComponents(bool *Input, int *BBox, int n, int m);

bool Estimate_transfo(float *points, Eigen::Matrix3f poseR, Eigen::Vector3f poset, float *Rotation, float *translation, int nb_match);

bool Estimate_transfo2D(float *points, float *Rotation, float *translation, int nb_match);

void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);

bool Horn_align(float *p, float *Rotation, float *translation, int  nb_match);



typedef enum {
  alignHorn,
  alignChenMedioni
} AlignmentMethod;


//
// Alignment methods:
//   horn_align does point-to-point
//   chen_medioni does point-to-plane
// Both methods calculate the transformation necessary to align
// src to dst, as a quaternion rotation and a translation.
//
// Meanings of parameters:
//   src        Source points to align
//   dst        Target positioning
//   nrmDst     Normals at points in dst
//   n          Number of point pairs
//   q[7]       Registration quaternion; 0..3 rot, 4..6 trans
//

void 
quaternion2matrix(double *q, double m[3][3]);

void
weigh_least_square(float **p,      // the model points  (source)
	   float **x,      // the measured points (destination)
	   int n,        // how many pairs
	   double q[7],  // registration quaternion
	   double d);

// Horn's method: calculates transformation from points to points

bool
horn_align(float *src,      // source points to align
	   int   n,        // how many pairs
	   double q[7]);   // registration quaternion
	   

#endif