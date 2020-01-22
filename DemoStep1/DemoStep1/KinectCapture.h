/**************************************************************************/
/* Author: Pavankumar Vasu Anasosalu									  */
/* Date:   March 08, 2013												  */
/* Last Modified: March 25, 2013										  */
/* Purpose: Contains the class definition for skeleton tracking			  */
/**************************************************************************/

/**** Includes ****/
#include <Windows.h>
#include <Ole2.h>
#include <NuiApi.h>
#include <NuiSensor.h> //#include <NuiSkeleton.h>
#include <NuiImageCamera.h>
#include <math.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/remove.h>
#include <thrust/generate.h>
#include <thrust/detail/type_traits.h>

#include <conio.h>
#include <algorithm>
#include <time.h>
#include <limits.h>

#ifdef _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#endif

// OpenGL Graphics includes
#include <GL/glew.h>
#if defined (__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// includes, cuda
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h

// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop

// includes for OpenCV matrix operations
//#include <cv.h>
//#include <highgui.h>

typedef struct
{
	float3 X_axis;
	float3 Y_axis;
	float3 Z_axis;
}CoordAxis;

class SkeletonTrack
{
	/****** Class variable declarations*******/
private:
	// Resolution of the streams
	static const NUI_IMAGE_RESOLUTION colorResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION depthResolution = NUI_IMAGE_RESOLUTION_640x480;
	
	// Mapped color coordinates from color frame to depth frame
	LONG*                         colorCoordinates;
	
	// Event handlers
	HANDLE						  rgbStream;
	HANDLE						  depthStream;
	HANDLE						  NextDepthFrameEvent;
	HANDLE						  NextColorFrameEvent;

	// Variables related to resolution assigned in constructor
	int							  CtoDdiv;
	long int					  cwidth;
	long int					  dwidth;
	long int					  cheight;
	long int					  dheight;

	// Actual sensor connected to the Computer
	INuiSensor*				      sensor;
	
	// Color and depth frames
	unsigned char*				  colordata;
	unsigned short*				  depthdata;
		
	/****** Class function declarations *******/
public:

	/* Constructor */
	SkeletonTrack();

	/* Destructor */
	~SkeletonTrack();

	/* Function that identifies the First Kinect sensor connected to the PC */ 
	HRESULT initKinect();

	/* Function to get the depth frame from the hardware */ 
	HRESULT getDepth(unsigned short *dest);

	/* Function to get the color frame from the hardware */
	HRESULT getColor(unsigned char *dest);

	/* Function to map color frame to the depth frame */
	HRESULT MapColorToDepth(BYTE* colorFrame, USHORT* depthFrame); 

	/* Function to synchronize the frame capture */
	void getKinectData();

	/**** Getters for the private members of the class ****/
	
	long int getCwidth()
	{
		return cwidth;
	}

	long int getDwidth()
	{
		return dwidth;
	}

	long int getCheight()
	{
		return cheight;
	}

	long int getDheight()
	{
		return dheight;
	}

	int      getCtoDdiv()
	{
		return CtoDdiv;
	}

	BYTE*    getColorframe()
	{
		return colordata;
	}

	USHORT*  getDepthframe()
	{
		return depthdata;
	}

	LONG*    getColorCoord()
	{
		return colorCoordinates;
	}

};



/**** class for utilities/functionalites required by SkeletonTrack class ****/
class SkeletonTrackUtils
{
public:
	/* Function to perform the cross product of two vectors */
	template<class TYPE> 
	void cross_product(TYPE *p1, TYPE *p2, TYPE *cross);

	/* Function of Utils class to perform Matrix Multiplication */
	template<class TYPE>
    void MatrixMul(TYPE *in_vect, TYPE *out_vect, Matrix4 &T);

	/* Function to normalize vector */
	template<class TYPE>
	void Normalize(TYPE *in_vect, TYPE *out_vect);
};


