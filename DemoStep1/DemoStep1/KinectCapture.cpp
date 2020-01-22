/**************************************************************************/
/* Author: Pavankumar Vasu Anasosalu									  */
/* Date:   March 08, 2013												  */
/* Last Modified: April 23, 2013										  */
/* Purpose: Contains the class definition for skeleton tracking			  */
/**************************************************************************/

/**** Includes ****/
#include "stdafx.h"
#include "KinectCapture.h"

/**** Constructor ****/
SkeletonTrack::SkeletonTrack()
{
	// initializing all the pointers and variables
	DWORD widthd = 0;
	DWORD heightd = 0;
	
	NuiImageResolutionToSize(depthResolution, widthd, heightd);
	dwidth  = static_cast<LONG>(widthd);
    dheight = static_cast<LONG>(heightd);
	
	NuiImageResolutionToSize(colorResolution, widthd, heightd);
	cwidth  = static_cast<LONG>(widthd);
    cheight = static_cast<LONG>(heightd);

    colordata        = (unsigned char *)  malloc(cwidth*cheight*4*sizeof(unsigned char));
	depthdata        = (unsigned short *) malloc(dwidth*dheight*sizeof(unsigned short));
	colorCoordinates = (LONG*)            malloc(dwidth*dheight*2*sizeof(LONG));

	CtoDdiv = cwidth/dwidth;
}

/**** Destructor ****/
SkeletonTrack::~SkeletonTrack()
{
	if (NULL != sensor)
    {
        sensor->NuiShutdown();
        sensor->Release();
    }

	free(colordata);
	free(depthdata);
	free(colorCoordinates);
}

/**** Initialization of Kinect Sensor ****/
HRESULT SkeletonTrack::initKinect()
{
	// Getting a working Kinect sesor
	int numSensors;
	HRESULT hr = NuiGetSensorCount(&numSensors);
	if(FAILED(hr))
		return hr;

	// identifying the sensor (We are assuming that there is only one sensor)
	hr = NuiCreateSensorByIndex(0,&sensor);
	if(FAILED(hr))
		return hr;

	// initializing the sensor and creating events for each frame
	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	
	if(SUCCEEDED(hr))
	{
		// Create an event that will be signaled when depth data is available
		NextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		// Create an event that will be signaled when color data is available
		NextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	}
	
	// initialize the sensor to open up depth stream
	hr = sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		depthResolution,
		NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,
		2,
		NextDepthFrameEvent,
		&depthStream);

	/*hr = sensor->NuiImageStreamSetImageFrameFlags(
         depthStream,
         NUI_IMAGE_DEPTH_NO_VALUE);*/

	
	if(FAILED(hr)) return hr;

	// Initialize sensor to open up color stream
	hr = sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		colorResolution,
		0,
		2,
		NextColorFrameEvent,
		&rgbStream);

	if(FAILED(hr)) return hr;  

	INuiColorCameraSettings *camSettings;
	hr = sensor->NuiGetColorCameraSettings(&camSettings);

	if(FAILED(hr))      return hr;

	hr = camSettings->SetAutoExposure(FALSE);
	if(FAILED(hr))      return hr;

	hr = camSettings->SetAutoWhiteBalance(FALSE);
	if(FAILED(hr))      return hr;

	hr = camSettings->SetWhiteBalance(4500);
	if(FAILED(hr))      return hr;

	return S_OK;
}

/**** Function to get the depth frame from the hardware ****/
HRESULT SkeletonTrack::getDepth(unsigned short *dest)
{
	NUI_IMAGE_FRAME imageFrame; 
	NUI_LOCKED_RECT LockedRect; 
	HRESULT hr;                 

	hr = sensor->NuiImageStreamGetNextFrame(depthStream,0,&imageFrame);
	if(FAILED(hr)) 		return hr;

	INuiFrameTexture *texture = imageFrame.pFrameTexture;
	hr = texture->LockRect(0,&LockedRect,NULL,0);
	if(FAILED(hr)) 		return hr;

	// Now copy the data to our own memory location
	if(LockedRect.Pitch != 0)
	{
		const GLushort* curr = (const unsigned short*) LockedRect.pBits;

		// copy the texture contents from current to destination
		memcpy( dest, curr, sizeof(unsigned short)*(dwidth*dheight) );
	}

	hr = texture->UnlockRect(0);
	if(FAILED(hr)) 		return hr;

	hr = sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
	if(FAILED(hr)) 		return hr;

	return S_OK;
}

/**** Function to get the color frame from the hardware ****/
HRESULT SkeletonTrack::getColor(unsigned char *dest)
{
	NUI_IMAGE_FRAME imageFrame; // structure containing all the metadata about the frame
	NUI_LOCKED_RECT LockedRect; // contains the pointer to the actual data
	HRESULT hr;                 // Error handling

	hr = sensor->NuiImageStreamGetNextFrame(rgbStream,0,&imageFrame);
	if(FAILED(hr))		return hr;

	INuiFrameTexture *texture = imageFrame.pFrameTexture;
	hr = texture->LockRect(0,&LockedRect,NULL,0);
	if(FAILED(hr))      return hr;

	// Now copy the data to our own memory location
	if(LockedRect.Pitch != 0)
	{
		const BYTE* curr = (const BYTE*) LockedRect.pBits;

		// copy the texture contents from current to destination
		memcpy( dest, curr, sizeof(BYTE)*(cwidth*cheight*4) );
	}

	hr = texture->UnlockRect(0);
	if(FAILED(hr))      return hr;

	hr = sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
	if(FAILED(hr))      return hr;

	return S_OK;
}

/**** Function to map color frame to the depth frame ****/
HRESULT SkeletonTrack::MapColorToDepth(BYTE* colorFrame, USHORT* depthFrame)
{
	HRESULT hr;

	// Find the location in the color image corresponding to the depth image
	hr = sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		colorResolution,
		depthResolution,
		dwidth*dheight,
		depthFrame,
		(dwidth*dheight)*2,
		colorCoordinates);

	if(FAILED(hr))    return hr;

	return S_OK;
}

/**** Function to synchronize the frame capture ****/
void SkeletonTrack::getKinectData()
{
	bool needToMapColorToDepth = false;
	HRESULT hr;

	while(true) {
		if( WAIT_OBJECT_0 == WaitForSingleObject(NextDepthFrameEvent, 0) )
		{
			// if we have received any valid new depth data we proceed to obtain new color data
			if ( (hr = getDepth(depthdata)) == S_OK )
			{
				if( WAIT_OBJECT_0 == WaitForSingleObject(NextColorFrameEvent, 0) )
				{
					// if we have received any valid new color data we proceed to extract skeletal information
					if ( (hr = getColor(colordata)) == S_OK )
					{
						MapColorToDepth((BYTE*)colordata, (USHORT*)depthdata); 
						return;
					}
				}
			}
		} 	
	}
}

/**** Function of Utils class to perform cross product ****/
template<class TYPE>
void SkeletonTrackUtils::cross_product(TYPE *p1, TYPE *p2, TYPE *cross)
{
	// this function basically takes the cross product of two vectors
	cross[0] = (p1[1]*p2[2]) - (p1[2]*p2[1]);
	cross[1] = (p1[2]*p2[0]) - (p1[0]*p2[2]);
	cross[2] = (p1[0]*p2[1]) - (p1[1]*p2[0]);
}

/**** Function of Utils class to perform Matrix Multiplication ****/
template<class TYPE>
void SkeletonTrackUtils::MatrixMul(TYPE *in_vect, TYPE *out_vect, Matrix4 &T)
{
	TYPE i_vector[4], o_vector[4];
	
	i_vector[0] = in_vect[0];
	i_vector[1] = in_vect[1];
	i_vector[2] = in_vect[2];
	i_vector[3] = 1.0;

	o_vector[0] = T.M11*i_vector[0] + T.M12*i_vector[1] + T.M13*i_vector[2] + T.M14*i_vector[3];
	o_vector[1] = T.M21*i_vector[0] + T.M22*i_vector[1] + T.M23*i_vector[2] + T.M24*i_vector[3];
	o_vector[2] = T.M31*i_vector[0] + T.M32*i_vector[1] + T.M33*i_vector[2] + T.M34*i_vector[3];
	o_vector[3] = T.M41*i_vector[0] + T.M42*i_vector[1] + T.M43*i_vector[2] + T.M44*i_vector[3];

	out_vect[0] = o_vector[0] / o_vector[3];
	out_vect[1] = o_vector[1] / o_vector[3];
	out_vect[2] = o_vector[2] / o_vector[3];
}

/**** Function to normalize vector ****/
template<class TYPE>
void SkeletonTrackUtils::Normalize(TYPE *in_vect, TYPE *out_vect)
{
	double mag = sqrt(pow(in_vect[0],2) + pow(in_vect[1],2) + pow(in_vect[2],2));

	out_vect[0] = (TYPE)in_vect[0] / mag;
	out_vect[1] = (TYPE)in_vect[1] / mag;
	out_vect[2] = (TYPE)in_vect[2] / mag;
}

/**** Function to compute product of 2 Matrix4 matrices T1 x T0 ****/
//Matrix4 SkeletonTrack::ComputeProduct(Matrix4& T0, NUI_SKELETON_BONE_ORIENTATION* boneOrientation, NUI_SKELETON_POSITION_INDEX idx)
//{
//	Matrix4 Product, T1;
//	T1 = boneOrientation[idx].hierarchicalRotation.rotationMatrix;
//
//	// Computing row 1
//	Product.M11 = T1.M11*T0.M11 + T1.M12*T0.M21 + T1.M13*T0.M31;
//	Product.M12 = T1.M11*T0.M12 + T1.M12*T0.M22 + T1.M13*T0.M32;
//	Product.M13 = T1.M11*T0.M13 + T1.M12*T0.M23 + T1.M13*T0.M33;
//	Product.M14 = 0;
//
//	// Computing row 2
//	Product.M21 = T1.M21*T0.M11 + T1.M22*T0.M21 + T1.M23*T0.M31;
//	Product.M22 = T1.M21*T0.M12 + T1.M22*T0.M22 + T1.M23*T0.M32;
//	Product.M23 = T1.M21*T0.M13 + T1.M22*T0.M23 + T1.M23*T0.M33;
//	Product.M24 = 0;
//
//	// Computing row 3
//	Product.M31 = T1.M31*T0.M11 + T1.M32*T0.M21 + T1.M33*T0.M31;
//	Product.M32 = T1.M31*T0.M12 + T1.M32*T0.M22 + T1.M33*T0.M32;
//	Product.M33 = T1.M31*T0.M13 + T1.M32*T0.M23 + T1.M33*T0.M33;
//	Product.M34 = 0;
//
//	// Computing row 4
//	Product.M41 = 0;
//	Product.M42 = 0;
//	Product.M43 = 0;
//	Product.M44 = 1;
//
//	return Product;
//}

/**** Function to compute inverse transformation from WS cartesian to Local coordinate system ****/
//void SkeletonTrack::ComputeInverseTransformatiom(Matrix4& T, Matrix4& T_inv)
//{
//	// get the rotation matrix from Matrix4 format to OpenCV cvMat format
//	CvMat *R		 = cvCreateMat(3,3,CV_32FC1);
//	CvMat *Tr		 = cvCreateMat(3,1,CV_32FC1);
//	CvMat *R_inv     = cvCreateMat(3,3,CV_32FC1);
//	CvMat *mR_inv_T  = cvCreateMat(3,1,CV_32FC1);
//
//	// populate the matrices
//	R->data.fl[0] = T.M11;   R->data.fl[1] = T.M12;   R->data.fl[2] = T.M13;
//	R->data.fl[3] = T.M21;   R->data.fl[4] = T.M22;   R->data.fl[5] = T.M23;
//	R->data.fl[6] = T.M31;   R->data.fl[7] = T.M32;   R->data.fl[8] = T.M33;
//
//	Tr->data.fl[0] = T.M14;  Tr->data.fl[1] = T.M24;  Tr->data.fl[2] = T.M34;
//
//	// Computing inverse of rotation R^{-1}
//	cvInv(R, R_inv, CV_SVD);
//
//	// Computing R^{-1} * T
//	cvMatMul(R_inv, Tr, mR_inv_T);
//
//	// negating the terms in mR_inv_T
//	mR_inv_T->data.fl[0] = mR_inv_T->data.fl[0] * -1.0;
//	mR_inv_T->data.fl[1] = mR_inv_T->data.fl[1] * -1.0;
//	mR_inv_T->data.fl[2] = mR_inv_T->data.fl[2] * -1.0;
//
//	// Setting the inverse matrix as
//	/*  [ R^{-1}  |  -R^{-1} * T ]
//	    [    0           1       ]
//	*/
//	T_inv.M11 = R_inv->data.fl[0];  T_inv.M12 = R_inv->data.fl[1];  T_inv.M13 = R_inv->data.fl[2];  T_inv.M14 = mR_inv_T->data.fl[0];
//	T_inv.M21 = R_inv->data.fl[3];  T_inv.M22 = R_inv->data.fl[4];  T_inv.M23 = R_inv->data.fl[5];  T_inv.M24 = mR_inv_T->data.fl[1];
//	T_inv.M31 = R_inv->data.fl[6];  T_inv.M32 = R_inv->data.fl[7];  T_inv.M33 = R_inv->data.fl[8];  T_inv.M34 = mR_inv_T->data.fl[2];
//	T_inv.M41 = 0;                  T_inv.M42 = 0;                  T_inv.M43 = 0;                  T_inv.M44 = 1;
//
//	// deallocate the memory allocated for matrices
//	cvReleaseMat(&R);
//	cvReleaseMat(&Tr);
//	cvReleaseMat(&R_inv);
//	cvReleaseMat(&mR_inv_T);
//}
