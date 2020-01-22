
#include "stdafx.h"
#include "Frame.h"

#define extern VERBOSE;


/***************************************************************************************/
/*************************** Methods for the class InputFrame **************************/
/***************************************************************************************/

void InputFrame::Draw(bool color) {
	
	/* on passe en mode VBO */
    glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);

	glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
	glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)));
	if (color)
		glColorPointer(3, GL_FLOAT , 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)+_n*_m*3*sizeof(float)));

	/* activation des tableaux de donnees */
    glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	if (color)
		glEnableClientState(GL_COLOR_ARRAY);

	/* rendu points */
	glDrawArrays(GL_POINTS, 0, _n*_m );

	if (color)
		glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

/***************************************************************************************/
/*************************** Methods for the class OffLineFrame **************************/
/***************************************************************************************/

void OffLineFrame::LoadFrame(string filename_depth, string filename_color) {
	
	cv::Mat depth_test;
	cv::Mat color_test;

	depth_test = cv::imread(filename_depth, CV_LOAD_IMAGE_UNCHANGED);
	//cv::imshow("depth",depth_test);
	/*cv::Mat tmp;
	_depth_test.convertTo(tmp, CV_32FC1);
	cv::Mat out;
	bilateralFilter(tmp, out, 5, 100.0, 100.0);
	out.convertTo(_depth_test, CV_16UC1);*/
	_depth_dev_test.upload(depth_test);
	depth_test.release();

	color_test = cv::imread(filename_color, CV_LOAD_IMAGE_UNCHANGED);
	//cv::imshow("color",color_test);
	_color_dev_test.upload(color_test);
	color_test.release();
	
	checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );
	// Copy RGBD data 
	gpu_cpy_char3(_color_dev_test, _RGB_dev, _n,  _m);
	// Compute Vertex position
	//checkCudaErrors( cudaMemset(_VMap_dev, 0, 3*_n*_m*sizeof(float)) );
	VertexMapKinect(_VMap_dev, _depth_dev_test, _n, _m);
	// Compute Normal orientation
	//checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);

	// Release VBO data 
	checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );
			
	return;
}


/***************************************************************************************/
/*************************** Methods for the class KinectFrame *************************/
/***************************************************************************************/

void KinectFrame::LoadKinectData(BYTE* h_colorFrame, USHORT* h_depthFrame, LONG* h_colorCoord) {

	/**** copy data from host memory to device memory location ****/
	// Copy color frame
	checkCudaErrors(cudaMemcpy(d_colorFrame, h_colorFrame, sizeof(BYTE)* 4 * _n * _m, cudaMemcpyHostToDevice));
	//memcpy(_rgb_char, h_colorFrame, sizeof(BYTE)* 4 * _n * _m);

	// Copy depth frame
	checkCudaErrors(cudaMemcpy(d_depthFrame, h_depthFrame, sizeof(USHORT) * _n * _m, cudaMemcpyHostToDevice));

	// copy color coordinates
	checkCudaErrors(cudaMemcpy(d_colorCoord, h_colorCoord, sizeof(LONG)* 2 * _n * _m, cudaMemcpyHostToDevice));

	checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );
	
	checkCudaErrors( cudaMemset(_VMap_dev,0, 3*_n *_m * sizeof(float)) );
	checkCudaErrors( cudaMemset(_NMap_dev,0, 3*_n * _m * sizeof(float)) );
	checkCudaErrors( cudaMemset(_RGB_dev,0, 3*_n * _m * sizeof(float)) );

	MapData2VBO_cu(d_colorFrame, d_depthFrame, d_colorCoord, _VMap_dev, _RGB_dev, _n, _m);

	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );

}


/***************************************************************************************/
/*************************** Methods for the class PredictedFrame ***********************/
/***************************************************************************************/

void PredictedFrame::print(char *filename) {

	float *VMap_dev = getVMap();
	float *NMap_dev = getNMap();
	float *RGB_dev = getRGB();

	float *VMap = (float *) malloc(3*_n*_m*sizeof(float));
	cudaMemcpy(VMap, (float *) VMap_dev,  3*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);
	float *NMap = (float *) malloc(3*_n*_m*sizeof(float));
	cudaMemcpy(NMap, (float *) NMap_dev,  3*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(_rgb, (float *) RGB_dev,  4*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);

	int nbVertex = 0;
	for (int i = 0; i < _n; i++) {
		for (int j = 0; j < _m; j++) {
			if (VMap[3*(i*_m+j)+2] != 0.0)
				nbVertex++;
		}
	}

	//
    //  Open the input file in "read text" mode.
    //
	ofstream  filestr;

	filestr.open (filename, fstream::out);

	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}

	filestr << "ply\n";
	filestr << "format ascii 1.0\n";
    
    //
    //  Write the header.
    //
	filestr << "comment File created by Diego Thomas\n";
	filestr << "element vertex " << nbVertex << "\n";
	filestr << "property float x\n";
	filestr << "property float y\n";
	filestr << "property float z\n";
	filestr << "property float nx\n";
	filestr << "property float ny\n";
	filestr << "property float nz\n";
	filestr << "property uchar red\n";
	filestr << "property uchar green\n";
	filestr << "property uchar blue\n";
	filestr << "property uchar alpha\n";
    int nbFace = 0;
	filestr << "element face " << nbFace << "\n";
	filestr << "property list uchar int vertex_indices\n";
	filestr << "end_header\n";
    
    //
    //  Write data.
    //
    for (int i = 0; i < _n; i++) {
		for (int j = 0; j < _m; j++) {
			if (VMap[3*(i*_m+j)+2] != 0.0)
				filestr << VMap[3*(i*_m+j)] << " " << VMap[3*(i*_m+j)+1] << " " << VMap[3*(i*_m+j)+2] << " "
				<< NMap[3*(i*_m+j)] << " " << NMap[3*(i*_m+j)+1] << " " << NMap[3*(i*_m+j)+2] << " "
				//<< 255 << " " << 255 << " " << 255 << " " << "255\n";
				<< static_cast<int>(_rgb[4*(i*_m+j)]*255.0) << " " << static_cast<int>(_rgb[4*(i*_m+j)+1]*255.0) << " " << static_cast<int>(_rgb[4*(i*_m+j)+2]*255.0) << " " << "255\n";
		}
	}

        
    filestr.close();
	free(VMap);
	free(NMap);
}

void PredictedFrame::save(char *filename, int indx) {
	char destfilename[100];

	cv::Mat imgd(_n, _m, CV_16UC1);
	int i,j,k;
	for (i=0,k=_n-1;i<_n;i++,k--) {
		for (j=0;j<_m;j++) {
			//cout << _depth[(i*_m + j)] << endl;
			imgd.at<unsigned short>(k,j) = unsigned short(_rgb[4*(i*_m + j)+3]*MAX_DEPTH*5000.0);
			/*img.at<cv::Vec3w>(k,j)[1] = unsigned short(_depth[(i*_m + j)]*5000.0);
			img.at<cv::Vec3w>(k,j)[0] = unsigned short(_depth[(i*_m + j)]*5000.0);*/
			//cout << img.at<cv::Vec3w>(k,j)[0] << endl;
		}
	}

	
	sprintf(destfilename, "%s\\Depth%d.tiff", filename, indx);
	if (! cv::imwrite(destfilename, imgd) )
			cout << "error print Depth" << endl;
	
	cv::Mat img(_n, _m, CV_16UC3);
	for (i=0,k=_n-1;i<_n;i++,k--) {
		for (j=0;j<_m;j++) {
			img.at<cv::Vec3w>(k,j)[2] = 200*unsigned short(_rgb[4*(i*_m + j)]*255.0);
			img.at<cv::Vec3w>(k,j)[1] = 200*unsigned short(_rgb[4*(i*_m + j)+1]*255.0);
			img.at<cv::Vec3w>(k,j)[0] = 200*unsigned short(_rgb[4*(i*_m + j)+2]*255.0);
		}
	}

	sprintf(destfilename, "%s\\RGB%d.tiff", filename, indx);
	if (! cv::imwrite(destfilename, img) )
			cout << "error print RGB" << endl;

	img.~Mat();
	imgd.~Mat();

}

void PredictedFrame::ReadFrame(cudaGraphicsResource_t *Resources) {
	
	cudaGraphicsResource_t Resources_tot [2];
	Resources_tot [0] = Resources[0];
	Resources_tot [1] = _Resources_t[0];

	checkCudaErrors( cudaGraphicsMapResources ( 2, Resources_tot) );

	cudaArray* My_Array;
	checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &My_Array, Resources[0], 0, 0) );		
	ReadFrame_cu(_VMap_dev, _RGB_dev, _NMap_dev, My_Array, _n, _m);

	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
		
	checkCudaErrors( cudaGraphicsUnmapResources (2,  Resources_tot) );		
}

void PredictedFrame::Draw(bool color, bool quad) {
	
	/* on passe en mode VBO */
    glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);

	glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
	glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)));
	if (color)
		glColorPointer(4, GL_FLOAT , 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)+_n*_m*3*sizeof(float)));

	/* activation des tableaux de donnees */
    glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	if (color)
		glEnableClientState(GL_COLOR_ARRAY);

	if (!quad) {
		/* rendu points */
		glDrawArrays(GL_POINTS, 0, _n*_m );
	} else {
		/* rendu indices */
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _index_buf);
		glDrawElements(GL_QUADS, 4*(_n-1)*(_m-1), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	if (color)
		glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PredictedFrame::Transform(float *pose, bool quad) {

	SetPoseMatrix(pose);
	
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	Resources [1] = _Resources_t[1];
	checkCudaErrors( cudaGraphicsMapResources (2, Resources) );

	TransformVertexMap(_VMap_dev, _RGB_dev, _n, _m);
	
	if (quad) {
		QuadTrimFrame(_VMap_dev, _Index_dev, _n, _m, 0.1);
	}

	checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	checkCudaErrors( cudaDeviceSynchronize() );
}

void PredictedFrame::Merge(InputFrame *frame) {

	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	if (DISPLAY_FRAME_IN)
		Resources [1] = frame->_Resources_t[0];

	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsMapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	if (frame->_offline)
		MergeDepthOff(_RGB_dev, ( (OffLineFrame *) frame)->_depth_dev_test, frame->getRGB(), _Mask_dev, _n, _m);
	else
		MergeDepthKinect(_RGB_dev, ( (KinectFrame *) frame)->d_depthFrame, frame->getRGB(), _Mask_dev, _n, _m);

	checkCudaErrors( cudaMemset(_VMap_dev, 0, 3*_n*_m*sizeof(float)) );
	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	VertexMapGL(_VMap_dev, _RGB_dev, _n, _m);
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
	
	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	/*cv::gpu::GpuMat tmp; 
	tmp.create(_n, _m, CV_8UC1);
	gpu_cpyC_char(tmp, _Mask_dev, _n, _m);
	cv::Mat tmp_host;
	tmp.download(tmp_host);
	cv::imshow("Mask", tmp_host);*/
}

void PredictedFrame::Cpy(InputFrame *frame) {
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	if (DISPLAY_FRAME_IN)
		Resources [1] = frame->_Resources_t[0];

	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsMapResources (2, Resources) );
	else 
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	checkCudaErrors( cudaMemcpy(_VMap_dev, frame->getVMap(), 3*_n*_m*sizeof(float), cudaMemcpyDeviceToDevice) );
	checkCudaErrors( cudaMemcpy(_NMap_dev, frame->getNMap(), 3*_n*_m*sizeof(float), cudaMemcpyDeviceToDevice) );

	// Add z buffer in RGBD data 
	if (frame->_offline)
		gpu_add_zbuffOff(( (OffLineFrame *) frame)->_depth_dev_test, frame->getRGB(), _RGB_dev, _n,  _m);
	else
		gpu_add_zbuffKinect(( (KinectFrame *) frame)->d_depthFrame, frame->getRGB(), _RGB_dev, _n,  _m);
	
	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsUnmapResources (1, Resources) );
}

void PredictedFrame::InitMask() {
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];

	checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	InitMask_cu(_Mask_dev, _RGB_dev, _n, _m);
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, Resources) );
}

void PredictedFrame::ProjectMask(float *pose) {
	SetPoseMatrix(pose);
	checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );

	ProjectMask_cu(_Mask_dev, _Mask_dev_swap, _VMap_dev, _n, _m);

	unsigned char *tmp = _Mask_dev;
	_Mask_dev = _Mask_dev_swap;	
	_Mask_dev_swap = tmp;
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );
}

void PredictedFrame::ProjectLabel(float *pose_loc, float *pose_glob, float *Equations_dev, int nbPlans) {
	SetPoseMatrix(pose_loc);
	checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );

	ProjectMask_cu(_Label_dev, _Label_dev_swap, _VMap_dev, _n, _m);

	unsigned char *tmp = _Label_dev;
	_Label_dev = _Label_dev_swap;	
	_Label_dev_swap = tmp;
	
	Segment_cu(_Label_dev, _VMap_dev, _NMap_dev, pose_glob, Equations_dev, nbPlans, 0.1, 0.9, _n, _m);
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );

	/*cv::gpu::GpuMat tmp_b; 
	tmp_b.create(_n, _m, CV_8UC1);
	gpu_cpyC_char(tmp_b, _Label_dev, _n, _m);
	cv::Mat tmp_host;
	tmp_b.download(tmp_host);
	cv::imshow("Label", tmp_host);*/
}

