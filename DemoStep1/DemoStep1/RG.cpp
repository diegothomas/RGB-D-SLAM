#include "stdafx.h"
#include "RG.h"

#define ACCEPT_THRESH 0.1

/************** Global variables ********************/
float *VMap_cpu;
float *NMap_cpu;
unsigned char *Label_cpu;

ParamThread *param[NB_THREADS];	
char filename_buff[100];

static void WINAPI SaveThread(void *lpParameter)
{
	ParamThread *param = static_cast<ParamThread *>(lpParameter);
	Graph *graph = param->_graph;
	PredictedFrame *frame = param->_frame;
	char *filename = param->_filename;
	int indx = param->_indx;
	
	graph->_pose(0,0) = param->_pose_loc[0]; graph->_pose(0,1) = param->_pose_loc[4]; graph->_pose(0,2) = param->_pose_loc[8]; graph->_pose(0,3) = param->_pose_loc[12];
	graph->_pose(1,0) = param->_pose_loc[1]; graph->_pose(1,1) = param->_pose_loc[5]; graph->_pose(1,2) = param->_pose_loc[9]; graph->_pose(1,3) = param->_pose_loc[13];
	graph->_pose(2,0) = param->_pose_loc[2]; graph->_pose(2,1) = param->_pose_loc[6]; graph->_pose(2,2) = param->_pose_loc[10]; graph->_pose(2,3) = param->_pose_loc[14];
	graph->_pose(3,0) = param->_pose_loc[3]; graph->_pose(3,1) = param->_pose_loc[7]; graph->_pose(3,2) = param->_pose_loc[11]; graph->_pose(3,3) = param->_pose_loc[15];

	graph->Clean();
	//graph->Clean(graph_global->_pose.data(), true);
	Eigen::Matrix4f ppp = Eigen::Matrix4f::Identity();
	//graph->Clean(ppp.data());
	//graph->Clean(ppp.data(), true);

	/*if (indx > 3) {
		for (int i=0;i<frame->_n;i++) {
			for (int j=0;j<frame->_m;j++) {
				if (frame_mask->_rgb[4*(i*frame->_m + j)] < 0.1) {
					frame->_depth[i*frame->_m + j] = 0.0;
				}
			}
		}
	}*/

	frame->save(filename, indx+1);
	graph->save(filename, indx);
	//graph->save2Mesh(filename);

	for (vector<Primitive *>::iterator it = graph->_nodes.begin(); it != graph->_nodes.end(); it++) {
		Primitive *Prim = (*it); 
		Prim->FreeCleanMem();
		delete Prim;
	}

	graph->_nodes.clear();
	graph->_curr_ind = 0;
	graph->_pose = Eigen::Matrix4f::Identity();
	graph->_Size = 0;
	graph->_curr_ind = 0;
	param->_finished = true;
}

void AllocVariables(int n, int m) {	
	VMap_cpu = (float *) malloc(3*n*m*sizeof(float));
	NMap_cpu = (float *) malloc(3*n*m*sizeof(float));
	Label_cpu = (unsigned char *) malloc(n*m*sizeof(unsigned char));

	for (int i = 0; i < NB_THREADS; i++)
			param[i] = new ParamThread(n,m);	
}

void FreeVariables() {
	free(VMap_cpu);
	free(NMap_cpu);
	
	for (int i = 0; i < NB_THREADS; i++)
		delete param[i];
}

bool RG::LoadFrame() {
	string filename_depth, filename_color;
	
	if (_count > _Size_data-1) { 
		if (_first_out) {
			_poseRPrev = Eigen::MatrixXf::Zero(3,3);
			_posetPrev = Eigen::Vector3f::Zero(3,1);
			cout << "Finished" << endl;
			Finish();
		}
		_first_out = false;
		return false;
	}

	if (_kinect) {
		if (_count == 0) {
			float current_time;
			float last_time = clock();
			current_time = clock();
			while ((current_time-last_time)/CLOCKS_PER_SEC < 1.0) {
				_Ske->getKinectData();
				current_time = clock();
			}
		}
		_Ske->getKinectData();
		((KinectFrame *) _frame)->LoadKinectData(_Ske->getColorframe(), _Ske->getDepthframe(), _Ske->getColorCoord());
	} else {
		if (!_real) {
			if (_RGBDRep) {
				filename_depth = string(_path) + "//" + _timestamp_depth[_count];
				filename_color = string(_path) + "//" + _timestamp_color[_count];
			} else {
				char filename_buff[100];
				if (_count+1 < 10) {
					sprintf(filename_buff, "%s//depth//00000%d.png", _path, _count+1);
				} else if (_count+1 < 100) {
					sprintf(filename_buff, "%s//depth//0000%d.png", _path, _count+1);
				} else if (_count+1 < 1000) {
					sprintf(filename_buff, "%s//depth//000%d.png", _path, _count+1);
				} else if (_count+1 < 10000) {
					sprintf(filename_buff, "%s//depth//00%d.png", _path, _count+1);
				}
				filename_depth = string(filename_buff);
				
				if (_count+1 < 10) {
					sprintf(filename_buff, "%s//color//00000%d.png", _path, _count+1);
				} else if (_count+1 < 100) {
					sprintf(filename_buff, "%s//color//0000%d.png", _path, _count+1);
				} else if (_count+1 < 1000) {
					sprintf(filename_buff, "%s//color//000%d.png", _path, _count+1);
				} else if (_count+1 < 10000) {
					sprintf(filename_buff, "%s//color//00%d.png", _path, _count+1);
				}
				filename_color = string(filename_buff);
			}
		} else {
			char filename_buff[100];
			sprintf(filename_buff, "%s//Depth_%d.tiff", _path, _count);
			filename_depth = string(filename_buff);
			sprintf(filename_buff, "%s//RGB_%d.tiff", _path, _count);
			filename_color = string(filename_buff);
		}
	
		((OffLineFrame *) _frame)->LoadFrame(filename_depth, filename_color);
	}	

	_count++;
	
	return true;
}

void RG::Finish() {	

	Matrix3f poseRinv;
	Vector3f posetinv;

	if (_first_in && _count > 30) {
		float pose [16];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				pose[j*4+i] = _poseR(i,j);	
		for (int i = 0; i < 3; i++)
			pose[3*4+i] = _poset(i);
		pose[3] = 0.0; pose[7] = 0.0; pose[11] = 0.0; pose[15] = 1.0;

		_graph->_BumpTab = NULL;
		_graph->_RGBTab = NULL;
		_graph->_MaskTab = NULL;
		_graph->_ParamTab = NULL;

		DetectPrimitives(_predicted_frame->_Resources_t, pose); 
		CreatePrimitives(_predicted_frame->_Resources_t, pose);
		//_graph->Update(_predicted_frame, pose);
		
		_first_in = false;

		return;
	}

	bool ok2Save = false;
	char Eqname[100];
	ifstream  filestr;
	sprintf(Eqname, "%s\\Dummy.txt", _path);
	filestr.open (Eqname, fstream::in);
	if (!filestr.is_open())
		ok2Save = true;
	filestr.close();


	if ((_count % 100) == 0 && param[0]->_finished && ok2Save) {

		_poseRPrev = _poseR;
		_posetPrev = _poset;

		// Reinit		
		sprintf(filename_buff, "Graph%d", _count);
				
		int count = 0;
		while(!param[count]->_finished) {
			if (count < NB_THREADS-1)
				count++;
			else
				count = 0;
		}

		DWORD dwID;
		DWORD dwRetVal = 0;

		if	( hthread[count] != 0 )
			dwRetVal = WaitForMultipleObjects(1, &hthread[count], TRUE, INFINITE);
		
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				param[count]->_pose_loc[j*4+i] = _poseR(i,j);	
		for (int i = 0; i < 3; i++)
			param[count]->_pose_loc[3*4+i] = _poset(i);
		param[count]->_pose_loc[3] = 0.0; param[count]->_pose_loc[7] = 0.0; param[count]->_pose_loc[11] = 0.0; param[count]->_pose_loc[15] = 1.0;	
				
		int id = 0;
		for (vector<Primitive *>::iterator it = _graph->_nodes.begin(); it != _graph->_nodes.end(); it++) {
			Primitive *Prim = (*it); 
			Prim->LoadOnCPU(_data_Bump[id], _data_RGB[id], _data_Mask[id]);
			Prim->Unload();
			param[count]->_graph->_nodes.push_back(Prim);
			id++;
		}
		_graph->_nodes.clear();

		//Copy gpu frame info to CPU
		checkCudaErrors( cudaMemcpy(param[count]->_frame->getRGBCPU(), _predicted_frame->getRGB(),  4 * _frame->_n * _frame->_m * sizeof(float), cudaMemcpyDeviceToHost) );
		
		param[count]->_filename = "C:\\Diego\\Data\\KinectV1\\Tmp";
		param[count]->_finished = false;
		param[count]->_indx = _indx;
		hthread[count] = CreateThread(NULL, 0,(LPTHREAD_START_ROUTINE)SaveThread,param[count],0,&dwID);
		
		_indx++;
		
		float pose [16];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				pose[j*4+i] = _poseR(i,j);	
		for (int i = 0; i < 3; i++)
			pose[3*4+i] = _poset(i);
		pose[3] = 0.0; pose[7] = 0.0; pose[11] = 0.0; pose[15] = 1.0;
		
		/*free(_Centers);	
		free(_Equations);
		checkCudaErrors( cudaFree(_Equations_dev) );
		_Equations_dev = NULL;
		_Equations = NULL;	
		_Centers = NULL;
		_nbPlan = 0;
		checkCudaErrors( cudaMemset(_predicted_frame->_Label_dev,0, _frame->_n * _frame->_m * sizeof(unsigned char)) );*/

		if (_graph->_BumpTab != NULL)
			checkCudaErrors( cudaFree(_graph->_BumpTab) );
		_graph->_BumpTab = NULL;
		if (_graph->_RGBTab != NULL)
			checkCudaErrors( cudaFree(_graph->_RGBTab) );
		_graph->_RGBTab = NULL;
		if (_graph->_MaskTab != NULL)
			checkCudaErrors( cudaFree(_graph->_MaskTab) );
		_graph->_MaskTab = NULL;
		if (_graph->_ParamTab != NULL)
			checkCudaErrors( cudaFree(_graph->_ParamTab) );
		_graph->_ParamTab = NULL;

		ReorganizeEqua(_predicted_frame->_Resources_t, pose);
		DetectPrimitives(_predicted_frame->_Resources_t, pose);
		CreatePrimitives(_predicted_frame->_Resources_t, pose);
		//_graph->Update(_predicted_frame, pose);
	}
}


/* 
Detect planes in a depth image
*/
void RG::DetectPrimitives(cudaGraphicsResource_t *Resources_predicted_frame, float *pose) {

	cv::gpu::GpuMat edgesR;
	cv::gpu::GpuMat edgesG;
	cv::gpu::GpuMat edgesB;
	
	cv::gpu::GpuMat img;
	img.create(_predicted_frame->getN(), _predicted_frame->getM(), CV_8UC1);
	
	cv::gpu::GpuMat img2;
	img2.create(_predicted_frame->getN(), _predicted_frame->getM(), CV_8UC1);

	cv::Mat tmp;

	checkCudaErrors( cudaGraphicsMapResources ( 1, Resources_predicted_frame) );

	/****** Compute edges from the Normal image **********/
	// Get edges from the red channel
	gpu_cpyN(img, _predicted_frame->getNMap(), _predicted_frame->getN(), _predicted_frame->getM(), 0);
	cv::gpu::Canny(img, edgesR, 120.0, 30.0, 3); 

	gpu_cpyN(img, _predicted_frame->getNMap(), _predicted_frame->getN(), _predicted_frame->getM(), 1);
	cv::gpu::Canny(img, edgesG, 120.0, 30.0, 3);  

	gpu_cpyN(img, _predicted_frame->getNMap(), _predicted_frame->getN(), _predicted_frame->getM(), 2);
	cv::gpu::Canny(img, edgesB, 120.0, 30.0, 3); 
		
	gpu_Sum(img, edgesR, edgesG, edgesB, _predicted_frame->getN(), _predicted_frame->getM());
	
	// Detect non-continuities in the depth image
	gpu_Discont(img, _predicted_frame->getVMap(), _predicted_frame->getN(), _predicted_frame->getM(), 0.03);
		
	img.download(tmp);
	//cv::imshow("grad_im", tmp);
	//cv::imwrite("grad_im.png", tmp);
	
	checkCudaErrors( cudaMemset(_predicted_frame->_Label_dev, 0, _predicted_frame->getN()*_predicted_frame->getM()*sizeof(unsigned char)) );

	gpu_Thresh(img2, img, 20, _predicted_frame->_Label_dev, _predicted_frame->getN(), _predicted_frame->getM());
		
	img2.download(tmp);
	//cv::imshow("edges", tmp);
			
	int erosion_size = 2;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );

	cv::Mat dilation_dst;
	
	/// Apply the erosion operation
	cv::erode( tmp, dilation_dst, element );
	
	//cv::imshow("dilation_dst", dilation_dst);

	//********************** Find connected components ********************************/
	cv::Mat output = cv::Mat::zeros(dilation_dst.size(), CV_8UC1);

    cv::Mat binary;
    std::vector < std::vector<cv::Point2i > > blobs;

    cv::threshold(dilation_dst, binary, 0.0, 1.0, cv::THRESH_BINARY);
	
	FindBlobs(binary, blobs);

	//size_t min_size = 5000;
	std::vector < std::vector<cv::Point2i > > blobsF;
	for(size_t i=0; i < blobs.size(); i++) {
		size_t curr_size = blobs[i].size();
		if (curr_size > MIN_SIZE_PLAN) {
			blobsF.push_back(blobs[i]);
		}
	}
	
	/*Test*/
	/*cv::Mat imgTest(_predicted_frame->getN(), _predicted_frame->getM(), CV_8UC3);
	for (int i = 0; i < _predicted_frame->getN(); i++) {
		for (int j = 0; j < _predicted_frame->getM(); j++) {
			if (binary.at<unsigned char>(i,j) > 0) {
				imgTest.at<cv::Vec3b>(i,j)[0] = 255;
				imgTest.at<cv::Vec3b>(i,j)[1] = 255;
				imgTest.at<cv::Vec3b>(i,j)[2] = 255;
			} else {
				imgTest.at<cv::Vec3b>(i,j)[0] = 0;
				imgTest.at<cv::Vec3b>(i,j)[1] = 0;
				imgTest.at<cv::Vec3b>(i,j)[2] = 0;
			}
		}
	}

	for(size_t i=0; i < blobs.size(); i++) {
		unsigned char R = unsigned char((float(rand())/RAND_MAX)*255.0);
		unsigned char G = unsigned char((float(rand())/RAND_MAX)*255.0);
		unsigned char B = unsigned char((float(rand())/RAND_MAX)*255.0);
		size_t curr_size = blobs[i].size();
		for (int j = 0; j < curr_size; j++) {
			if (binary.at<unsigned char>(blobs[i][j].y,blobs[i][j].x) > 0) {
				imgTest.at<cv::Vec3b>(blobs[i][j].y,blobs[i][j].x)[2] = R;
				imgTest.at<cv::Vec3b>(blobs[i][j].y,blobs[i][j].x)[1] = G;
				imgTest.at<cv::Vec3b>(blobs[i][j].y,blobs[i][j].x)[0] = B;
			}
		}
	}

	cv::imwrite("Segmented.png", imgTest);*/

	/* END TEST*/

	int nbPlans = int(blobsF.size());
	
	if (nbPlans == 0) {
		_Detect = false;
		checkCudaErrors( cudaGraphicsUnmapResources (1,  Resources_predicted_frame) );
		return;
	}
	_Detect = true;
	_curr_plan_idx = 0;

	// Do median filter
	checkCudaErrors( cudaMemcpy(VMap_cpu, _predicted_frame->getVMap(), 3*_predicted_frame->getN()*_predicted_frame->getM()*sizeof(float), cudaMemcpyDeviceToHost) );
	checkCudaErrors( cudaMemcpy(NMap_cpu, _predicted_frame->getNMap(), 3*_predicted_frame->getN()*_predicted_frame->getM()*sizeof(float), cudaMemcpyDeviceToHost) );
	checkCudaErrors( cudaMemcpy(Label_cpu, _predicted_frame->_Label_dev, _predicted_frame->getN()*_predicted_frame->getM()*sizeof(unsigned char), cudaMemcpyDeviceToHost) );
	
	
	int n = _predicted_frame->getN();
	int m = _predicted_frame->getM();
	_nbPlan = 0;

	//int *countC = (int *) malloc(_nbPlan*sizeof(int));
	//for (int i = 0; i < _nbPlan; i++) {		
	//	_Centers[3*i] = 0.0;
	//	_Centers[3*i+1] = 0.0;
	//	_Centers[3*i+2] = 0.0;
	//	countC[i] = 0;
	//}

	//for (int i = 0; i < n; i++) {
	//	for (int j = 0; j < m; j++) {
	//		int idx = Label_cpu[(i*m+j)];
	//		if (idx < 1)
	//			continue;

	//		//float nmle [3];
	//		//nmle [0] = ThePlans[idx][0];
	//		//nmle [1] = ThePlans[idx][1];
	//		//nmle [2] = ThePlans[idx][2];
	//							
	//		_Centers[3*(idx-1)] = _Centers[3*(idx-1)] + VMap_cpu[3*(i*m+j)]*pose[0] + VMap_cpu[3*(i*m+j)+1]*pose[4] + VMap_cpu[3*(i*m+j)+2]*pose[8] + pose[12];
	//		_Centers[3*(idx-1)+1] = _Centers[3*(idx-1)+1] + VMap_cpu[3*(i*m+j)]*pose[1] + VMap_cpu[3*(i*m+j)+1]*pose[5] + VMap_cpu[3*(i*m+j)+2]*pose[9] + pose[13];
	//		_Centers[3*(idx-1)+2] = _Centers[3*(idx-1)+2] + VMap_cpu[3*(i*m+j)]*pose[2] + VMap_cpu[3*(i*m+j)+1]*pose[6] + VMap_cpu[3*(i*m+j)+2]*pose[10] + pose[14];
	//		countC[(idx-1)] = countC[(idx-1)]+1;

	//		//float dist = (pt [0]*nmle [0] + pt [1]*nmle [1] + pt [2]*nmle [2]);
	//		//Dists[i][j] = dist; //.push_back(dist);
	//	}
	//}			
	//
	//for (int i = 0; i < _nbPlan; i++) {		
	//	_Centers[3*i] = _Centers[3*i] / float(countC[i]);
	//	_Centers[3*i+1] = _Centers[3*i+1] / float(countC[i]);
	//	_Centers[3*i+2] = _Centers[3*i+2] / float(countC[i]);
	//}
	//free(countC);

	///////Test
	//tmp = cv::Mat(_predicted_frame->getN(), _predicted_frame->getM(), CV_8UC3) ;
	//for (int i = 0; i < _predicted_frame->getN(); i++) {
	//	for (int j = 0; j < _predicted_frame->getM(); j++) {
	//		tmp.at<cv::Vec3b>(i,j)[0] = unsigned char((NMap_cpu[3*(i*_predicted_frame->getM()+j)]+1.0)*100.0);
	//		tmp.at<cv::Vec3b>(i,j)[1] = unsigned char((NMap_cpu[3*(i*_predicted_frame->getM()+j)+1]+1.0)*100.0);
	//		tmp.at<cv::Vec3b>(i,j)[2] = unsigned char((NMap_cpu[3*(i*_predicted_frame->getM()+j)+2]+1.0)*100.0);
	//	}
	//}
	//cv::imshow("NMAP_CPU", tmp);
	///////Test
	
	vector<vector<float>> Nmles_x;
	vector<vector<float>> Nmles_y;
	vector<vector<float>> Nmles_z;
	vector<vector<float>> Dists;
	vector<float *> ThePlans;
	for (int j = 0; j < nbPlans; j++) {
		vector<float> tmpx;
		Nmles_x.push_back(tmpx);
		vector<float> tmpy;
		Nmles_y.push_back(tmpy);
		vector<float> tmpz;
		Nmles_z.push_back(tmpz);
		vector<float> tmpd;
		Dists.push_back(tmpd);	
		float *eq = new float[10];
		ThePlans.push_back(eq);		
	}

	for(int i=0; i < nbPlans; i++) {
		int curr_size = int(blobsF[i].size());
		Nmles_x[i].resize(curr_size);
		Nmles_y[i].resize(curr_size);
		Nmles_z[i].resize(curr_size);

		int count = 0;
		//#pragma omp parallel num_threads(NUM_THREADS)
		//{
		//	#pragma omp for
			for(int j=0; j < curr_size; j++) {
				int x = blobsF[i][j].x;
				int y = blobsF[i][j].y;

				if (NMap_cpu[3*(y*m+x)] == 0.0 && NMap_cpu[3*(y*m+x)+1] == 0.0 && NMap_cpu[3*(y*m+x)+2] == 0.0)
					continue;

				count++;

				float nmle [3];
				nmle [0] = NMap_cpu[3*(y*m+x)]*pose[0] + NMap_cpu[3*(y*m+x)+1]*pose[4] + NMap_cpu[3*(y*m+x)+2]*pose[8];
				nmle [1] = NMap_cpu[3*(y*m+x)]*pose[1] + NMap_cpu[3*(y*m+x)+1]*pose[5] + NMap_cpu[3*(y*m+x)+2]*pose[9];
				nmle [2] = NMap_cpu[3*(y*m+x)]*pose[2] + NMap_cpu[3*(y*m+x)+1]*pose[6] + NMap_cpu[3*(y*m+x)+2]*pose[10];

				Nmles_x[i][j] = nmle [0]; //.push_back(nmle [0]);
				Nmles_y[i][j] = nmle [1]; //.push_back(nmle [1]);
				Nmles_z[i][j] = nmle [2]; //.push_back(nmle [2]);
				
			}
		//}

			if(count < 1000) {
				blobsF[i].clear();
			}
	}

	for(int i=0; i < nbPlans; i++) {
		if (blobsF[i].size() == 0) {
			blobsF.erase(blobsF.begin() + i);
			Nmles_x.erase(Nmles_x.begin() + i);
			Nmles_y.erase(Nmles_y.begin() + i);
			Nmles_z.erase(Nmles_z.begin() + i);
			Dists.erase(Dists.begin() + i);
			ThePlans.erase(ThePlans.begin() + i);
			i--;
			nbPlans--;
		}
	}
	cout << "nb new Plans: " << nbPlans << endl;


	for (int j = 0; j < nbPlans; j++) {
		std::sort (Nmles_x[j].begin(), Nmles_x[j].end()); 
		std::sort (Nmles_y[j].begin(), Nmles_y[j].end()); 
		std::sort (Nmles_z[j].begin(), Nmles_z[j].end()); 
		 
		float nmle [3];
		nmle [0] = Nmles_x[j][Nmles_x[j].size()/2];
		nmle [1] = Nmles_y[j][Nmles_y[j].size()/2];
		nmle [2] = Nmles_z[j][Nmles_z[j].size()/2];
		 
		float norm_nmle = sqrt(nmle [0]*nmle [0] + nmle [1]*nmle [1] + nmle [2]*nmle [2]);
		nmle [0] = nmle [0]/norm_nmle;
		nmle [1] = nmle [1]/norm_nmle;
		nmle [2] = nmle [2]/norm_nmle;

		ThePlans[j][0] = nmle [0];
		ThePlans[j][1] = nmle [1];
		ThePlans[j][2] = nmle [2];
	}
	
	float *CentersTmp = (float *) malloc(3*(_nbPlan+nbPlans)*sizeof(float));
	for (int i = 0; i < _nbPlan; i++) {
		CentersTmp[3*i] = _Centers[3*i];
		CentersTmp[3*i+1] = _Centers[3*i+1];
		CentersTmp[3*i+2] = _Centers[3*i+2];
	}

	for(int i=0; i < nbPlans; i++) {
		int curr_size = int(blobsF[i].size());
		Dists[i].resize(curr_size);
		float center_curr [3];
		center_curr [0] = 0.0;
		center_curr [1] = 0.0;
		center_curr [2] = 0.0;
		//#pragma omp parallel num_threads(NUM_THREADS)
		//{
		//	#pragma omp for
			for(int j=0; j < curr_size; j++) {
				int x = blobsF[i][j].x;
				int y = blobsF[i][j].y;

				float nmle [3];
				nmle [0] = ThePlans[i][0];
				nmle [1] = ThePlans[i][1];
				nmle [2] = ThePlans[i][2];
				
				float pt [3];
				pt [0] = VMap_cpu[3*(y*m+x)]*pose[0] + VMap_cpu[3*(y*m+x)+1]*pose[4] + VMap_cpu[3*(y*m+x)+2]*pose[8] + pose[12];
				pt [1] = VMap_cpu[3*(y*m+x)]*pose[1] + VMap_cpu[3*(y*m+x)+1]*pose[5] + VMap_cpu[3*(y*m+x)+2]*pose[9] + pose[13];
				pt [2] = VMap_cpu[3*(y*m+x)]*pose[2] + VMap_cpu[3*(y*m+x)+1]*pose[6] + VMap_cpu[3*(y*m+x)+2]*pose[10] + pose[14];
				
				center_curr [0] = center_curr [0] + pt [0];
				center_curr [1] = center_curr [1] + pt [1];
				center_curr [2] = center_curr [2] + pt [2];

				float dist = (pt [0]*nmle [0] + pt [1]*nmle [1] + pt [2]*nmle [2]);
				Dists[i][j] = dist; //.push_back(dist);
				Label_cpu[y*m+x] = unsigned char(i+_nbPlan+1);
			}
		//}
			
		CentersTmp[3*(i+_nbPlan)] = center_curr [0] / float(curr_size);
		CentersTmp[3*(i+_nbPlan)+1] = center_curr [1] / float(curr_size);
		CentersTmp[3*(i+_nbPlan)+2] = center_curr [2] / float(curr_size);

	}
	if (_Centers != NULL)
		free(_Centers);
	_Centers = CentersTmp;	

	checkCudaErrors( cudaMemcpy(_predicted_frame->_Label_dev, Label_cpu, _predicted_frame->getN()*_predicted_frame->getM()*sizeof(unsigned char), cudaMemcpyHostToDevice) );

	for (int j = 0; j < nbPlans; j++) {
		std::sort (Dists[j].begin(), Dists[j].end()); 
		 
		float dist = Dists[j][Dists[j].size()/2];

		ThePlans[j][3] = dist;
	}

	Nmles_x.clear();
	Nmles_y.clear();
	Nmles_z.clear();
	Dists.clear();	
	//// End median filter

	// Compute e1 and e2 vectors for each plan
	for (int j = 0; j < nbPlans; j++) {
		float nmle [3];
		nmle [0] = ThePlans[j][0];
		nmle [1] = ThePlans[j][1];
		nmle [2] = ThePlans[j][2];
		
		float e1 [3];
		float e2 [3];
		float norm_e;
		//e1
		if (nmle[0] > nmle[1] && nmle[0] > nmle[2]) {
			//%z vec n
			e1[0] = -nmle[1]; e1[1] = nmle[0]; e1[2] = 0.0;
			norm_e = sqrt(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
			e1[0] /= norm_e; e1[1] /= norm_e; e1[2] /= norm_e;
		} else if (nmle[1] > nmle[0] && nmle[1] > nmle[2]) {
			//%x vec n
			e1[0] = 0.0; e1[1] = -nmle[2]; e1[2] = nmle[1];
			norm_e = sqrt(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
			e1[0] /= norm_e; e1[1] /= norm_e; e1[2] /= norm_e;
		} else {
			//%y vec n
			e1[0] = nmle[2]; e1[1] = 0.0; e1[2] = -nmle[0];
			norm_e = sqrt(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
			e1[0] /= norm_e; e1[1] /= norm_e; e1[2] /= norm_e;
		}

		//e2 
		e2[0] = nmle[1]*e1[2] - nmle[2]*e1[1];
		e2[1] = nmle[2]*e1[0] - nmle[0]*e1[2];
		e2[2] = nmle[0]*e1[1] - nmle[1]*e1[0];

		float *eq = (float *) malloc(10*sizeof(float));
		eq[0] = nmle[0]; eq[1] = nmle[1]; eq[2] = nmle[2];
		eq[3] = e1[0]; eq[4] = e1[1]; eq[5] = e1[2];
		eq[6] = e2[0]; eq[7] = e2[1]; eq[8] = e2[2];
		eq[9] = ThePlans[j][3];
		free(ThePlans[j]);
		ThePlans[j] = eq;
	}

	float *EquationsTmp = (float *) malloc(10*(_nbPlan+nbPlans)*sizeof(float));

	for (int i = 0; i < _nbPlan; i++) {
		EquationsTmp[10*i] = _Equations[10*i];
		EquationsTmp[10*i+1] = _Equations[10*i+1];
		EquationsTmp[10*i+2] = _Equations[10*i+2];
		EquationsTmp[10*i+3] = _Equations[10*i+3];
		EquationsTmp[10*i+4] = _Equations[10*i+4];
		EquationsTmp[10*i+5] = _Equations[10*i+5];
		EquationsTmp[10*i+6] = _Equations[10*i+6];
		EquationsTmp[10*i+7] = _Equations[10*i+7];
		EquationsTmp[10*i+8] = _Equations[10*i+8];
		EquationsTmp[10*i+9] = _Equations[10*i+9];
	}

	for (int j = 0; j < nbPlans; j++) {
		EquationsTmp[10*(j+_nbPlan)] = ThePlans[j][0];
		EquationsTmp[10*(j+_nbPlan)+1] = ThePlans[j][1];
		EquationsTmp[10*(j+_nbPlan)+2] = ThePlans[j][2];
		EquationsTmp[10*(j+_nbPlan)+3] = ThePlans[j][3];
		EquationsTmp[10*(j+_nbPlan)+4] = ThePlans[j][4];
		EquationsTmp[10*(j+_nbPlan)+5] = ThePlans[j][5];
		EquationsTmp[10*(j+_nbPlan)+6] = ThePlans[j][6];
		EquationsTmp[10*(j+_nbPlan)+7] = ThePlans[j][7];
		EquationsTmp[10*(j+_nbPlan)+8] = ThePlans[j][8];
		EquationsTmp[10*(j+_nbPlan)+9] = ThePlans[j][9];
	}
	if (_Equations != NULL)
		free(_Equations);
	_Equations = EquationsTmp;
	
	_nbPlan += nbPlans;
	//cout << "nbPlans: " << _nbPlan << endl;

	/*if (_Equations_dev != NULL)
		checkCudaErrors( cudaFree(_Equations_dev) );
	
	checkCudaErrors( cudaMalloc((void **) &_Equations_dev,10*_nbPlan*sizeof(float)) );
	checkCudaErrors( cudaMemcpy(_Equations_dev, _Equations, 10*_nbPlan*sizeof(float), cudaMemcpyHostToDevice) );

	checkCudaErrors( cudaDeviceSynchronize() );

	Segment_cu(_predicted_frame->_Label_dev, _predicted_frame->getVMap(), _predicted_frame->getNMap(), pose, _Equations_dev, _nbPlan, ACCEPT_THRESH, 0.8, n, m);*/
	
	checkCudaErrors( cudaGraphicsUnmapResources (1,  Resources_predicted_frame) );

}


/* Iteration for plane detection
*/
void RG::CreatePrimitives(cudaGraphicsResource_t *Resources_predicted_frame, float *pose) {
	// Create all primitive with the size of the corresponding BBox.
	// For each primitive, center is the lower left corner of the bounding box with width Size_frag [0] and height Size_frag [1]

	int *Assoc = (int *) malloc(_nbPlan * sizeof(int));
	int count = 0;
	for (int i = 0; i < _nbPlan; i++) {
		float scale [2] = {2.0/RES_PLANE, 2.0/RES_PLANE};
		int Size_frag [2];
		float center [2];

		Assoc[i] = -1;
		
		Size_frag [0] = _graph->_SizeVBUFF[0]; 
		Size_frag [1] = _graph->_SizeVBUFF[1];

		/*** Project current center into plane equation ***/
		float pt [2];
		pt[0] = _Centers[3*i]*_Equations[10*i+3] + _Centers[3*i+1]*_Equations[10*i+4] + _Centers[3*i+2]*_Equations[10*i+5]; // e1
		pt[1] = _Centers[3*i]*_Equations[10*i+6] + _Centers[3*i+1]*_Equations[10*i+7] + _Centers[3*i+2]*_Equations[10*i+8]; // e2

		center [0] = pt[0] - (float(Size_frag [0])*RES_PLANE)/2.0;
		center [1] = pt[1] - (float(Size_frag [1])*RES_PLANE)/2.0;

		Primitive *ThePrim = new Primitive(&_Equations[10*i], &_Equations[10*i+3], &_Equations[10*i+6], _Equations[10*i+9], scale, Size_frag, center, &_Centers[3*i]);

		//cout << "Anchor point: " << _Centers[3*i] << " " << _Centers[3*i+1]<< " " << _Centers[3*i+2] << endl; 

		//if (_graph->exist(ThePrim, &_Equations[10*i], _Equations[10*i+9], center)) {
		//	delete ThePrim;
		//	//cout << "The Prim Deleted" << endl;
		//	continue;
		//}

		if (!_graph->AddPrimitive(ThePrim)) {
			delete ThePrim;
			cout << "The Prim Non added" << endl;
			continue;
		}
		ThePrim->Allocate();
		Assoc[i] = count;
		count++;
	}

	float *EquationsTmp = (float *) malloc(10*count*sizeof(float));
	float *CentersTmp = (float *) malloc(3*count*sizeof(float));

	for (int i = 0; i < _nbPlan; i++) {
		if (Assoc[i] != -1) {
			EquationsTmp[10*Assoc[i]] = _Equations[10*i];
			EquationsTmp[10*Assoc[i]+1] = _Equations[10*i+1];
			EquationsTmp[10*Assoc[i]+2] = _Equations[10*i+2];
			EquationsTmp[10*Assoc[i]+3] = _Equations[10*i+3];
			EquationsTmp[10*Assoc[i]+4] = _Equations[10*i+4];
			EquationsTmp[10*Assoc[i]+5] = _Equations[10*i+5];
			EquationsTmp[10*Assoc[i]+6] = _Equations[10*i+6];
			EquationsTmp[10*Assoc[i]+7] = _Equations[10*i+7];
			EquationsTmp[10*Assoc[i]+8] = _Equations[10*i+8];
			EquationsTmp[10*Assoc[i]+9] = _Equations[10*i+9];

	
			CentersTmp[3*Assoc[i]] = _Centers[3*i];
			CentersTmp[3*Assoc[i]+1] = _Centers[3*i+1];
			CentersTmp[3*Assoc[i]+2] = _Centers[3*i+2];			
		}		
	}

	free(_Centers);	
	free(_Equations);
	checkCudaErrors( cudaFree(_Equations_dev) );
	_Equations_dev = NULL;

	_Equations = EquationsTmp;	
	_Centers = CentersTmp;
	_nbPlan = count;
	
	free(Assoc);

	if (_Equations_dev != NULL)
		checkCudaErrors( cudaFree(_Equations_dev) );
	checkCudaErrors( cudaMalloc((void **) &_Equations_dev,10*_nbPlan*sizeof(float)) );
	checkCudaErrors( cudaMemcpy(_Equations_dev, _Equations, 10*_nbPlan*sizeof(float), cudaMemcpyHostToDevice) );

	checkCudaErrors( cudaDeviceSynchronize() );
	
	checkCudaErrors( cudaGraphicsMapResources ( 1, Resources_predicted_frame) );

	Segment_cu(_predicted_frame->_Label_dev, _predicted_frame->getVMap(), _predicted_frame->getNMap(), pose, _Equations_dev, _nbPlan, ACCEPT_THRESH, 0.8, _predicted_frame->_n, _predicted_frame->_m);
	
	checkCudaErrors( cudaGraphicsUnmapResources (1,  Resources_predicted_frame) );
	
	cout << "nbPlans: " << _nbPlan << endl;
	_curr_plan_idx = 0;	
	_Size = _graph->_nodes.size();
}


// Align current frame with predicted frame
bool RG::AlignFrameGaussNewton(cudaGraphicsResource_t Resources_frame, cudaGraphicsResource_t Resources_predicted_frame) {
	bool loss_track;
	float w_sift = _graph->_nodes.size() > 0 ? 10.0/float(_graph->_nodes.size()) : 1;
		
	cudaGraphicsResource_t Resources [2];
	Resources [0] = Resources_frame;
	Resources [1] = Resources_predicted_frame;

	checkCudaErrors( cudaGraphicsMapResources ( 2, Resources) );

	gpu_cpy(_img1, _frame->getRGB(), _frame->getN(), _frame->getM());

	cv::gpu::Sobel(_img1, _Gx, CV_32FC1, 1, 0, 3);
	cv::gpu::Sobel(_img1, _Gy, CV_32FC1, 0, 1, 3);
	
	/*cv::Mat tmp;
	_Gx.download(tmp);
	cv::imshow("_Gx", tmp);	*/
	
	float *VMap_prev; float *NMap_prev;
	float *VMap; float *NMap;
		
	// Load current camera pose
	/*_poseRPrev = _poseR;
	_posetPrev = _poset;*/

	Matrix3f Rcurr = _poseR;
	Vector3f tcurr = _poset;
		
	VMap_prev = _predicted_frame->getVMap(); 
	NMap_prev = _predicted_frame->getNMap();

	VMap = _frame->getVMap(); 
	NMap = _frame->getNMap();

	int fact = int(pow((float)2.0, (int)_lvl-1));

	Matrix3f Rcurr_inv = Rcurr.inverse();
	Vector3f tcurr_inv = -Rcurr_inv*tcurr;

	for (int lvl = _lvl-1; lvl > -1; lvl--) {

		Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A_sift;
		Eigen::Matrix<float, 6, 1> b_sift;

		Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
		Eigen::Matrix<float, 6, 1> b;
		
		loss_track = false;
		bool cvg = false;
		int iter = 0;
		float det;
		while (!cvg) {

			Rcurr = Rcurr_inv * _poseRPrev;
			tcurr = Rcurr_inv * _posetPrev + tcurr_inv;

			checkCudaErrors( cudaMemcpy(_Rcurr_dev, Rcurr.data(), 9*sizeof(float), cudaMemcpyHostToDevice) );
			checkCudaErrors( cudaMemcpy(_tcurr_dev, tcurr.data(), 3*sizeof(float), cudaMemcpyHostToDevice) );

			//Fill in A and b
			EstimateSystemGaussNewton(_Rcurr_dev, _tcurr_dev, VMap, NMap, _frame->getRGB(), _Gx, _Gy, 
										VMap_prev, NMap_prev, _predicted_frame->getRGB(), _intr_dev, _distThres, _angleThres, _frame->getN(), _frame->getM(), A.data (), b.data (), fact);
			
			//checking nullspace
			det = A.determinant ();

			if (fabs (det) < 1e-15 || det != det)
			{
				if (det != det) cout << "qnan" << endl;
				cout << "det null" << endl;
				
				checkCudaErrors( cudaGraphicsUnmapResources (2,  Resources) );
				return false;
			}

			Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
			double q[4];
			double norm = (result (3)*result (3) + result (4)*result (4) + result (5)*result (5));
			q[1] = result (3); q[2] = result (4); q[3] = result (5); q[0] = sqrt(1-norm);

			double tmp [3][3];
			quaternion2matrix(q, tmp); 

			Matrix3f Rinc;
			Rinc(0,0) = float(tmp[0][0]); Rinc(0,1) = float(tmp[0][1]); Rinc(0,2) = float(tmp[0][2]);
			Rinc(1,0) = float(tmp[1][0]); Rinc(1,1) = float(tmp[1][1]); Rinc(1,2) = float(tmp[1][2]);
			Rinc(2,0) = float(tmp[2][0]); Rinc(2,1) = float(tmp[2][1]); Rinc(2,2) = float(tmp[2][2]);
			Vector3f tinc = result.head<3> ();
			tcurr_inv = Rinc * tcurr_inv + tinc;
			Rcurr_inv = Rinc * Rcurr_inv;

			if ((iter > _max_iter[lvl] || ((Rinc-Matrix3f::Identity()).norm() < _thres_cvg_alpha && tinc.norm() < _thres_cvg_tr))) {
				cvg = true;
			}

			/*if (lvl > -1 && ((_poseR-Rcurr).norm() > 0.3 || (_poset-tcurr).norm() > 0.2)) {
				cout << "out at lvl : " << lvl << endl;
				Rcurr = _poseR;
				tcurr = _poset;
				Rcurr_inv = Rcurr.inverse();
				tcurr_inv = -Rcurr_inv*tcurr;
			}*/

			iter++;
		}

		fact /= 2;
	}
	
	checkCudaErrors( cudaGraphicsUnmapResources (2,  Resources) );

	/////////// Check for track loss //////////////
	if ((Rcurr - Matrix3f::Identity()).norm() > 0.3 || (tcurr).norm() > 0.3) {
		loss_track = true;
	}
	
	/////////// Check for track loss //////////////
	if (loss_track) {
		cout << "camera track lost: " << _count << endl;
		Vector3f dummy = Vector3f::Zero();
		dummy(0) = 100.0;
		return false;
	}

	////////// Update ///////////	
	Rcurr = Rcurr_inv.inverse();
	tcurr = -Rcurr*tcurr_inv;
	_poseR = Rcurr;
	_poset = tcurr;

	/*cout << Rcurr_inv << endl;
	cout << tcurr_inv << endl;*/
	
	return true;
}

void RG::ReorganizeEqua(cudaGraphicsResource_t *Resources_predicted_frame, float *pose) {
	int *Buff_dev;
	checkCudaErrors( cudaMalloc((void**)&Buff_dev, _nbPlan * sizeof(int)) );
	checkCudaErrors( cudaMemset(Buff_dev,0, _nbPlan * sizeof(int)) );

	float *Centers_dev;
	checkCudaErrors( cudaMalloc((void**)&Centers_dev, 3*_nbPlan * sizeof(float)) );
	float *count_dev;
	checkCudaErrors( cudaMalloc((void**)&count_dev, _nbPlan * sizeof(float)) );

	checkCudaErrors( cudaGraphicsMapResources (1, Resources_predicted_frame) );

	VotePlan_cu(_predicted_frame->_Label_dev, _predicted_frame->_VMap_dev, Centers_dev, count_dev, pose, _predicted_frame->getN(), _predicted_frame->getM());
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, Resources_predicted_frame) );
		
	float *count_c = (float *) malloc(_nbPlan * sizeof(float));
	checkCudaErrors( cudaMemcpy(count_c, count_dev, _nbPlan*sizeof(float), cudaMemcpyDeviceToHost) );
	
	int *Assoc = (int *) malloc(_nbPlan * sizeof(int));
	int count = 0;
	for (int i = 0; i < _nbPlan; i++) {
		if (int(count_c[i]) > MIN_SIZE_PLAN) {
			count++;
			Assoc[i] = count;
		} else {
			Assoc[i] = 0;
		}
	}
		
	checkCudaErrors( cudaMemcpy(Buff_dev, Assoc, _nbPlan*sizeof(int), cudaMemcpyHostToDevice) );
	AffectPlan_cu(_predicted_frame->_Label_dev, Buff_dev, _predicted_frame->getN(), _predicted_frame->getM());

	float *EquationsTmp = (float *) malloc(10*count*sizeof(float));
	checkCudaErrors( cudaMemcpy(_Equations, _Equations_dev, 10*_nbPlan*sizeof(float), cudaMemcpyDeviceToHost) );
	float *CentersTmp = (float *) malloc(3*count*sizeof(float));
	checkCudaErrors( cudaMemcpy(_Centers, Centers_dev, 3*_nbPlan*sizeof(float), cudaMemcpyDeviceToHost) );	

	for (int i = 0; i < _nbPlan; i++) {
		if (Assoc[i] != 0) {
			EquationsTmp[10*(Assoc[i]-1)] = _Equations[10*i];
			EquationsTmp[10*(Assoc[i]-1)+1] = _Equations[10*i+1];
			EquationsTmp[10*(Assoc[i]-1)+2] = _Equations[10*i+2];
			EquationsTmp[10*(Assoc[i]-1)+3] = _Equations[10*i+3];
			EquationsTmp[10*(Assoc[i]-1)+4] = _Equations[10*i+4];
			EquationsTmp[10*(Assoc[i]-1)+5] = _Equations[10*i+5];
			EquationsTmp[10*(Assoc[i]-1)+6] = _Equations[10*i+6];
			EquationsTmp[10*(Assoc[i]-1)+7] = _Equations[10*i+7];
			EquationsTmp[10*(Assoc[i]-1)+8] = _Equations[10*i+8];
			EquationsTmp[10*(Assoc[i]-1)+9] = _Equations[10*i+9];

	
			CentersTmp[3*(Assoc[i]-1)] = _Centers[3*i] / count_c[i];
			CentersTmp[3*(Assoc[i]-1)+1] = _Centers[3*i+1] / count_c[i];
			CentersTmp[3*(Assoc[i]-1)+2] = _Centers[3*i+2] / count_c[i];			
		}		
	}

	free(_Centers);	
	free(count_c);	
	checkCudaErrors( cudaFree(Centers_dev) );
	checkCudaErrors( cudaFree(count_dev) );
	free(_Equations);
	checkCudaErrors( cudaFree(_Equations_dev) );
	_Equations_dev = NULL;

	_Equations = EquationsTmp;	
	_Centers = CentersTmp;
	_nbPlan = count;
	
	free(Assoc);
	checkCudaErrors( cudaFree(Buff_dev) );
}