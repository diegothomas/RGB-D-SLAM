#include "stdafx.h"
#include "Graph.h"

/************ Global functions *********/

bool overlap(Primitive *Prim1, Primitive *Prim2) {

	if (Prim2 == NULL)
		return false;

	float *center_prim = Prim1->_Shift;
	int *size = Prim1->_Size;
	// Test if the projected bounding box is included
	float *corners = Prim2->getCorners();
	// Project each corners onto the plane
	float *e1 = Prim1->_e1;
	float *e2 = Prim1->_e2;
	float projll [2];
	projll [0] = corners[0]*e1[0] + corners[1]*e1[1] + corners[2]*e1[2];
	projll [1] = corners[0]*e2[0] + corners[1]*e2[1] + corners[2]*e2[2];
	float projlr [2];
	projlr [0] = corners[3]*e1[0] + corners[4]*e1[1] + corners[5]*e1[2];
	projlr [1] = corners[3]*e2[0] + corners[4]*e2[1] + corners[5]*e2[2];
	float projur [2];
	projur [0] = corners[6]*e1[0] + corners[7]*e1[1] + corners[8]*e1[2];
	projur [1] = corners[6]*e2[0] + corners[7]*e2[1] + corners[8]*e2[2];
	float projul [2];
	projul [0] = corners[9]*e1[0] + corners[10]*e1[1] + corners[11]*e1[2];
	projul [1] = corners[9]*e2[0] + corners[10]*e2[1] + corners[11]*e2[2];

	//Compute bbox
	float BBox [4];
	BBox[0] = min(projll [0], min(projlr [0], min(projur [0], projul [0])));
	BBox[1] = max(projll [0], max(projlr [0], max(projur [0], projul [0])));
	BBox[2] = min(projll [1], min(projlr [1], min(projur [1], projul [1])));
	BBox[3] = max(projll [1], max(projlr [1], max(projur [1], projul [1])));

	// Horizontal overlap
	bool hoverlap = (float(BBox [0]) < center_prim[0] + float(size[0])*RES_PLANE) && (center_prim [0] < float(BBox [0]) + float(BBox[1]-BBox[0]));
	// Vertical overlap
	bool voverlap = (float(BBox [2]) < center_prim[1] + float(size[1])*RES_PLANE) && (center_prim [1] < float(BBox [2]) + float(BBox[3]-BBox[2]));

	delete corners;

	return (hoverlap && voverlap);
}

/************** Graph class ****************/

void Graph::Update(PredictedFrame *frame, float *pose) {	
	_Size = _nodes.size();

	if (_nodes.size() == 0)
		return;

	if (_BumpTab == NULL) {
		checkCudaErrors( cudaMalloc((void **) &_BumpTab, _Size*sizeof(unsigned short *)) );
		checkCudaErrors( cudaMalloc((void **) &_RGBTab, _Size*sizeof(unsigned char *)) );
		checkCudaErrors( cudaMalloc((void **) &_MaskTab, _Size*sizeof(unsigned char *)) );
		checkCudaErrors( cudaMalloc((void **) &_ParamTab, _Size*sizeof(float *)) );

		for (int i =0; i < _Size; i++) {
			Primitive *Prim = _nodes[i];
			AffectShort(_BumpTab, Prim->_Bump_dev, i);
			AffectChar(_RGBTab, Prim->_RGB_dev, i);
			AffectChar(_MaskTab, Prim->_Mask_dev, i);
			Prim->SetParam();
			Affect(_ParamTab, Prim->_param_dev, i);
		}
	}

	checkCudaErrors( cudaGraphicsMapResources (1, frame->_Resources_t) );

	Update_cu(_BumpTab, _RGBTab, _MaskTab, _ParamTab, frame->_VMap_dev, frame->_RGB_dev, frame->_Mask_dev, frame->_Label_dev, pose, _SizeVBUFF [0], _SizeVBUFF [1], frame->getN(), frame->getM());

	checkCudaErrors( cudaGraphicsUnmapResources (1, frame->_Resources_t) );

	//Primitive *Prim = _nodes[_curr_ind];
	/*if (!Prim->Update(frame, mask, pose, _frame_buf, &_texturePrim, _SizeVBUFF, _Resources_t)) {
		delete Prim;
		_nodes.erase(_nodes.begin()+_curr_ind);
		_curr_ind--;
		return;
	}*/

	/*_nodes[_curr_ind] = NULL;
	int tmp;
	if (included(Prim, 0.1, &tmp) == -1) {
		_nodes[_curr_ind] = Prim;
	} else {
		delete Prim;
		_nodes.erase(_nodes.begin()+_curr_ind);
		_curr_ind--;
	}*/
}

void Graph::save(char *filename, int indx) {
	
	Primitive *ThePrim;
	char destfilename[100];
	float epsilon = 0.1;
	float factorBump = 30000.0f/epsilon;

	ofstream  filestr;

	// Svae curr pose
	sprintf(destfilename, "%s\\Pose%d.txt", filename, indx+1);
	filestr.open (destfilename, fstream::out);
	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}

	filestr << _pose(0,0) << " " << _pose(1,0) << " " << _pose(2,0) << " " << _pose(3,0) <<
			" " << _pose(0,1) << " " << _pose(1,1) << " " << _pose(2,1) << " " << _pose(3,1) <<
			" " << _pose(0,2) << " " << _pose(1,2) << " " << _pose(2,2) << " " << _pose(3,2) <<
			" " << _pose(0,3) << " " << _pose(1,3) << " " << _pose(2,3) << " " << _pose(3,3) << endl;

	filestr.close();

	if (indx == -1)
		return;

	sprintf(destfilename, "%s\\Graph-eq%d.txt", filename, indx);
	filestr.open (destfilename, fstream::out);
	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}

	filestr << _nodes.size() << endl;
	int count = 0;
    for (vector<Primitive *>::iterator it = _nodes.begin(); it != _nodes.end(); it++) {
		// Save current Primitive as mesh file
		ThePrim = (*it);

		if (ThePrim->_Bump == NULL) {
			cout << "Bump is null" << endl;
			filestr << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 <<
			" " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 <<
			" " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 <<
			" " << 0 << " " << 0 << endl;
			count++;
			continue;
		}

		// save equations
		float *nmle = ThePrim->_nmle;
		float *e1 = ThePrim->_e1;
		float *e2 = ThePrim->_e2;
		float *shift = ThePrim->_Shift;
		float *scale = ThePrim->_scale;
		int *size = ThePrim->_Size;
		float *anchor = ThePrim->_anchor;

		filestr << nmle[0] << " " << nmle[1]<< " " << nmle[2] << " " << ThePrim->_dist << " " << e1[0] <<
			" " << e1[1] << " " << e1[2] << " " << e2[0] << " " << e2[1] << " " << e2[2] <<
			" " << shift[0] << " " << shift[1] << " " << scale[0] << " " << scale[1] <<
			" " << size[0] << " " << size[1] <<
			" " << anchor[0] << " " << anchor[1] << " " << anchor[2] <<endl;

		int n_prim = ThePrim->_Size[0]; 
		int m_prim = ThePrim->_Size[1]; 
						
		int i,j,k;
		cv::Mat img(n_prim, m_prim, CV_16UC3);
		for (i=0,k=n_prim-1;i<n_prim;i++,k--) {
			for (j=0;j<m_prim;j++) {
				/*img.at<cv::Vec3w>(k,j)[2] = (ushort) 65535.0 * (TheBump[3*((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[1] = (ushort) 65535.0 * (TheBump[3*((int)i*m_prim + (int)j)+1]);
				img.at<cv::Vec3w>(k,j)[0] = (ushort) 65535.0 * (TheBump[3*((int)i*m_prim + (int)j)+2] + epsilon)/(2.0*epsilon);*/
				img.at<cv::Vec3w>(k,j)[2] = (ushort) ThePrim->_Bump[3*((int)i*m_prim + (int)j)];
				img.at<cv::Vec3w>(k,j)[1] = (ushort) ThePrim->_Bump[3*((int)i*m_prim + (int)j)+1];
				img.at<cv::Vec3w>(k,j)[0] = (ushort) ThePrim->_Bump[3*((int)i*m_prim + (int)j)+2];
				/*img.at<cv::Vec3b>(k,j)[2] = (uchar) (255.0*float(TheBump[3*((int)i*m_prim + (int)j)])/60000.0);
				img.at<cv::Vec3b>(k,j)[1] = (uchar) (255.0*float(TheBump[3*((int)i*m_prim + (int)j)+1])/60000.0);
				img.at<cv::Vec3b>(k,j)[0] = (uchar) (255.0*float(TheBump[3*((int)i*m_prim + (int)j)+2])/60000.0);*/
			}
		}
				
		sprintf(destfilename, "%s\\Bump%d-%d.tif", filename, indx, count);
		/*if (! cv::imwrite(destfilename, img) ) {
			cout << "error print bump" << endl;
			return;
		}*/
		while (! cv::imwrite(destfilename, img) ) {
			cout << ".";
		}
		
		for (i=0,k=n_prim-1;i<n_prim;i++,k--) {
			for (j=0;j<m_prim;j++) {
				img.at<cv::Vec3w>(k,j)[2] = (ushort) 200*unsigned short(ThePrim->_Mask_char[((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[1] = (ushort) 200*unsigned short(ThePrim->_Mask_char[((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[0] = (ushort) 200*unsigned short(ThePrim->_Mask_char[((int)i*m_prim + (int)j)]);
				/*img.at<cv::Vec3w>(k,j)[2] = 200*unsigned short(TheMask[((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[1] = 200*unsigned short(TheMask[((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[0] = 200*unsigned short(TheMask[((int)i*m_prim + (int)j)]);*/
				/*img.at<cv::Vec3b>(k,j)[2] = (uchar) TheMask[(int)i*m_prim + (int)j];
				img.at<cv::Vec3b>(k,j)[1] = (uchar) TheMask[(int)i*m_prim + (int)j];
				img.at<cv::Vec3b>(k,j)[0] = (uchar) TheMask[(int)i*m_prim + (int)j];*/
			}
		}
				
		sprintf(destfilename, "%s\\Mask%d-%d.tif", filename, indx, count);
		/*if (!cv::imwrite(destfilename, img)) {
			cout << "error print Mask" << endl;
			return;
		}*/
		while (! cv::imwrite(destfilename, img) ) {
			cout << ".";
		}
		
		for (i=0,k=n_prim-1;i<n_prim;i++,k--) {
			for (j=0;j<m_prim;j++) {
				img.at<cv::Vec3w>(k,j)[2] = 200*unsigned short(ThePrim->_RGB[3*((int)i*m_prim + (int)j)]);
				img.at<cv::Vec3w>(k,j)[1] = 200*unsigned short(ThePrim->_RGB[3*((int)i*m_prim + (int)j)+1]);
				img.at<cv::Vec3w>(k,j)[0] = 200*unsigned short(ThePrim->_RGB[3*((int)i*m_prim + (int)j)+2]);
				/*img.at<cv::Vec3b>(k,j)[2] = (uchar) TheRGB[3*((int)i*m_prim + (int)j)];
				img.at<cv::Vec3b>(k,j)[1] = (uchar) TheRGB[3*((int)i*m_prim + (int)j)+1];
				img.at<cv::Vec3b>(k,j)[0] = (uchar) TheRGB[3*((int)i*m_prim + (int)j)+2];*/
			}
		}
				
		sprintf(destfilename, "%s\\RGB%d-%d.tif", filename, indx, count);
		/*if (!cv::imwrite(destfilename, img)) {
			cout << "error print RGB" << endl;
			return;
		}*/
		while (! cv::imwrite(destfilename, img) ) {
			cout << ".";
		}

		img.~Mat();
		count++;
	}

	
    filestr.close();

	sprintf(destfilename, "%s\\Dummy%d.txt", filename, indx);
	filestr.open (destfilename, fstream::out);
	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}
    filestr.close();

}

bool Graph::exist(Primitive *Prim_in, float *nmle, float dist, float *center) {

	Primitive *prim;
	float *nmle_prim;
	float *center_prim;
	int *size;
	float error_nmle, error_dist, error_center;
	for (vector<Primitive *>::iterator it = _nodes.begin(); it != _nodes.end(); it++) {
		prim = (*it);
		if (prim == NULL)
			continue;
		
		if (prim->_Bump_dev == NULL)
			continue;

		nmle_prim = prim->_nmle;
		center_prim = prim->_Shift;
		size = prim->_Size;
		
		error_nmle = sqrt((nmle[0]-nmle_prim[0])*(nmle[0]-nmle_prim[0]) + (nmle[1]-nmle_prim[1])*(nmle[1]-nmle_prim[1]) + (nmle[2]-nmle_prim[2])*(nmle[2]-nmle_prim[2]));
		error_dist = fabs(dist-prim->_dist);
		error_center = sqrt((center[0]-center_prim[0])*(center[0]-center_prim[0]) + (center[1]-center_prim[1])*(center[1]-center_prim[1]));

		// Test if the projected bounding box is included
		float *corners = Prim_in->getCorners();
		// Project each corners onto the plane
		float *e1 = prim->_e1;
		float *e2 = prim->_e2;
		float projll [2];
		projll [0] = corners[0]*e1[0] + corners[1]*e1[1] + corners[2]*e1[2];
		projll [1] = corners[0]*e2[0] + corners[1]*e2[1] + corners[2]*e2[2];
		float projlr [2];
		projlr [0] = corners[3]*e1[0] + corners[4]*e1[1] + corners[5]*e1[2];
		projlr [1] = corners[3]*e2[0] + corners[4]*e2[1] + corners[5]*e2[2];
		float projur [2];
		projur [0] = corners[6]*e1[0] + corners[7]*e1[1] + corners[8]*e1[2];
		projur [1] = corners[6]*e2[0] + corners[7]*e2[1] + corners[8]*e2[2];
		float projul [2];
		projul [0] = corners[9]*e1[0] + corners[10]*e1[1] + corners[11]*e1[2];
		projul [1] = corners[9]*e2[0] + corners[10]*e2[1] + corners[11]*e2[2];

		//Compute bbox
		float BBox [4];
		BBox[0] = min(projll [0], min(projlr [0], min(projur [0], projul [0])));
		BBox[1] = max(projll [0], max(projlr [0], max(projur [0], projul [0])));
		BBox[2] = min(projll [1], min(projlr [1], min(projur [1], projul [1])));
		BBox[3] = max(projll [1], max(projlr [1], max(projur [1], projul [1])));

		// Horizontal overlap
		bool hoverlap = (float(BBox [0]) < center_prim[0] + float(size[0])*RES_PLANE) && (center_prim [0] < float(BBox [0]) + float(BBox[1]-BBox[0]));
		// Vertical overlap
		bool voverlap = (float(BBox [2]) < center_prim[1] + float(size[1])*RES_PLANE) && (center_prim [1] < float(BBox [2]) + float(BBox[3]-BBox[2]));

		float Shift[2];
		Shift[0] = min(center_prim [0], BBox[0]);
		Shift[1] = min(center_prim [1], BBox[2]);
		
		int NewSize[2];
		NewSize[0] = max(size[0]+int((center_prim[0] - Shift[0])/RES_PLANE), int((BBox[1]-BBox[0])/RES_PLANE)+int((BBox[0] - Shift[0])/RES_PLANE));
		NewSize[1] = max(size[1]+int((center_prim[1] - Shift[1])/RES_PLANE), int((BBox[3]-BBox[2])/RES_PLANE)+int((BBox[2] - Shift[1])/RES_PLANE));	

		bool included = int(((BBox[1]-BBox[0])/RES_PLANE)*((BBox[3]-BBox[2])/RES_PLANE)) < size[0]*size[1];
		delete corners;
		if (error_nmle < 0.4 && error_dist < 1.0e-1 && (hoverlap && voverlap) && included) {
			//prim->Merge(Prim_in, Shift, NewSize);
			//cout << "merge" << endl;
			return true;
		}
	}

	return false;
}

int Graph::included(Primitive *Prim_in, float thresh, int *nbPts, bool GPU) {

	if (GPU) {
		//checkCudaErrors( cudaGraphicsMapResources (2, Prim_in->_Resources_t) );
		*nbPts = 0;
	}

	Primitive *prim;
	float perc;
	int count_prim = 0;
	for (vector<Primitive *>::iterator it = _nodes.begin(); it != _nodes.end(); it++) {
		prim = (*it);
		if (prim == NULL) {			
			count_prim++;
			continue;
		}

		//if (GPU)
		perc = Overlap_cu(Prim_in->_Bump_dev, Prim_in->_Mask_dev, Prim_in->_param_dev, prim->_param_dev, Prim_in->_Size[0], Prim_in->_Size[1], prim->_Size[0], prim->_Size[1]);
		//else
		//	perc = Overlap_count(Prim_in, prim, thresh, nbPts);
		
		if (GPU && perc > 0.8) {	
			//checkCudaErrors( cudaGraphicsUnmapResources (2, Prim_in->_Resources_t) );
			//cout << "GPU Prim deleted because more than 90perc overlapping" << endl;
			return count_prim;
		}

		//if (!GPU && perc > 0.6 && overlap(Prim_in, prim)) {	
			//cout << " CPU Prim deleted because more than 90perc overlapping" << endl;
		//	return count_prim;
		//}

		count_prim++;
	}
	
	//if (GPU)
	//	checkCudaErrors( cudaGraphicsUnmapResources (2, Prim_in->_Resources_t) );

	return -1;
}

void Graph::Clean() {
	for (vector<Primitive *>::iterator it = _nodes.begin(); it != _nodes.end(); it++) {
		// Clean attributes of the primitive
		Primitive *ThePrim = (*it);
		ThePrim->Clean();
	}
}

