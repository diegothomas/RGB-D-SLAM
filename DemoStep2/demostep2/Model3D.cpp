#include "stdafx.h"
#include "Model3D.h"


bool POSE_CSTRN = true;

struct ParamThread
{
	PlanarPatch *Patch;
	float **VMap;
	float **NMap;
};

static void WINAPI ComputeVNNMAP(void *lpParameter)
{
	ParamThread *param = static_cast<ParamThread *>(lpParameter);
	param->Patch->ComputeVMAPNMAP(param->VMap, param->NMap);
}

bool identical(PlanarPatch * P1, PlanarPatch * P2) {
	if (P1 == NULL || P2 == NULL)
		return false;
	
	float *nmle = P1->getNmle();
	float *nmle_prim = P2->getNmle();	
	float *center =P1->getShift();
	float *center_prim =P2->getShift();
	int *size = P2->getSize();
		
	float error_nmle = sqrt((nmle[0]-nmle_prim[0])*(nmle[0]-nmle_prim[0]) + (nmle[1]-nmle_prim[1])*(nmle[1]-nmle_prim[1]) + (nmle[2]-nmle_prim[2])*(nmle[2]-nmle_prim[2]));
	float error_dist = fabs(P1->getDist()-P2->getDist());
	float error_center = sqrt((center[0]-center_prim[0])*(center[0]-center_prim[0]) + (center[1]-center_prim[1])*(center[1]-center_prim[1]));

	// Test if the projected bounding box is included
	float *corners = P1->getCorners();
	// Project each corners onto the plane
	float *e1 = P2->getE1();
	float *e2 = P2->getE2();
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
	if (error_nmle < 0.4 && error_dist < 1.0e-1 && (hoverlap && voverlap) /*&& included*/) {
		return true;
	}

	return false;
}

bool Register(vector <PlanarPatch *> Set1, vector <PlanarPatch *> Set2, vector <KeyFrame *> ListKey, KeyFrame *RefKey){
	bool res = true;
	/*ParamThread **param1;
	param1 = (ParamThread **) malloc(Set1.size()*sizeof(ParamThread *));
	HANDLE *hthread1;
	hthread1 = (HANDLE *) malloc(Set1.size()*sizeof(HANDLE));

	ParamThread **param2;
	param2 = (ParamThread **) malloc(Set2.size()*sizeof(ParamThread *));
	HANDLE *hthread2;
	hthread2 = (HANDLE *) malloc(Set2.size()*sizeof(HANDLE));

	int count = 0;
	DWORD dwID;*/

	/***Compute Positions and Normals for all planar patches ***/
	vector <float **> VMapList1;
	vector <float **> NMapList1;
	for (vector <PlanarPatch *>::iterator itP = Set1.begin(); itP < Set1.end(); itP++) {
			PlanarPatch *currP = (*itP);
			int n = currP->getSizeN();
			int m = currP->getSizeM();
	
			float **VMap, **NMap;
			VMap = (float **) malloc(n*sizeof(float *));
			if (VMap == NULL)
				perror ("The following error occurred when allocating _VMap in ComputeVMapNmap");
			NMap = (float **) malloc(n*sizeof(float *));
			if (NMap == NULL)
				perror ("The following error occurred when allocating _NMap in ComputeVMapNmap");

			for (int i = 0; i < n; i++) {
				VMap[i] = (float *) malloc(3*m*sizeof(float));
				if (VMap[i] == NULL)
					perror ("The following error occurred when allocating _VMap[i] in ComputeVMapNmap");
				memset(VMap[i], 0, 3*m*sizeof(float));
				NMap[i] = (float *) malloc(3*m*sizeof(float));
				if (NMap[i] == NULL)
					perror ("The following error occurred when allocating _NMap[i] in ComputeVMapNmap");
				memset(NMap[i], 0, 3*m*sizeof(float));
			}
			VMapList1.push_back(VMap);
			NMapList1.push_back(NMap);
			// do parallel
			/*param1[count] = new ParamThread();
			param1[count]->Patch = currP;
			param1[count]->VMap = VMap;
			param1[count]->NMap = NMap;
			hthread1[count] = CreateThread(NULL, 0,(LPTHREAD_START_ROUTINE)ComputeVNNMAP,param1[count],0,&dwID);
			count++;*/
			currP->ComputeVMAPNMAP(VMap, NMap);
	}

	//count = 0;
	vector <float **> VMapList2;
	vector <float **> NMapList2;
	for (vector <PlanarPatch *>::iterator itP = Set2.begin(); itP < Set2.end(); itP++) {
			PlanarPatch *currP = (*itP);
			int n = currP->getSizeN();
			int m = currP->getSizeM();
	
			float **VMap, **NMap;
			VMap = (float **) malloc(n*sizeof(float *));
			if (VMap == NULL)
				perror ("The following error occurred when allocating _VMap in ComputeVMapNmap");
			NMap = (float **) malloc(n*sizeof(float *));
			if (NMap == NULL)
				perror ("The following error occurred when allocating _NMap in ComputeVMapNmap");

			for (int i = 0; i < n; i++) {
				VMap[i] = (float *) malloc(3*m*sizeof(float));
				if (VMap[i] == NULL)
					perror ("The following error occurred when allocating _VMap[i] in ComputeVMapNmap");
				memset(VMap[i], 0, 3*m*sizeof(float));
				NMap[i] = (float *) malloc(3*m*sizeof(float));
				if (NMap[i] == NULL)
					perror ("The following error occurred when allocating _NMap[i] in ComputeVMapNmap");
				memset(NMap[i], 0, 3*m*sizeof(float));
			}
			VMapList2.push_back(VMap);
			NMapList2.push_back(NMap);
			// do parallel
			/*param2[count] = new ParamThread();
			param2[count]->Patch = currP;
			param2[count]->VMap = VMap;
			param2[count]->NMap = NMap;
			hthread2[count] = CreateThread(NULL, 0,(LPTHREAD_START_ROUTINE)ComputeVNNMAP,param2[count],0,&dwID);
			count++;*/
			currP->ComputeVMAPNMAP(VMap, NMap);
	}

	/*** Initialize transformation using KeyFrames whenever it is posible ***/
	float *Transfo = (float *) malloc(16*sizeof(float));
	for (int i = 0; i < 16; i++)
		Transfo[i] = 0.0;
	Transfo[0] = 1.0; Transfo[5] = 1.0; Transfo[10] = 1.0; Transfo[15] = 1.0;

	// do parallel
	cout << "size of ListKey: " << ListKey.size() << endl;
	for (vector<KeyFrame *>::iterator it = ListKey.begin(); it != ListKey.end(); it++) {
		KeyFrame *curr = (*it);
		if (RefKey->Align(curr, Transfo) > 50)// more than 30 good correspondences to align RefKey to curr
			break;
	}

	/*** Perform ICP between Set1 and Set 2 with Transfo as initialisation (i.e. Set1 is aligned and transformed to mathch Set2) ***/
	Eigen::Matrix3f Rot;
	Eigen::Vector3f tcurr;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rot(i,j) = Transfo[j*4+i];
		}
	}
	
	tcurr(0) = Transfo[12];
	tcurr(1) = Transfo[13];
	tcurr(2) = Transfo[14];
	
	cout << "SIFT: " << Rot << endl;
	cout << tcurr << endl;
	
	Eigen::MatrixXf ThePose = Eigen::Matrix4f::Identity();

	int iter = 0;
	bool converged = false;

	float pt[3];
	float nmle[3];

	while (iter < 10 && !converged) {

		/*** Compute correspondences ***/
		int i = 0;
		float Mat[27];
		for (int k = 0; k < 27; k++)			
			Mat[k] = 0.0;

		int indx1 = 0;
		for (vector <PlanarPatch *>::iterator itP = Set1.begin(); itP < Set1.end(); itP++) {
			PlanarPatch *currP = (*itP);
			float **VMap = VMapList1[indx1];
			float **NMap = NMapList1[indx1];
			indx1++;

			/*#pragma omp parallel num_threads(10)
			{
				#pragma omp for*/
				for (int row = 0; row < currP->getSizeN(); row++) {				
					for (int col = 0; col < currP->getSizeM(); col++) {
						pt [0] = VMap[row][3*col]*Rot(0,0) + VMap[row][3*col+1]*Rot(0,1) + VMap[row][3*col+2]*Rot(0,2) + tcurr(0);
						pt [1] = VMap[row][3*col]*Rot(1,0) + VMap[row][3*col+1]*Rot(1,1) + VMap[row][3*col+2]*Rot(1,2) + tcurr(1);
						pt [2] = VMap[row][3*col]*Rot(2,0) + VMap[row][3*col+1]*Rot(2,1) + VMap[row][3*col+2]*Rot(2,2) + tcurr(2);
					
						nmle [0] = NMap[row][3*col]*Rot(0,0) + NMap[row][3*col+1]*Rot(0,1) + NMap[row][3*col+2]*Rot(0,2);
						nmle [1] = NMap[row][3*col]*Rot(1,0) + NMap[row][3*col+1]*Rot(1,1) + NMap[row][3*col+2]*Rot(1,2);
						nmle [2] = NMap[row][3*col]*Rot(2,0) + NMap[row][3*col+1]*Rot(2,1) + NMap[row][3*col+2]*Rot(2,2);
					
						if (nmle[2] < 0.0)
							continue;

						// Search for corresponding point
						bool found_coresp = false;
						float min_dist = 1000.0;
						float pointClose[3];
						float pointCoord[3];
						unsigned short depth;
					
						int indx2 = 0;
						for (vector <PlanarPatch *>::iterator itP2 = Set2.begin(); itP2 < Set2.end(); itP2++) {
							PlanarPatch *currDesst = (*itP2);
							float **VMap2 = VMapList2[indx2];
							float **NMap2 = NMapList2[indx2];
							unsigned short ** Mask = currDesst->getMask();
							indx2++;
						
							float *nmle2 = currDesst->getNmle();

							float scal = nmle[0]*nmle2[0] + nmle[1]*nmle2[1] + nmle[2]*nmle2[2];
							if (scal < 0.0)
								continue;

							float *e1 = currDesst->getE1();
							float *e2 = currDesst->getE2();
							float *shift = currDesst->getShift();
							float *scale = currDesst->getScale();

							//Project point onto plane
							float proj_a = pt[0]*e1[0] + pt[1]*e1[1] + pt[2]*e1[2]; // e1
							float proj_b = pt[0]*e2[0] + pt[1]*e2[1] + pt[2]*e2[2]; // e2

							proj_a = (proj_a-shift[0])*scale[0]/2.0;
							proj_b = (proj_b-shift[1])*scale[1]/2.0; 

							int ind_i = int(proj_a);
							int ind_j = int(proj_b);

							if (ind_i > currDesst->getSizeN()-1 || ind_j > currDesst->getSizeM()-1 || ind_i  < 0 || ind_j  < 0 ) 
								continue;

							if (Mask[ind_i][ind_j] < 10)
								continue;
						
							float pt2[3];
							pt2 [0] = VMap2[ind_i][3*ind_j];
							pt2 [1] = VMap2[ind_i][3*ind_j+1];
							pt2 [2] = VMap2[ind_i][3*ind_j+2];

							float dist = sqrt((pt[0]-pt2[0])*(pt[0]-pt2[0]) + (pt[1]-pt2[1])*(pt[1]-pt2[1]) + (pt[2]-pt2[2])*(pt[2]-pt2[2]));
						
							if (dist < min_dist) {
								min_dist = dist;
								pointClose[0] = pt2[0];
								pointClose[1] = pt2[1];
								pointClose[2] = pt2[2];
							}

						}

						if (min_dist < 0.1) 
							found_coresp = true;

						float weight;
						float JD[18];
						float JRot[18];
						float row[7];
						if (found_coresp)
						{
							weight = 1.0; //0.0012/(0.0012 + 0.0019*(s[3]-0.4)*(s[3]-0.4));

							JD[0] = 1.0; JD[3] = 0.0; JD[6] = 0.0;	JD[9] = 0.0;			JD[12] = 2.0*pt[2];		JD[15] = -2.0*pt[1]; 
							JD[1] = 0.0; JD[4] = 1.0; JD[7] = 0.0;	JD[10] = -2.0*pt[2];	JD[13] = 0.0;			JD[16] = 2.0*pt[0]; 
							JD[2] = 0.0; JD[5] = 0.0; JD[8] = 1.0;	JD[11] = 2.0*pt[1];		JD[14] = -2.0*pt[0];	JD[17] = 0.0; 
				
							JRot[0] = 0.0; JRot[3] = 0.0; JRot[6] = 0.0;	JRot[9] = 0.0;			JRot[12] = 2.0*nmle[2];	JRot[15] = -2.0*nmle[1]; 
							JRot[1] = 0.0; JRot[4] = 0.0; JRot[7] = 0.0;	JRot[10] = -2.0*nmle[2];	JRot[13] = 0.0;			JRot[16] = 2.0*nmle[0]; 
							JRot[2] = 0.0; JRot[5] = 0.0; JRot[8] = 0.0;	JRot[11] = 2.0*nmle[1];	JRot[14] = -2.0*nmle[0];	JRot[17] = 0.0; 

							row[0] = weight*(-(nmle[0]*JD[0] + nmle[1]*JD[1] + nmle[2]*JD[2]) + JRot[0]*(pointClose[0]-pt[0]) + JRot[1]*(pointClose[1]-pt[1]) + JRot[2]*(pointClose[2]-pt[2]));
							row[1] = weight*(-(nmle[0]*JD[3] + nmle[1]*JD[4] + nmle[2]*JD[5]) + JRot[3]*(pointClose[0]-pt[0]) + JRot[4]*(pointClose[1]-pt[1]) + JRot[5]*(pointClose[2]-pt[2]));
							row[2] = weight*(-(nmle[0]*JD[6] + nmle[1]*JD[7] + nmle[2]*JD[8]) + JRot[6]*(pointClose[0]-pt[0]) + JRot[7]*(pointClose[1]-pt[1]) + JRot[8]*(pointClose[2]-pt[2]));
							row[3] = weight*(-(nmle[0]*JD[9] + nmle[1]*JD[10] + nmle[2]*JD[11]) + JRot[9]*(pointClose[0]-pt[0]) + JRot[10]*(pointClose[1]-pt[1]) + JRot[11]*(pointClose[2]-pt[2]));
							row[4] = weight*(-(nmle[0]*JD[12] + nmle[1]*JD[13] + nmle[2]*JD[14]) + JRot[12]*(pointClose[0]-pt[0]) + JRot[13]*(pointClose[1]-pt[1]) + JRot[14]*(pointClose[2]-pt[2]));
							row[5] = weight*(-(nmle[0]*JD[15] + nmle[1]*JD[16] + nmle[2]*JD[17]) + JRot[15]*(pointClose[0]-pt[0]) + JRot[16]*(pointClose[1]-pt[1]) + JRot[17]*(pointClose[2]-pt[2]));

							row[6] = -weight*(nmle[0]*(pointClose[0]-pt[0]) + nmle[1]*(pointClose[1]-pt[1]) + nmle[2]*(pointClose[2]-pt[2]));

							int shift_id = 0;
							for (int k = 0; k < 6; ++k)        //rows
							{
								for (int l = k; l < 7; ++l)          // cols + b
								{
									Mat[shift_id] = Mat[shift_id] + row[k] * row[l];
									shift_id++;
								}
							}
						}
					}				
				}
			//}
		}

		Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
		Eigen::Matrix<float, 6, 1> b;
		int shift_id = 0;
		for (int i = 0; i < 6; ++i) {  //rows
			for (int j = i; j < 7; ++j)    // cols + b
			{
				float value = Mat[shift_id++];
				if (j == 6)       // vector b
					b(i) = value;
				else
					A(j,i) = A(i,j) = value;
			}
		}

		//checking nullspace
		float det = A.determinant ();

		if (fabs (det) < 1e-15 || det != det)
		{
			if (det != det) cout << "qnan" << endl;
			cout << "det null" << endl;
			res = false;
			goto ENDREGISTER;//return;
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

		tcurr= Rinc * tcurr + tinc;
		Rot = Rinc * Rot;

		if (((Rinc-Matrix3f::Identity()).norm() < 1.0e-6 && tinc.norm() < 1.0e-6)) {
			converged = true;
		}

		iter++;
	}

	cout << "Rot: " << Rot << " Translation: " << tcurr;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ThePose(i,j) = Rot(i,j);
		}
		ThePose(i,3) = tcurr(i);
	}


	for (vector <PlanarPatch *>::iterator itP = Set1.begin(); itP < Set1.end(); itP++) {
		PlanarPatch *currP = (*itP);
		currP->Transform(ThePose);
	}
	RefKey->Transform(ThePose);

ENDREGISTER:

	free(Transfo);

	int i = 0;
	for (vector <PlanarPatch *>::iterator itP = Set1.begin(); itP < Set1.end(); itP++) {
			PlanarPatch *currP = (*itP);
			int n = currP->getSizeN();
	
			float **VMap, **NMap;
			VMap = VMapList1[i];			
			NMap = NMapList1[i];

			for (int j = 0; j < n; j++) {
				free(VMap[j]);
				free(NMap[j]);
			}
			free(VMap);
			free(NMap);
			i++;
	}
	VMapList1.clear();
	NMapList1.clear();

	i = 0;
	for (vector <PlanarPatch *>::iterator itP = Set2.begin(); itP < Set2.end(); itP++) {
			PlanarPatch *currP = (*itP);
			int n = currP->getSizeN();
	
			float **VMap, **NMap;
			VMap = VMapList2[i];			
			NMap = NMapList2[i];

			for (int j = 0; j < n; j++) {
				free(VMap[j]);
				free(NMap[j]);
			}
			free(VMap);
			free(NMap);
			i++;
	}
	VMapList2.clear();
	NMapList2.clear();

	return res;
}

/***************************************************************************************/
/**************************** Methods for the class Model3D ****************************/
/***************************************************************************************/


void Model3D::DrawGraph(bool keyf) {
	if (keyf) {
		/*** Draw Key Frames ***/
		glColor3f(0.0, 1.0, 0.0);
		for (vector<KeyFrame *>::iterator it = _Keyframes.begin(); it != _Keyframes.end(); it++) {
			glPushMatrix();
			glTranslatef( (*it)->_pose(0,3), (*it)->_pose(1,3), (*it)->_pose(2,3) );
			gluSphere(gluNewQuadric(), 0.1, 10, 10);
			glPopMatrix();

			glBegin(GL_LINES);
			glLineWidth(2);

			glVertex3f((*it)->_pose(0,3), (*it)->_pose(1,3), (*it)->_pose(2,3));
			glVertex3f((*it)->_pose(0,3) + 0.5*(*it)->_pose(0,0), (*it)->_pose(1,3) + 0.5*(*it)->_pose(1,0), (*it)->_pose(2,3) + 0.5*(*it)->_pose(2,0));
		
			glVertex3f((*it)->_pose(0,3), (*it)->_pose(1,3), (*it)->_pose(2,3));
			glVertex3f((*it)->_pose(0,3) + 0.5*(*it)->_pose(0,1), (*it)->_pose(1,3) + 0.5*(*it)->_pose(1,1), (*it)->_pose(2,3) + 0.5*(*it)->_pose(2,1));
		
			glVertex3f((*it)->_pose(0,3), (*it)->_pose(1,3), (*it)->_pose(2,3));
			glVertex3f((*it)->_pose(0,3) + 0.5*(*it)->_pose(0,2), (*it)->_pose(1,3) + 0.5*(*it)->_pose(1,2), (*it)->_pose(2,3) + 0.5*(*it)->_pose(2,2));

			glEnd();
		}	
	}

	glColor3f(1.0, 0.0, 0.0);
	//display graph
	PlanarPatch *ThePrim;
	if (_MergedPrim.empty()) {
		for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++) {
			ThePrim = (*it);
			if (ThePrim == NULL)
				continue;
			
			glColor3f(ThePrim->_colorPrim[0], ThePrim->_colorPrim[1], ThePrim->_colorPrim[2]);
			float *center = ThePrim->getCenter(0.5, 0.5);

			glPushMatrix();
			glTranslatef( center[0], center[1], center[2] );
			gluSphere(gluNewQuadric(), 0.1, 10, 10);
			glPopMatrix();

			delete center;
		}
	} else {
		for (vector<PlanarPatch *>::iterator it = _MergedPrim.begin(); it != _MergedPrim.end(); it++) {
			ThePrim = (*it);
			if (ThePrim == NULL)
				continue;
			
			glColor3f(ThePrim->_colorPrim[0], ThePrim->_colorPrim[1], ThePrim->_colorPrim[2]);
			float *center = ThePrim->getCenter(0.5, 0.5);

			glPushMatrix();
			glTranslatef( center[0], center[1], center[2] );
			gluSphere(gluNewQuadric(), 0.1, 10, 10);
			glPopMatrix();

			delete center;
		}
	}
	
	if (keyf) {
		/*** Draw edges ***/
		glBegin(GL_LINES);
		glLineWidth(2);

		glColor3f(0.0, 0.0, 0.0);
		double *v = new double [7];
		for (OptimizableGraph::EdgeSet::const_iterator it=_optimizer.edges().begin(); it!=_optimizer.edges().end(); ++it){
			EdgeSE3* e = (EdgeSE3*) (*it);
			VertexSE3* v1 = (VertexSE3*) e->vertices()[0];
			v1->getEstimateData(v);
			glVertex3f(float(v[0]), float(v[1]), float(v[2]));

			VertexSE3* v2 = (VertexSE3*) e->vertices()[1];
			v2->getEstimateData(v);
			glVertex3f(float(v[0]), float(v[1]), float(v[2]));
		}
		delete [] v;
		glEnd();
	}
	
	glColor3f(1.0, 1.0, 1.0);
}

void Model3D::LoadModel3D(char *filename) {
	char destfilename[100];

	for (int i = 0; i < 100; i++) {
		sprintf(destfilename, "%s\\MPrim%d", filename, i);
		PlanarPatch *Prim = new PlanarPatch(1024, 1024, destfilename);
		Prim->LoadEquation(destfilename);

		//if (Prim->getSizeN() < 2000 && Prim->getSizeM() < 2000)
			AddPrim(Prim);
		
		//vector<PlanarPatch *> PrimList;
		//if (Prim->getSizeN() > 512 || Prim->getSizeM() > 512) {
		//	//Segment the primitive in multiple subimages
		//	PrimList = Prim->Segment();
		//	for (int j = 0; j < PrimList.size(); j++) {
		//		AddPrim(PrimList[j]);
		//	}
		//	delete Prim;
		//} else {
		//	/*if (Prim->getSizeN()*Prim->getSizeM() < 128*128) {
		//		Prim->setNM(128, 128, 0);
		//	} else if (Prim->getSizeN()*Prim->getSizeM() < 256*256) {
		//		Prim->setNM(256, 256, 1);
		//	} else {*/
		//		Prim->setNM(512, 512, 2);
		//	//}
		//
		//	AddPrim(Prim);
		//}

		/*if (Prim->getSizeN()*Prim->getSizeM() < 128*128) {
			Prim->setNM(128, 128, 0);
		} else if (Prim->getSizeN()*Prim->getSizeM() < 256*256) {
			Prim->setNM(256, 256, 1);
		} else if (Prim->getSizeN()*Prim->getSizeM() < 512*512) {
			Prim->setNM(512, 512, 2);
		} else if (Prim->getSizeN()*Prim->getSizeM() < 1024*1024) {
			Prim->setNM(1024, 1024, 3);
		} else if (Prim->getSizeN()*Prim->getSizeM() < 2048*2048) {
			Prim->setNM(2048, 2048, 4);
		} else {
			cout << "Error prim to big!" << endl;
			delete Prim;
			continue;
		}*/
		
		//AddPrim(Prim);		
	}
}

void Model3D::UpdateWorking(float x, float y, float z, float lx, float ly, float lz, char *filename) { 
//	char destfilename[100];

	int i = 0;
	for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++) {
		PlanarPatch * Prim = (PlanarPatch *) (*it);
		//sprintf(destfilename, "%s\\Prim%d", filename, i);
		i++;
		
		Prim->Load(Prim->getFilename());

		_WorkingPrim.push_back(Prim);

	}
}

void Model3D::UpdateVisible(float x, float y, float z, float lx, float ly, float lz) {

	//for (vector<PlanarPatch *>::iterator it = _ViewedPrim.begin(); it != _ViewedPrim.end(); it++) {
	//	PlanarPatch * Prim = (*it);

	//	if (!Prim->IsVisible(x, y, z, lx, ly, lz)) {
	//		Prim->UnloadFromDevice();
	//		/*int lvl_dta = Prim->getlvlData();
	//		int lvl_vbo = 0;
	//		if (lvl_dta < 2) {
	//			lvl_vbo = 0;
	//		} else if (lvl_dta < 3) {
	//			lvl_vbo = 1;
	//		} else {
	//			lvl_vbo = 2;
	//		}
	//		_idx_data_BCM[lvl_dta].push_back(Prim->getidDt());
	//		_idx_data_VBO[lvl_vbo].push_back(Prim->getidVBO());*/

	//		it = _ViewedPrim.erase(it)-1;
	//	}
	//}

	int count = 0;

	for (vector<PlanarPatch *>::iterator it = _WorkingPrim.begin(); it != _WorkingPrim.end(); it++) {
		PlanarPatch * Prim = (PlanarPatch *) (*it);
		if (Prim->IsView() /*|| !Prim->IsVisible(x, y, z, lx, ly, lz)*/)
			continue;
		
		//int lvl_dta = Prim->getlvlData();
		//int lvl_vbo = 0;
		//if (lvl_dta < 2) {
		//	lvl_vbo = 0;
		//} else if (lvl_dta < 3) {
		//	lvl_vbo = 1;
		//} else {
		//	lvl_vbo = 2;
		//}

		//if (_idx_data_BCM[lvl_dta].empty() || _idx_data_VBO[lvl_vbo].empty()){
		//	cout << "Not enough memory space!" << endl;
		//	continue;
		//}		

		////cout << "lvl_dta: " << lvl_dta << ", lvl_vbo: " << lvl_vbo << endl;

		//int idDt = _idx_data_BCM[lvl_dta].back();
		//_idx_data_BCM[lvl_dta].pop_back();
		//int idVBO = _idx_data_VBO[lvl_vbo].back();
		//_idx_data_VBO[lvl_vbo].pop_back();
		//Prim->LoadToDevice(Prim->getlvl());

		/*size_t available, total;
		cudaMemGetInfo(&available, &total);
		cout << "Available memory: " << available << "Total Memory: " << total << endl;*/

		//Prim->setIdx(idDt, idVBO);

		//cout << "idDt: " << idDt << ", idVBO: " << idVBO << endl;

		//Prim->UpdateVBO();

		_ViewedPrim.push_back(Prim);
		Prim->SetView(true);
		Prim->setViewId(_ViewedPrim.size()-1);

		/*count++;
		if (count == 2)
			return;*/
	}

}

void Model3D::AddKeyFrame(KeyFrame *Key_in) {
	/*** Add a new vertix in the optimizable graph that correspond to the pose of the KeyFrame ***/
	int sizegraph = _Keyframes.size()+_AllPrim.size();
	double *v = Key_in->getQuaternion();
	VertexSE3* prim_v =  new VertexSE3;
	prim_v->setId(sizegraph);
	prim_v->setEstimateDataImpl(v);
	_optimizer.addVertex(prim_v);
	Key_in->setIndx(sizegraph);

	/*** Add the keyframe pose constraint into the graph ***/
	if (sizegraph > 0 && POSE_CSTRN) {
		EdgeSE3* rigid_cstr;
		rigid_cstr = new EdgeSE3;
		rigid_cstr->vertices()[0] = _optimizer.vertex(Key_in->getIndx());
		rigid_cstr->vertices()[1] = _optimizer.vertex(_Keyframes.back()->getIndx());
		rigid_cstr->setMeasurementFromState();	
		_optimizer.addEdge(rigid_cstr);
	}

	_Keyframes.push_back(Key_in);
}

void Model3D::AddPrim(PlanarPatch *Prim_in) {	
	/*** Add a new vertix in the optimizable graph that correspond to the anchor point of the patch ***/
	int sizegraph = _Keyframes.size()+_AllPrim.size();
	double *v = Prim_in->getQuaternion();
	VertexSE3* prim_v =  new VertexSE3;
	prim_v->setId(sizegraph);
	prim_v->setEstimateDataImpl(v);
	_optimizer.addVertex(prim_v);
	Prim_in->setIndx(sizegraph);
		
	/*** Add the visibility constraint into the graph ***/
	EdgeSE3* cam_cstr = new EdgeSE3;
	cam_cstr->vertices()[0] = _optimizer.vertex(Prim_in->getIndx());
	cam_cstr->vertices()[1] = _optimizer.vertex(_Keyframes.back()->getIndx());
	cam_cstr->setMeasurementFromState();
	_optimizer.addEdge(cam_cstr);

	_Keyframes.back()->_nodes.push_back(_AllPrim.size());
	_AllPrim.push_back(Prim_in);
	_ViewedPrim.push_back(Prim_in);
	Prim_in->ComputeTriangulationLight();
	Prim_in->SetView(true);
}

//void Model3D::AddPrim(PlanarPatch *Prim_in) {
//	
//	PlanarPatch *WorkingPatch = NULL;
//
//	/*** First test if the primitive has a valid match in the global model and process the two cases***/
//	float *nmle = Prim_in->getNmle();
//	float dist = Prim_in->getDist();
//	float *center = Prim_in->getCenter(0.0,0.0);
//
//	PlanarPatch *prim;
//	float *nmle_prim;
//	float *center_prim;
//	int *size;
//	float error_nmle, error_dist;
//	bool merged = false;
//	for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++) {
//		prim = (*it);
//		if (prim == NULL)
//			continue;
//		
//		if (prim->getBump() == NULL)
//			continue;
//
//		nmle_prim = prim->getNmle();
//		center_prim = prim->getShift();
//		size = prim->getSize();
//		
//		error_nmle = sqrt((nmle[0]-nmle_prim[0])*(nmle[0]-nmle_prim[0]) + (nmle[1]-nmle_prim[1])*(nmle[1]-nmle_prim[1]) + (nmle[2]-nmle_prim[2])*(nmle[2]-nmle_prim[2]));
//		error_dist = fabs(dist-prim->getDist());
//
//		// Test if the projected bounding box is included
//		float *corners = Prim_in->getCorners();
//		// Project each corners onto the plane
//		float *e1 = prim->getE1();
//		float *e2 = prim->getE2();
//		float projll [2];
//		projll [0] = corners[0]*e1[0] + corners[1]*e1[1] + corners[2]*e1[2];
//		projll [1] = corners[0]*e2[0] + corners[1]*e2[1] + corners[2]*e2[2];
//		float projlr [2];
//		projlr [0] = corners[3]*e1[0] + corners[4]*e1[1] + corners[5]*e1[2];
//		projlr [1] = corners[3]*e2[0] + corners[4]*e2[1] + corners[5]*e2[2];
//		float projur [2];
//		projur [0] = corners[6]*e1[0] + corners[7]*e1[1] + corners[8]*e1[2];
//		projur [1] = corners[6]*e2[0] + corners[7]*e2[1] + corners[8]*e2[2];
//		float projul [2];
//		projul [0] = corners[9]*e1[0] + corners[10]*e1[1] + corners[11]*e1[2];
//		projul [1] = corners[9]*e2[0] + corners[10]*e2[1] + corners[11]*e2[2];
//
//		//Compute bbox
//		float BBox [4];
//		BBox[0] = min(projll [0], min(projlr [0], min(projur [0], projul [0])));
//		BBox[1] = max(projll [0], max(projlr [0], max(projur [0], projul [0])));
//		BBox[2] = min(projll [1], min(projlr [1], min(projur [1], projul [1])));
//		BBox[3] = max(projll [1], max(projlr [1], max(projur [1], projul [1])));
//
//		// Horizontal overlap
//		bool hoverlap = (float(BBox [0]) < center_prim[0] + float(size[0])*RES_PLANE) && (center_prim [0] < float(BBox [0]) + float(BBox[1]-BBox[0]));
//		// Vertical overlap
//		bool voverlap = (float(BBox [2]) < center_prim[1] + float(size[1])*RES_PLANE) && (center_prim [1] < float(BBox [2]) + float(BBox[3]-BBox[2]));
//
//		float Shift[2];
//		Shift[0] = min(center_prim [0], BBox[0]);
//		Shift[1] = min(center_prim [1], BBox[2]);
//		
//		int NewSize[2];
//		NewSize[0] = max(size[0]+int((center_prim[0] - Shift[0])/RES_PLANE), int((BBox[1]-BBox[0])/RES_PLANE)+int((BBox[0] - Shift[0])/RES_PLANE));
//		NewSize[1] = max(size[1]+int((center_prim[1] - Shift[1])/RES_PLANE), int((BBox[3]-BBox[2])/RES_PLANE)+int((BBox[2] - Shift[1])/RES_PLANE));	
//
//		bool included = true;// int(((BBox[1]-BBox[0])/RES_PLANE)*((BBox[3]-BBox[2])/RES_PLANE)) < size[0]*size[1];
//		delete corners;
//		if (error_nmle < 0.4 && error_dist < 1.0e-1 && (hoverlap && voverlap) && included) {
//			prim->Merge(Prim_in, Shift, NewSize);
//			prim->LinearInterpolation();
//			cout << "merge" << endl;
//			merged = true;
//			WorkingPatch = prim;
//			break;
//		}
//	}
//
//	delete center;
//	
//	if (merged) {
//		delete Prim_in;
//	} else {
//		_AllPrim.push_back(Prim_in);
//		WorkingPatch = Prim_in;
//	}
//
//	/*** Generate the triangulation for the newly added Patch ***/
//
//	if (WorkingPatch)
//		WorkingPatch->ComputeTriangulation();
//}

void Model3D::AddGeoCnstr() {	
		
	/*** Add the geometric constraints into the graph ***/
	for (int i = 0; i < _ViewedPrim.size(); i++) {
		for (int j = i+1; j < _ViewedPrim.size(); j++) {
			EdgeSE3* cam_cstr = new EdgeSE3;
			cam_cstr->vertices()[0] = _optimizer.vertex(_ViewedPrim[i]->getIndx());
			cam_cstr->vertices()[1] = _optimizer.vertex(_ViewedPrim[j]->getIndx());
			cam_cstr->setMeasurementFromState();
			_optimizer.addEdge(cam_cstr);
		}
	}

	//_ViewedPrim.clear();
}

void Model3D::AddIdCnstr(vector <PlanarPatch *> Set1, vector <PlanarPatch *> Set2) {	
		
	/*** Add the identity constraints into the graph ***/
	for (int i = 0; i < Set1.size(); i++) {
		for (int j = 0; j < Set2.size(); j++) {
			if (identical(Set1[i], Set2[j])) {
				EdgeSE3* cam_cstr = new EdgeSE3;
				cam_cstr->vertices()[0] = _optimizer.vertex(Set1[i]->getIndx());
				cam_cstr->vertices()[1] = _optimizer.vertex(Set2[j]->getIndx());
				cam_cstr->setMeasurementFromState();
				_optimizer.addEdge(cam_cstr);
			}
		}
	}

	//_ViewedPrim.clear();
}

void Model3D::FragmentRegistration() {

	/***Step 1: Identify set of neighbourhing keyframes ***/
	vector <KeyFrame *> ListKey;
	vector <int> ListIndx;
	int indx = 0;
	for (vector<KeyFrame *>::iterator it = _Keyframes.begin(); it != _Keyframes.end(); it++) {
		KeyFrame *curr = (*it);
		if (curr != _Keyframes.back() && IsNeighboor(curr, _Keyframes.back())) {
			ListKey.push_back(curr);
			ListIndx.push_back(indx);
		}
		indx++;
	}

	cout << "Step 1 done : " << ListKey.size() << endl;
	
	/***Step 2: Break the set of neighbourhing keyframes into sets of consecutive KeyFrames***/	
	vector <vector <KeyFrame *>> MultiListKey;	
	vector <KeyFrame *> currList;
	int i = 0;
	int prev_indx = -1;
	for (vector<KeyFrame *>::iterator it = ListKey.begin(); it != ListKey.end(); it++) {
		KeyFrame *curr = (*it);
		indx = ListIndx[i];
		if (prev_indx == -1 || indx == prev_indx+1) {
			currList.push_back(curr);
		} else {
			MultiListKey.push_back(currList);
			currList.clear();
			currList.push_back(curr);
		}
		prev_indx = indx;
		i++;
	}
	MultiListKey.push_back(currList);

	cout << "Step 2 done : " << MultiListKey.size() << endl;
	
	/***Step 3: Register the set of planar patches linked to the last keyFrame to all sets of KeyFrames iteratively***/
	KeyFrame *LastKey = _Keyframes.back();
	vector <PlanarPatch *> LastSet;
	for (vector<int>::iterator it = LastKey->_nodes.begin(); it != LastKey->_nodes.end(); it++) {
		LastSet.push_back(_AllPrim[(*it)]);
	}

	for (i = 0; i < MultiListKey.size(); i++) {
		//Step 3-a: Register set of lastKeyFrame to the one of all Key
		vector <PlanarPatch *> CurrSet;
		for (vector<KeyFrame *>::iterator itK = MultiListKey[i].begin(); itK != MultiListKey[i].end(); itK++) {
			KeyFrame *CurrKey = (*itK);
			for (vector<int>::iterator it = CurrKey->_nodes.begin(); it != CurrKey->_nodes.end(); it++) {
				CurrSet.push_back(_AllPrim[(*it)]);
			}
		}

		if (Register(LastSet, CurrSet, MultiListKey[i], _Keyframes.back())) {
			//Step 3-b: Add all identity constraints
			for (vector<PlanarPatch *>::iterator it = LastSet.begin(); it != LastSet.end(); it++) {
				PlanarPatch *CurrPatch = (*it);
				VertexSE3 *prim_v = (VertexSE3 *) _optimizer.vertex(CurrPatch->getIndx());
				double *v = CurrPatch->getQuaternion();
				prim_v->setEstimateDataImpl(v);
				delete v;
			}
			AddIdCnstr(LastSet, CurrSet);

			//Step 3-c: Optimize the graph
			if (!MultiListKey[i].empty())
				OptimizeGraph(MultiListKey[i][0]->_nodes, LastSet, MultiListKey[i][0], _Keyframes.back());
		}

		CurrSet.clear();
	}

	cout << "Step 3 done" << endl;

	LastSet.clear();
}

void Model3D::OptimizeGraph(vector <int> Set1, vector <PlanarPatch *> Set2, KeyFrame *MatchKey, KeyFrame *RefKey) {

	// Fix local models related to lastSet and CurrSet
	for (vector<int>::iterator it = Set1.begin(); it != Set1.end(); it++) {
		PlanarPatch *CurrPatch = _AllPrim[(*it)];
		VertexSE3 *prim_v = (VertexSE3 *) _optimizer.vertex(CurrPatch->getIndx());
		prim_v->setFixed(true);
	}

	for (vector<PlanarPatch *>::iterator it = Set2.begin(); it != Set2.end(); it++) {
		PlanarPatch *CurrPatch = (*it);
		VertexSE3 *prim_v = (VertexSE3 *) _optimizer.vertex(CurrPatch->getIndx());
		prim_v->setFixed(true);
	}

	//Fix the two keyframes
	VertexSE3 *prim_k = (VertexSE3 *) _optimizer.vertex(MatchKey->getIndx());
	prim_k->setFixed(true);
	
	prim_k = (VertexSE3 *) _optimizer.vertex(RefKey->getIndx());
	double *v_k = RefKey->getQuaternion();
	prim_k->setEstimateDataImpl(v_k);
	prim_k->setFixed(true);
	delete v_k;

	//_optimizer.save("fusion_before.g2o");
	_optimizer.setVerbose(true);
	cout << "Optimizing" << endl;
	_optimizer.initializeOptimization();
	_optimizer.optimize(20);
	cout << "done." << endl;
	//_optimizer.save("fusion_after.g2o");

	//Affect new positions to all vertices
	for (vector<PlanarPatch *>::iterator it = _AllPrim.begin(); it != _AllPrim.end(); it++) {
		PlanarPatch *Prim = (*it);
		VertexSE3* prim_v = (VertexSE3*) _optimizer.vertex(Prim->getIndx());

		double *v = new double [7];
		prim_v->getEstimateData(v);

		Prim->SetFromQuaternion(v);

		prim_v->setFixed(false);
		delete v;
	}

	for (vector<KeyFrame *>::iterator it = _Keyframes.begin(); it != _Keyframes.end(); it++) {
		KeyFrame *cam = (*it);
		VertexSE3* prim_v = (VertexSE3*) _optimizer.vertex(cam->getIndx());

		double *v = new double [7];
		prim_v->getEstimateData(v);

		cam->SetFromQuaternion(v);

		prim_v->setFixed(false);
		delete v;
	}

}

void Model3D::UpdateGroups() {
	int nbNewPrim = _ViewedPrim.size();
	int nbPrim = _AllPrim.size();

	/*** Find the group that corresponds to each new primitive ***/
	for (int i = 0; i < _ViewedPrim.size(); i++) {
		bool stop = false;
		for (vector<Group *>::iterator it = _Groups.begin(); it != _Groups.end(); it++) {
			Group *currGroup = (*it);
			for (vector<int>::iterator it2 = currGroup->_indices.begin(); it2 != currGroup->_indices.end(); it2++) {
				if (identical(_ViewedPrim[i], _AllPrim[(*it2)])) {
					// add new primitive in the group
					currGroup->_indices.push_back(nbPrim - nbNewPrim + i);
					_ViewedPrim[i]->_colorPrim[0] = currGroup->_colorGroup[0];
					_ViewedPrim[i]->_colorPrim[1] = currGroup->_colorGroup[1];
					_ViewedPrim[i]->_colorPrim[2] = currGroup->_colorGroup[2];
					stop = true;
					break;
				}
			}
			if (stop)
				break;
		}
		if (!stop) {
			//create a new group
			Group *currGroup = new Group();
			currGroup->_indices.push_back(nbPrim - nbNewPrim + i);
			currGroup->_colorGroup[0] = float(rand())/RAND_MAX;
			currGroup->_colorGroup[1] = float(rand())/RAND_MAX;
			currGroup->_colorGroup[2] = float(rand())/RAND_MAX;
			_Groups.push_back(currGroup);
			_ViewedPrim[i]->_colorPrim[0] = currGroup->_colorGroup[0];
			_ViewedPrim[i]->_colorPrim[1] = currGroup->_colorGroup[1];
			_ViewedPrim[i]->_colorPrim[2] = currGroup->_colorGroup[2];
		}
	}

	cout << "nb groups: " << _Groups.size() << endl;
	if (_Groups.size() == 44) {
		cout << "Error coming" << endl; 
	}


}

void Model3D::Update() {
	char Dumname[100];
	char Eqname[100];
	char Bumpname[100];
	char RGBname[100];
	char Maskname[100];
	string line;
	char tmpline[200];
	int nbPrim;
	
	ifstream  filestr;
	sprintf(Dumname, "%s\\Dummy%d.txt", _path, _indx);
	filestr.open (Dumname, fstream::in);
	while (!filestr.is_open()) {
		return;
	}
	filestr.close();
	//remove ( Dumname );

	/****** A new KeyFrame is available ******/
	KeyFrame *Frame = new KeyFrame(_path, _indx+1);
	AddKeyFrame(Frame);

	/****** A new set of planar patches is available ******/

	// Load the set of equations and get number of patches
	sprintf(Eqname, "%s\\Graph-eq%d.txt", _path, _indx);
	filestr.open (Eqname, fstream::in);
	while (!filestr.is_open()) {
		cout << "Could not open " << Eqname << endl;
		_indx++;
		return;
	}

	getline (filestr,line);
	strcpy(tmpline, line.c_str());
	sscanf (tmpline,"%d", &nbPrim);
	// Load the new patches into the global mesh

	for (int p = 0; p < nbPrim; p++) {
		// Load current patch

		// load equations
		float nmle[3];
		float e1[3];
		float e2[3];
		float shift[2];
		float scale[2];
		int size[2];
		float dist;
		float anchor[3];
		
		getline (filestr,line);
		strcpy(tmpline, line.c_str());
		sscanf (tmpline,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %f %f %f", &nmle[0], &nmle[1], &nmle[2], &dist, 
			&e1[0], &e1[1], &e1[2], &e2[0], &e2[1], &e2[2], &shift[0], &shift[1], &scale[0], &scale[1], &size[0], &size[1], &anchor[0], &anchor[1], &anchor[2]);

		if (size[0]*size[1] == 0)
			continue;

		// load attributes
		unsigned short **Bump = (unsigned short **) malloc(size[0]*sizeof(unsigned short *));
		if (Bump == NULL)
			perror ("The following error occurred when allocating Bump in Load Graph");

		unsigned char **RGB = (unsigned char **) malloc(size[0]*sizeof(unsigned char *));
		if (RGB == NULL)
			perror ("The following error occurred when allocating RGB in Load Graph");
		
		unsigned short **Mask = (unsigned short **) malloc(size[0]*sizeof(unsigned short *));
		if (Mask == NULL)
			perror ("The following error occurred when allocating Mask in Load Graph");

		for (int q = 0; q < size[0]; q++) {
			Bump[q] = (unsigned short *) malloc(3*size[1]*sizeof(unsigned short));
			if (Bump[q] == NULL)
				perror ("The following error occurred when allocating Bump[q] in Load Graph");
			RGB[q] = (unsigned char *) malloc(3*size[1]*sizeof(unsigned char));
			if (RGB[q] == NULL)
				perror ("The following error occurred when allocating RGB[q] in Load Graph");
			Mask[q] = (unsigned short *) malloc(size[1]*sizeof(unsigned short));
			if (Mask[q] == NULL)
				perror ("The following error occurred when allocating Mask[q] in Load Graph");
		}

		cv::Mat img_bump;
		cv::Mat img_rgb; 
		cv::Mat img_mask;
		sprintf(Bumpname, "%s\\Bump%d-%d.tif", _path, _indx, p);
		img_bump = cv::imread(Bumpname, CV_LOAD_IMAGE_UNCHANGED);
		if(! img_bump.data ) 
			cout <<  "Could not open or find the img_bump" << std::endl ;

		sprintf(RGBname, "%s\\RGB%d-%d.tif", _path, _indx, p);
		img_rgb = cv::imread(RGBname, CV_LOAD_IMAGE_UNCHANGED);
		if(! img_rgb.data ) 
			cout <<  "Could not open or find the img_rgb" << std::endl ;
		
		sprintf(Maskname, "%s\\Mask%d-%d.tif", _path, _indx, p);
		img_mask = cv::imread(Maskname, CV_LOAD_IMAGE_UNCHANGED);
		if(! img_mask.data ) 
			cout <<  "Could not open or find the img_mask" << std::endl ;
				
		int i,j,k;
		for (i=0,k=size[0]-1;i<size[0];i++,k--) {
			for (j=0;j<size[1];j++) {
				Bump[k][3*j] = unsigned short(img_bump.at<cv::Vec3w>(i,j)[2]);
				Bump[k][3*j+1] = unsigned short(img_bump.at<cv::Vec3w>(i,j)[1]);
				Bump[k][3*j+2] = unsigned short(img_bump.at<cv::Vec3w>(i,j)[0]);	

				RGB[k][3*j] = unsigned char(img_rgb.at<cv::Vec3w>(i,j)[2]/200);
				RGB[k][3*j+1] = unsigned char(img_rgb.at<cv::Vec3w>(i,j)[1]/200);
				RGB[k][3*j+2] = unsigned char(img_rgb.at<cv::Vec3w>(i,j)[0]/200);
				
				Mask[k][j] = img_mask.at<cv::Vec3w>(i,j)[0]/200;	
			}
		}

		img_bump.release();
		img_rgb.release();
		img_mask.release();
		

		//remove ( Bumpname );
		//remove ( RGBname );
		//remove ( Maskname );
				
		PlanarPatch *ThePrim = new PlanarPatch(nmle, e1, e2, dist, scale, size, shift, anchor);
		ThePrim->Allocate(Bump, RGB, Mask);	

		if (ThePrim->Clean())
			AddPrim(ThePrim);
		else
			delete ThePrim;
	}
	
	filestr.close();
	//remove ( Eqname );

	cout << "nb of Patches: " << _AllPrim.size() << endl; 

	_indx++;

	/*** Add geometric constraints ***/
	AddGeoCnstr();

	/*** Do fragment registration ***/
	//cout << "FragmentRegistration" << endl;
	FragmentRegistration();

	/*** Group identical planar patches ***/
	UpdateGroups();
	
	/*for (int i = 0; i < _AllPrim.size(); i++) {
		_AllPrim[i]->SetView(true);
		_AllPrim[i]->ComputeTriangulation();
	}*/

	_ViewedPrim.clear();
}

void Model3D::Merge() {
	// Compute equations of the merged prim as the mean
	for (vector<Group *>::iterator it = _Groups.begin(); it != _Groups.end(); it++) {
		Group *currGroup = (*it);
		float nmle [3] = {0.0,0.0,0.0};
		float dist = 0.0;
		float count = 0.0;

		for (vector<int>::iterator it2 = currGroup->_indices.begin(); it2 != currGroup->_indices.end(); it2++) {
			PlanarPatch *prim = _AllPrim[(*it2)];
			float *currnmle = prim->getNmle();
			float currdist = prim->getDist();

			nmle[0] = (count*nmle[0] + currnmle[0]) / (count + 1.0);
			nmle[1] = (count*nmle[1] + currnmle[1]) / (count + 1.0);
			nmle[2] = (count*nmle[2] + currnmle[2]) / (count + 1.0);
			dist = (count*dist + currdist) / (count + 1.0);
			count += 1.0;
		}

		float mag = sqrt(nmle[0]*nmle[0] + nmle[1]*nmle[1] + nmle[2]*nmle[2]);
		currGroup->_nmle[0] = nmle[0]/mag;
		currGroup->_nmle[1] = nmle[1]/mag;
		currGroup->_nmle[2] = nmle[2]/mag;
		currGroup->_dist = dist;
	}


	for (vector<Group *>::iterator it = _Groups.begin(); it != _Groups.end(); it++) {
		Group *currGroup = (*it);
		PlanarPatch *MergedPrim = new PlanarPatch(1,1);
		MergedPrim->setNmle(currGroup->_nmle);
		MergedPrim->setDist(currGroup->_dist);
		MergedPrim->setScale(_AllPrim[0]->getScale());
		
		float *nmle_prim = MergedPrim->getNmle();
		float *e1 = MergedPrim->getE1();
		float *e2 = MergedPrim->getE2();
		float *center_prim;
		int *size;

		bool first = true;

		for (vector<int>::iterator it2 = currGroup->_indices.begin(); it2 != currGroup->_indices.end(); it2++) {
			center_prim = MergedPrim->getShift();
			size = MergedPrim->getSize();

			PlanarPatch *prim = _AllPrim[(*it2)];
			if (prim == NULL)
				continue;
		
			if (prim->getBump() == NULL)
				continue;

			// Test if the projected bounding box is included
			float *corners = prim->getCorners();
			// Project each corners onto the plane
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
			
			float Shift[2];
			if (first) {
				Shift[0] = BBox[0];
				Shift[1] = BBox[2];
			} else {
				Shift[0] = min(center_prim [0], BBox[0]);
				Shift[1] = min(center_prim [1], BBox[2]);
			}
		
			int NewSize[2];
			if (first) {
				NewSize[0] = int((BBox[1]-BBox[0])/RES_PLANE);
				NewSize[1] = int((BBox[3]-BBox[2])/RES_PLANE);
			} else {
				NewSize[0] = max(size[0]+int((center_prim[0] - Shift[0])/RES_PLANE), int((BBox[1]-BBox[0])/RES_PLANE)+int((BBox[0] - Shift[0])/RES_PLANE));
				NewSize[1] = max(size[1]+int((center_prim[1] - Shift[1])/RES_PLANE), int((BBox[3]-BBox[2])/RES_PLANE)+int((BBox[2] - Shift[1])/RES_PLANE));	
			}

			//cout << "New size: " << NewSize[0] << " " << NewSize[1] << endl;

			delete corners;

			MergedPrim->Merge(prim, Shift, NewSize);
			first = false;
		}
		MergedPrim->LinearInterpolation();
		MergedPrim->SetView(true);
		MergedPrim->ComputeTriangulation();
		cout << "merge" << endl;
		_MergedPrim.push_back(MergedPrim);
		MergedPrim->_colorPrim[0] = currGroup->_colorGroup[0];
		MergedPrim->_colorPrim[1] = currGroup->_colorGroup[1];
		MergedPrim->_colorPrim[2] = currGroup->_colorGroup[2];
	}
	_Merged = true;
}