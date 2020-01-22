/* Author: Diego Thomas
 * Date Created: 14/03/2014
 * Last Modification: 14/03/2014
 * Main file for a real time 3D construction of local 3D models from RGB-D data
 */

#include "stdafx.h"
#include "RG.h"

GLuint window;

RG *RGProcess;
bool Running = false;

/*** Camera variables for OpenGL ***/
GLfloat intrinsics[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Calib[11] = {580.8857, 583.317, 319.5, 239.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 8000.0}; // Kinect data
float Znear = 0.5;
float Zfar = 10.0;
GLfloat light_pos[] = { 1.0, 1.0, 0.0, 0.0 };
GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };

bool first;
float pose[16];
float pose_inv[16];

float current_time;
float last_time;
float my_count;
float fps;

Matrix3f poseRinv;
Vector3f posetinv;
Matrix3f poseR;
Vector3f poset;

int width_r = 640;
int	height_r = 480;

int Init() {
	
	GLenum code;
			
	cudaError_t cudaStatus;

	int dev_idx = 0;//gpuGetMaxGflopsDeviceId();

	/* Set up the device for OpenGL to work with */
	cudaStatus = cudaGLSetGLDevice(dev_idx);
    if (cudaStatus != cudaSuccess) {
        cout << "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?" << endl;
        return 1;
    }

	/* initialisation de GLEW */
	code = glewInit();
	if(code != GLEW_OK)
	{
		cout << "impossible d'initialiser GLEW : " << glewGetErrorString(code) << endl;
	}
			
	Running = false;
						
	current_time = clock();
	last_time = clock();
	my_count = 0.0;
	first = true;

	/*** Initialize OpenGL ***/
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	// enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// set material properties which will be assigned by glColor
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	
	return 0;
}

void reshape(int width_in, int height_in)
{	
	glViewport(0, 0, width_in, height_in) ;
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	// Set up camera intrinsics
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
}

void DisplayPredictedFBO() {
	glBindFramebuffer(GL_FRAMEBUFFER, RGProcess->_frame_buf);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, RGProcess->_textureId, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, RGProcess->_DepthRendererId, 0);
		
	//// clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity (); 
	glViewport (0, 0, width_r, height_r);

	RGProcess->_predicted_frame->Draw(true, true);

	glBindFramebuffer(GL_FRAMEBUFFER, 0); // unbind
}

void DisplayFrame() {
	glViewport (0, 0, width_r, height_r);
	if (lighting) {
		glEnable ( GL_LIGHTING ) ;
		glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
		glEnable ( GL_LIGHT0 ) ;
	}

	RGProcess->_frame->Draw(false);

	glViewport (width, 0, width_r, height_r);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();  	
	RGProcess->_predicted_frame->Draw(false);

	if (lighting) {
		glDisable(GL_LIGHT0);
		glDisable ( GL_LIGHTING ) ;
	}
}

void display(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	// Set up camera intrinsics
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();

	if (!Running)
		goto ENDDISPLAY;
	
	/* Load Input RGB-D image */
	float time_A = clock();
	if (!RGProcess->LoadFrame()) {
		exit(1);
		goto ENDDISPLAY;
	}	
	float time_B = clock();
	if (TIMING)
		cout << "Load Timing: " << time_B - time_A << endl; 
	/* End Load Input RGB-D image */
		
	/* Init semi global model at the first frame */
	if (RGProcess->_count == 2) {
		RGProcess->_predicted_frame->Cpy(RGProcess->_frame);
		RGProcess->_predicted_frame->InitMask();
	}
	
	/* Display Input RGB-D image and predicted one onto the screen */
	time_A = clock();
	DisplayFrame();
	time_B = clock();
	if (TIMING)
		cout << "Display/READ Timing: " << time_B - time_A << endl; 
	/* End display Input RGB-D image and predicted one onto the screen */
			
	/* Add new planar patch in the graph if non assigned fragments still remain */	
	/*if (RGProcess->_Detect) {
		time_A = clock();
		RGProcess->IterDetectPrimitives();
		cout << "nb Prim: " << RGProcess->_graph->_nodes.size() << endl;
		time_B = clock();
		if (TIMING)
			cout << "IterDetectPrimitives Timing: " << time_B - time_A << endl; 
	}*/
	
	/* Align Current RGB-D image with the predicted one */	
	time_A = clock();
	if (!RGProcess->AlignFrameGaussNewton(RGProcess->_frame->_Resources_t[0], RGProcess->_predicted_frame->_Resources_t[0])) {
		RGProcess->_predicted_frame->Cpy(RGProcess->_frame);
		RGProcess->_predicted_frame->InitMask();
		RGProcess->_poseRPrev = RGProcess->_poseR;
		RGProcess->_posetPrev = RGProcess->_poset;
		cout << "align out" << endl;
		goto ENDDISPLAY;
	}
	time_B = clock();
	if (TIMING)
		cout << "AlignFrameGaussNewton Timing: " << time_B - time_A << endl; 
							
	poseRinv = RGProcess->_poseR.inverse();
	posetinv = -poseRinv*RGProcess->_poset;

	poseR = poseRinv * RGProcess->_poseRPrev;
	poset = poseRinv * RGProcess->_posetPrev + posetinv;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			pose_inv[j*4+i] = poseR(i,j);	
	for (int i = 0; i < 3; i++)
		pose_inv[3*4+i] = poset(i);
	pose_inv[3] = 0.0; pose_inv[7] = 0.0; pose_inv[11] = 0.0; pose_inv[15] = 1.0;

	/* Transform vertices values of the predicted image into current frame coordinates */	
	time_A = clock();
	RGProcess->_predicted_frame->Transform(pose_inv, true);
	time_B = clock();
	if (TIMING)
		cout << "Transform Timing: " << time_B - time_A << endl; 
	
	/* Compute new RGB-D and Mask image for the current camera image plane using FBO */
	time_A = clock();
	DisplayPredictedFBO();
	RGProcess->_predicted_frame->ReadFrame(RGProcess->_Resources_t);
		
	poseRinv = poseR.inverse();
	posetinv = -poseRinv*poset;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			pose_inv[j*4+i] = poseRinv(i,j);
			pose[j*4+i] = RGProcess->_poseR(i,j);
		}
	for (int i = 0; i < 3; i++) {
		pose_inv[3*4+i] = posetinv(i);
		pose[3*4+i] = RGProcess->_poset(i);
	}
	pose_inv[3] = 0.0; pose_inv[7] = 0.0; pose_inv[11] = 0.0; pose_inv[15] = 1.0;
	RGProcess->_predicted_frame->ProjectMask(pose_inv);
	RGProcess->_predicted_frame->ProjectLabel(pose_inv, pose, RGProcess->_Equations_dev, RGProcess->_nbPlan);
	
	time_B = clock();
	if (TIMING)
		cout << "ReadFrame ReadFrameMask Timing: " << time_B - time_A << endl;
		
	RGProcess->_poseRPrev = RGProcess->_poseR;
	RGProcess->_posetPrev = RGProcess->_poset;
		
	/* Merge Predicted and input images */
	time_A = clock();
	RGProcess->_predicted_frame->Merge(RGProcess->_frame);
	time_B = clock();
	if (TIMING)
		cout << "Merge Timing: " << time_B - time_A << endl; 
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			pose[j*4+i] = RGProcess->_poseR(i,j);	
	for (int i = 0; i < 3; i++)
		pose[3*4+i] = RGProcess->_poset(i);
	pose[3] = 0.0; pose[7] = 0.0; pose[11] = 0.0; pose[15] = 1.0;
	RGProcess->_graph->Update(RGProcess->_predicted_frame, pose);

	//
	///* Update attributes of the current planar patch */
	//for (int i = 0; i < 1/*RGProcess->_graph->_nodes.size()*/; i++) {
	//	//RGProcess->_graph->Update(RGProcess->_predicted_frame, RGProcess->_mask_frame, RGProcess->_frame, pose, pose_inv, RGProcess->_intr_dev);
	//	RGProcess->_Size = RGProcess->_graph->_nodes.size();
	//	if (RGProcess->_Size == 0 && !RGProcess->_Detect)
	//		cout << "ERROR NO  _Detect" << endl;
	//	RGProcess->_graph->_curr_ind = RGProcess->_graph->_curr_ind+1 < RGProcess->_Size ? RGProcess->_graph->_curr_ind+1 : 0;
	//}
	
	/* Prepare for next input frame */
	time_A = clock();
	RGProcess->Finish();
	time_B = clock();
	if (TIMING)
		cout << "Finish Timing: " << time_B - time_A << endl; 
		
ENDDISPLAY:

	my_count++;
	current_time = clock();
	if ((current_time-last_time)/CLOCKS_PER_SEC > 1.0) {
		fps = my_count / ((current_time - last_time)/CLOCKS_PER_SEC);
		last_time = current_time;
		my_count = 0.0;
		cout << "fps: " << fps << endl;
		/*size_t available, total;
		cudaMemGetInfo(&available, &total);
		cout << "begin detect == 0: Available memory: " << available << "Total Memory: " << total << endl;*/
	}

	glutSwapBuffers() ;
	glutPostRedisplay() ;

}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 's':
		for (int i = 0; i < NB_THREADS; i++)
			WaitForMultipleObjects(1, &RGProcess->hthread[i], TRUE, INFINITE);

		//Copy unsigned char mask to the unsigned short mask
		for (vector<Primitive *>::iterator it = RGProcess->_graph->_nodes.begin(); it != RGProcess->_graph->_nodes.end(); it++) {
			Primitive *Prim = (*it); 
			Prim->CpyToShortMask();
		}
		//RGProcess->_graph->Clean(RGProcess->_graph_global->_pose.data(), true);
		RGProcess->_graph->save("Graph", 0);
		//RGProcess->_graph->save2Mesh("GraphGlob");
		break;
	case 27 /* Esc */:
		for (int i = 0; i < NB_THREADS; i++)
			WaitForMultipleObjects(1, &RGProcess->hthread[i], TRUE, INFINITE);
		delete RGProcess;
		exit(1) ;
	}
}

/***** Function to handle right click of Mouse for subwindow 1*****/
void Right_menu(int val)
{
}

void KinectLive(int value)
{
	Running = true;
	/*** Live Kinect data ***/
	intrinsics[0] = 2.0*580.8857/640.0; intrinsics[5] = 2.0*583.317/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0;
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);
	cout << "Running on live kinect." << endl;
	RGProcess = new RG(height, width, "C:\\Diego\\Data\\Tmp", Calib, 3, 0.1, sin (40.0 * PI / 180.f), 0, true, true, false); 
}

void KinectOffLine(int value)
{
	Running = true;
	/*** Offline Kinect data ***/
	intrinsics[0] = 2.0*580.8857/640.0; intrinsics[5] = 2.0*583.317/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0;
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);
	switch (value)
	{
		case 0: 
			cout << "Running on DataScan2." << endl;
			RGProcess = new RG(height, width, "C:\\Diego\\Data\\DataScan2", Calib, 3, 0.1, sin (40.0 * PI / 180.f), 8500, true, false, false);
			break;
		case 1: 
			break;
		case 2: 
			break;
		case 3: 
			break;
		default:
			break;
	}
}

void KinectV2OffLine(int value)
{
	Running = true;
	/*** Offline Kinect data ***/
	intrinsics[0] = 2.0;//*315.4/512.0; 
	intrinsics[5] = 2.0;//*316.3/424.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0;
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);
	Calib[0] = 512.0; Calib[1] = 424.0; Calib[2] = 255.5; Calib[3] = 211.5; Calib[4] = 0.0; Calib[5] = 0.0; Calib[6] = 0.0; Calib[7] = 0.0; Calib[8] = 0.0; Calib[9] = 4.5; Calib[10] = 255.0*255.0;
	width_r = 512;
	height_r = 424;
	switch (value)
	{
		case 0: 
			cout << "Running on Set1." << endl; 
			RGProcess = new RG(424, 512, "C:\\Diego\\Data\\KinectV2\\Set1", Calib, 3, 0.1, sin (40.0 * PI / 180.f), 1750, true, false, false);
			break;
		case 1: 
			break;
		case 2: 
			break;
		case 3: 
			break;
		default:
			break;
	}
}

void RGBDRep(int value)
{
	Running = true;
	/*** RGBD repository ***/
	intrinsics[0] = 2.0*525.0/640.0; intrinsics[5] = 2.0*525.0/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0; 
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);
	Calib[0] = 525.0; Calib[1] = 525.0; Calib[2] = 319.5; Calib[3] = 239.5; Calib[4] = 0.2624; Calib[5] = -0.9531; Calib[6] = -0.0054; Calib[7] = 0.0026; Calib[8] = 1.1633; Calib[9] = 1.035; Calib[10] = 5000.0;
	switch (value)
	{
		case 0: 
			cout << "Running on rgbd_dataset_freiburg1_xyz." << endl;
			RGProcess = new RG(height, width, "C:\\Diego\\Data\\rgbd_dataset_freiburg1_xyz", Calib, 3, 0.05, sin (20.0 * PI / 180.f), 0);
			break;
		case 1: 
			break;
		case 2: 
			break;
		case 3: 
			break;
		default:
			break;
	}
}

void Zhou(int value)
{
	Running = true;
	/*** Zhou data ***/
	intrinsics[0] = 2.0*525.0/640.0; intrinsics[5] = 2.0*525.0/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0;
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);
	Calib[0] = 525.0; Calib[1] = 525.0; Calib[2] = 319.5; Calib[3] = 239.5; Calib[4] = 0.2624; Calib[5] = -0.9531; Calib[6] = -0.0054; Calib[7] = 0.0026; Calib[8] = 1.1633; Calib[9] = 1.0; Calib[10] = 1000.0;
	switch (value)
	{
		case 0: 
			cout << "Running on data lounge." << endl;
			RGProcess = new RG(height, width, "C:\\Diego\\Data\\lounge", Calib, 3, 0.1, sin (40.0 * PI / 180.f), 2999, false, false, false); 
			RGProcess->_Size_data = 2999;
			break;
		case 1: 
			cout << "Running on data copyroom." << endl;
			RGProcess = new RG(height, width, "C:\\Diego\\Data\\copyroom", Calib, 3, 0.1, sin (40.0 * PI / 180.f), 5489, false, false, false); 
			RGProcess->_Size_data = 5489;
			break;
		case 2: 
			break;
		case 3: 
			break;
		default:
			break;
	}
}

int _tmain(int argc, char* argv[])
{		
	glutInit(&argc, argv) ;
	
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	glutInitWindowSize(2*width, height);

	window = glutCreateWindow("GP") ;
	
	if (Init() != 0)
		return 1;

	int Kinect_Live = glutCreateMenu (KinectLive);
		glutAddMenuEntry ( "Run", 0 );

	int Kinect_OffLine = glutCreateMenu (KinectOffLine);
		glutAddMenuEntry ( "Run Data2", 0 );
		
	int KinectV2_OffLine = glutCreateMenu (KinectV2OffLine);
		glutAddMenuEntry ( "Run Set1", 0 );
		
	int RGBD_Rep = glutCreateMenu (RGBDRep);
		glutAddMenuEntry ( "Run office_long_household", 0 );
		
	int Zhou_etal = glutCreateMenu (Zhou);
		glutAddMenuEntry ( "Run lounge", 0 );
		glutAddMenuEntry ( "Run copyroom", 1 );

	int menu_general = glutCreateMenu (Right_menu);
		glutAddSubMenu("Kinect live",Kinect_Live);
		glutAddSubMenu("Kinect off-line",Kinect_OffLine);
		glutAddSubMenu("Kinect V2 off-line",KinectV2_OffLine);
		glutAddSubMenu("RGB-D repository",RGBD_Rep);
		glutAddSubMenu("Zhou et.al.",Zhou_etal);

	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);

	glutKeyboardFunc(keyboard) ;

	glutMainLoop() ;

	return 0 ;
}

