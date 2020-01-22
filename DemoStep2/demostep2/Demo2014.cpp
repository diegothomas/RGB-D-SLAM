// Vizu.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Model3D.h"

GLuint window;

float current_time;
float last_time;
float my_count;
float fps;

Model3D *TheMesh;
GLfloat intrinsics[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Znear = 0.01;
float Zfar = 30.0;

// angle of rotation for the camera direction
float anglex = 0.0f;
float angley = 0.0f;
 
// actual vector representing the camera's direction
float lx=0.0f, ly=0.0f, lz=-1.0f;
float lxStrap=-1.0f,lyStrap=0.0f,lzStrap=0.0f;
 
// XZ position of the camera
float x=0.0f, y=1.0f, z=5.0f;
 
// the key states. These variables will be zero
//when no key is being presses
float deltaAnglex = 0.0f;
float deltaAngley = 0.0f;
float deltaMove = 0;
float deltaStrap = 0;
int xOrigin = -1;
int yOrigin = -1;
bool color = true;
bool light = false;
bool wire = false;

bool SELECT_MODE = false;
bool EDIT_MODE = false;

bool ready_to_select = false;
bool has_selected = false;
bool ready_to_rubb = false;
bool ready_to_draw = false;
bool ready_to_extrude = false;
bool ready_to_dig = false;
bool is_rubbing = false;
bool is_drawing = false;
bool is_extruding = false;
bool is_digging = false;
bool keyf = true;

int curr_indx = 0;
int pix_i, pix_j;
int size_brush = 2;

GLfloat light_pos[] = { 1.0, 1.0, 0.0, 0.0 };
GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };

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
			
						
	current_time = clock();
	last_time = clock();
	my_count = 0.0;

	/*** Initialize OpenGL ***/
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	// enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// set material properties which will be assigned by glColor
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable( GL_TEXTURE_2D );
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glAlphaFunc(GL_GREATER,0.95);
	glEnable(GL_ALPHA_TEST);
	
	TheMesh = new Model3D("C:\\Diego\\Data\\KinectV1\\Tmp");
	//TheMesh->LoadModel3D("F:\\Projects\\data\\Tmp_2_04");
	//TheMesh->UpdateWorking(x, y, z, lx, ly, lz, "F:\\Projects\\data\\Tmp_2_04");
	
	intrinsics[0] = 2.0*580.8857/640.0; intrinsics[5] = 2.0*583.317/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0; 
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);

	return 0;
}

void reshape(int width_in, int height_in)
{	
	glViewport(0, 0, 2*width_in, 2*height_in) ;
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	// Set up camera intrinsics
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	
}

void computePos(float deltaMove, float deltaStrap) {
 
	x += deltaMove * lx * 0.1f + deltaStrap * lxStrap * 0.1f ;
	y += deltaMove * ly * 0.1f + deltaStrap * lyStrap * 0.1f ;
	z += deltaMove * lz * 0.1f + deltaStrap * lzStrap * 0.1f ;

}

void GetIdx()
{
	float *image = new float[3*width*height];
	glPixelStorei(GL_PACK_ALIGNMENT,1);
	glReadBuffer(GL_FRONT);
	glReadPixels(width, height, width, height, GL_RGB, GL_FLOAT, image);
	curr_indx = int(image[3*(width*(height-pix_j-1)+pix_i)]*255.0);
	delete [] image;
	image = 0;
}

void display(void) {

	if (deltaMove || deltaStrap)
		computePos(deltaMove, deltaStrap);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);				// Black Background
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	// Set up camera intrinsics
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	// Set the camera
	gluLookAt(	x, y, z,
			x+lx, y+ly,  z+lz,
			0.0f, 1.0f,  0.0f);

	// Get inverse Modelview matrix
	GLfloat ModelMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, ModelMatrix);
	Eigen::Matrix4f ModelEigen;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ModelEigen(i,j) = ModelMatrix[j*4+i];
		}
	}
	Eigen::Matrix4f InverseModelMatrix = ModelEigen.inverse();
	//Get the object space light vector
	VECTOR3D objectLightPosition= VECTOR3D(0.0,0.0,0.0);
	objectLightPosition.x = InverseModelMatrix(0,0)*light_pos[0] + InverseModelMatrix(0,1)*light_pos[1] + InverseModelMatrix(0,2)*light_pos[2] + InverseModelMatrix(0,3)*light_pos[3];
	objectLightPosition.y = InverseModelMatrix(1,0)*light_pos[0] + InverseModelMatrix(1,1)*light_pos[1] + InverseModelMatrix(1,2)*light_pos[2] + InverseModelMatrix(1,3)*light_pos[3];
	objectLightPosition.z = InverseModelMatrix(2,0)*light_pos[0] + InverseModelMatrix(2,1)*light_pos[1] + InverseModelMatrix(2,2)*light_pos[2] + InverseModelMatrix(2,3)*light_pos[3];

	TheMesh->Update(); //Visible(x, y, z, lx, ly, lz);
	/*if (light) {
		glEnable ( GL_LIGHTING ) ;
		glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
		glEnable ( GL_LIGHT0 ) ;
	}*/

	if (wire) {
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	} else {
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	}

	glViewport(0, height, width, height) ;
	glColor4f(1.0, 1.0, 1.0, 1.0);
	TheMesh->Draw(objectLightPosition, light, color);
	
	glViewport(0, 0, width, height);
	TheMesh->DrawGraph(keyf);

	/*if (light)  {
		glDisable(GL_LIGHT0);
		glDisable ( GL_LIGHTING ) ;
	}*/

	if (SELECT_MODE) {
		glViewport(width, height, width, height) ;
		glColor4f(1.0, 1.0, 1.0, 1.0);
		TheMesh->DrawIdx();

		if (has_selected) {
			has_selected = false;
			int prev_indx = curr_indx;
			GetIdx();
			cout << "curr_indx: " << curr_indx << endl;
			if (curr_indx > TheMesh->Size()-1)
				curr_indx = prev_indx;
		}

		glViewport(0, 0, width, height) ;
		//TheMesh->DrawPrim(objectLightPosition, light, color, curr_indx);
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();	
		glOrtho(0.0,640,480,1.0,-1.0,1.0);
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		glViewport(width, 0, width, height) ;
		TheMesh->DrawPrimFlat(curr_indx);
	}

	if (EDIT_MODE) {
		if (is_rubbing) {
			TheMesh->RubbPrim(curr_indx, pix_i, pix_j, size_brush);
		}
		if (is_drawing) {
			TheMesh->DrawPrim(curr_indx, pix_i, pix_j, size_brush);
		}
		if (is_extruding) {
			TheMesh->ExtrudePrim(curr_indx, pix_i, pix_j, size_brush);
		}
		if (is_digging) {
			TheMesh->DigPrim(curr_indx, pix_i, pix_j, size_brush);
		}
	}

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
	case 'c':
		color = !color;
		break;
	case 'l':
		light = !light;
		break;
	case 'w':
		wire = !wire;
		break;
	case 'k':
		keyf = !keyf;
		break;
	case 'i':
		TheMesh->ReInit();
		break;
	case 'm':
		TheMesh->Merge();
		break;
	case 'u':
		TheMesh->ComputeTriangulation(curr_indx);
		break;
	case 's':
		if (SELECT_MODE) {
			ready_to_select = true;
		}
		break;
	case 'd':
		if (EDIT_MODE) {
			TheMesh->setInvisible(curr_indx);
		}
		break;		
	case 'r':
		if (EDIT_MODE) {
			ready_to_rubb = !ready_to_rubb;
		}
		break;
	case 'p':
		if (EDIT_MODE) {
			ready_to_draw = !ready_to_draw;
		}
		break;		
	case 'e':
		if (EDIT_MODE) {
			ready_to_extrude = !ready_to_extrude;
		}
		break;		
	case 'h':
		if (EDIT_MODE) {
			ready_to_dig = !ready_to_dig;
		}
		break;
	case '+':
		size_brush++;
		break;
	case '-':
		size_brush = max(0,size_brush-1);
		break;
	case 27 /* Esc */:
		delete TheMesh;
		exit(1) ;
	}
}

void keyUp(unsigned char key, int x, int y) {
	switch (key) {
	case 's':
		if (SELECT_MODE) {
			ready_to_select = false;
		}
		break;
	}
}
 
void pressKey(int key, int xx, int yy) {
 
       switch (key) {
             case GLUT_KEY_UP : deltaMove = 0.5f; break;
             case GLUT_KEY_DOWN : deltaMove = -0.5f; break;
             case GLUT_KEY_LEFT : deltaStrap = 0.5f; break;
             case GLUT_KEY_RIGHT : deltaStrap = -0.5f; break;
       }
} 
 
void releaseKey(int key, int x, int y) { 	
 
        switch (key) {
             case GLUT_KEY_LEFT :
             case GLUT_KEY_RIGHT :
             case GLUT_KEY_UP :
             case GLUT_KEY_DOWN : deltaMove = 0; deltaStrap = 0; break;
        }
} 

void mouseMove(int x, int y) { 	

	if (is_rubbing || is_drawing || is_extruding || is_digging) {
		if (x > width-1 && y > height-1) {
			pix_i = (x - width);
			pix_j = (y - height);
		}
		return;
	}

	if (ready_to_rubb || ready_to_draw || ready_to_extrude || ready_to_dig)
		return;
 
    // this will only be true when the left button is down
    if (xOrigin >= 0 || yOrigin >= 0) {
		 
		// update deltaAngle
		deltaAnglex = (x - xOrigin) * 0.001f;
		deltaAngley = (y - yOrigin) * 0.001f;
 
		// update camera's direction
		//lx = sin(anglex + deltaAnglex);
		//lz = -cos(anglex + deltaAnglex);
		lx = sin(anglex + deltaAnglex);
		ly = cos(anglex + deltaAnglex) * sin(-(angley + deltaAngley));
		lz = -cos(anglex + deltaAnglex) * cos(-(angley + deltaAngley));

		// update camera's direction
		//lxStrap = cos(anglex + deltaAnglex);
		//lzStrap = sin(anglex + deltaAnglex);
		lxStrap = -cos(anglex + deltaAnglex);
		lyStrap = sin(anglex + deltaAnglex) * sin(-(angley + deltaAngley));
		lzStrap = -sin(anglex + deltaAnglex) * cos(-(angley + deltaAngley));		
	}
}

void mouseButton(int button, int state, int x, int y) {
 
	// only start motion if the left button is pressed
	if (button == GLUT_LEFT_BUTTON) {

		if (ready_to_rubb || ready_to_draw || ready_to_extrude || ready_to_dig) {
			if (state != GLUT_UP) {
				pix_i = x;
				pix_j = y;
				if (pix_i > width-1 && pix_j > height-1) {
					pix_i = pix_i-width;
					pix_j = pix_j-height;
					//cout << "current pixel: " << pix_i << " " << pix_j << endl;
					is_rubbing = ready_to_rubb;
					is_drawing = ready_to_draw;
					is_extruding = ready_to_extrude;
					is_digging = ready_to_dig;
					return;
				}
			} else {
				is_rubbing = false;
				is_drawing = false;
				is_extruding = false;
				is_digging = false;
				TheMesh->UpdateTextures(curr_indx);
				return;
			}
		} else {
			if (ready_to_select) {
				if (state != GLUT_UP) {
					pix_i = x;
					pix_j = y;
					if (pix_i < width && pix_j < height) {
						pix_i = pix_i-width;
						//cout << "current pixel: " << pix_i << " " << pix_j << endl;
						has_selected = true;
					}
				}
			}
		}
 
		// when the button is released
		if (state == GLUT_UP) {
			anglex += deltaAnglex;
			angley += deltaAngley;
			xOrigin = -1;
			yOrigin = -1;
		}
		else  {// state = GLUT_DOWN
			xOrigin = x;
			yOrigin = y;
		}
		
	}
}

/***** Function to handle right click of Mouse for subwindow 1*****/
void Select_mode(int val)
{
	switch(val) {
	case 0:
		SELECT_MODE = true;
		break;
	case 1:
		EDIT_MODE = true;
		cout << "Edit Mode ON: 'd': delete object, 'r': rubber, 'p': pencil" << endl;
		break;
	default:
		break;
	}
}

int _tmain(int argc, char* argv[])
{
	glutInit(&argc, argv) ;
	
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	glutInitWindowSize(2*width, 2*height);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("Vizu") ;
	
	if (Init() != 0)
		return 1;

	int menu_general = glutCreateMenu (Select_mode);
		glutAddMenuEntry("Selection",0);
		glutAddMenuEntry("Edition",1);

	glutAttachMenu(GLUT_RIGHT_BUTTON);

	//glutReshapeFunc(reshape);
	glutDisplayFunc(display);

	glutKeyboardFunc(keyboard) ;
	glutKeyboardUpFunc(keyUp);

	glutSpecialFunc(pressKey);
	glutSpecialUpFunc(releaseKey);
 
	// here are the two new functions
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);

	glutMainLoop() ;

	return 0 ;
}

