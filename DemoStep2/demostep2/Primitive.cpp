#include "stdafx.h"
#include "Primitive.h"


int round (float a) {
	if ((a - static_cast<int>(a)) > 0.5)
		return static_cast<int>(a)+1;
	return static_cast<int>(a);
}

template <typename T>
void DrawTriangle(Delaunay::Triangle triangle, T **data, T color) {
	VECTOR2D v0 = VECTOR2D(triangle.vertex(0).x(), triangle.vertex(0).y());
	VECTOR2D v1 = VECTOR2D(triangle.vertex(1).x(), triangle.vertex(1).y());
	VECTOR2D v2 = VECTOR2D(triangle.vertex(2).x(), triangle.vertex(2).y());

	int count = 0;

	if (v1.y < v0.y && v1.y <= v2.y) {
		VECTOR2D tmp = v0;
		v0 = v1;
		v1 = tmp;
	} else if (v2.y < v0.y && v2.y <= v1.y) {
		VECTOR2D tmp = v0;
		v0 = v2;
		v2 = tmp;
	}

	if (v2.y > v1.y) {
		VECTOR2D tmp = v1;
		v1 = v2;
		v2 = tmp;
	}

	if (v2.y == v0.y == v1.y) {
		cout << "Error flat triangle" << endl;
		return;
	}
	
	VECTOR2D AB = v1-v0;
	VECTOR2D AC = v2-v0;
	
	VECTOR3D AB3D = VECTOR3D(AB.x, AB.y, 1.0);
	VECTOR3D AC3D = VECTOR3D(AC.x, AC.y, 1.0);
	VECTOR3D nmle = AB3D.CrossProduct(AC3D);

	if (v2.y == v0.y) {
		goto DOWN;
	}

	if (nmle.z > 0.0) {
		VECTOR2D tmp = AB;
		AB = AC;
		AC = tmp;
	}

	for (int j = v0.y; j < v2.y+1; j++) {
		// left summit
		float lambda = (j-v0.y)/AB.y;
		VECTOR2D left = v0 + lambda*AB;
		// right summit
		lambda = (j-v0.y)/AC.y;
		VECTOR2D right = v0 + lambda*AC;
		int lb = round(left.x);
		int rb = round(right.x);
		for (int i = lb; i < rb+1; i++) {
			data[i][j] = color;
			count++;
		}
	}

	if (v2.y == v1.y) {
		if (count == 0)
			cout << "No coloring 1" << endl;
		return;
	}

	if (nmle.z > 0.0) {
		AB = v1-v2;
		for (int j = v2.y+1; j < v1.y+1; j++) {
			// left summit
			float lambda = (j-v2.y)/AB.y;
			VECTOR2D left = v2 + lambda*AB;
			// right summit
			lambda = (j-v0.y)/AC.y;
			VECTOR2D right = v0 + lambda*AC;
			int lb = round(left.x);
			int rb = round(right.x);
			for (int i = lb; i < rb+1; i++) {
				data[i][j] = color;
				count++;
			}
		}
	} else {
		AC = v1-v2;
		for (int j = v2.y+1; j < v1.y+1; j++) {
			// left summit
			float lambda = (j-v0.y)/AB.y;
			VECTOR2D left = v0 + lambda*AB;
			// right summit
			lambda = (j-v2.y)/AC.y;
			VECTOR2D right = v2 + lambda*AC;
			int lb = round(left.x);
			int rb = round(right.x);
			for (int i = lb; i < rb+1; i++) {
				data[i][j] = color;
				count++;
			}
		}
	}
	if (count == 0)
		cout << "No coloring 2" << endl;

	return;

DOWN:
	AB = v0-v1;
	AC = v2-v1;

	AB3D = VECTOR3D(AB.x, AB.y, 1.0);
	AC3D = VECTOR3D(AC.x, AC.y, 1.0);
	nmle = AB3D.CrossProduct(AC3D);

	if (nmle.z < 0.0) {
		VECTOR2D tmp = AB;
		AB = AC;
		AC = tmp;
	}

	for (int j = v1.y; j > v0.y-1; j--) {
		// left summit
		float lambda = (j-v1.y)/AB.y;
		VECTOR2D left = v1 + lambda*AB;
		// right summit
		lambda = (j-v1.y)/AC.y;
		VECTOR2D right = v1 + lambda*AC;
		int lb = round(left.x);
		int rb = round(right.x);
		for (int i = lb; i < rb+1; i++) {
			data[i][j] = color;
			count++;
		}
	}
	
	if (count == 0)
		cout << "No coloring 3" << endl;

	return;
}


VECTOR3D ComputeTangentS(VECTOR3D normal, VECTOR3D position, int i, int j, float **VMap, int n, int m) {
	if (i > n-1 || j > m-1)
		return VECTOR3D(0.0, 0.0, 0.0);

	VECTOR3D neigh = VECTOR3D(VMap[i][3*j], VMap[i][3*j+1], VMap[i][3*j+2]);
	VECTOR3D diff = neigh - position;
	float prodscal = diff.DotProduct(normal);
	VECTOR3D res = diff - prodscal*normal;
	res.Normalize();
	return res;
}

VECTOR3D ComputeTangentT(VECTOR3D normal, VECTOR3D position, VECTOR3D sTangent, int i, int j, float **VMap, int n, int m) {
	if (i > n-1 || j > m-1)
		return VECTOR3D(0.0, 0.0, 0.0);

	VECTOR3D neigh = VECTOR3D(VMap[i][3*j], VMap[i][3*j+1], VMap[i][3*j+2]);
	VECTOR3D diff = neigh - position;
	float prodscal = diff.DotProduct(normal);
	VECTOR3D res = diff - prodscal*normal;
	res.Normalize();
	prodscal = res.DotProduct(sTangent);
	res = res - prodscal*sTangent;
	res.Normalize();
	return res;
}

bool is_in_triangle(Delaunay::Triangle triangle, int k, int l) {
	VECTOR3D v0 = VECTOR3D(triangle.vertex(0).x(), triangle.vertex(0).y(), 1.0);
	VECTOR3D v1 = VECTOR3D(triangle.vertex(1).x(), triangle.vertex(1).y(), 1.0);
	VECTOR3D v2 = VECTOR3D(triangle.vertex(2).x(), triangle.vertex(2).y(), 1.0);
	VECTOR3D pos = VECTOR3D(float(k), float(l), 1.0);

	VECTOR3D deltav01 = v1-v0;
	VECTOR3D deltav02 = v2-v0;
	VECTOR3D delta = pos-v0;
	
	VECTOR3D crossref = deltav01.CrossProduct(deltav02);
	VECTOR3D cross = deltav01.CrossProduct(delta);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	cross = delta.CrossProduct(deltav02);
	if (cross.DotProduct(crossref) < 0.0)
		return false;

	deltav01 = v2-v1;
	deltav02 = v0-v1;
	delta = pos-v1;
	crossref = deltav01.CrossProduct(deltav02);
	cross = deltav01.CrossProduct(delta);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	cross = delta.CrossProduct(deltav02);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	
	return true;
}

bool PlanarPatch::is_in_triangle_b(int i, int k, int l) {
	VECTOR3D v0 = VECTOR3D(_vertices[i].s*float(_n), _vertices[i].t*float(_m), 1.0);
	VECTOR3D v1 = VECTOR3D(_vertices[i+1].s*float(_n), _vertices[i+1].t*float(_m), 1.0);
	VECTOR3D v2 = VECTOR3D(_vertices[i+2].s*float(_n), _vertices[i+2].t*float(_m), 1.0);
	VECTOR3D pos = VECTOR3D(float(k), float(l), 1.0);

	VECTOR3D deltav01 = v1-v0;
	VECTOR3D deltav02 = v2-v0;
	VECTOR3D delta = pos-v0;
	
	VECTOR3D crossref = deltav01.CrossProduct(deltav02);
	VECTOR3D cross = deltav01.CrossProduct(delta);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	cross = delta.CrossProduct(deltav02);
	if (cross.DotProduct(crossref) < 0.0)
		return false;

	deltav01 = v2-v1;
	deltav02 = v0-v1;
	delta = pos-v1;
	crossref = deltav01.CrossProduct(deltav02);
	cross = deltav01.CrossProduct(delta);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	cross = delta.CrossProduct(deltav02);
	if (cross.DotProduct(crossref) < 0.0)
		return false;
	
	return true;
}

void PlanarPatch::Transform(Eigen::MatrixXf ThePose) {
	// Transform primitive equation
	float tmp[3];
	tmp[0] = ThePose(0,0)*_e1[0] + ThePose(0,1)*_e1[1] + ThePose(0,2)*_e1[2];
	tmp[1] = ThePose(1,0)*_e1[0] + ThePose(1,1)*_e1[1] + ThePose(1,2)*_e1[2];
	tmp[2] = ThePose(2,0)*_e1[0] + ThePose(2,1)*_e1[1] + ThePose(2,2)*_e1[2];
	_e1[0] = tmp[0]; _e1[1] = tmp[1]; _e1[2] = tmp[2];
		
	tmp[0] = ThePose(0,0)*_e2[0] + ThePose(0,1)*_e2[1] + ThePose(0,2)*_e2[2];
	tmp[1] = ThePose(1,0)*_e2[0] + ThePose(1,1)*_e2[1] + ThePose(1,2)*_e2[2];
	tmp[2] = ThePose(2,0)*_e2[0] + ThePose(2,1)*_e2[1] + ThePose(2,2)*_e2[2];
	_e2[0] = tmp[0]; _e2[1] = tmp[1]; _e2[2] = tmp[2];
		
	tmp[0] = ThePose(0,0)*_nmle[0] + ThePose(0,1)*_nmle[1] + ThePose(0,2)*_nmle[2];
	tmp[1] = ThePose(1,0)*_nmle[0] + ThePose(1,1)*_nmle[1] + ThePose(1,2)*_nmle[2];
	tmp[2] = ThePose(2,0)*_nmle[0] + ThePose(2,1)*_nmle[1] + ThePose(2,2)*_nmle[2];
	_nmle[0] = tmp[0]; _nmle[1] = tmp[1]; _nmle[2] = tmp[2];

	tmp[0] = ThePose(0,3)*_e1[0] + ThePose(1,3)*_e1[1] + ThePose(2,3)*_e1[2];
	tmp[1] = ThePose(0,3)*_e2[0] + ThePose(1,3)*_e2[1] + ThePose(2,3)*_e2[2];
	tmp[2] = ThePose(0,3)*_nmle[0] + ThePose(1,3)*_nmle[1] + ThePose(2,3)*_nmle[2];

	_Shift[0] = _Shift[0] + tmp[0];
	_Shift[1] = _Shift[1] + tmp[1];
	_dist = _dist + tmp[2];
	
	tmp[0] = ThePose(0,0)*_anchor[0] + ThePose(0,1)*_anchor[1] + ThePose(0,2)*_anchor[2];
	tmp[1] = ThePose(1,0)*_anchor[0] + ThePose(1,1)*_anchor[1] + ThePose(1,2)*_anchor[2];
	tmp[2] = ThePose(2,0)*_anchor[0] + ThePose(2,1)*_anchor[1] + ThePose(2,2)*_anchor[2];
	_anchor[0] = tmp[0]; _anchor[1] = tmp[1]; _anchor[2] = tmp[2];
}

/***************************************************************************************/
/*************************** Methods for the class Primitive ***************************/
/***************************************************************************************/


void Primitive::Draw(bool color) {
	//int fact = int(pow(2.0,_lvl));

	//int frag_n = _Size[0]/fact;
	//int frag_m = _Size[1]/fact;

	///* on passe en mode VBO */
 //   glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);

	//glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
	//glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(frag_n*frag_m*3*sizeof(float)));
	//if (color)
	//	glColorPointer(4, GL_FLOAT , 0, BUFFER_OFFSET(frag_n*frag_m*3*sizeof(float)+frag_n*frag_m*3*sizeof(float)));

	///* activation des tableaux de donnees */
 //   glEnableClientState(GL_VERTEX_ARRAY);
	//glEnableClientState(GL_NORMAL_ARRAY);
	//if (color)
	//	glEnableClientState(GL_COLOR_ARRAY);

	////glDrawArrays(GL_POINTS, 0, frag_n*frag_m );
	///* rendu indices */
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _index_buf);
	//glDrawElements(GL_QUADS, 4*(frag_n-1)*(frag_m-1), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	//if (color)
	//	glDisableClientState(GL_COLOR_ARRAY);
	//glDisableClientState(GL_NORMAL_ARRAY);
	//glDisableClientState(GL_VERTEX_ARRAY);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Primitive::DrawMesh(bool color) {
}

void Primitive::Load(char *filename){
	return;
}

/***************************************************************************************/
/************************* Methods for the class PlanarPatch ***************************/
/***************************************************************************************/

double *PlanarPatch::getQuaternion() {
	double *v = new double [7];

	float *t = getCenter(0.5, 0.5);
	v[0] = double(t[0]);
	v[1] = double(t[1]);
	v[2] = double(t[2]);

	/*v[0] = double(_anchor[0]);
	v[1] = double(_anchor[1]);
	v[2] = double(_anchor[2]);*/

	Eigen::Matrix3d Rot;
	Rot(0,0) = double(_e1[0]); Rot(0,1) = double(_e2[0]); Rot(0,2) = double(_nmle[0]);
	Rot(1,0) = double(_e1[1]); Rot(1,1) = double(_e2[1]); Rot(1,2) = double(_nmle[1]); 
	Rot(2,0) = double(_e1[2]); Rot(2,1) = double(_e2[2]); Rot(2,2) = double(_nmle[2]);

	float *r = new float [4];
	//matrix2quaternion(Rot, r);
	Eigen::Quaterniond q((Eigen::Matrix3d) Rot);
	Eigen::Quaterniond normalize(q);
	v[3] = q.x(); v[4] = q.y(); v[5] = q.z(); v[6] = q.w();

	// normalize quaternion
	/*float norm = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);

	v[3] = double(r[1]/norm);
	v[4] = double(r[2]/norm);
	v[5] = double(r[3]/norm);
	v[6] = double(r[0]/norm);*/

	delete r;

	return v;
}

void PlanarPatch::SetFromQuaternion(double *v) {
	//double Mat [3][3];
	double q[4];
	q[0] = v[6]; //qw
	q[1] = v[3]; //qx
	q[2] = v[4]; //qy
	q[3] = v[5]; //qz

	Eigen::Matrix3d Mat = Quaterniond(v[6], v[3], v[4], v[5]).toRotationMatrix();
	//quaternion2matrix(q, Mat);
	_nmle[0] = float (Mat(0,2));
	_nmle[1] = float (Mat(1,2));
	_nmle[2] = float (Mat(2,2));

	_e1[0] = float (Mat(0,0));
	_e1[1] = float (Mat(1,0));
	_e1[2] = float (Mat(2,0));
	
	_e2[0] = float (Mat(0,1));
	_e2[1] = float (Mat(1,1));
	_e2[2] = float (Mat(2,1));

	if (!CheckOrthoBasis(_e1,_e2,_nmle)) {
		cout << "ERROR: Non orthonormal basis" << endl;
		return;
	}

	float center [3];
	center [0] = float (v[0]);
	center [1] = float (v[1]);
	center [2] = float (v[2]);

	/*float llcorner [3];
	llcorner [0] = center [0] - _e2[0]*_scale[0]*float(_Size[0])/2.0 - _e1[0]*_scale[1]*float(_Size[1])/2.0;
	llcorner [1] = float (v[1]);
	llcorner [2] = float (v[2]);*/
	
	_Shift[1] = center [0]*_e2[0] +  center [1]*_e2[1] + center [2]*_e2[2] - float(_Size[1])/_scale[1];
	_Shift[0] = center [0]*_e1[0] +  center [1]*_e1[1] + center [2]*_e1[2] - float(_Size[0])/_scale[0];
	_dist = center [0]*_nmle[0] +  center [1]*_nmle[1] + center [2]*_nmle[2];

	return;
}

void PlanarPatch::DrawMesh(bool color) {
}

void PlanarPatch::DrawMeshBump(VECTOR3D objectLightPosition, bool light, bool color, bool drawBumps) {
	if (!_view)
		return;

	//Loop through vertices
	for(int i=0; i<_numVertices; ++i)
	{
		VECTOR3D lightVector=objectLightPosition-_vertices[i].position;
		
		//Calculate tangent space light vector
		_vertices[i].tangentSpaceLight.x=
			_vertices[i].sTangent.DotProduct(lightVector);
		_vertices[i].tangentSpaceLight.y=
			_vertices[i].tTangent.DotProduct(lightVector);
		_vertices[i].tangentSpaceLight.z=
			_vertices[i].normal.DotProduct(lightVector);
	}

	//Draw bump pass
	if(drawBumps)
	{
		//Bind normal map to texture unit 0
		glBindTexture(GL_TEXTURE_2D, _normalMap);
		glEnable(GL_TEXTURE_2D);

		//Bind normalisation cube map to texture unit 1
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _normalisationCubeMap);
		glEnable(GL_TEXTURE_CUBE_MAP_ARB);
		glActiveTextureARB(GL_TEXTURE0_ARB);

		//Set vertex arrays for torus
		glVertexPointer(3, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].position);
		glEnableClientState(GL_VERTEX_ARRAY);

		//Send texture coords for normal map to unit 0
		glTexCoordPointer(2, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].s);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		//Send tangent space light vectors for normalisation to unit 1
		glClientActiveTextureARB(GL_TEXTURE1_ARB);
		glTexCoordPointer(3, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].tangentSpaceLight);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTextureARB(GL_TEXTURE0_ARB);


		//Set up texture environment to do (tex0 dot tex1)*color
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_ARB);
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB_ARB, GL_TEXTURE);
		glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB_ARB, GL_REPLACE);

		glActiveTextureARB(GL_TEXTURE1_ARB);
	
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_ARB);
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB_ARB, GL_TEXTURE);
		glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB_ARB, GL_DOT3_RGB_ARB);
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_RGB_ARB, GL_PREVIOUS_ARB);
	
		glActiveTextureARB(GL_TEXTURE0_ARB);
	
		//Draw
		glDrawElements(GL_TRIANGLES, _numIndices, GL_UNSIGNED_INT, _indices);

		//Disable textures
		glDisable(GL_TEXTURE_2D);

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glDisable(GL_TEXTURE_CUBE_MAP_ARB);
		glActiveTextureARB(GL_TEXTURE0_ARB);

		//disable vertex arrays
		glDisableClientState(GL_VERTEX_ARRAY);
	
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
		glClientActiveTextureARB(GL_TEXTURE1_ARB);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTextureARB(GL_TEXTURE0_ARB);

		//Return to standard modulate texenv
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	}

	//If we are drawing both passes, enable blending to multiply them together
	if(drawBumps && color)
	{
		//Enable multiplicative blending
		glBlendFunc(GL_DST_COLOR, GL_ZERO);
		glEnable(GL_BLEND);
	}

	//Perform a second pass to color the torus
	if(color)
	{
		if(!drawBumps && light)
		{
			glLightfv(GL_LIGHT1, GL_POSITION, VECTOR4D(objectLightPosition));
			glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
			glLightfv(GL_LIGHT1, GL_AMBIENT, black);
			glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
			glEnable(GL_LIGHT1);
			glEnable(GL_LIGHTING);

			glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
		}

		//Bind decal texture
		if(color)
		{
			glBindTexture(GL_TEXTURE_2D, _texture);
		} else {
			glBindTexture(GL_TEXTURE_2D, _textureb);
		}
		glEnable(GL_TEXTURE_2D);

		//Set vertex arrays for mesh
		glVertexPointer(3, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].position);
		glEnableClientState(GL_VERTEX_ARRAY);

		glNormalPointer(GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].normal);
		glEnableClientState(GL_NORMAL_ARRAY);

		glTexCoordPointer(2, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].s);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		//Draw torus
		glDrawElements(GL_TRIANGLES, _numIndices, GL_UNSIGNED_INT, _indices);

		if(!drawBumps)
			glDisable(GL_LIGHTING);

		//Disable texture
		glDisable(GL_TEXTURE_2D);

		//disable vertex arrays
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	}

	//Disable blending if it is enabled
	if(drawBumps && color)
		glDisable(GL_BLEND);
}

void PlanarPatch::DrawIdx(int indx) {
	
	if (!_view)
		return;

	glColor4f(float(indx)/255.0, float(indx)/255.0, float(indx)/255.0, 1.0);

	//Bind decal texture
	glBindTexture(GL_TEXTURE_2D, _textureb);
	glEnable(GL_TEXTURE_2D);

	//Set vertex arrays for mesh
	glVertexPointer(3, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].position);
	glEnableClientState(GL_VERTEX_ARRAY);

	glTexCoordPointer(2, GL_FLOAT, sizeof(PRIM_VERTEX), &_vertices[0].s);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	//Draw torus
	glDrawElements(GL_TRIANGLES, _numIndices, GL_UNSIGNED_INT, _indices);

	//Disable texture
	glDisable(GL_TEXTURE_2D);

	//disable vertex arrays
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

void PlanarPatch::DrawMeshFlat() {
	
	if (!_view)
		return;

	glColor4f(1.0, 1.0, 1.0, 1.0);

	glBindTexture( GL_TEXTURE_2D, _texture );
	glEnable(GL_TEXTURE_2D);
	glBegin( GL_QUADS );
	glTexCoord2d(0.0,0.0);												
	glVertex2f(0.0,0.0); //glVertex3f(v0[0], v0[1], v0[2]); //glVertex2d(0.0,0.0);
	glTexCoord2d(float(_Size[0])/float(_n),0.0);						
	glVertex2f(640,0.0); //glVertex3f(v1[0], v1[1], v1[2]); //glVertex2d(1.0,0.0);
	glTexCoord2d(float(_Size[0])/float(_n),float(_Size[1])/float(_m));	
	glVertex2f(640,480); //glVertex3f(v2[0], v2[1], v2[2]); //glVertex2d(1.0,1.0);
	glTexCoord2d(0.0,float(_Size[1])/float(_m));						
	glVertex2f(0.0,480); //glVertex3f(v3[0], v3[1], v3[2]); //glVertex2d(0.0,1.0);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void PlanarPatch::Load(char *filename){
	if (_Bump != NULL)
		return;

	char destfilename[100];
	string line;
	char tmpline[200];
	cv::Mat img_bump_x, img_bump_y, img_bump_z, img_rgb, img_mask;
	ifstream  filestr;
	
	//sprintf(destfilename, "%s\\eq.txt", filename);
	//filestr.open (destfilename, fstream::in);
	//while (!filestr.is_open()) {
	//	cout << "Could not open " << destfilename << endl;
	//	return;
	//}

	//// load equations
	//getline (filestr,line);
	//strcpy(tmpline, line.c_str());
	//sscanf (tmpline,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d", &_nmle[0], &_nmle[1], &_nmle[2], &_dist, 
	//	&_e1[0], &_e1[1], &_e1[2], &_e2[0], &_e2[1], &_e2[2], &_Shift[0], &_Shift[1], &_scale[0], &_scale[1], &_Size[0], &_Size[1]);

	//filestr.close();
	
	sprintf(destfilename, "%s\\Bump_x.png", filename);
	img_bump_x = cv::imread(destfilename, CV_LOAD_IMAGE_UNCHANGED);
	sprintf(destfilename, "%s\\Bump_y.png", filename);
	img_bump_y = cv::imread(destfilename, CV_LOAD_IMAGE_UNCHANGED);
	sprintf(destfilename, "%s\\Bump_z.png", filename);
	img_bump_z = cv::imread(destfilename, CV_LOAD_IMAGE_UNCHANGED);
	sprintf(destfilename, "%s\\RGB.png", filename);
	img_rgb = cv::imread(destfilename, CV_LOAD_IMAGE_UNCHANGED);
	sprintf(destfilename, "%s\\Mask.png", filename);
	img_mask = cv::imread(destfilename, CV_LOAD_IMAGE_UNCHANGED);

	_n = 1; _m = 1;

	while (_n < _Size[0])
		_n *= 2;
	
	while (_m < _Size[1])
		_m *= 2;

	BYTE * data = (BYTE *) malloc( _n * _m * 4 );
	memset(data,0,_n * _m * 4);

	/*_Bump = (unsigned short ***) malloc(5*sizeof(unsigned short **));
	_RGB = (unsigned char ***) malloc(5*sizeof(unsigned char **));
	_Mask = (unsigned char ***) malloc(5*sizeof(unsigned char **));

	for (int lvl = 0; lvl < 5; lvl++) {
		
		int fact = int(pow(2.0,lvl));

		int frag_n = _Size[0]/fact;
		int frag_m = _Size[1]/fact;

		_Bump[lvl] = (unsigned short **) malloc(frag_n*sizeof(unsigned short *));
		if (_Bump[lvl] == NULL)
			perror ("The following error occurred when allocating Bump in Load Graph");

		_RGB[lvl] = (unsigned char **) malloc(frag_n*sizeof(unsigned char *));
		if (_RGB[lvl] == NULL)
			perror ("The following error occurred when allocating RGB in Load Graph");
		
		_Mask[lvl] = (unsigned char **) malloc(frag_n*sizeof(unsigned char *));
		if (_Mask[lvl] == NULL)
			perror ("The following error occurred when allocating Mask in Load Graph");

		for (int i = 0; i < frag_n; i++) {
			_Bump[lvl][i] = (unsigned short *) malloc(3*frag_m*sizeof(unsigned short));
			if (_Bump[lvl][i] == NULL)
				perror ("The following error occurred when allocating Bump[i] in Load Graph");
			_RGB[lvl][i] = (unsigned char *) malloc(3*frag_m*sizeof(unsigned char));
			if (_RGB[lvl][i] == NULL)
				perror ("The following error occurred when allocating RGB[i] in Load Graph");
			_Mask[lvl][i] = (unsigned char *) malloc(frag_m*sizeof(unsigned char));
			if (_Mask[lvl][i] == NULL)
				perror ("The following error occurred when allocating Mask[i] in Load Graph");
		}
	}*/

	cout << "Size: " << _Size[0] << " " << _Size[1] << endl;
	cout << "_n, _m: " << _n << " " << _m << endl;
	/*cout << "_prev_Size: " << _prev_Size[0] << " " << _prev_Size[1] << endl;
	cout << "start i: " << max(_prev_Size[0] - (_Slide[0]*511+512), 0) << " end " << min(_prev_Size[0] - _Slide[0]*511, _prev_Size[0]) << endl;
	cout << "start j: " << _Slide[1]*511 << " end " << min(_Slide[1]*511+512, _prev_Size[1]) << endl;*/
	int i,j,k,l;
	/*for (i=max(_prev_Size[0] - (_Slide[0]*510+512), 0), k = _Size[0]-1; i < (_prev_Size[0] - _Slide[0]*510); i++, k--) {
		for (j=_Slide[1]*510,l= 0;j<min(_Slide[1]*510+512, _prev_Size[1]);j++, l++) {*/

	for (i=0; i < _n; i++) {
		for (j=0;j<_m;j++) {
			data[4*(j*_n + i)] = 255;
			data[4*(j*_n + i)+1] = 255;
			data[4*(j*_n + i)+2] = 255;
			data[4*(j*_n + i)+3] = 0;
		}
	}

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			/*_Bump[0][k][3*l] = (unsigned short)  60000.0*float(img_bump_x.at<unsigned char>(i,j))/255.0;
			_Bump[0][k][3*l+1] = (unsigned short)  60000.0*float(img_bump_y.at<unsigned char>(i,j))/255.0;
			_Bump[0][k][3*l+2] = img_bump_z.at<ushort>(i,j);

			_RGB[0][k][3*l] = img_rgb.at<cv::Vec3b>(i,j)[2];
			_RGB[0][k][3*l+1] = img_rgb.at<cv::Vec3b>(i,j)[1];
			_RGB[0][k][3*l+2] = img_rgb.at<cv::Vec3b>(i,j)[0];	*/

			data[4*(l*_n + k)] = img_rgb.at<cv::Vec3b>(i,j)[2];
			data[4*(l*_n + k)+1] = img_rgb.at<cv::Vec3b>(i,j)[1];
			data[4*(l*_n + k)+2] = img_rgb.at<cv::Vec3b>(i,j)[0];
			data[4*(l*_n + k)+3] = img_mask.at<unsigned char>(i,j) > 20 ? 255 : 0;
				
			//_Mask[0][k][l] = img_mask.at<unsigned char>(i,j) > 20 ? img_mask.at<unsigned char>(i,j) : 0;
		}
	}

	char* window_name = "Sobel Demo - Simple Edge Detector";
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	/// Gradient X
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::Mat grad;

	//cv::GaussianBlur( img_bump_z, img_bump_z, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

	cv::Sobel( img_bump_z, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	cv::Sobel( img_bump_z, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	/*cv::imshow( window_name, grad );

	cv::waitKey(0);*/
		
	_Controls.clear();
	
	/*_ControlsIdx.push_back(Point2D(0,0));
	_ControlsIdx.push_back(Point2D(_n,0));
	_ControlsIdx.push_back(Point2D(_n,_m));
	_ControlsIdx.push_back(Point2D(0,_m));*/
	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {

			if(grad.at<unsigned char>(i,j) > 50 && img_mask.at<unsigned char>(i,j) > 20) {
				_Controls.push_back(Point3D(float(k),float(j), 0.0));
			}
		}
	}
	
	cout << _Controls.size() << " nb of control points" << endl;
	
	//Create pyramid
	/*for (int lvl = 1; lvl < 5; lvl++) {

		int fact = int(pow(2.0,lvl));

		int frag_n = _Size[0]/fact;
		int frag_m = _Size[1]/fact;

		for (i = 0; i < frag_n; i++) {
			for (j = 0; j < frag_m; j++) {
				_Bump[lvl][i][3*j] = _Bump[0][i*fact][3*j*fact];
				_Bump[lvl][i][3*j+1] = _Bump[0][i*fact][3*j*fact+1];
				_Bump[lvl][i][3*j+2] = _Bump[0][i*fact][3*j*fact+2];

				_RGB[lvl][i][3*j] = _RGB[0][i*fact][3*j*fact];
				_RGB[lvl][i][3*j+1] = _RGB[0][i*fact][3*j*fact+1];
				_RGB[lvl][i][3*j+2] = _RGB[0][i*fact][3*j*fact+2];

				_Mask[lvl][i][j] = _Mask[0][i*fact][j*fact];
			}
		}
	}*/
	

	// select our current texture

	glBindTexture( GL_TEXTURE_2D, _texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );


	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			data[4*(l*_n + k)] = 200;
			data[4*(l*_n + k)+1] = 200;
			data[4*(l*_n + k)+2] = 200;
			data[4*(l*_n + k)+3] = img_mask.at<unsigned char>(i,j) > 20 ? 255 : 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _textureb );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );


	/// Build Normal maps
	// Build data
	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			data[4*(l*_n + k)] = 255;
			data[4*(l*_n + k)+1] = 0;
			data[4*(l*_n + k)+2] = 0;
			data[4*(l*_n + k)+3] = img_mask.at<unsigned char>(i,j) > 20 ? 255 : 0;
		}
	}

	//load
	glBindTexture(GL_TEXTURE_2D, _normalMap);
	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGBA8, _n, _m,
					0, GL_RGBA, GL_UNSIGNED_BYTE, data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	
	glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _normalisationCubeMap);
	GenerateNormalisationCubeMap();
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);


	// free buffer
	free( data );

	/******* Compute trinagulation 3D ******/

	// simplification by clustering using erase-remove idiom
	double cell_size = 10.0;
	_Controls.erase(CGAL::grid_simplify_point_set(_Controls.begin(), _Controls.end(), cell_size),
				_Controls.end());
	// Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
	std::vector<Point3D>(_Controls).swap(_Controls);
		
	cout << _Controls.size() << " nb of control points after simplication" << endl;

	_ControlsIdx.clear();
	for (vector<Point3D>::iterator it = _Controls.begin(); it != _Controls.end(); it++) {
		_ControlsIdx.push_back(Point2D(int(it->x()), int(it->y())));
	}

	/******** Compute Triangulation *********/
	//create a delaunay triangulation
    _dt.insert(_ControlsIdx.begin(), _ControlsIdx.end());

	//iterate through the faces
	_Controls.clear();
	i = 0;
    Delaunay::Finite_faces_iterator it;
    for (it = _dt.finite_faces_begin(); it != _dt.finite_faces_end(); it++)
    {
		i++;

		int ctrlsidx[3][2];

		for (k = 0; k < int(3); k++) {
			ctrlsidx[k][0] = int(_dt.triangle(it).vertex(k).x());
			ctrlsidx[k][1] = int(_dt.triangle(it).vertex(k).y());

			if (ctrlsidx[k][0] < _Size[0] && ctrlsidx[k][1] < _Size[1]) {
				float Shift_ind[2];
				Shift_ind[0] = float(img_bump_x.at<unsigned char>(_Size[0]-ctrlsidx[k][0]-1,ctrlsidx[k][1]))/255.0;
				Shift_ind[1] = float(img_bump_y.at<unsigned char>(_Size[0]-ctrlsidx[k][0]-1,ctrlsidx[k][1]))/255.0;

				float x = (ctrlsidx[k][0]+Shift_ind[0])*2.0/_scale[0] + _Shift[0];
				float y = (ctrlsidx[k][1]+Shift_ind[1])*2.0/_scale[1] + _Shift[1];

				float d = _dist + ((float(img_bump_z.at<ushort>(_Size[0]-ctrlsidx[k][0]-1,ctrlsidx[k][1]))/2000.0)-15.0);

				_Controls.push_back(Point3D(x*_e1[0] + y*_e2[0] + d*_nmle[0], x*_e1[1] + y*_e2[1] + d*_nmle[1], x*_e1[2] + y*_e2[2] + d*_nmle[2]));
			} else {
				float x = (ctrlsidx[k][0])*2.0/_scale[0] + _Shift[0];
				float y = (ctrlsidx[k][1])*2.0/_scale[1] + _Shift[1];

				float d = _dist;

				_Controls.push_back(Point3D(x*_e1[0] + y*_e2[0] + d*_nmle[0], x*_e1[1] + y*_e2[1] + d*_nmle[1], x*_e1[2] + y*_e2[2] + d*_nmle[2]));
			}
		}
    }
		
	_Size[0] = _n;
	_Size[1] = _m;

	
	img_bump_x.~Mat();
	img_bump_y.~Mat();
	img_bump_z.~Mat();
	img_rgb.~Mat();
	img_mask.~Mat();


	return;
}

void PlanarPatch::LoadEquation(char *filename){
	char destfilename[100];
	string line;
	char tmpline[200];
	ifstream  filestr;
	
	sprintf(destfilename, "%s\\eq.txt", filename);
	filestr.open (destfilename, fstream::in);
	while (!filestr.is_open()) {
		cout << "Could not open " << destfilename << endl;
		return;
	}

	// load equations
	getline (filestr,line);
	strcpy(tmpline, line.c_str());
	sscanf (tmpline,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d", &_nmle[0], &_nmle[1], &_nmle[2], &_dist, 
		&_e1[0], &_e1[1], &_e1[2], &_e2[0], &_e2[1], &_e2[2], &_Shift[0], &_Shift[1], &_scale[0], &_scale[1], &_Size[0], &_Size[1]);

	filestr.close();

	_prev_Size[0] = _Size[0]; _prev_Size[1] = _Size[1];
	_Slide[0] = 0; _Slide[1] = 0;
	
	return;
}

bool PlanarPatch::IsVisible(float x, float y, float z, float lx, float ly, float lz) {

	float *center = getCenter(0.5, 0.5);

	bool res = false;
	float dist = sqrt((x-center[0])*(x-center[0]) + (y-center[1])*(y-center[1]) + (z-center[2])*(z-center[2]));
	float prodscal = ((center[0]-x)*lx+ (center[1]-y)*ly + (center[2]-z)*lz)/(dist);
	if (dist < 20.0 && prodscal > 0.2)
		res = true;

	delete center;
	int prec_lvl = _lvl;

	if (dist < 1.0) {
		_lvl = 0;
	} else if (dist < 2.0) {
		_lvl = 1;
	} else if (dist < 4.0) {
		_lvl = 2;
	} else if (dist < 8.0) {
		_lvl = 3;
	} else {
		_lvl = 4;
	}

	return (res && (_lvl == prec_lvl || !_view));
}

vector<PlanarPatch *>  PlanarPatch::Segment() {
	vector<PlanarPatch *> res;
	PlanarPatch *Prim;

	int res_n = _Size[0];
	int res_m = _Size[1];

	int s_i = 0;
	int s_j = 0;

	while (res_n > 512 || res_m > 512) {
		Prim = new PlanarPatch(512,512);
		Prim->setEquation(this);
		Prim->set_Slide_Shift(s_i, s_j);
		if (res_n > 512) {
			if (res_m > 512) {
				Prim->setSize(512, 512);
			} else {
				Prim->setSize(512, res_m);
			}
			s_i++;
			res_n -= 510;
		} else {
			if (res_m > 512) {
				Prim->setSize(res_n, 512);
			} else {
				Prim->setSize(res_n, res_m);
			}
			s_i = 0;
			s_j ++;
			res_n = _Size[0];
			res_m -= 510;
		}
		Prim->setNM(512, 512, 2);
		res.push_back(Prim);
	}

	return res;
}

void PlanarPatch::set_Slide_Shift(int s_i, int s_j) {
	_Slide[0] = s_i;
	_Slide[1] = s_j;

	_Shift[0] = _Shift[0] + float(s_i*510)*(2.0/_scale[0]);
	_Shift[1] = _Shift[1] + float(s_j*510)*(2.0/_scale[1]);
}

void PlanarPatch::ComputeTriangulation() {
	_n = 1; _m = 1;

	while (_n < _Size[0])
		_n *= 2;
	
	while (_m < _Size[1])
		_m *= 2;

	BYTE * data = (BYTE *) malloc( _n * _m * 4 );
	memset(data,0,_n * _m * 4);
	BYTE *dataPtr;
	
	//cout << "Size: " << _Size[0] << " " << _Size[1] << endl;
	//cout << "_n, _m: " << _n << " " << _m << endl;

	dataPtr = data;
	int i,j,k,l;
	for (i=0; i < _n; i++) {
		for (j=0;j<_m;j++) {
			dataPtr[0] = 255; //data[4*(j*_n + i)] = 255;
			dataPtr[1] = 255; //data[4*(j*_n + i)+1] = 255;
			dataPtr[2] = 255; //data[4*(j*_n + i)+2] = 255;
			dataPtr[3] = 0; //data[4*(j*_n + i)+3] = 0;
			dataPtr += 4;
		}
	}

	cv::Mat img_bump_z(_Size[0], _Size[1], CV_16UC1);
	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++)  {
			data[4*(l*_n + k)] = _RGB[k][3*l]; 
			data[4*(l*_n + k)+1] = _RGB[k][3*l+1];
			data[4*(l*_n + k)+2] = _RGB[k][3*l+2];
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
			img_bump_z.at<unsigned short>(i,j) = _Mask[k][l] > 10 ? _Bump[k][3*l+2] : 0;
		}
	}

	char* window_name = "Sobel Demo - Simple Edge Detector";
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	/// Gradient X
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::Mat grad;

	//cv::GaussianBlur( img_bump_z, img_bump_z, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
	//cv::imshow( window_name, img_bump_z );

	cv::Sobel( img_bump_z, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	cv::Sobel( img_bump_z, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	//cv::imshow( window_name, grad );

	/*cv::waitKey(0);*/
		
	_Controls.clear();
	
	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {

			if(grad.at<unsigned char>(i,j) > 50 && _Mask[k][l] > 10) {
				/*int lb = max(0, k-60);
				int lu = min(_Size[0], k+60);
				int rb = max(0, j-60);
				int ru = min(_Size[1], j+60);
				for (int p = lb; p < lu; p+=30)
					for (int q = rb; q < ru; q+=30) {
						if (_Mask[p][q] > 10)
							_Controls.push_back(Point3D(float(p),float(q), 0.0));
					}*/
				_Controls.push_back(Point3D(float(k),float(j), 0.0));
			}
		}
	}
	
	//cout << _Controls.size() << " nb of control points" << endl;
	
	glBindTexture( GL_TEXTURE_2D, _texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			data[4*(l*_n + k)] = 255;
			data[4*(l*_n + k)+1] = 255;
			data[4*(l*_n + k)+2] = 255;
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _textureb );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	/******* Compute trinagulation 3D ******/

	// simplification by clustering using erase-remove idiom
	double cell_size = 2.0;
	_Controls.erase(CGAL::grid_simplify_point_set(_Controls.begin(), _Controls.end(), cell_size),
				_Controls.end());
	// Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
	std::vector<Point3D>(_Controls).swap(_Controls);
		
	//cout << _Controls.size() << " nb of control points after simplication" << endl;

	_ControlsIdx.clear();
	/*_ControlsIdx.push_back(Point2D(0, 0));
	_ControlsIdx.push_back(Point2D(0, _Size[1]-1));
	_ControlsIdx.push_back(Point2D(_Size[0]-1, _Size[1]-1));
	_ControlsIdx.push_back(Point2D(_Size[0]-1, 0));*/
	for (vector<Point3D>::iterator it = _Controls.begin(); it != _Controls.end(); it++) {
		_ControlsIdx.push_back(Point2D(int(it->x()), int(it->y())));
	}

	/******** Compute Triangulation *********/
	//create a delaunay triangulation
	_dt.clear();
    _dt.insert(_ControlsIdx.begin(), _ControlsIdx.end());

	_numVertices= _ControlsIdx.size();
	_numIndices=_dt.number_of_faces()*3;

    _vertices=new PRIM_VERTEX[_numVertices];
	 if(!_vertices)
    {
        printf("Unable to allocate memory for Prim vertices\n");
        return;
    }

    _indices=new unsigned int[_numIndices];
    if(!_indices)
    {
        printf("Unable to allocate memory for Prim indices\n");
        return;
    }
	
	// Allocate buffer for unsigned short
	typedef unsigned short image_type;
	if (_dt.number_of_faces() > 65535) {
		// Allocate buffer for unsigned short
		typedef unsigned int image_type;
		cout << "int type" << endl;
	}

	image_type **imageDta = (image_type **) malloc(_n * sizeof(image_type *));
	if (imageDta == NULL)
		perror ("The following error occurred when allocating imageDta");
		
	for (int i = 0; i < _n; i++) {
		imageDta[i] = (image_type *) malloc(_m*sizeof(image_type));
		if (imageDta[i] == NULL)
			perror ("The following error occurred when allocating imageDta[i]");
		memset(imageDta[i], 0, _m * sizeof(image_type));
	}

	// Compute Vertex map and Normal map
	float **VMap = (float **) malloc(_Size[0] * sizeof(float *));
	if (VMap == NULL)
		perror ("The following error occurred when allocating TheBump in Merge");
	float **NMap = (float **) malloc(_Size[0] * sizeof(float *));
	if (NMap == NULL)
		perror ("The following error occurred when allocating TheBump in Merge");

	for (int i = 0; i < _Size[0]; i++) {
		VMap[i] = (float *) malloc(3*_Size[1]*sizeof(float));
		if (VMap[i] == NULL)
			perror ("The following error occurred when allocating TheBump[i] in Load Graph");
		memset(VMap[i], 0, 3*_Size[1] * sizeof(float));
		NMap[i] = (float *) malloc(3*_Size[1]*sizeof(float));
		if (NMap[i] == NULL)
			perror ("The following error occurred when allocating TheRGB[i] in Load Graph");
		memset(NMap[i], 0, 3*_Size[1] * sizeof(float));
	}

	ComputeVMAPNMAP(VMap, NMap);

	//iterate through the vertices
	int ind=0;
	int ctrlsidx[2];
	Delaunay::Finite_vertices_iterator itv;
    for (itv = _dt.finite_vertices_begin(); itv != _dt.finite_vertices_end(); itv++)
    {
		ctrlsidx[0] = int((*itv).point().x());
		ctrlsidx[1] = int((*itv).point().y());

		if (ctrlsidx[0] < _Size[0] && ctrlsidx[1] < _Size[1]) {
			float Shift_ind[2];
			_vertices[ind].position = VECTOR3D(VMap[ctrlsidx[0]][3*ctrlsidx[1]], VMap[ctrlsidx[0]][3*ctrlsidx[1]+1], VMap[ctrlsidx[0]][3*ctrlsidx[1]+2]);
		} else {
			float x = (ctrlsidx[0])*2.0/_scale[0] + _Shift[0];
			float y = (ctrlsidx[1])*2.0/_scale[1] + _Shift[1];

			float d = _dist;

			_vertices[ind].position = VECTOR3D(x*_e1[0] + y*_e2[0] + d*_nmle[0], x*_e1[1] + y*_e2[1] + d*_nmle[1], x*_e1[2] + y*_e2[2] + d*_nmle[2]);
		}

		_vertices[ind].s = float((*itv).point().x())/float(_n);
		_vertices[ind].t = float((*itv).point().y())/float(_m);
		_vertices[ind].weight = 0.0;

		// Compute Normal and tangents
		_vertices[ind].normal = VECTOR3D(0.0,0.0,0.0); //NMap[ctrlsidx[0]][3*ctrlsidx[1]], NMap[ctrlsidx[0]][3*ctrlsidx[1]+1], NMap[ctrlsidx[0]][3*ctrlsidx[1]+2]); 
		_vertices[ind].sTangent = VECTOR3D(0.0,0.0,0.0); //ComputeTangentS(_vertices[ind].normal, _vertices[ind].position, ctrlsidx[0]+1, ctrlsidx[1], VMap, _Size[0], _Size[1]);
		_vertices[ind].tTangent = VECTOR3D(0.0,0.0,0.0); //ComputeTangentT(_vertices[ind].normal, _vertices[ind].position, _vertices[ind].sTangent, ctrlsidx[0], ctrlsidx[1]+1, VMap, _Size[0], _Size[1]);

		ind++;

	}

	//iterate through the faces
	_Controls.clear();
	ind = 0;
    Delaunay::Finite_faces_iterator it;
    for (it = _dt.finite_faces_begin(); it != _dt.finite_faces_end(); it++)
    {
		
		for (k = 0; k < int(3); k++) {
			unsigned int j = 0;
			Delaunay::Finite_vertices_iterator itTmp;
			for (itTmp = _dt.finite_vertices_begin(); itTmp != _dt.finite_vertices_end(); itTmp++)
			{
				if (_dt.triangle(it).vertex(k).x() == (*itTmp).point().x() && _dt.triangle(it).vertex(k).y() == (*itTmp).point().y())
					break;
				j++;
			}
			_indices[3*ind+k] = j;
		}
		ind++;
    }

	for (int i = 0; i < _numIndices; i+=3) {
		// Compute matrix tangent space transform
		VECTOR3D v0 = _vertices[_indices[i]].position;
		VECTOR3D v1 = _vertices[_indices[i+1]].position;
		VECTOR3D v2 = _vertices[_indices[i+2]].position;
 
		// Shortcuts for UVs
		VECTOR2D uv0 = VECTOR2D(_vertices[_indices[i]].s, _vertices[_indices[i]].t);
		VECTOR2D uv1 = VECTOR2D(_vertices[_indices[i+1]].s, _vertices[_indices[i+1]].t);
		VECTOR2D uv2 = VECTOR2D(_vertices[_indices[i+2]].s, _vertices[_indices[i+2]].t);
 
		// Edges of the triangle : position delta
		VECTOR3D deltaPos1 = v1-v0;
		VECTOR3D deltaPos2 = v2-v0;
 
		// UV delta
		VECTOR2D deltaUV1 = uv1-uv0;
		VECTOR2D deltaUV2 = uv2-uv0;

		float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		VECTOR3D tangent = (deltaPos1 * deltaUV2.y   - deltaPos2 * deltaUV1.y)*r;
		tangent.Normalize();
		VECTOR3D bitangent = (deltaPos2 * deltaUV1.x   - deltaPos1 * deltaUV2.x)*r;
		bitangent = bitangent - tangent*tangent.DotProduct(bitangent);
		bitangent.Normalize();
		VECTOR3D normale = VECTOR3D(_nmle [0], _nmle [1], _nmle [2]); //deltaPos1.CrossProduct(deltaPos2);
		//normale.Normalize();
		if ((_nmle [0]*normale.x + _nmle [1]*normale.y + _nmle [2]*normale.z) < 0.0) {
			normale = -normale;
			tangent = -tangent;
			bitangent = -bitangent;
			//cout << "reversed normal" << endl;
		}

		_vertices[_indices[i]].normal += normale;//(_vertices[_indices[i]].weight*_vertices[_indices[i]].normal + normale)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].normal += normale;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].normal + normale)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].normal += normale;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].normal + normale)/(_vertices[_indices[i+2]].weight + 1.0);
		
		_vertices[_indices[i]].sTangent += tangent;//= (_vertices[_indices[i]].weight*_vertices[_indices[i]].sTangent + tangent)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].sTangent += tangent;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].sTangent + tangent)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].sTangent += tangent;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].sTangent + tangent)/(_vertices[_indices[i+2]].weight + 1.0);
		
		_vertices[_indices[i]].tTangent += bitangent;//= (_vertices[_indices[i]].weight*_vertices[_indices[i]].tTangent + bitangent)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].tTangent += bitangent;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].tTangent + bitangent)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].tTangent += bitangent;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].tTangent + bitangent)/(_vertices[_indices[i+2]].weight + 1.0);

		_vertices[_indices[i]].weight += 1.0;
		_vertices[_indices[i+1]].weight += 1.0;
		_vertices[_indices[i+2]].weight += 1.0;
	}
	
	for (int i = 0; i < _numVertices; i++) {
		_vertices[i].normal.Normalize();
		_vertices[i].sTangent = _vertices[i].sTangent - _vertices[i].normal*_vertices[i].normal.DotProduct(_vertices[i].sTangent);
		_vertices[i].sTangent.Normalize();
		_vertices[i].tTangent = _vertices[i].tTangent - _vertices[i].normal*_vertices[i].normal.DotProduct(_vertices[i].tTangent);
		_vertices[i].tTangent = _vertices[i].tTangent - _vertices[i].sTangent*_vertices[i].sTangent.DotProduct(_vertices[i].tTangent);
		_vertices[i].tTangent.Normalize();
	}

	image_type clr = 1;
	for (it = _dt.finite_faces_begin(); it != _dt.finite_faces_end(); it++)
	{
		DrawTriangle<image_type>(_dt.triangle(it), imageDta, clr);
		clr++;
	}

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			// Find corresponding triangle
			ind = int(imageDta[k][l])-1;
			float Nml [3];
			bool ok_go = false;
			if (ind != -1) {
				VECTOR2D X = VECTOR2D(float(k), float(l));
				VECTOR2D A = VECTOR2D(_vertices[_indices[3*ind]].s*float(_n), _vertices[_indices[3*ind]].t*float(_m));
				VECTOR2D B = VECTOR2D(_vertices[_indices[3*ind+1]].s*float(_n), _vertices[_indices[3*ind+1]].t*float(_m));
				VECTOR2D C = VECTOR2D(_vertices[_indices[3*ind+2]].s*float(_n), _vertices[_indices[3*ind+2]].t*float(_m));
				VECTOR2D AB = B-A;
				VECTOR2D BC = C-B;
				VECTOR2D AC = C-A;
				VECTOR2D AX = X-A;
				VECTOR2D BX = X-B;
				VECTOR2D CX = X-C;

				//compute intersection btw AX and BC
				// A
				float f1;
				if (AX.x != 0.0 && BC.y != 0.0)
					f1 = (B.x +(BC.x*(A.y - B.y))/BC.y -A.x)/AX.x;
				float f2;
				if (BC.y != 0.0)
					f2 = A.x*BC.x/BC.y;
				float lambda;
				if (f2 != 0.0)
					f2 = f1/f2;
				float w_a;
				if (X.x == A.x && X.y == A.y)
					w_a = 1.0;
				else
					w_a = 1.0 - (X-A).GetLength()/(A+lambda*AX).GetLength();
					
				// B
				if (BX.x != 0.0 && AC.y != 0.0)
					f1 = (A.x +(AC.x*(B.y - A.y))/AC.y -B.x)/BX.x;
				if (AC.y != 0.0)
					f2 = B.x*AC.x/AC.y;
				if (f2 != 0.0)
					lambda = f1/f2;
				float w_b;
				if (X.x == B.x && X.y == B.y)
					w_b = 1.0;
				else
					w_b = 1.0 - (X-B).GetLength()/(B+lambda*BX).GetLength();
					
				// C
				if (CX.x != 0.0 && AB.y != 0.0)
					f1 = (A.x +(AB.x*(C.y - A.y))/AB.y -C.x)/CX.x;
				if (AB.y != 0.0)
					f2 = C.x*AB.x/AB.y;
				if (f2 != 0.0)
					lambda = f1/f2;
				float w_c;
				if (X.x == C.x && X.y == C.y)
					w_c = 1.0;
				else
					w_c = 1.0 - (X-C).GetLength()/(C+lambda*CX).GetLength();

				// Compute weighted normals, tangents and bitangents
				VECTOR3D normale = w_a*_vertices[_indices[3*ind]].normal + w_b*_vertices[_indices[3*ind+1]].normal + w_c*_vertices[_indices[3*ind+2]].normal;
				VECTOR3D tangent = w_a*_vertices[_indices[3*ind]].sTangent + w_b*_vertices[_indices[3*ind+1]].sTangent + w_c*_vertices[_indices[3*ind+2]].sTangent;
				VECTOR3D bitangent = w_a*_vertices[_indices[3*ind]].tTangent + w_b*_vertices[_indices[3*ind+1]].tTangent + w_c*_vertices[_indices[3*ind+2]].tTangent;

				//normalize normal vector
				normale.Normalize();
				float tmp = normale.GetLength();
					
				//orthogonolize and normalize tangent vector
				tangent = tangent - normale*normale.DotProduct(tangent);
				tangent.Normalize();
				tmp = tangent.GetLength();

				//orthogonolize and normalize bitangent vector
				bitangent = bitangent - normale*normale.DotProduct(bitangent);
				bitangent = bitangent - tangent*tangent.DotProduct(bitangent);
				bitangent.Normalize();
				tmp = bitangent.GetLength();

				Nml[0] = tangent.x*NMap[k][3*l] + tangent.y*NMap[k][3*l+1] + tangent.z*NMap[k][3*l+2];
				Nml[1] = bitangent.x*NMap[k][3*l] + bitangent.y*NMap[k][3*l+1] + bitangent.z*NMap[k][3*l+2];
				Nml[2] = normale.x*NMap[k][3*l] + normale.y*NMap[k][3*l+1] + normale.z*NMap[k][3*l+2]; 
				if ((_nmle [0]*Nml[0] + _nmle [1]*Nml[1] + _nmle [2]*Nml[2]) < 0.0) {
					Nml[0] = -Nml[0];
					Nml[1] = -Nml[1];
					Nml[2] = -Nml[2];
				}
				ok_go = true;
			}

			data[4*(l*_n + k)] = ok_go ? BYTE(255.0*(Nml[0]+1.0)/2.0) : 255; // ind == -1 ? 255 : ind/2;//
			data[4*(l*_n + k)+1] = ok_go ? BYTE(255.0*(Nml[1]+1.0)/2.0) : 0;
			data[4*(l*_n + k)+2] = ok_go ? BYTE(255.0*(Nml[2]+1.0)/2.0) : 0;
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
			if (!ok_go)
				data[4*(l*_n + k)+3] = 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _normalMap );
	//glBindTexture( GL_TEXTURE_2D, _texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	// free buffer
	free( data );

	//Create normalisation cube map
	glBindTexture(GL_TEXTURE_CUBE_MAP_ARB, _normalisationCubeMap);
	GenerateNormalisationCubeMap();
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_ARB, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	for (int i = 0; i < _Size[0]; i++) {
		if (VMap[i] != NULL)
			free(VMap[i]);
		if (NMap[i] != NULL)
			free(NMap[i]);
	}
	if (VMap != NULL)
		free(VMap);
	if (NMap != NULL)
		free(NMap);
}

void PlanarPatch::ComputeTriangulationLight() {
	_n = 1; _m = 1;

	while (_n < _Size[0])
		_n *= 2;
	
	while (_m < _Size[1])
		_m *= 2;

	BYTE * data = (BYTE *) malloc( _n * _m * 4 );
	memset(data,0,_n * _m * 4);
	BYTE *dataPtr;

	dataPtr = data;
	int i,j,k,l;
	for (i=0; i < _n; i++) {
		for (j=0;j<_m;j++) {
			dataPtr[0] = 255; //data[4*(j*_n + i)] = 255;
			dataPtr[1] = 255; //data[4*(j*_n + i)+1] = 255;
			dataPtr[2] = 255; //data[4*(j*_n + i)+2] = 255;
			dataPtr[3] = 0; //data[4*(j*_n + i)+3] = 0;
			dataPtr += 4;
		}
	}

	cv::Mat img_bump_z(_Size[0], _Size[1], CV_16UC1);
	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++)  {
			data[4*(l*_n + k)] = _RGB[k][3*l]; 
			data[4*(l*_n + k)+1] = _RGB[k][3*l+1];
			data[4*(l*_n + k)+2] = _RGB[k][3*l+2];
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
			img_bump_z.at<unsigned short>(i,j) = _Mask[k][l] > 10 ? _Bump[k][3*l+2] : 0;
		}
	}


	glBindTexture( GL_TEXTURE_2D, _texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			data[4*(l*_n + k)] = 255;
			data[4*(l*_n + k)+1] = 255;
			data[4*(l*_n + k)+2] = 255;
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _textureb );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	/******* Compute simple triangulation 3D ******/

	_ControlsIdx.clear();
	_ControlsIdx.push_back(Point2D(0, 0));
	_ControlsIdx.push_back(Point2D(0, _Size[1]-1));
	_ControlsIdx.push_back(Point2D(_Size[0]-1, _Size[1]-1));
	_ControlsIdx.push_back(Point2D(_Size[0]-1, 0));

	/******** Compute Triangulation *********/
	//create a delaunay triangulation
	_dt.clear();
    _dt.insert(_ControlsIdx.begin(), _ControlsIdx.end());

	_numVertices= _ControlsIdx.size();
	_numIndices=_dt.number_of_faces()*3;

    _vertices=new PRIM_VERTEX[_numVertices];
	 if(!_vertices)
    {
        printf("Unable to allocate memory for Prim vertices\n");
        return;
    }

    _indices=new unsigned int[_numIndices];
    if(!_indices)
    {
        printf("Unable to allocate memory for Prim indices\n");
        return;
    }
	
	// Allocate buffer for unsigned short
	//typedef unsigned short image_type;
	//if (_dt.number_of_faces() > 65535) {
	//	// Allocate buffer for unsigned short
	//	typedef unsigned int image_type;
	//	cout << "int type" << endl;
	//}

	//image_type **imageDta = (image_type **) malloc(_n * sizeof(image_type *));
	//if (imageDta == NULL)
	//	perror ("The following error occurred when allocating imageDta");
	//	
	//for (int i = 0; i < _n; i++) {
	//	imageDta[i] = (image_type *) malloc(_m*sizeof(image_type));
	//	if (imageDta[i] == NULL)
	//		perror ("The following error occurred when allocating imageDta[i]");
	//	memset(imageDta[i], 0, _m * sizeof(image_type));
	//}

	//iterate through the vertices
	int ind=0;
	int ctrlsidx[2];
	Delaunay::Finite_vertices_iterator itv;
    for (itv = _dt.finite_vertices_begin(); itv != _dt.finite_vertices_end(); itv++)
    {
		ctrlsidx[0] = int((*itv).point().x());
		ctrlsidx[1] = int((*itv).point().y());

		float x = (ctrlsidx[0])*2.0/_scale[0] + _Shift[0];
		float y = (ctrlsidx[1])*2.0/_scale[1] + _Shift[1];

		float d = _dist;

		_vertices[ind].position = VECTOR3D(x*_e1[0] + y*_e2[0] + d*_nmle[0], x*_e1[1] + y*_e2[1] + d*_nmle[1], x*_e1[2] + y*_e2[2] + d*_nmle[2]);
		
		_vertices[ind].s = float((*itv).point().x())/float(_n);
		_vertices[ind].t = float((*itv).point().y())/float(_m);
		_vertices[ind].weight = 0.0;

		// Compute Normal and tangents
		_vertices[ind].normal = VECTOR3D(0.0,0.0,0.0); //NMap[ctrlsidx[0]][3*ctrlsidx[1]], NMap[ctrlsidx[0]][3*ctrlsidx[1]+1], NMap[ctrlsidx[0]][3*ctrlsidx[1]+2]); 
		_vertices[ind].sTangent = VECTOR3D(0.0,0.0,0.0); //ComputeTangentS(_vertices[ind].normal, _vertices[ind].position, ctrlsidx[0]+1, ctrlsidx[1], VMap, _Size[0], _Size[1]);
		_vertices[ind].tTangent = VECTOR3D(0.0,0.0,0.0); //ComputeTangentT(_vertices[ind].normal, _vertices[ind].position, _vertices[ind].sTangent, ctrlsidx[0], ctrlsidx[1]+1, VMap, _Size[0], _Size[1]);

		ind++;

	}

	//iterate through the faces
	ind = 0;
    Delaunay::Finite_faces_iterator it;
    for (it = _dt.finite_faces_begin(); it != _dt.finite_faces_end(); it++)
    {
		
		for (k = 0; k < int(3); k++) {
			unsigned int j = 0;
			Delaunay::Finite_vertices_iterator itTmp;
			for (itTmp = _dt.finite_vertices_begin(); itTmp != _dt.finite_vertices_end(); itTmp++)
			{
				if (_dt.triangle(it).vertex(k).x() == (*itTmp).point().x() && _dt.triangle(it).vertex(k).y() == (*itTmp).point().y())
					break;
				j++;
			}
			_indices[3*ind+k] = j;
		}
		ind++;
    }

	for (int i = 0; i < _numIndices; i+=3) {
		// Compute matrix tangent space transform
		VECTOR3D v0 = _vertices[_indices[i]].position;
		VECTOR3D v1 = _vertices[_indices[i+1]].position;
		VECTOR3D v2 = _vertices[_indices[i+2]].position;
 
		// Shortcuts for UVs
		VECTOR2D uv0 = VECTOR2D(_vertices[_indices[i]].s, _vertices[_indices[i]].t);
		VECTOR2D uv1 = VECTOR2D(_vertices[_indices[i+1]].s, _vertices[_indices[i+1]].t);
		VECTOR2D uv2 = VECTOR2D(_vertices[_indices[i+2]].s, _vertices[_indices[i+2]].t);
 
		// Edges of the triangle : position delta
		VECTOR3D deltaPos1 = v1-v0;
		VECTOR3D deltaPos2 = v2-v0;
 
		// UV delta
		VECTOR2D deltaUV1 = uv1-uv0;
		VECTOR2D deltaUV2 = uv2-uv0;

		float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		VECTOR3D tangent = (deltaPos1 * deltaUV2.y   - deltaPos2 * deltaUV1.y)*r;
		tangent.Normalize();
		VECTOR3D bitangent = (deltaPos2 * deltaUV1.x   - deltaPos1 * deltaUV2.x)*r;
		bitangent = bitangent - tangent*tangent.DotProduct(bitangent);
		bitangent.Normalize();
		VECTOR3D normale = VECTOR3D(_nmle [0], _nmle [1], _nmle [2]); //deltaPos1.CrossProduct(deltaPos2);
		//normale.Normalize();
		if ((_nmle [0]*normale.x + _nmle [1]*normale.y + _nmle [2]*normale.z) < 0.0) {
			normale = -normale;
			tangent = -tangent;
			bitangent = -bitangent;
			//cout << "reversed normal" << endl;
		}

		_vertices[_indices[i]].normal += normale;//(_vertices[_indices[i]].weight*_vertices[_indices[i]].normal + normale)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].normal += normale;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].normal + normale)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].normal += normale;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].normal + normale)/(_vertices[_indices[i+2]].weight + 1.0);
		
		_vertices[_indices[i]].sTangent += tangent;//= (_vertices[_indices[i]].weight*_vertices[_indices[i]].sTangent + tangent)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].sTangent += tangent;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].sTangent + tangent)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].sTangent += tangent;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].sTangent + tangent)/(_vertices[_indices[i+2]].weight + 1.0);
		
		_vertices[_indices[i]].tTangent += bitangent;//= (_vertices[_indices[i]].weight*_vertices[_indices[i]].tTangent + bitangent)/(_vertices[_indices[i]].weight + 1.0);
		_vertices[_indices[i+1]].tTangent += bitangent;//= (_vertices[_indices[i+1]].weight*_vertices[_indices[i+1]].tTangent + bitangent)/(_vertices[_indices[i+1]].weight + 1.0);
		_vertices[_indices[i+2]].tTangent += bitangent;//= (_vertices[_indices[i+2]].weight*_vertices[_indices[i+2]].tTangent + bitangent)/(_vertices[_indices[i+2]].weight + 1.0);

		_vertices[_indices[i]].weight += 1.0;
		_vertices[_indices[i+1]].weight += 1.0;
		_vertices[_indices[i+2]].weight += 1.0;
	}
	
	for (int i = 0; i < _numVertices; i++) {
		_vertices[i].normal.Normalize();
		_vertices[i].sTangent = _vertices[i].sTangent - _vertices[i].normal*_vertices[i].normal.DotProduct(_vertices[i].sTangent);
		_vertices[i].sTangent.Normalize();
		_vertices[i].tTangent = _vertices[i].tTangent - _vertices[i].normal*_vertices[i].normal.DotProduct(_vertices[i].tTangent);
		_vertices[i].tTangent = _vertices[i].tTangent - _vertices[i].sTangent*_vertices[i].sTangent.DotProduct(_vertices[i].tTangent);
		_vertices[i].tTangent.Normalize();
	}

	// free buffer
	free( data );
}

void PlanarPatch::Merge(PlanarPatch *Prim_in, float *NewCenter, int *NewSize) {

	if (Prim_in == NULL)
		return;

	unsigned short **TheBump = (unsigned short **) malloc(NewSize [0] * sizeof(unsigned short *));
	if (TheBump == NULL)
		perror ("The following error occurred when allocating TheBump in Merge");
	unsigned short **TheMask = (unsigned short **) malloc(NewSize [0] * sizeof(unsigned short *));
	if (TheMask == NULL)
		perror ("The following error occurred when allocating TheBump in Merge");
	unsigned char **TheRGB = (unsigned char **) malloc(NewSize [0] * sizeof(unsigned char *));
	if (TheRGB == NULL)
		perror ("The following error occurred when allocating TheBump in Merge");

	for (int i = 0; i < NewSize[0]; i++) {
		TheBump[i] = (unsigned short *) malloc(3*NewSize[1]*sizeof(unsigned short));
		if (TheBump[i] == NULL)
			perror ("The following error occurred when allocating TheBump[i] in Load Graph");
		memset(TheBump[i], 0, 3*NewSize [1] * sizeof(unsigned short));
		TheRGB[i] = (unsigned char *) malloc(3*NewSize[1]*sizeof(unsigned char));
		if (TheRGB[i] == NULL)
			perror ("The following error occurred when allocating TheRGB[i] in Load Graph");
		memset(TheRGB[i], 0, 3*NewSize [1] * sizeof(unsigned char));
		TheMask[i] = (unsigned short *) malloc(NewSize[1]*sizeof(unsigned short));
		if (TheMask[i] == NULL)
			perror ("The following error occurred when allocating TheMask[i] in Load Graph");
		memset(TheMask[i], 0, NewSize [1] * sizeof(unsigned short));
	}

	// Project This primitive onto new images
	for (int i = 0; i < _Size[0]; i++) {
		for (int j = 0; j < _Size[1]; j++) {
			if (_Mask[i][j] > unsigned short(10)) {		
				unsigned short scal = _Bump[i][3*j+2]; 
				unsigned char rgb [3];
				rgb [0] = _RGB[i][3*j];
				rgb [1] = _RGB[i][3*j+1];
				rgb [2] = _RGB[i][3*j+2];

				float Shift_ind[2];
				Shift_ind[0] = float(_Bump[i][3*j])/60000.0;
				Shift_ind[1] = float(_Bump[i][3*j+1])/60000.0;

				float x, y;
				x = (i+Shift_ind[0])*2.0/_scale[0] + _Shift[0];
				y = (j+Shift_ind[1])*2.0/_scale[1] + _Shift[1];

				float a, b;
				a = x-NewCenter[0];
				b = y-NewCenter[1];

				a = (a*_scale[0])/2.0f;
				b = (b*_scale[1])/2.0f;

				int ind_i = int(a);
				int ind_j = int(b);

				if (ind_i < 0 || ind_i > NewSize[0]-1 || ind_j < 0 || ind_j > NewSize[1]-1)
					continue;

				unsigned short shift [2];
				shift[0] = unsigned short((a-float(ind_i))*60000.0);
				shift[1] = unsigned short((b-float(ind_j))*60000.0);

				//if (TheMask[ind_i][ind_j] < _Mask[i][j]) {
					TheBump[ind_i][3*ind_j] = shift[0];
					TheBump[ind_i][3*ind_j+1] = shift[1];
					TheBump[ind_i][3*ind_j+2] = scal;

					TheRGB[ind_i][3*ind_j] = rgb[0];
					TheRGB[ind_i][3*ind_j+1] = rgb[1];
					TheRGB[ind_i][3*ind_j+2] = rgb[2];

					TheMask[ind_i][ind_j] = _Mask[i][j];
				//}
				//} else if (/*mask_val > mask_prev*/TheMask[ind_i][ind_j] < unsigned short(1000) && abs(int(scal) - int(TheBump[ind_i][3*ind_j+2])) < 60)  {
				//	TheBump[ind_i][3*ind_j] = shift[0];
				//	TheBump[ind_i][3*ind_j+1] = shift[1];
				//	TheBump[ind_i][3*ind_j+2] = scal;

				//	TheRGB[ind_i][3*ind_j] = rgb[0];
				//	TheRGB[ind_i][3*ind_j+1] = rgb[1];
				//	TheRGB[ind_i][3*ind_j+2] = rgb[2];

				//	TheMask[ind_i][ind_j] = _Mask[i][j];

				//	TheBump[ind_i][3*ind_j] = unsigned short( (float(TheBump[ind_i][3*ind_j])*float(mask_prev) + float(shift[0])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
				//	TheBump[ind_i][3*ind_j+1] = unsigned short( (float(TheBump[ind_i][3*ind_j+1])*float(mask_prev) + float(shift[1])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
				//	TheBump[ind_i][3*ind_j+2] = unsigned short( (float(TheBump[ind_i][3*ind_j+2])*float(mask_prev) + float(scal)*float(mask_val))/(float(mask_prev) + float(mask_val)) );

				//	//if (TheRGB[3*(ind_i*NewSize[1] + ind_j)] < 250 || TheRGB[3*(ind_i*NewSize[1] + ind_j)+1] > 5 || TheRGB[3*(ind_i*NewSize[1] + ind_j)+2] > 5) {
				//		/*if (rgb[0] > 250 && rgb[1] < 5 && rgb[2] < 5) {
				//			TheRGB[ind_i][3*ind_j] = 255;
				//			TheRGB[ind_i][3*ind_j+1] = 0;
				//			TheRGB[ind_i][3*ind_j+2] = 0;
				//		} else {
				//			TheRGB[ind_i][3*ind_j] = unsigned char( (float(TheRGB[ind_i][3*ind_j])*float(mask_prev) + float(rgb[0])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
				//			TheRGB[ind_i][3*ind_j+1] = unsigned char( (float(TheRGB[ind_i][3*ind_j+1])*float(mask_prev) + float(rgb[1])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
				//			TheRGB[ind_i][3*ind_j+2] = unsigned char( (float(TheRGB[ind_i][3*ind_j+2])*float(mask_prev) + float(rgb[2])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
				//		}*/
				//	//}

				//	TheMask[ind_i][ind_j] = unsigned short( min(1000/*65535*/, 10 + mask_prev + mask_val) );
				//}
			}
		}
	}

	// Project Prim_in primitive onto new images
	for (int i = 0; i < Prim_in->_Size[0]; i++) {
		for (int j = 0; j < Prim_in->_Size[1]; j++) {
			if (Prim_in->_Mask[i][j] > MASKTHRESH) {
				float Shift_ind[2];
				Shift_ind[0] = float(Prim_in->_Bump[i][3*j])/60000.0;
				Shift_ind[1] = float(Prim_in->_Bump[i][3*j+1])/60000.0;

				float x, y, d;
				x = (float(i)+Shift_ind[0])*2.0/Prim_in->_scale[0] + Prim_in->_Shift[0];
				y = (float(j)+Shift_ind[1])*2.0/Prim_in->_scale[1] + Prim_in->_Shift[1];

				d = Prim_in->_dist + ((float(Prim_in->_Bump[i][3*j+2])/2000.0)-15.0); 

				double point [3];
				point[0] = double (x*Prim_in->_e1[0] + y*Prim_in->_e2[0] + d*Prim_in->_nmle[0]);
				point[1] = double (x*Prim_in->_e1[1] + y*Prim_in->_e2[1] + d*Prim_in->_nmle[1]);
				point[2] = double (x*Prim_in->_e1[2] + y*Prim_in->_e2[2] + d*Prim_in->_nmle[2]);
				
				//double point [3];
				//point[0] = double (Prim_in->_VMap[i][3*j]);//Mat_in[0]*pt[0] + Mat_in[4]*pt[1] + Mat_in[8]*pt[2] + Mat_in[12];
				//point[1] = double (Prim_in->_VMap[i][3*j+1]);//Mat_in[1]*pt[0] + Mat_in[5]*pt[1] + Mat_in[9]*pt[2] + Mat_in[13];
				//point[2] = double (Prim_in->_VMap[i][3*j+2]);//Mat_in[2]*pt[0] + Mat_in[6]*pt[1] + Mat_in[10]*pt[2] + Mat_in[14];

				double error_dist = (point[0]*double (_nmle[0]) + point[1]*double (_nmle[1]) + point[2]*double (_nmle[2])) - double (_dist);

				if (fabs(error_dist) > 0.05/*15.0*/) {
					continue;
				}

				double proj [3];
				proj[0] = point[0] - error_dist*double (_nmle[0]);
				proj[1] = point[1] - error_dist*double (_nmle[1]);
				proj[2] = point[2] - error_dist*double (_nmle[2]);
        
				double proj_a, proj_b, a, b;
				proj_a = proj[0]*double (_e1[0]) + proj[1]*double (_e1[1]) + proj[2]*double (_e1[2]);
				proj_b = proj[0]*double (_e2[0]) + proj[1]*double (_e2[1]) + proj[2]*double (_e2[2]); 

				a = proj_a-double (NewCenter[0]); 
				b = proj_b-double (NewCenter[1]); 

				a = (a*double (_scale[0]))/2.0;
				b = (b*double (_scale[1]))/2.0;

				int ind_i = int(a);
				int ind_j = int(b);

				if (ind_i < 0 || ind_i > NewSize[0] - 1 || ind_j < 0 || ind_j > NewSize[1] - 1)
					continue;

				unsigned short shift [2];
				shift[0] = unsigned short((a-double(ind_i))*60000.0);
				shift[1] = unsigned short((b-double(ind_j))*60000.0);
				unsigned short scal = unsigned short(((error_dist+15.0))*2000.0);
				
				unsigned char rgb [3];
				rgb [0] = Prim_in->_RGB[i][3*j];
				rgb [1] = Prim_in->_RGB[i][3*j+1];
				rgb [2] = Prim_in->_RGB[i][3*j+2];

				int mask_val = int(Prim_in->_Mask[i][j])-10;
				int mask_prev = int(TheMask[ind_i][ind_j]) - 10;

				if (mask_prev <= 0) {
					TheBump[ind_i][3*ind_j] = shift[0];
					TheBump[ind_i][3*ind_j+1] = shift[1];
					TheBump[ind_i][3*ind_j+2] = scal;

					TheRGB[ind_i][3*ind_j] = rgb[0];
					TheRGB[ind_i][3*ind_j+1] = rgb[1];
					TheRGB[ind_i][3*ind_j+2] = rgb[2];

					TheMask[ind_i][ind_j] = Prim_in->_Mask[i][j];
				} else if (TheMask[ind_i][ind_j] < unsigned short(1000)/* && abs(int(error_dist) - int(TheBump[ind_i][3*ind_j+2])) < 60*/) {

					TheBump[ind_i][3*ind_j] = unsigned short( (float(TheBump[ind_i][3*ind_j])*float(mask_prev) + float(shift[0])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
					TheBump[ind_i][3*ind_j+1] = unsigned short( (float(TheBump[ind_i][3*ind_j+1])*float(mask_prev) + float(shift[1])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
					TheBump[ind_i][3*ind_j+2] = unsigned short( (float(TheBump[ind_i][3*ind_j+2])*float(mask_prev) + float(scal)*float(mask_val))/(float(mask_prev) + float(mask_val)) );

					TheRGB[ind_i][3*ind_j] = unsigned char( (float(TheRGB[ind_i][3*ind_j])*float(mask_prev) + float(rgb[0])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
					TheRGB[ind_i][3*ind_j+1] = unsigned char( (float(TheRGB[ind_i][3*ind_j+1])*float(mask_prev) + float(rgb[1])*float(mask_val))/(float(mask_prev) + float(mask_val)) );
					TheRGB[ind_i][3*ind_j+2] = unsigned char( (float(TheRGB[ind_i][3*ind_j+2])*float(mask_prev) + float(rgb[2])*float(mask_val))/(float(mask_prev) + float(mask_val)) );

					TheMask[ind_i][ind_j] = unsigned short( min(1000, 10 + mask_prev + mask_val) );
				}
			}
		}
	}
	
	for (int i = 0; i < _Size[0]; i++) {
		free(_Bump[i]);
		free(_RGB[i]);
		free(_Mask[i]);
	}
	free(_Bump);
	free(_RGB);
	free(_Mask);
	
	_Shift [0] = NewCenter[0];
	_Shift [1] = NewCenter[1];

	_Size[0] = NewSize [0];
	_Size[1] = NewSize [1];		

	_Bump = TheBump;
	_RGB = TheRGB;
	_Mask = TheMask;
}

void PlanarPatch::ComputeVMAPNMAP(float **VMap, float **NMap) {
	int n = _Size[0];
	int m = _Size[1];

		
	/*********** Compute vertices ******************/
	#pragma omp parallel num_threads(NUM_THREADS)
	{
		#pragma omp for
		for (int i = 0; i < n; i++) {
			float pt [3];
			float pt_t [3];
			float Shift_ind[2];
			float x, y, d;
			for (int j = 0; j < m; j++) {
				if (_Mask[i][j] <= unsigned short(10)) {
					VMap[i][3*j]= 0.0;
					VMap[i][3*j+1] = 0.0;
					VMap[i][3*j+2] = 0.0;
					continue;
				}

				Shift_ind[0] = float(_Bump[i][3*j])/60000.0;
				Shift_ind[1] = float(_Bump[i][3*j+1])/60000.0;

				x = (float(i)+Shift_ind[0])*2.0/_scale[0] + _Shift[0];
				y = (float(j)+Shift_ind[1])*2.0/_scale[1] + _Shift[1];

				d = _dist + ((float(_Bump[i][3*j+2])/2000.0)-15.0);
					
				pt[0] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
				pt[1] = x*_e1[1] + y*_e2[1] + d*_nmle[1];
				pt[2] = x*_e1[2] + y*_e2[2] + d*_nmle[2];
			
				VMap[i][3*j] = pt[0];
				VMap[i][3*j+1] = pt[1];
				VMap[i][3*j+2] = pt[2];
			}
		}
	}

	/*********** Compute normals ******************/

	
	#pragma omp parallel num_threads(NUM_THREADS)
	{
		#pragma omp for
		for (int i = 1; i < n-1; i++) {
			float p1 [3];
			float p2 [3];
			float p3 [3];
			float n_p [3];
			float n_p1 [3];
			float n_p2 [3];
			float n_p3 [3];
			float n_p4 [3];
			float norm_n;
			unsigned short n_tot = 0;
			for (int j = 1; j < m-1; j++) {
				if (VMap[i][3*j] == 0.0 && VMap[i][3*j+1] == 0.0 && VMap[i][3*j+2] == 0.0) {
					NMap[i][3*j] = 0.0;
					NMap[i][3*j+1] = 0.0;
					NMap[i][3*j+2] = 0.0;
					continue;
				}
				int idx_out = i*m+j;
				int idx = i*m+j;
			
				p1[0] = VMap[i][3*j];
				p1[1] = VMap[i][3*j+1];
				p1[2] = VMap[i][3*j+2];

				n_p1[0] = 0.0; n_p1[1] = 0.0; n_p1[2] = 0.0;
				n_p2[0] = 0.0; n_p2[1] = 0.0; n_p2[2] = 0.0;
				n_p3[0] = 0.0; n_p3[1] = 0.0; n_p3[2] = 0.0;
				n_p4[0] = 0.0; n_p4[1] = 0.0; n_p4[2] = 0.0;

				////////////////////////// Triangle 1 /////////////////////////////////

				idx = (i+1)*m + j;
				p2[0] = VMap[i+1][3*j];
				p2[1] = VMap[i+1][3*j+1];
				p2[2] = VMap[i+1][3*j+2];

				idx = i*m + (j+1);
				p3[0] = VMap[i][3*(j+1)];
				p3[1] = VMap[i][3*(j+1)+1];
				p3[2] = VMap[i][3*(j+1)+2];

				if (p2[2] != 0.0 && p3[2] != 0.0) {
					n_p1[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
					n_p1[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
					n_p1[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

					norm_n = (n_p1[0]*n_p1[0] + n_p1[1]*n_p1[1] + n_p1[2]*n_p1[2]);

					if (norm_n != 0.0) {
						//n_p1[0] = n_p1[0] / sqrt(norm_n);
						//n_p1[1] = n_p1[1] / sqrt(norm_n);
						//n_p1[2] = n_p1[2] / sqrt(norm_n);

						n_tot++;
					}
				}

				////////////////////////// Triangle 2 /////////////////////////////////

				idx = i*m + (j+1);
				p2[0] = VMap[i][3*(j+1)];
				p2[1] = VMap[i][3*(j+1)+1];
				p2[2] = VMap[i][3*(j+1)+2];

				idx = (i-1)*m + j;
				p3[0] = VMap[i-1][3*j];
				p3[1] = VMap[i-1][3*j+1];
				p3[2] = VMap[i-1][3*j+2];

				if (p2[2] != 0.0 && p3[2] != 0.0) {
					n_p2[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
					n_p2[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
					n_p2[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

					norm_n = (n_p2[0]*n_p2[0] + n_p2[1]*n_p2[1] + n_p2[2]*n_p2[2]);

					if (norm_n != 0.0) {
						//n_p2[0] = n_p2[0] / sqrt(norm_n);
						//n_p2[1] = n_p2[1] / sqrt(norm_n);
						//n_p2[2] = n_p2[2] / sqrt(norm_n);

						n_tot++;
					}
				}

				////////////////////////// Triangle 3 /////////////////////////////////

				idx = (i-1)*m + j;
				p2[0] = VMap[i-1][3*j];
				p2[1] = VMap[i-1][3*j+1];
				p2[2] = VMap[i-1][3*j+2];

				idx = i*m + (j-1);
				p3[0] = VMap[i][3*(j-1)];
				p3[1] = VMap[i][3*(j-1)+1];
				p3[2] = VMap[i][3*(j-1)+2];

				if (p2[2] != 0.0 && p3[2] != 0.0) {
					n_p3[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
					n_p3[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
					n_p3[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

					norm_n = (n_p3[0]*n_p3[0] + n_p3[1]*n_p3[1] + n_p3[2]*n_p3[2]);

					if (norm_n != 0) {
						//n_p3[0] = n_p3[0] / sqrt(norm_n);
						//n_p3[1] = n_p3[1] / sqrt(norm_n);
						//n_p3[2] = n_p3[2] / sqrt(norm_n);

						n_tot++;
					}
				}

				////////////////////////// Triangle 4 /////////////////////////////////

				idx = i*m+ (j-1);
				p2[0] = VMap[i][3*(j-1)];
				p2[1] = VMap[i][3*(j-1)+1];
				p2[2] = VMap[i][3*(j-1)+2];

				idx = (i+1)*m + j;
				p3[0] = VMap[i+1][3*j];
				p3[1] = VMap[i+1][3*j+1];
				p3[2] = VMap[i+1][3*j+2];

				if (p2[2] != 0.0 && p3[2] != 0.0) {
					n_p4[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
					n_p4[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
					n_p4[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

					norm_n = (n_p4[0]*n_p4[0] + n_p4[1]*n_p4[1] + n_p4[2]*n_p4[2]);

					if (norm_n != 0) {
						//n_p4[0] = n_p4[0] / sqrt(norm_n);
						//n_p4[1] = n_p4[1] / sqrt(norm_n);
						//n_p4[2] = n_p4[2] / sqrt(norm_n);

						n_tot++;
					}
				}

				if (n_tot == 0) {
					NMap[i][3*j] = 0.0;
					NMap[i][3*j+1] = 0.0;
					NMap[i][3*j+2] = 0.0;
					continue;
				}

				n_p[0] = (n_p1[0] + n_p2[0] + n_p3[0] + n_p4[0]);//float(n_tot);
				n_p[1] = (n_p1[1] + n_p2[1] + n_p3[1] + n_p4[1]);//float(n_tot);
				n_p[2] = (n_p1[2] + n_p2[2] + n_p3[2] + n_p4[2]);//float(n_tot);

				norm_n = sqrt(n_p[0]*n_p[0] + n_p[1]*n_p[1] + n_p[2]*n_p[2]);

				if (norm_n != 0.0) {
					NMap[i][3*j] = n_p[0]/norm_n;
					NMap[i][3*j+1] = n_p[1]/norm_n;
					NMap[i][3*j+2] = n_p[2]/norm_n;
				} else {
					NMap[i][3*j] = 0.0;
					NMap[i][3*j+1] = 0.0;
					NMap[i][3*j+2] = 0.0;
				}
			}
		}
	}
}

int PlanarPatch::RefineCPU() {
	
	int n = _Size[0];
	int m = _Size[1];
	
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

	ComputeVMAPNMAP(VMap, NMap);	
	
	vector<float> Nmles_x;
	vector<float> Nmles_y;
	vector<float> Nmles_z;

	for (int i =0; i < _Size[0]; i ++) {
		for (int j =0; j < _Size[1]; j ++) {
			float nmle [3];
			nmle [0] = NMap[i][3*j];
			nmle [1] = NMap[i][3*j+1];
			nmle [2] = NMap[i][3*j+2];
			
			if (nmle [0] != 0.0 || nmle [1] != 0.0 || nmle [2] != 0.0) {
				Nmles_x.push_back(nmle [0]);
				Nmles_y.push_back(nmle [1]);
				Nmles_z.push_back(nmle [2]);
			}
		}
	}
	
	float pt [3];
	vector<float> Dists;					

	// Compute median normal and distance
	if (Nmles_x.size() < 100) {
		free(VMap);
		VMap = NULL;
		free(NMap);
		NMap = NULL;
		return -1;
	}

	std::sort (Nmles_x.begin(), Nmles_x.end()); 
	std::sort (Nmles_y.begin(), Nmles_y.end()); 
	std::sort (Nmles_z.begin(), Nmles_z.end()); 
		
	_nmle [0] = Nmles_x [Nmles_x.size()/2];
	_nmle [1] = Nmles_y [Nmles_y.size()/2];
	_nmle [2] = Nmles_z [Nmles_z.size()/2];

	float norm_e;
	norm_e = sqrt(_nmle[0]*_nmle[0] + _nmle[1]*_nmle[1] + _nmle[2]*_nmle[2]);
	_nmle [0] = _nmle [0]/norm_e;
	_nmle [1] = _nmle [1]/norm_e;
	_nmle [2] = _nmle [2]/norm_e;

	if (_nmle[0] > _nmle[1] && _nmle[0] > _nmle[2]) {
		//%z vec n
		_e1[0] = -_nmle[1]; _e1[1] = _nmle[0]; _e1[2] = 0.0;
		norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
		_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
	} else if (_nmle[1] > _nmle[0] && _nmle[1] > _nmle[2]) {
		//%x vec n
		_e1[0] = 0.0; _e1[1] = -_nmle[2]; _e1[2] = _nmle[1];
		norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
		_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
	} else {
		//%y vec n
		_e1[0] = _nmle[2]; _e1[1] = 0.0; _e1[2] = -_nmle[0];
		norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
		_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
	}

	//e2 
	_e2[0] = _nmle[1]*_e1[2] - _nmle[2]*_e1[1];
	_e2[1] = _nmle[2]*_e1[0] - _nmle[0]*_e1[2];
	_e2[2] = _nmle[0]*_e1[1] - _nmle[1]*_e1[0];
	
	Nmles_x.clear();
	Nmles_y.clear();
	Nmles_z.clear();
	
	float **proj = (float **) malloc(_Size[0]*sizeof(float *));
	if (proj == NULL)
		perror ("The following error occurred when allocating proj in RefineCPU");

	for (int i = 0; i < _Size[0]; i++) {
		proj[i] = (float *) malloc(3*_Size[1]*sizeof(float));
		if (proj[i] == NULL)
			perror ("The following error occurred when allocating proj[i] in RefineCPU");
	}

	int BBox [4] = {100000, -100000, 100000, -100000};
	float res = RES_PLANE;
	for (int i =0; i < _Size[0]; i ++) {
		for (int j =0; j < _Size[1]; j ++) {
			//float pt [3];
			pt [0] = VMap[i][3*j];
			pt [1] = VMap[i][3*j+1];
			pt [2] = VMap[i][3*j+2];

			float nmle [3];
			nmle [0] = NMap[i][3*j];
			nmle [1] = NMap[i][3*j+1];
			nmle [2] = NMap[i][3*j+2];

			if (nmle [0] == 0.0 && nmle [1] == 0.0 && nmle [2] == 0.0) {
				proj[i][3*j] = 0.0; 
				proj[i][3*j+1] = 0.0; 
				proj[i][3*j+2] = 0.0; 
				VMap[i][3*j] = 0.0;
				VMap[i][3*j+1] = 0.0;
				VMap[i][3*j+2] = 0.0;
				NMap[i][3*j] = 0.0;
				NMap[i][3*j+1] = 0.0;
				NMap[i][3*j+2] = 0.0;
				continue;
			}
								
			float dist_curr = (pt [0]*_nmle [0] + pt [1]*_nmle [1] + pt [2]*_nmle [2]);
				
			pt [0] = pt [0] - dist_curr*_nmle [0];
			pt [1] = pt [1] - dist_curr*_nmle [1];
			pt [2] = pt [2] - dist_curr*_nmle [2];

			proj[i][3*j] = pt[0]*_e1[0] + pt[1]*_e1[1] + pt[2]*_e1[2]; // e1
			proj[i][3*j+1] = pt[0]*_e2[0] + pt[1]*_e2[1] + pt[2]*_e2[2]; // e2

			BBox [0] = min(BBox [0], int(proj[i][3*j]/res)-1);
			BBox [1] = max(BBox [1], int(proj[i][3*j]/res)+1);
			BBox [2] = min(BBox [2], int(proj[i][3*j+1]/res)-1);
			BBox [3] = max(BBox [3], int(proj[i][3*j+1]/res)+1);
						
			float dist_angle = (nmle [0]*_nmle [0] + nmle [1]*_nmle [1] + nmle [2]*_nmle [2]);
			if (dist_angle < 0.8) {				
				proj[i][3*j] = 0.0; 
				proj[i][3*j+1] = 0.0; 
				proj[i][3*j+2] = 0.0; 
				VMap[i][3*j] = 0.0;
				VMap[i][3*j+1] = 0.0;
				VMap[i][3*j+2] = 0.0;
				NMap[i][3*j] = 0.0;
				NMap[i][3*j+1] = 0.0;
				NMap[i][3*j+2] = 0.0;
				continue;
			}
			
			proj[i][3*j+2] = dist_curr;

			Dists.push_back(dist_curr);
		}
	}

	if (Dists.size() < 500) {
		for (int i = 0; i < _Size[0]; i++) {
			free(VMap[i]);
		}
		free(VMap);
		VMap = NULL;

		for (int i = 0; i < _Size[0]; i++) {
			free(NMap[i]);
		}
		free(NMap);
		NMap = NULL;

		for (int i = 0; i < _Size[0]; i++) {
			free(proj[i]);
		}
		free(proj);
		proj = NULL;
		return -1;
	}
	std::sort (Dists.begin(), Dists.end()); 

	_dist = Dists[Dists.size()/2];
	Dists.clear();
	
	if (BBox[0] == 100000) {
		//cout << "Box erase" << endl;
		for (int i = 0; i < _Size[0]; i++) {
			free(VMap[i]);
		}
		free(VMap);
		VMap = NULL;

		for (int i = 0; i < _Size[0]; i++) {
			free(NMap[i]);
		}
		free(NMap);
		NMap = NULL;

		for (int i = 0; i < _Size[0]; i++) {
			free(proj[i]);
		}
		free(proj);
		proj = NULL;

		_Size[0] = 0;
		_Size[1] = 0;
		return -1;
	}

	int new_size [2];
	new_size [0] = BBox[1] - BBox[0];
	new_size [1] = BBox[3] - BBox[2];

	int count_l = 32;

	while (new_size [0]-count_l > 0) {
		count_l += 32;
	}

	int count_m = 32;
	while (new_size [1]-count_m > 0) {
		count_m += 32;
	}

	new_size [0] = count_l;
	new_size [1] = count_m;

	int shift_l = (new_size [0] - (BBox[1] - BBox[0]))/2;
	int shift_m = (new_size [1] - (BBox[3] - BBox[2]))/2;

	_Shift [0] = float(BBox[0] - shift_l)*res;
	_Shift [1] = float(BBox[2] - shift_m)*res;	
	
	for (int i = 0; i < _Size[0]; i++) {
		free(_Bump[i]);
	}
	free(_Bump);
	_Bump = NULL;

	_Bump = (unsigned short **) malloc(new_size[0]*sizeof(unsigned short *));
	if (_Bump == NULL)
		perror ("The following error occurred when allocating _Bump in RefineCPU");

	unsigned char **RGBTmp = (unsigned char **) malloc(new_size[0]*sizeof(unsigned char *));
	if (RGBTmp == NULL)
		perror ("The following error occurred when allocating RGBTmp in RefineCPU");
		
	unsigned short **MaskTmp = (unsigned short **) malloc(new_size[0]*sizeof(unsigned short *));
	if (MaskTmp == NULL)
		perror ("The following error occurred when allocating MaskTmp in RefineCPU");
	
	for (int i = 0; i < new_size[0]; i++) {
		_Bump[i] = (unsigned short *) malloc(3*new_size[1]*sizeof(unsigned short));
		if (_Bump[i] == NULL)
			perror ("The following error occurred when allocating _Bump[i] in RefineCPU");
		RGBTmp[i] = (unsigned char *) malloc(3*new_size[1]*sizeof(unsigned char));
		if (RGBTmp[i] == NULL)
			perror ("The following error occurred when allocating RGBTmp[i] in RefineCPU");
		MaskTmp[i] = (unsigned short *) malloc(new_size[1]*sizeof(unsigned short));
		if (MaskTmp[i] == NULL)
			perror ("The following error occurred when allocating MaskTmp[i] in RefineCPU");
	}
	
	/************************* Init attributes ***************************************/
	for (int i = 0; i < new_size[0]; i ++) {
		for (int j = 0; j < new_size[1]; j ++) {
			_Bump[i][3*j] = 0;
			_Bump[i][3*j+1] = 0;
			_Bump[i][3*j+2] = 0;
			
			RGBTmp[i][3*j] = 0;
			RGBTmp[i][3*j+1] = 0;
			RGBTmp[i][3*j+2] = 0;

			MaskTmp[i][j] = 10;
		}
	}

	/************************* Populate into new images *****************************/

	for (int i = 0; i < _Size[0]; i ++) {
		for (int j = 0; j < _Size[1]; j ++) {

			if (fabs(proj[i][3*j+2]-_dist) > 0.02/*EPSILON*/)
				continue;

			float proj_a = proj[i][3*j];//vcurr_l[0]*param[8] + vcurr_l[1]*param[9] + vcurr_l[2]*param[10]; // e1
			float proj_b = proj[i][3*j+1];//vcurr_l[0]*param[11] + vcurr_l[1]*param[12] + vcurr_l[2]*param[13]; // e2

			proj_a = (proj_a-_Shift[0])*_scale[0]/2.0; //shift[0]
			proj_b = (proj_b-_Shift[1])*_scale[1]/2.0; //Shift[1];

			int ind_i = int(proj_a);
			int ind_j = int(proj_b);

			if (ind_i > new_size[0]-1 || ind_j > new_size[1]-1 || ind_i  < 0 || ind_j  < 0 ) 
				continue;

			int idx_prim = ind_i*new_size[1] + ind_j;
			float shift [2];
			shift[0] = proj_a - float(ind_i);
			shift[1] = proj_b - float(ind_j);

			if (_Mask[i][j] > MaskTmp[ind_i][ind_j]) {
				_Bump[ind_i][3*ind_j] = unsigned short(shift[0]*60000.0);
				_Bump[ind_i][3*ind_j+1] = unsigned short(shift[1]*60000.0);
				_Bump[ind_i][3*ind_j+2] = unsigned short(((proj[i][3*j+2]-_dist)+15.0)*2000.0);
				
				RGBTmp[ind_i][3*ind_j] = _RGB[i][3*j];
				RGBTmp[ind_i][3*ind_j+1] = _RGB[i][3*j+1];
				RGBTmp[ind_i][3*ind_j+2] = _RGB[i][3*j+2];				

				MaskTmp[ind_i][ind_j]= _Mask[i][j];
			}
		}
	}

	for (int i = 0; i < _Size[0]; i++) {
		free(_RGB[i]);
		free(_Mask[i]);
		free(proj[i]);
		free(VMap[i]);
		free(NMap[i]);
	}
	
	free(_RGB);
	free(_Mask);
	free(VMap);
	free(NMap);

	free(proj);
	proj = NULL;
	
	_Size[0] = new_size [0];
	_Size[1] = new_size [1];

	_RGB = RGBTmp;
	_Mask = MaskTmp;

	return 1;
}

bool PlanarPatch::Clean() {

	bool opencv_use = true;
	cv::Mat output;
	/*cv::Mat img_bump;
	cv::Mat img_color;*/
	cv::Mat img_mask;
	cv::Mat element;
	cv::Mat dilation_dst;
    cv::Mat binary;
	std::vector < std::vector<cv::Point2i > > blobs;
	
	/*img_bump = cv::Mat(_Size[0], _Size[1], CV_16UC3);
	img_color = cv::Mat(_Size[0], _Size[1], CV_16UC3);*/
	img_mask = cv::Mat(_Size[0], _Size[1], CV_8UC1);

	int i,j,k;
	for (i=0,k=_Size[0]-1;i<_Size[0];i++,k--) {
		for (j=0;j<_Size[1];j++) {
			img_mask.at<unsigned char>(k,j) = _Mask[i][j] > 10 ? 255 : 0;
		}
	}
		
    //cv::imshow("input", img_mask);

	int erosion_size = 4;
	element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
	

	/// Apply the dilation operation
	cv::erode( img_mask, dilation_dst, element );
	
	/// Apply the erosion operation
	cv::dilate( dilation_dst, img_mask, element );

	//********************** Find connected components ********************************/
	output = cv::Mat::zeros(img_mask.size(), CV_8UC1);

    cv::threshold(img_mask, binary, 0.0, 1.0, cv::THRESH_BINARY);

	FindBlobs(binary, blobs);

	/*********** Get maximal connect component ****************/
	size_t max_indx = -1;
	size_t max_size = 0;
	for(size_t i=0; i < blobs.size(); i++) {
		size_t curr_size = blobs[i].size();
		if (curr_size > max_size) {
			max_size = curr_size;
			max_indx = i;
		}
	}

	if (max_indx == -1)
		return false;

//	if (!final) {
	for(size_t i=0; i < blobs.size(); i++) {
		for(size_t j=0; j < blobs[i].size(); j++) {
			int x = blobs[i][j].x;
			int y = blobs[i][j].y;

			output.at<unsigned char>(y,x) = 255;
		}
	}
	/*} else {
		for (size_t i=0; i < blobs.size(); i++) {
			for(size_t j=0; j < blobs[i].size(); j++) {
				int x = blobs[i][j].x;
				int y = blobs[i][j].y;
				output.at<unsigned char>(y,x) = 255;
			}
		}			
	}*/

   //cv::imshow("binary", img_mask);
    /*cv::imshow("labelled", output);
    cv::waitKey(0);*/

REFINE_PRIM:

	/**************** If not final only remove non-connected points and refine equation *******************/
	//if (!final) {
		if (opencv_use) {
			for (i=0,k=_Size[0]-1;i<_Size[0];i++,k--) {
				for (j=0;j<_Size[1];j++) {
					_Mask[i][j] = _Mask[i][j]*(output.at<unsigned char>(k,j)/255);
				}
			}
		}

		int retval = 1; //RefineCPU();

		if (retval == -1)
			return false;

		//return (retval == 1);
	//}*/


	/**************** Do linear interpolation *****************************/
	//LinearInterpolation();

    return true;
	
}

void PlanarPatch::LinearInterpolation() {
	int i,j,k;

	for (i=0,k=_Size[0]-1;i<_Size[0];i++,k--) {
		for (j=0;j<_Size[1];j++) {

			if(_Mask[i][j] > 100)
				continue;
	
			int size_inter = 1;

BEGIN_INTER:
			int bl = max(0, i-size_inter);
			int br = min(_Size[0]-1, i+size_inter);
			
			int ul = max(0, j-size_inter);
			int ur = min(_Size[1]-1, j+size_inter);

			float bmp [3];
			bmp [0] = bmp [1] = bmp [2] = 0;
			float rgb [3];
			rgb [0] = rgb [1] = rgb [2] = 0;
			int mask = 0;
			float weight = 0.0;

			float pix[2];
			pix[0] = _Mask[i][j] > unsigned short(10) ? float(i) + float(_Bump[i][3*j])/60000.0 : float(i);
			pix[1] = _Mask[i][j] > unsigned short(10) ? float(j) + float(_Bump[i][3*j+1])/60000.0 : float(j);

			float curr_pix[2];
			float val;
			float mask_avg = 0.0;
			float count = 0;
			for (int h = bl; h < br; h++) {
				for (int l = ul; l < ur; l++) {
					if (_Mask[h][l] > unsigned short(11)) {
						curr_pix[0] = float(h) + float(_Bump[h][3*l])/60000.0;
						curr_pix[1] = float(l) + float(_Bump[h][3*l+1])/60000.0;

						val = sqrt((pix[0]-curr_pix[0])*(pix[0]-curr_pix[0]) + (pix[1]-curr_pix[1])*(pix[1]-curr_pix[1]));
						
						mask_avg += float(int(_Mask[h][l])-10)/(1.0+val*val);
						count+= 1.0/(1.0+val*val);

						val = float(int(_Mask[h][l]) - 10)/(1.0+val*val);

						bmp [0] = bmp [0] + val*float(_Bump[h][3*l]);
						bmp [1] = bmp [1] + val*float(_Bump[h][3*l+1]);
						bmp [2] = bmp [2] + val*float(_Bump[h][3*l+2]);
						
						rgb [0] = rgb [0] + val*float(_RGB[h][3*l]);
						rgb [1] = rgb [1] + val*float(_RGB[h][3*l+1]);
						rgb [2] = rgb [2] + val*float(_RGB[h][3*l+2]);

						//mask += int(val*float((_Mask[i*_Size[1] + j])-10));

						weight += val;

					}
				}
			}

			if (weight > 30.0) {
				_Bump[i][3*j] = _Mask[i][j] > unsigned short(10) ? _Bump[i][3*j] : 0;//unsigned short(float(bmp[0])/weight);
				_Bump[i][3*j+1] = _Mask[i][j] > unsigned short(10) ? _Bump[i][3*j+1] : 0;//unsigned short(float(bmp[1])/weight);
				_Bump[i][3*j+2] = unsigned short(bmp[2]/weight);
				
				_RGB[i][3*j] = unsigned char(rgb[0]/weight);
				_RGB[i][3*j+1] = unsigned char(rgb[1]/weight);
				_RGB[i][3*j+2] = unsigned char(rgb[2]/weight);

				_Mask[i][j] =  _Mask[i][j] > unsigned short(10) ? _Mask[i][j] : 11; //unsigned short(mask_avg/count);
			} else {
				size_inter *= 2;
				//cout << "size_inter: " << size_inter <<endl;
				if (size_inter < 10)
					goto BEGIN_INTER;
			}
		}
	}


	cv::Mat img_mask;
	cv::Mat element;
	cv::Mat dilation_dst;
    cv::Mat binary;
	
	img_mask = cv::Mat(_Size[0], _Size[1], CV_8UC1);

	for (i=0,k=_Size[0]-1;i<_Size[0];i++,k--) {
		for (j=0;j<_Size[1];j++) {
			img_mask.at<unsigned char>(k,j) = _Mask[i][j] > 10 ? 255 : 0;
		}
	}

	int erosion_size = 2;
	element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
		
	/// Apply the erosion operation
	cv::erode( img_mask, dilation_dst, element );

	erosion_size = 1;
	element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
	
	/// Apply the dilation operation
	cv::dilate( dilation_dst, img_mask, element );

    cv::threshold(img_mask, binary, 0.0, 1.0, cv::THRESH_BINARY);

	for (i=0,k=_Size[0]-1;i<_Size[0];i++,k--) {
		for (j=0;j<_Size[1];j++) {
			_Mask[i][j] = _Mask[i][j]*binary.at<bool>(k,j);
		}
	}
}

void PlanarPatch::Rubb(int pix_i, int pix_j, int s) {
	float u = float(pix_i)/float(width);
	float v = float(pix_j)/float(height);

	int i = int(u*float(_Size[0]));
	int j = int(v*float(_Size[1]));

	if (i > _Size[0]-1 || j > _Size[1]-1)
		return;
	
	int ll = max(0, i-s);
	int ul = min(_Size[0]-1, i+s);
	
	int lr = max(0, j-s);
	int ur = min(_Size[1]-1, j+s);

	for (int l = ll; l < ul+1; l++) {
		for (int k = lr; k < ur+1; k++) {
			_Mask[l][k] = 0;
			_RGB[l][3*k] = 0;
			_RGB[l][3*k+1] = 0;
			_RGB[l][3*k+2] = 0;
			_Bump[l][3*k] = 0;
			_Bump[l][3*k+1] = 0;
			_Bump[l][3*k+2] = 0;
		}
	}
}

void PlanarPatch::Draw(int pix_i, int pix_j, int s) {

	float u = float(pix_i)/float(width);
	float v = float(pix_j)/float(height);

	int i = int(u*float(_Size[0]));
	int j = int(v*float(_Size[1]));

	cout << i <<endl;
	if (i > _Size[0]-1 || j > _Size[1]-1)
		return;
	
	int ll = max(0, i-s);
	int upl = min(_Size[0]-1, i+s);
	
	int lr = max(0, j-s);
	int upr = min(_Size[1]-1, j+s);

	for (int li = ll; li < upl+1; li++) {
		for (int k = lr; k < upr+1; k++) {

			if(_Mask[li][k] > unsigned short(10))
				continue;
	
			int size_inter = 1;

BEGIN_INTER_BIS:
			int bl = max(0, li-size_inter);
			int br = min(_Size[0]-1, li+size_inter);
			
			int ul = max(0, k-size_inter);
			int ur = min(_Size[1]-1, k+size_inter);

			float bmp [3];
			bmp [0] = bmp [1] = bmp [2] = 0;
			float rgb [3];
			rgb [0] = rgb [1] = rgb [2] = 0;
			int mask = 0;
			float weight = 0.0;

			float pix[2];
			pix[0] = _Mask[li][k] > unsigned short(10) ? float(li) + float(_Bump[li][3*k])/60000.0 : float(li);
			pix[1] = _Mask[li][k] > unsigned short(10) ? float(k) + float(_Bump[li][3*k+1])/60000.0 : float(k);

			float curr_pix[2];
			float val;
			float mask_avg = 0.0;
			float count = 0;
			for (int h = bl; h < br; h++) {
				for (int l = ul; l < ur; l++) {
					if (_Mask[h][l] > unsigned short(10)) {
						curr_pix[0] = float(h) + float(_Bump[h][3*l])/60000.0;
						curr_pix[1] = float(l) + float(_Bump[h][3*l+1])/60000.0;

						val = sqrt((pix[0]-curr_pix[0])*(pix[0]-curr_pix[0]) + (pix[1]-curr_pix[1])*(pix[1]-curr_pix[1]));
						
						mask_avg += float(int(_Mask[h][l])-10)/(1.0+val*val);
						count+= 1.0/(1.0+val*val);

						val = 1.0;//float(int(_Mask[h][l]) - 10)/(1.0+val*val);

						bmp [0] = bmp [0] + val*float(_Bump[h][3*l]);
						bmp [1] = bmp [1] + val*float(_Bump[h][3*l+1]);
						bmp [2] = bmp [2] + val*float(_Bump[h][3*l+2]);
						
						rgb [0] = rgb [0] + val*float(_RGB[h][3*l]);
						rgb [1] = rgb [1] + val*float(_RGB[h][3*l+1]);
						rgb [2] = rgb [2] + val*float(_RGB[h][3*l+2]);

						//mask += int(val*float((_Mask[i*_Size[1] + j])-10));

						weight += val;

					}
				}
			}

			if (weight > 0.0) {
				_Bump[li][3*k] = _Mask[li][j] > unsigned short(10) ? _Bump[li][3*k] : 0;//unsigned short(float(bmp[0])/weight);
				_Bump[li][3*k+1] = _Mask[li][j] > unsigned short(10) ? _Bump[li][3*k+1] : 0;//unsigned short(float(bmp[1])/weight);
				_Bump[li][3*k+2] = unsigned short(bmp[2]/weight);
				
				_RGB[li][3*k] = unsigned char(rgb[0]/weight);
				_RGB[li][3*k+1] = unsigned char(rgb[1]/weight);
				_RGB[li][3*k+2] = unsigned char(rgb[2]/weight);

				_Mask[li][k] =  20;//_Mask[i][j] > unsigned short(10) ? _Mask[i][j] : unsigned short(mask_avg/count);
			} else {
				size_inter *= 2;
				//cout << "size_inter: " << size_inter <<endl;
				if (size_inter < 33/*17*/)
					goto BEGIN_INTER_BIS;
			}
		}
	}

}

void PlanarPatch::Extrude(int pix_i, int pix_j, int s) {

	float u = float(pix_i)/float(width);
	float v = float(pix_j)/float(height);

	int i = int(u*float(_Size[0]));
	int j = int(v*float(_Size[1]));

	cout << i <<endl;
	if (i > _Size[0]-1 || j > _Size[1]-1)
		return;
	
	int ll = max(0, i-s);
	int upl = min(_Size[0]-1, i+s);
	
	int lr = max(0, j-s);
	int upr = min(_Size[1]-1, j+s);

	for (int li = ll; li < upl+1; li++) {
		for (int k = lr; k < upr+1; k++) {

			if(_Mask[li][k] <= unsigned short(10))
				continue;
	
			int size_inter = 1;

			float bump_i = 1.0 - float((li-i)*(li-i))/float(s*s);
			float bump_j = 1.0 - float((k-j)*(k-j))/float(s*s);
			_Bump[li][3*k+2] = _Bump[li][3*k+2] + unsigned short(bump_i*bump_j*5);
		}
	}

}

void PlanarPatch::Dig(int pix_i, int pix_j, int s) {

	float u = float(pix_i)/float(width);
	float v = float(pix_j)/float(height);

	int i = int(u*float(_Size[0]));
	int j = int(v*float(_Size[1]));

	cout << i <<endl;
	if (i > _Size[0]-1 || j > _Size[1]-1)
		return;
	
	int ll = max(0, i-s);
	int upl = min(_Size[0]-1, i+s);
	
	int lr = max(0, j-s);
	int upr = min(_Size[1]-1, j+s);

	for (int li = ll; li < upl+1; li++) {
		for (int k = lr; k < upr+1; k++) {

			if(_Mask[li][k] <= unsigned short(10))
				continue;
	
			int size_inter = 1;

			float bump_i = 1.0 - float((li-i)*(li-i))/float(s*s);
			float bump_j = 1.0 - float((k-j)*(k-j))/float(s*s);
			_Bump[li][3*k+2] = _Bump[li][3*k+2] - unsigned short(bump_i*bump_j*5);
		}
	}

}

void PlanarPatch::UpdateTextures() {

	BYTE * data = (BYTE *) malloc( _n * _m * 4 );
	memset(data,0,_n * _m * 4);
	BYTE *dataPtr;

	dataPtr = data;
	int i,j,k,l;
	for (i=0; i < _n; i++) {
		for (j=0;j<_m;j++) {
			dataPtr[0] = 255; 
			dataPtr[1] = 255; 
			dataPtr[2] = 255; 
			dataPtr[3] = 0;
			dataPtr += 4;
		}
	}

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++)  {
			data[4*(l*_n + k)] = _RGB[k][3*l]; 
			data[4*(l*_n + k)+1] = _RGB[k][3*l+1];
			data[4*(l*_n + k)+2] = _RGB[k][3*l+2];
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	for (i=0, k = _Size[0]-1; i < _Size[0]; i++, k--) {
		for (j=0,l= 0;j<_Size[1];j++, l++) {
			data[4*(l*_n + k)] = 255;
			data[4*(l*_n + k)+1] = 255;
			data[4*(l*_n + k)+2] = 255;
			data[4*(l*_n + k)+3] = _Mask[k][l] > 10 ? 255 : 0;
		}
	}

	glBindTexture( GL_TEXTURE_2D, _textureb );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_NEAREST );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 4, _n, _m,
                   GL_RGBA, GL_UNSIGNED_BYTE, data );

	// free buffer
	free( data );
}