#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#pragma once

#include "Normalisation_Cube_Map.h"

#define BUFFER_OFFSET(a) ((char*)NULL + (a))

class PRIM_VERTEX
{
public:
	VECTOR3D position;
	float s, t;
	float weight;
	VECTOR3D sTangent, tTangent;
	VECTOR3D normal;
	VECTOR3D tangentSpaceLight;
};

class Primitive {
protected:

	unsigned short **_Bump;
	unsigned char **_RGB;
	unsigned short **_Mask;

	//********** Parameters of the primitive ******************/
	//float *_param_dev;
		
	int _lvl;
	int _lvl_data;

	// Pointers to data on device
	/*unsigned short *_Bump_dev;
	unsigned char *_Mask_dev;
	unsigned char *_RGBC_dev;

	float * _RGB_dev;
	float * _VMap_dev;
	float * _NMap_dev;
	unsigned int *_Index_dev;

	GLuint _frame_buf;
	GLuint _index_buf;
	cudaGraphicsResource_t _Resources_t[2];
	size_t _buf_size;*/

	GLuint _texture;
	GLuint _textureb;
	//Normal map
	GLuint _normalMap;
	//Normalisation cube map
	GLuint _normalisationCubeMap;

	int _n, _m; // dimensions of the Primitive

	// Size	
	int _Size[2];

	bool _view;
	int _viewIdx;
	
	int _idDt;
	int _idVBO;

public:

	/* Empty constructor */
	Primitive(int n, int m): _n(n), _m(m) {
		_Bump = NULL;
		_RGB = NULL;
		_Mask = NULL;
		_view = true;
		
		// allocate a texture name
		glGenTextures( 1, &_texture );
		glGenTextures( 1, &_textureb );
		glGenTextures( 1, &_normalMap );
		glGenTextures( 1, &_normalisationCubeMap );
	};
	
	/* Empty destructor */
	virtual ~Primitive() {
		glDeleteTextures( 1, &_texture );
		glDeleteTextures( 1, &_textureb );
		glDeleteTextures( 1, &_normalMap );
		glDeleteTextures( 1, &_normalisationCubeMap );
	};

	inline int getN() {return _n;}
	inline int getM() {return _m;} 
	inline int getSizeN() {return _Size[0];}
	inline int getSizeM() {return _Size[1];} 
	inline int getlvlData() {return _lvl_data;} 
	inline int getlvl() {return _lvl;} 
	
	inline unsigned short ** getBump() { return _Bump;}
	inline unsigned short ** getMask() { return _Mask;}

	inline bool IsView() {return _view;}

	inline void setNM(int n, int m, int lvl) {_n = n; _m = m; _lvl_data = lvl;}
	inline void SetView(bool view) {_view = view;}
	inline void setSize(int n, int m) {_Size[0] = n; _Size[1] = m;};
	

	inline int getidDt() {return _idDt;}
	inline int getidVBO() {return _idVBO;}
	inline void setIdx(int idDt, int idVBO) {_idDt = idDt; _idVBO = idVBO;}
	inline int getViewId() {return _viewIdx;}
	inline void setViewId(int viewId) {_viewIdx = viewId;}


	void Draw(bool color = true);

	virtual void DrawMesh(bool color = true);

	virtual void Load(char *filename);
	
};

class PlanarPatch : public Primitive {
private:
	// Normal vector
	float _nmle [3];
	// Basis vector 1
	float _e1 [3];
	// Basis vector 2
	float _e2 [3];
	// Distance to origine
	float _dist;
	// Scaling factors
	float _scale[2];
	int _prev_Size[2];
	// Center point shift relative to the origine projection
	float _Shift[2];
	// shifting position of subwindow inside big window
	int _Slide[2];
	// Anchor point
	float _anchor[3];

	char _filename[100];
	
	vector <Point2D> _ControlsIdx;
	vector <Point3D> _Controls;
	Delaunay _dt;
	Triangulation T;

	int _numVertices;
	int _numIndices;

	unsigned int * _indices;
	PRIM_VERTEX * _vertices;
	
	int _indx_g2o;

public:
	
	float _colorPrim[3];
	
	// constructor
	PlanarPatch(int n, int m): Primitive(n,m) { 
		_lvl = 2;
		_lvl_data = 0;
	};

	PlanarPatch(float *nmle, float *e1, float *e2, float dist, float *scale, int *Size, float *Shift, float *anchor): Primitive(1,1) {
		_nmle[0] = nmle[0]; _nmle[1] = nmle[1]; _nmle[2] = nmle[2];
		_e1[0] = e1[0]; _e1[1] = e1[1]; _e1[2] = e1[2];
		_e2[0] = e2[0]; _e2[1] = e2[1]; _e2[2] = e2[2];
		_dist = dist;
		_scale[0] = scale[0]; _scale[1] = scale[1];
		_Size[0] = Size[0]; _Size[1] = Size[1];
		_prev_Size[0] = Size[0]; _prev_Size[1] = Size[1];
		_Shift[0] = Shift[0]; _Shift[1] = Shift[1];
		_anchor[0] = anchor[0]; anchor[1] = anchor[1]; anchor[2] = anchor[2];
		
		_lvl = 2;
		_lvl_data = 0;

		_Bump = NULL;
		_RGB = NULL;
		_Mask = NULL;		
	};
	
	PlanarPatch(int n, int m, char *filename): Primitive(n,m) { 
		_lvl = 2;
		_lvl_data = 0;
		sprintf(_filename, "%s", filename);
	};

	// destructor
	virtual ~PlanarPatch() { 	
		if (_Bump != NULL) {
				for (int i = 0; i < _Size[0]; i++) {
					if (_Bump[i] != NULL)
						free(_Bump[i]);
					if (_RGB[i] != NULL)
						free(_RGB[i]);
					if (_Mask[i] != NULL)
						free(_Mask[i]);
				}
			if (_Bump != NULL)
			free(_Bump);
			if (_RGB != NULL)
			free(_RGB);
			if (_Mask != NULL)
			free(_Mask);

			_Bump = NULL;
			_RGB = NULL;
			_Mask = NULL;
		}

		if(_indices)
			delete [] _indices;
		_indices=NULL;

		if(_vertices)
			delete [] _vertices;
		_vertices=NULL;
	};
	
	void DrawMeshBump(VECTOR3D objectLightPosition, bool light, bool color = true, bool drawBumps = true);

	virtual void DrawMesh(bool color = true);
		
	virtual void Load(char *filename);
	
	double *getQuaternion();

	void SetFromQuaternion(double *v);
	
	void DrawIdx(int indx);

	void DrawMeshFlat();

	void LoadEquation(char *filename);
	
	bool IsVisible(float x, float y, float z, float lx, float ly, float lz);

	void Allocate(unsigned short **Bump, unsigned char **RGB, unsigned short **Mask) {
		_Bump = Bump;
		_RGB = RGB;
		_Mask = Mask;
	}

	void ComputeTriangulation();
	
	void ComputeTriangulationLight();

	void Merge(PlanarPatch *Prim_in, float *NewCenter, int *NewSize);

	void ComputeVMAPNMAP(float **VMap, float **NMap);

	int RefineCPU();

	bool Clean();

	void LinearInterpolation();
	
	vector<PlanarPatch *> Segment();

	void set_Slide_Shift(int s_i, int s_j);
	
	void Rubb(int pix_i, int pix_j, int s = 2);

	void Draw(int pix_i, int pix_j, int s = 2);
	
	void Extrude(int pix_i, int pix_j, int s = 2);
	
	void Dig(int pix_i, int pix_j, int s = 2);
	
	void UpdateTextures();

	bool is_in_triangle_b(int i, int k, int l);
	
	void Transform(Eigen::MatrixXf ThePose);


	//////////////////////////////////////////////////////////////////////////////////////////////
	inline float *getCenter(float w, float h) {
		float *res = new float [3];
		res[0] = _dist*_nmle[0] + (_Shift[0]+w*float(2.0*_Size[0])/_scale[0])*_e1[0] + (_Shift[1]+h*float(2.0*_Size[1])/_scale[1])*_e2[0];
		res[1] = _dist*_nmle[1] + (_Shift[0]+w*float(2.0*_Size[0])/_scale[0])*_e1[1] + (_Shift[1]+h*float(2.0*_Size[1])/_scale[1])*_e2[1];
		res[2] = _dist*_nmle[2] + (_Shift[0]+w*float(2.0*_Size[0])/_scale[0])*_e1[2] + (_Shift[1]+h*float(2.0*_Size[1])/_scale[1])*_e2[2];
		return res;
	}

	inline float *getNmle() {return _nmle;}
	inline float *getE1() {return _e1;}
	inline float *getE2() {return _e2;}
	inline float *getShift() {return _Shift;}
	inline float *getScale() {return _scale;}
	inline float getDist() {return _dist;}
	inline char *getFilename() {return _filename;}
	inline int *getSize() {return _Size;}
	inline int getIndx() {return _indx_g2o;}
	inline void setIndx(int indx) {_indx_g2o = indx;}
	
	void setNmle(float *nmle) {
		_nmle[0] = nmle[0];
		_nmle[1] = nmle[1];
		_nmle[2] = nmle[2];

		float norm_e;
		if (nmle[0] > nmle[1] && nmle[0] > nmle[2]) {
			//%z vec n
			_e1[0] = -nmle[1]; _e1[1] = nmle[0]; _e1[2] = 0.0;
			norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
			_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
		} else if (nmle[1] > nmle[0] && nmle[1] > nmle[2]) {
			//%x vec n
			_e1[0] = 0.0; _e1[1] = -nmle[2]; _e1[2] = nmle[1];
			norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
			_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
		} else {
			//%y vec n
			_e1[0] = nmle[2]; _e1[1] = 0.0; _e1[2] = -nmle[0];
			norm_e = sqrt(_e1[0]*_e1[0] + _e1[1]*_e1[1] + _e1[2]*_e1[2]);
			_e1[0] /= norm_e; _e1[1] /= norm_e; _e1[2] /= norm_e;
		}

		//e2 
		_e2[0] = nmle[1]*_e1[2] - nmle[2]*_e1[1];
		_e2[1] = nmle[2]*_e1[0] - nmle[0]*_e1[2];
		_e2[2] = nmle[0]*_e1[1] - nmle[1]*_e1[0];
	}
	inline void setDist(float dist) {_dist = dist;}
	inline void setScale(float *scale) {
		_scale[0] = scale[0];
		_scale[1] = scale[1];
	}
	inline void setEquation(PlanarPatch *Prim) {
		float *tmp = Prim->getNmle();
		_nmle[0] = tmp[0]; _nmle[1] = tmp[1]; _nmle[2] = tmp[2];
		tmp = Prim->getE1();
		_e1[0] = tmp[0]; _e1[1] = tmp[1]; _e1[2] = tmp[2];
		tmp = Prim->getE2();
		_e2[0] = tmp[0]; _e2[1] = tmp[1]; _e2[2] = tmp[2];
		_dist = Prim->getDist();
		tmp = Prim->getShift();
		_Shift[0] = tmp[0]; _Shift[1] = tmp[1];
		tmp = Prim->getScale();
		_scale[0] = tmp[0]; _scale[1] = tmp[1]; 

		int *prev_Size = Prim->getSize();
		_prev_Size[0] = prev_Size[0]; _prev_Size[1] = prev_Size[1];
		
		sprintf(_filename, "%s", Prim->getFilename());
	}

	inline float *getCorners() {
		float *res = new float [12];
		float x, y, d;
		x = _Shift[0];
		y = _Shift[1];

		d = _dist;

		res[0] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[1]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[2]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0] + (float(_Size[0]))*2.0/_scale[0];
		y = _Shift[1];
		res[3] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[4]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[5]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0] + (float(_Size[0]))*2.0/_scale[0];
		y = _Shift[1] + (float(_Size[1]))*2.0/_scale[1];
		res[6] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[7]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[8]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];
		
		x = _Shift[0];//*2.0/_scale[0];
		y = _Shift[1] + (float(_Size[1]))*2.0/_scale[1];
		res[9] = x*_e1[0] + y*_e2[0] + d*_nmle[0];
		res[10]  = x*_e1[1] + y*_e2[1] + d*_nmle[1];
		res[11]  = x*_e1[2] + y*_e2[2] + d*_nmle[2];

		return res;
	}	

};


#endif