#ifndef __PRIMITIVEKERNEL_H
#define __PRIMITIVEKERNEL_H

#include "cudaTypes.cuh"

///***** Global variable accessor ******////

void AllocPosePrimMatrix();
void SetPosePrimMatrix(float *pose_CPU);
void FreePosePrimMatrix();

void AllocBufPrim();
void FreeBufPrim();

///**** Function definitions ****/
void VertexFromBump(float *VMap_VBO, float *RGB_VBO, float *Mask_VBO, unsigned short *Bump_dev, unsigned char *RGB_bump_dev, unsigned char *Mask_dev, float *param_dev, int n, int m, int lvl);

void QuadTrim(float *VMap, unsigned char *Mask, unsigned int *indices_dev, unsigned short min_conf, unsigned int n, unsigned int m, float thresh, int lvl);

void UpdateBump_cu(unsigned short *Bump_dev, unsigned char *RGB_dev, unsigned char *Mask_dev, float *VMap, float *NMap, float *RGB, float *Mask, 
													float *param, int N_prim, int M_prim, int n, int m);

void Update_cu(unsigned short **BumpTab, unsigned char **RGBTab, unsigned char **MaskTab, float **ParamTab, float *VMap, float *RGB, unsigned char *Mask, 
						unsigned char *Label, float *pose, int N_prim, int M_prim, int n, int m);

void DetectContour_cu(unsigned short *Bump_dev, unsigned char *RGB_dev, unsigned char *Mask_dev, float *VMap, float *intr,
													float *param, int N_prim, int M_prim, int n, int m);

void BBOX_cu(unsigned char *Mask, int *BBox, int n, int m);

unsigned int Count_cu(unsigned char *Mask, int n, int m);

float Overlap_cu(unsigned short *Bump, unsigned char *Mask, float *param, float *param_in, int n, int m, int N_prim, int M_prim);

void SetRGBPrim_cu(unsigned char *RGB, cudaArray* Array, int n, int m);

void ReadVtxMask_cu(float *param, unsigned short *Bump, unsigned char *Mask, cudaArray* Array, int n, int m);


#endif