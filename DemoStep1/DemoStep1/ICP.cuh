#ifndef __ICP_H
#define __ICP_H

#include "cudaTypes.cuh"

void AllocBuffers(int n, int m);
void FreeBuffers();

void EstimateSystemGaussNewton(float *Rcurr_dev, float *tcurr_dev,
					float *VMap, float *NMap, float *RGB, cv::gpu::DevMem2D_<float> Gx, cv::gpu::DevMem2D_<float> Gy,
					float *VMap_prev, float *NMap_prev, float *RGB_prev,
					float *intr, float distThres, float angleThres, int n_row, int m_col, float *A, float *b, int fact);

#endif