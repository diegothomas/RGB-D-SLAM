#include "ICP.cuh"


/////**************Global Variables ********/////////////////////////
float *buf_dev; 
float *outbuf_dev;
float *buf;

void AllocBuffers(int n, int m) {
	checkCudaErrors( cudaMalloc((void**)&buf_dev, 27*n*m*sizeof(float) * sizeof(float)) );
	checkCudaErrors( cudaMalloc((void **) &outbuf_dev, 27*sizeof(float)) );
	buf = (float *) malloc (27*sizeof(float));
}

void FreeBuffers() {
	checkCudaErrors( cudaFree(buf_dev) );
	checkCudaErrors( cudaFree(outbuf_dev) );
	free(buf);
}

//*************** Define Cuda Kernels ***********//////////////////

static __device__ __forceinline__ int flattenedThreadId()
{
return threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
}

template<int CTA_SIZE>
static __device__ __forceinline__ void reduce(volatile float* buffer)
{
	int tid = flattenedThreadId();
	float val =  buffer[tid];

	if (CTA_SIZE >= 1024) { if (tid < 512) buffer[tid] = val = val + buffer[tid + 512]; __syncthreads(); }
	if (CTA_SIZE >=  512) { if (tid < 256) buffer[tid] = val = val + buffer[tid + 256]; __syncthreads(); }
	if (CTA_SIZE >=  256) { if (tid < 128) buffer[tid] = val = val + buffer[tid + 128]; __syncthreads(); }
	if (CTA_SIZE >=  128) { if (tid <  64) buffer[tid] = val = val + buffer[tid +  64]; __syncthreads(); }

	if (tid < 32)
	{
		if (CTA_SIZE >=   64) { buffer[tid] = val = val + buffer[tid +  32]; }
		if (CTA_SIZE >=   32) { buffer[tid] = val = val + buffer[tid +  16]; }
		if (CTA_SIZE >=   16) { buffer[tid] = val = val + buffer[tid +   8]; }
		if (CTA_SIZE >=    8) { buffer[tid] = val = val + buffer[tid +   4]; }
		if (CTA_SIZE >=    4) { buffer[tid] = val = val + buffer[tid +   2]; }
		if (CTA_SIZE >=    2) { buffer[tid] = val = val + buffer[tid +   1]; }
	}
}

__global__ void ReduceKernel (float *buf, float *output, int length) {
	float *beg = &buf[blockIdx.x*length];
    float *end = beg + length;

    int tid = threadIdx.x;

    float sum = 0.0;
    for (float *t = beg + tid; t < end; t += STRIDE)
        sum += *t;

    __shared__ float smem[STRIDE];

    smem[tid] = sum;
    __syncthreads ();

	reduce<STRIDE>(smem);
			
	if (tid == 0) {			
		output[blockIdx.x] = smem[0];
	}
}

__device__ __forceinline__ bool searchGauss (int indx, float *intr, float *Rcurr, float *tcurr, float *VMap, float *NMap, float *RGB, cv::gpu::DevMem2D_<float> Gx, cv::gpu::DevMem2D_<float> Gy,
										float *VMap_prev, float *NMap_prev, float *RGB_prev, float distThres, float angleThres, int n_row, int m_col, float *n, float *d, float *s, float *rgb_val) {
		// Rcurr and Tcurr should be the transformation that align VMap to VMap_prev

        float ncurr[3];
		float nprev_cp[3];
		float nprev[3];
		float ncross[3];
		float vcurr[3];	
		float vprev[3];
		float vprev_cp[3];
		int p_indx[2];	
		float intsy_curr;
		float intsy_prev;

		nprev[0] = NMap_prev[3*indx];
		nprev[1] = NMap_prev[3*indx+1];
		nprev[2] = NMap_prev[3*indx+2];

		if (nprev[0] == 0.0 && nprev[1] == 0.0 && nprev[2] == 0.0)
			return false;

		vprev[0] = VMap_prev[3*indx];
		vprev[1] = VMap_prev[3*indx+1];
		vprev[2] = VMap_prev[3*indx+2];
				
		vprev_cp[0] = Rcurr[0]*vprev[0] + Rcurr[3]*vprev[1] + Rcurr[6]*vprev[2] + tcurr[0]; //Rcurr is row major
		vprev_cp[1] = Rcurr[1]*vprev[0] + Rcurr[4]*vprev[1] + Rcurr[7]*vprev[2] + tcurr[1];
		vprev_cp[2] = Rcurr[2]*vprev[0] + Rcurr[5]*vprev[1] + Rcurr[8]*vprev[2] + tcurr[2];

		nprev_cp[0] = Rcurr[0]*nprev[0] + Rcurr[3]*nprev[1] +Rcurr[6]*nprev[2]; //Rcurr is row major
		nprev_cp[1] = Rcurr[1]*nprev[0] + Rcurr[4]*nprev[1] +Rcurr[7]*nprev[2];
		nprev_cp[2] = Rcurr[2]*nprev[0] + Rcurr[5]*nprev[1] +Rcurr[8]*nprev[2];

		intsy_prev = (RGB_prev[4*indx]+RGB_prev[4*indx+1]+RGB_prev[4*indx+2])/3.0;
			
		p_indx[0] = min(m_col-1, max(0, __float2int_rn((vprev_cp[0]/fabs(vprev_cp[2]))*intr[0] + intr[2]))); 
		p_indx[1] = min(n_row-1, max(0, __float2int_rn((vprev_cp[1]/fabs(vprev_cp[2]))*intr[1] + intr[3]))); 

		int indx_proj = 3*(p_indx[1]*m_col + p_indx[0]);
		ncurr[0] = NMap[indx_proj];
		ncurr[1] = NMap[indx_proj+1];
		ncurr[2] = NMap[indx_proj+2];
		
		if (ncurr[0] == 0.0 && ncurr[1] == 0.0 && ncurr[2] == 0.0)
			return false;
								
		vcurr[0] = VMap[indx_proj];
		vcurr[1] = VMap[indx_proj+1];
		vcurr[2] = VMap[indx_proj+2];
		
		float dist = sqrt((vprev_cp[0]-vcurr[0])*(vprev_cp[0]-vcurr[0]) + (vprev_cp[1]-vcurr[1])*(vprev_cp[1]-vcurr[1]) + (vprev_cp[2]-vcurr[2])*(vprev_cp[2]-vcurr[2]));
		if (dist > distThres)
			return false;
	
		ncross[0] = ncurr[1]*nprev_cp[2] - ncurr[2]*nprev_cp[1];
		ncross[1] = -ncurr[0]*nprev_cp[2] + ncurr[2]*nprev_cp[0];
		ncross[2] = ncurr[0]*nprev_cp[1] - ncurr[1]*nprev_cp[0];
		
		float angle = sqrt(ncross[0]*ncross[0] + ncross[1]*ncross[1] +ncross[2]*ncross[2]);
		if (angle > angleThres)
			return false;
		
		intsy_curr = (RGB[3*(p_indx[1]*m_col + p_indx[0])]+RGB[3*(p_indx[1]*m_col + p_indx[0])+1]+RGB[3*(p_indx[1]*m_col + p_indx[0])+2])/3.0;

		n[0] = nprev_cp[0]; n[1] = nprev_cp[1]; n[2] = nprev_cp[2];
		d[0] = vprev_cp[0]; d[1] = vprev_cp[1]; d[2] = vprev_cp[2];
		s[0] = vcurr[0]; s[1] = vcurr[1]; s[2] = vcurr[2]; s[3] = -vcurr[2];
		rgb_val[0] = Gx(p_indx[1], p_indx[0])/8.0;
		rgb_val[1] = Gy(p_indx[1], p_indx[0])/8.0; 
		rgb_val[2] = (intsy_curr - intsy_prev);

		return true; //(dist < distThres && angle < angleThres && (ncurr[0] != 0.0 || ncurr[1] != 0.0 || ncurr[2] != 0.0) && vprev_cp[2] < 0.0 && (nprev[0] != 0.0 || nprev[1] != 0.0 || nprev[2] != 0.0));

}

__device__ __forceinline__ void ProcessMatchKernelGaussNewto (float *intr, float *Rcurr, float *tcurr, float *VMap, float *NMap, float *RGB, cv::gpu::DevMem2D_<float> Gx, cv::gpu::DevMem2D_<float> Gy,
										float *VMap_prev, float *NMap_prev, float *RGB_prev, float distThres, float angleThres, int n_row, int m_col, float *buf, int fact) {

	 // identifiant de thread ? deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_L_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_L_Y;
    int idx = (i*fact)*m_col + (j*fact);

	float n[3], d[3], s[4], rgb_val[3];
    bool found_coresp = false;
	float weight = 1.0;
	float lambda = 0.03;

    if ((i*fact) < n_row && (j*fact) < m_col)
        found_coresp = searchGauss (idx, intr, Rcurr, tcurr, VMap, NMap, RGB, Gx, Gy, VMap_prev, NMap_prev, RGB_prev, distThres, angleThres, n_row, m_col, n, d, s, rgb_val);

	float row[7];
	row[0] = row[1] = row[2] = row[3] = row[4] = row[5] = row[6] = 0.0;
	float row_rgb[7];
	row_rgb[0] = row_rgb[1] = row_rgb[2] = row_rgb[3] = row_rgb[4] = row_rgb[5] = row_rgb[6] = 0.0;
	float JD[18];
	float JRot[18];
	float JProj[6];

	// row [0 -> 5] = A^t = [skew(s) | Id(3,3)]^t*n
    if (found_coresp)
    {
		weight = 0.0012/(0.0012 + 0.0019*(s[3]-0.4)*(s[3]-0.4));

		JD[0] = 1.0; JD[3] = 0.0; JD[6] = 0.0;	JD[9] = 0.0;		JD[12] = 2.0*d[2];	JD[15] = -2.0*d[1]; 
		JD[1] = 0.0; JD[4] = 1.0; JD[7] = 0.0;	JD[10] = -2.0*d[2]; JD[13] = 0.0;		JD[16] = 2.0*d[0]; 
		JD[2] = 0.0; JD[5] = 0.0; JD[8] = 1.0;	JD[11] = 2.0*d[1];	JD[14] = -2.0*d[0]; JD[17] = 0.0; 
				
		JRot[0] = 0.0; JRot[3] = 0.0; JRot[6] = 0.0;	JRot[9] = 0.0;			JRot[12] = 2.0*n[2];	JRot[15] = -2.0*n[1]; 
		JRot[1] = 0.0; JRot[4] = 0.0; JRot[7] = 0.0;	JRot[10] = -2.0*n[2];	JRot[13] = 0.0;			JRot[16] = 2.0*n[0]; 
		JRot[2] = 0.0; JRot[5] = 0.0; JRot[8] = 0.0;	JRot[11] = 2.0*n[1];	JRot[14] = -2.0*n[0];	JRot[17] = 0.0; 

		row[0] = weight*(-(n[0]*JD[0] + n[1]*JD[1] + n[2]*JD[2]) + JRot[0]*(s[0]-d[0]) + JRot[1]*(s[1]-d[1]) + JRot[2]*(s[2]-d[2]));
		row[1] = weight*(-(n[0]*JD[3] + n[1]*JD[4] + n[2]*JD[5]) + JRot[3]*(s[0]-d[0]) + JRot[4]*(s[1]-d[1]) + JRot[5]*(s[2]-d[2]));
		row[2] = weight*(-(n[0]*JD[6] + n[1]*JD[7] + n[2]*JD[8]) + JRot[6]*(s[0]-d[0]) + JRot[7]*(s[1]-d[1]) + JRot[8]*(s[2]-d[2]));
		row[3] = weight*(-(n[0]*JD[9] + n[1]*JD[10] + n[2]*JD[11]) + JRot[9]*(s[0]-d[0]) + JRot[10]*(s[1]-d[1]) + JRot[11]*(s[2]-d[2]));
		row[4] = weight*(-(n[0]*JD[12] + n[1]*JD[13] + n[2]*JD[14]) + JRot[12]*(s[0]-d[0]) + JRot[13]*(s[1]-d[1]) + JRot[14]*(s[2]-d[2]));
		row[5] = weight*(-(n[0]*JD[15] + n[1]*JD[16] + n[2]*JD[17]) + JRot[15]*(s[0]-d[0]) + JRot[16]*(s[1]-d[1]) + JRot[17]*(s[2]-d[2]));

		row[6] = -weight*(n[0]*(s[0]-d[0]) + n[1]*(s[1]-d[1]) + n[2]*(s[2]-d[2]));
		
		JProj[0] = intr[0]/fabs(d[2]);	JProj[2] = 0.0;					JProj[4] = -d[0]*intr[0]/(d[2]*d[2]);	
		JProj[1] = 0.0;					JProj[3] = intr[1]/fabs(d[2]);	JProj[5] = -d[1]*intr[1]/(d[2]*d[2]);

		//////////////////////
		row_rgb[0] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[0] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[1] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[2]);
		row_rgb[1] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[3] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[4] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[5]);
		row_rgb[2] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[6] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[7] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[8]);
		row_rgb[3] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[9] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[10] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[11]);
		row_rgb[4] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[12] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[13] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[14]);
		row_rgb[5] = lambda*weight*((rgb_val[0]*JProj[0] + rgb_val[1]*JProj[1])*JD[15] + (rgb_val[0]*JProj[2] + rgb_val[1]*JProj[3])*JD[16] + (rgb_val[0]*JProj[4] + rgb_val[1]*JProj[5])*JD[17]);

		row_rgb[6] = -lambda*weight*rgb_val[2];

		//row[0] = s[1]*n[2] - s[2]*n[1]; 
		//row[1] = -s[0]*n[2] + s[2]*n[0];
		//row[2] = s[0]*n[1] - s[1]*n[0];
		//
		//row[3] = n[0];
		//row[4] = n[1];
		//row[5] = n[2];

		//row[6] = n[0]*(d[0]-s[0]) + n[1]*(d[1]-s[1]) + n[2]*(d[2]-s[2]); //b
		//weight = 0.0012/(0.0012 + 0.0019*(s[3]-0.4)*(s[3]-0.4));
    }

	////////////// Compute A^t*A and A^t*b ///////////////////////////

	__shared__ float smem[THREAD_SIZE];

	int tid = flattenedThreadId();
	
    int shift = 0;
    for (int k = 0; k < 6; ++k)        //rows
    {
        #pragma unroll
        for (int l = k; l < 7; ++l)          // cols + b
        {
			__syncthreads ();
			smem[tid] = row[k] * row[l] + row_rgb[k] * row_rgb[l];
			__syncthreads ();
						
			reduce<THREAD_SIZE>(smem);
			
			if (tid == 0) {			
				buf[blockIdx.x + blockIdx.y*gridDim.x + (shift++)*(gridDim.x*gridDim.y)] = smem[0];
			}
		}
    }
        
}

__global__ void MatchKernelGaussNewton(float *intr, float *Rcurr, float *tcurr, float *VMap, float *NMap, float *RGB, cv::gpu::DevMem2D_<float> Gx, cv::gpu::DevMem2D_<float> Gy,
										float *VMap_prev, float *NMap_prev, float *RGB_prev, float distThres, float angleThres, int n_row, int m_col, float *buf, int fact) {

		ProcessMatchKernelGaussNewto(intr, Rcurr, tcurr, VMap, NMap, RGB, Gx, Gy, VMap_prev, NMap_prev, RGB_prev, distThres, angleThres, n_row, m_col, buf, fact);
}


//*************** Define cuda functions **********////////////////////

void EstimateSystemGaussNewton(float *Rcurr_dev, float *tcurr_dev, float *VMap, float *NMap, float *RGB, cv::gpu::DevMem2D_<float> Gx, cv::gpu::DevMem2D_<float> Gy,
					float *VMap_prev, float *NMap_prev, float *RGB_prev,
					float *intr, float distThres, float angleThres, int n_row, int m_col, float *A, float *b, int fact) {

	dim3 dimBlock(THREAD_SIZE_L_X, THREAD_SIZE_L_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n_row/fact, dimBlock.x);
	dimGrid.y = divUp (m_col/fact, dimBlock.y);
	
	checkCudaErrors( cudaMemset(buf_dev, 0, 27*dimGrid.x*dimGrid.y*sizeof(float)) );
	
	MatchKernelGaussNewton<<<dimGrid, dimBlock>>>(intr, Rcurr_dev, tcurr_dev, VMap, NMap, RGB, Gx, Gy, VMap_prev, NMap_prev, RGB_prev, distThres, angleThres, n_row, m_col, buf_dev, fact);
	
	checkCudaErrors( cudaMemset(outbuf_dev, 0, 27*sizeof(float)) );

	ReduceKernel<<<27, STRIDE>>>(buf_dev, outbuf_dev, dimGrid.x*dimGrid.y);
	
	checkCudaErrors( cudaMemcpy(buf, outbuf_dev, 27*sizeof(float), cudaMemcpyDeviceToHost) );
	
	int shift = 0;
	for (int i = 0; i < 6; ++i) {  //rows
		for (int j = i; j < 7; ++j)    // cols + b
		{
			float value = buf[shift++];
			if (j == 6)       // vector b
				b[i] = value;
			else
				A[j * 6 + i] = A[i * 6 + j] = value;
		}
	}

	return;
}