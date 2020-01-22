#include "Primitive.cuh"

/******* Global variables ******/
texture<float4, cudaTextureType2D, cudaReadModeElementType> texRef;

float *pose_prim;
unsigned int *res_dev;

void AllocPosePrimMatrix() {
	checkCudaErrors( cudaMalloc((void**)&pose_prim, 16 * sizeof(float)) );
}

void SetPosePrimMatrix(float *pose_CPU) {
	checkCudaErrors( cudaMemcpy(pose_prim, pose_CPU,  16 * sizeof(float), cudaMemcpyHostToDevice) );
}

void FreePosePrimMatrix() {
	checkCudaErrors( cudaFree(pose_prim) );
}

void AllocBufPrim() {
	checkCudaErrors( cudaMalloc((void **)&res_dev, sizeof(unsigned int)) );
}

void FreeBufPrim() {
	checkCudaErrors( cudaFree(res_dev) );
}


////**** Kernel definition *****/
__device__ __forceinline__ void VertexFromBumpProcess(float *VMap, unsigned short *Bump, unsigned char *RGB_bump, unsigned short *Mask_dev, float *param, int n, int m, int lvl)
{
    float x, y, d, ind_i, ind_j;
	    
	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*(m/lvl) + j;
	
	int i_ref = lvl*i;
	int j_ref = lvl*j;
    int idx_ref = i_ref*m + j_ref;

    if (i > n-1 || j > m-1) 
        return;

    if (Mask_dev[idx_ref] < 11) {
		VMap[3*idx] = 0.0;
		VMap[3*idx+1] = 0.0;
		VMap[3*idx+2] = 0.0;
        return;
	}

    // Attention indexes are inversed when reading the matrix (shift_i -> e1 -> x; shift_j -> e2 -> y; j - > m_prim -> e1)
    ind_i = float(i_ref);
    ind_j = float(j_ref);

	float Shift_ind[2];
	Shift_ind[0] = float(Bump[3*idx_ref])/60000.0;
	Shift_ind[1] = float(Bump[3*idx_ref+1])/60000.0;

	x = (ind_i+Shift_ind[0])*2.0/param[4] + param[6];
    y = (ind_j+Shift_ind[1])*2.0/param[5] + param[7];

    d = param[3] + ((float(Bump[3*idx_ref+2])/2000.0)-15.0);

    VMap[3*idx] = x*param[8] + y*param[11] + d*param[0];
    VMap[3*idx+1] = x*param[9] + y*param[12] + d*param[1];
    VMap[3*idx+2] = x*param[10] + y*param[13] + d*param[2];
}
__global__ void VertexFromBumpKernel(float *VMap, unsigned short *Bump, unsigned char *RGB_bump, unsigned short *Mask_dev, float *param, int n, int m, int lvl)
{
    VertexFromBumpProcess(VMap, Bump, RGB_bump, Mask_dev, param, n, m, lvl);

}

__device__ __forceinline__ void VertexOnlyFromBumpProcess(float *VMap, unsigned short *Bump, unsigned short *Mask_dev, float *param, int n, int m, int lvl)
{
    float x, y, d, ind_i, ind_j;
	    
	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*(m/lvl) + j;
	
	int i_ref = lvl*i;
	int j_ref = lvl*j;
    int idx_ref = i_ref*m + j_ref;

    if (i > n-1 || j > m-1) 
        return;

    if (Mask_dev[idx_ref] < 11) {
		VMap[3*idx] = 0.0;
		VMap[3*idx+1] = 0.0;
		VMap[3*idx+2] = 0.0;
        return;
	}

    // Attention indexes are inversed when reading the matrix (shift_i -> e1 -> x; shift_j -> e2 -> y; j - > m_prim -> e1)
    ind_i = float(i_ref);
    ind_j = float(j_ref);

	float Shift_ind[2];
	Shift_ind[0] = float(Bump[3*idx_ref])/60000.0;
	Shift_ind[1] = float(Bump[3*idx_ref+1])/60000.0;

	x = (ind_i+Shift_ind[0])*2.0/param[4] + param[6];
    y = (ind_j+Shift_ind[1])*2.0/param[5] + param[7];

    d = param[3] + ((float(Bump[3*idx_ref+2])/2000.0)-15.0);

    VMap[3*idx] = x*param[8] + y*param[11] + d*param[0];
    VMap[3*idx+1] = x*param[9] + y*param[12] + d*param[1];
    VMap[3*idx+2] = x*param[10] + y*param[13] + d*param[2];
}
__global__ void VertexOnlyFromBumpKernel(float *VMap, unsigned short *Bump, unsigned short *Mask_dev, float *param, int n, int m, int lvl)
{
    VertexOnlyFromBumpProcess(VMap, Bump, Mask_dev, param, n, m, lvl);

}

__device__ __forceinline__ void VertexRGBFromBumpProcess(float *VMap, float *RGB, unsigned short *Bump, unsigned char *RGB_bump, unsigned char *Mask_dev, float *param, int n, int m, int lvl)
{
    float x, y, d, ind_i, ind_j;
	    
	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	int i_ref = lvl*i;
	int j_ref = lvl*j;
    //int idx_ref = i_ref*m + j_ref;
	

    if (i > n-1 || j > m-1) 
        return;

    if (Mask_dev[idx] < 11) {
		VMap[3*idx] = 0.0;
		VMap[3*idx+1] = 0.0;
		VMap[3*idx+2] = 0.0;
		
		RGB[4*idx] = 0.0;
		RGB[4*idx+1] = 0.0;
		RGB[4*idx+2] = 0.0;
		RGB[4*idx+3] = 0.0;
        return;
	}
	

    // Attention indexes are inversed when reading the matrix (shift_i -> e1 -> x; shift_j -> e2 -> y; j - > m_prim -> e1)
    ind_i = float(i_ref);
    ind_j = float(j_ref);

	float Shift_ind[2];
	Shift_ind[0] = float(Bump[3*idx])/60000.0;
	Shift_ind[1] = float(Bump[3*idx+1])/60000.0;

	x = (ind_i+Shift_ind[0])*2.0/param[4] + param[6];
    y = (ind_j+Shift_ind[1])*2.0/param[5] + param[7];

    d = param[3] + ((float(Bump[3*idx+2])/2000.0)-15.0);

    VMap[3*idx] = x*param[8] + y*param[11] + d*param[0];
    VMap[3*idx+1] = x*param[9] + y*param[12] + d*param[1];
    VMap[3*idx+2] = x*param[10] + y*param[13] + d*param[2];
	
	RGB[4*idx] = float(RGB_bump[3*idx])/255.0;
	RGB[4*idx+1] = float(RGB_bump[3*idx+1])/255.0;
	RGB[4*idx+2] = float(RGB_bump[3*idx+2])/255.0;
	RGB[4*idx+3] = 1.0;
}
__global__ void VertexRGBFromBumpKernel(float *VMap, float *RGB, unsigned short *Bump, unsigned char *RGB_bump, unsigned char *Mask_dev, float *param, int n, int m, int lvl)
{
    VertexRGBFromBumpProcess(VMap, RGB, Bump, RGB_bump, Mask_dev, param, n, m, lvl);

}

__device__ __forceinline__ void NormalProcess(float *NMap, float *VMap, int n, int m, bool inverse) {
	float p1 [3];
    float p2 [3];
    float p3 [3];
    float n_p [3];
    float n_p1 [3];
    float n_p2 [3];
    float n_p3 [3];
    float n_p4 [3];
    float norm_n;


    // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
    int idx_out = i*m + j;

    unsigned short n_tot = 0;

    if (i > n-1 || j > m-1)
        return;
        
    if ( i < 1 || i > n-2 || j < 1 || j > m-2 || VMap[3*idx+2] == 0.0) {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
        return;
    }

    p1[0] = VMap[3*idx];
    p1[1] = VMap[3*idx+1];
    p1[2] = VMap[3*idx+2];

    n_p1[0] = 0.0; n_p1[1] = 0.0; n_p1[2] = 0.0;
    n_p2[0] = 0.0; n_p2[1] = 0.0; n_p2[2] = 0.0;
    n_p3[0] = 0.0; n_p3[1] = 0.0; n_p3[2] = 0.0;
    n_p4[0] = 0.0; n_p4[1] = 0.0; n_p4[2] = 0.0;

    ////////////////////////// Triangle 1 /////////////////////////////////

    idx = (i+1)*m + j;
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = i*m + (j+1);
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p1[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p1[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p1[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p1[0]*n_p1[0] + n_p1[1]*n_p1[1] + n_p1[2]*n_p1[2]);

        if (norm_n != 0.0) {
            n_p1[0] = n_p1[0] / sqrt(norm_n);
            n_p1[1] = n_p1[1] / sqrt(norm_n);
            n_p1[2] = n_p1[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 2 /////////////////////////////////

    idx = i*m + (j+1);
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = (i-1)*m + j;
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p2[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p2[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p2[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p2[0]*n_p2[0] + n_p2[1]*n_p2[1] + n_p2[2]*n_p2[2]);

        if (norm_n != 0.0) {
            n_p2[0] = n_p2[0] / sqrt(norm_n);
            n_p2[1] = n_p2[1] / sqrt(norm_n);
            n_p2[2] = n_p2[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 3 /////////////////////////////////

    idx = (i-1)*m + j;
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = i*m + (j-1);
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p3[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p3[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p3[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p3[0]*n_p3[0] + n_p3[1]*n_p3[1] + n_p3[2]*n_p3[2]);

        if (norm_n != 0) {
            n_p3[0] = n_p3[0] / sqrt(norm_n);
            n_p3[1] = n_p3[1] / sqrt(norm_n);
            n_p3[2] = n_p3[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 4 /////////////////////////////////

    idx = i*m+ (j-1);
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = (i+1)*m + j;
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p4[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p4[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p4[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p4[0]*n_p4[0] + n_p4[1]*n_p4[1] + n_p4[2]*n_p4[2]);

        if (norm_n != 0) {
            n_p4[0] = n_p4[0] / sqrt(norm_n);
            n_p4[1] = n_p4[1] / sqrt(norm_n);
            n_p4[2] = n_p4[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    if (n_tot == 0) {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
        return;
    }

    n_p[0] = (n_p1[0] + n_p2[0] + n_p3[0] + n_p4[0])/float(n_tot);
    n_p[1] = (n_p1[1] + n_p2[1] + n_p3[1] + n_p4[1])/float(n_tot);
    n_p[2] = (n_p1[2] + n_p2[2] + n_p3[2] + n_p4[2])/float(n_tot);

    norm_n = sqrt(n_p[0]*n_p[0] + n_p[1]*n_p[1] + n_p[2]*n_p[2]);

    if (norm_n != 0) {
		if (inverse) {
			NMap[3*idx_out] = -n_p[0]/norm_n;
			NMap[3*idx_out+1] =  -n_p[1]/norm_n;
			NMap[3*idx_out+2] = -n_p[2]/norm_n;
		} else {
			NMap[3*idx_out] = n_p[0]/norm_n;
			NMap[3*idx_out+1] =  n_p[1]/norm_n;
			NMap[3*idx_out+2] = n_p[2]/norm_n;
		}
    } else {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
	}

    return;
}
__global__ void NormalKernel(float *NMap, float *VMap, int n, int m, bool inverse) {
	NormalProcess(NMap, VMap, n, m, inverse);
}

__device__ __forceinline__ void QuadTrimProcess(unsigned int *index_dev, float *VMap, unsigned char *Mask, unsigned short min_conf, int n, int m, float thresh, int lvl)
{
	int VIdx [4];

	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*(m/lvl-1) + j;
	
	if (i > n/lvl-2 || j > m/lvl-2)
		return;

	VIdx[0] = i*m/lvl+j; 
	VIdx[1] = i*m/lvl+j+1; 
	VIdx[2] = (i+1)*m/lvl+j+1; 
	VIdx[3] = (i+1)*m/lvl+j; 

	float diff1 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[1]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[1]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[1]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[1]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[1]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[1]+2])); 

	float diff2 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[2]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[2]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[2]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[2]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[2]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[2]+2])); 

	float diff3 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[3]+2])); 

	float diff4 = sqrt((VMap[3*VIdx[1]]-VMap[3*VIdx[2]])*(VMap[3*VIdx[1]]-VMap[3*VIdx[2]]) + (VMap[3*VIdx[1]+1]-VMap[3*VIdx[2]+1])*(VMap[3*VIdx[1]+1]-VMap[3*VIdx[2]+1])
						+ (VMap[3*VIdx[1]+2]-VMap[3*VIdx[2]+2])*(VMap[3*VIdx[1]+2]-VMap[3*VIdx[2]+2])); 

	float diff5 = sqrt((VMap[3*VIdx[1]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[1]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[1]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[1]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[1]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[1]+2]-VMap[3*VIdx[3]+2])); 

	float diff6 = sqrt((VMap[3*VIdx[2]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[2]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[2]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[2]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[2]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[2]+2]-VMap[3*VIdx[3]+2])); 

	float max_diff = max(diff1, diff2);
	max_diff = max(diff3, max_diff);
	max_diff = max(diff4, max_diff);
	max_diff = max(diff5, max_diff);
	max_diff = max(diff6, max_diff);
	if (Mask[VIdx[0]] < 11 || Mask[VIdx[1]] < 11 || Mask[VIdx[2]] < 11 || Mask[VIdx[3]] < 11 || max_diff > thresh) {
		index_dev[4*idx] = 0;
		index_dev[4*idx+1] = 0;
		index_dev[4*idx+2] = 0;
		index_dev[4*idx+3] = 0;
	} else {
		index_dev[4*idx] = unsigned int(VIdx[0]);
		index_dev[4*idx+1] = unsigned int(VIdx[3]);
		index_dev[4*idx+2] = unsigned int(VIdx[2]);
		index_dev[4*idx+3] = unsigned int(VIdx[1]);
	}

}
__global__ void QuadTrimKernel(unsigned int *index_dev, float *VMap, unsigned char *Mask, unsigned short min_conf, int n, int m, float thresh, int lvl)
{
	QuadTrimProcess(index_dev, VMap, Mask, min_conf, n, m, thresh, lvl);

}

__device__ __forceinline__ void UpdateBumpProcess(unsigned short *Bump_dev, unsigned char *RGB_dev, unsigned char *Mask_dev, float *VMap, float *NMap, float *RGB, float *Mask, 
													float *param, float *pose, int N_prim, int M_prim, int n, int m)
{
	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
		
	if (i > n-1 || j > m-1)
		return;
		
	float  mask_vis = Mask[4*idx+1]*255.0;
	
	float  mask_tmp = Mask[4*idx]*255.0;
	unsigned char mask_curr = unsigned char(__float2int_rn(mask_tmp));

	float vcurr[3];
	vcurr[0] = VMap[3*idx];
	vcurr[1] = VMap[3*idx+1];
	vcurr[2] = VMap[3*idx+2];

	float ncurr[3];
	ncurr[0] = NMap[3*idx];
	ncurr[1] = NMap[3*idx+1];
	ncurr[2] = NMap[3*idx+2];

	if (ncurr[0] == 0.0 && ncurr[1] == 0.0 && ncurr[2] == 0.0) {
		return;
	}

	// Transform current point
	float vcurr_l[3];
	vcurr_l[0] = pose[0]*vcurr[0] + pose[4]*vcurr[1] + pose[8]*vcurr[2] + pose[12];
	vcurr_l[1] = pose[1]*vcurr[0] + pose[5]*vcurr[1] + pose[9]*vcurr[2] + pose[13];
	vcurr_l[2] = pose[2]*vcurr[0] + pose[6]*vcurr[1] + pose[10]*vcurr[2] + pose[14];

	float ncurr_l[3];
	ncurr_l[0] = pose[0]*ncurr[0] + pose[4]*ncurr[1] + pose[8]*ncurr[2];
	ncurr_l[1] = pose[1]*ncurr[0] + pose[5]*ncurr[1] + pose[9]*ncurr[2];
	ncurr_l[2] = pose[2]*ncurr[0] + pose[6]*ncurr[1] + pose[10]*ncurr[2];
	
	float error_dist = (vcurr_l[0]*param[0] + vcurr_l[1]*param[1] + vcurr_l[2]*param[2]) - param[3];
    float error_alpha = (ncurr_l[0]*param[0] + ncurr_l[1]*param[1] + ncurr_l[2]*param[2]);

	if (fabs(error_dist) > EPSILON || fabs(error_alpha) < ALPHA) {
		return;
	}

	float proj_a = vcurr_l[0]*param[8] + vcurr_l[1]*param[9] + vcurr_l[2]*param[10]; // e1
	float proj_b = vcurr_l[0]*param[11] + vcurr_l[1]*param[12] + vcurr_l[2]*param[13]; // e2

	proj_a = (proj_a-param[6])*param[4]/2.0; //shift[0]
	proj_b = (proj_b-param[7])*param[5]/2.0; //Shift[1];

	int ind_i = __float2int_rd(proj_a);
	int ind_j = __float2int_rd(proj_b);

	if (ind_i > N_prim-1 || ind_j > M_prim-1 || ind_i  < 0 || ind_j  < 0 ) 
		return;

	int idx_prim = ind_i*M_prim + ind_j;
		
	unsigned char mask_ref = Mask_dev[idx_prim];

	__syncthreads ();
	
	if (mask_vis < 10.0 && mask_ref != 1) {
		if  (mask_ref > 11)
			Mask_dev[idx_prim] = Mask_dev[idx_prim] - 1; //atomicSub(&Mask[idxBump [0]*size[1] + idxBump [1]], 1);
		else {
			Mask_dev[idx_prim] = 10; //atomicExch(&Mask[idxBump [0]*size[1] + idxBump [1]], 10);
			Bump_dev[3*idx_prim] = 0; //atomicExch(&Bump[3*(idxBump [0]*size[1] + idxBump [1])], 0);
			Bump_dev[3*idx_prim+1] = 0; //atomicExch(&Bump[3*(idxBump [0]*size[1] + idxBump [1])+1], 0);
			Bump_dev[3*idx_prim+2] = 0; //atomicExch(&Bump[3*(idxBump [0]*size[1] + idxBump [1])+2], 0);
			RGB_dev[3*idx_prim] = 0; //atomicExch(&RGB[3*(idxBump [0]*size[1] + idxBump [1])], 0);
			RGB_dev[3*idx_prim+1] = 0; //atomicExch(&RGB[3*(idxBump [0]*size[1] + idxBump [1])+1], 0);
			RGB_dev[3*idx_prim+2] = 0; //atomicExch(&RGB[3*(idxBump [0]*size[1] + idxBump [1])+2], 0);
		}
		return;
	}

	float shift [2];
	shift[0] = proj_a - float(ind_i);
	shift[1] = proj_b - float(ind_j);

	///// Critical Section ? ///////
	//unsigned char old_mask = Mask_dev[idx_prim];
	//if (mask_curr > Mask_dev[idx_prim]) {
	//	Mask_dev[idx_prim] = mask_curr; //atomicMax(&Mask[idxBump [0]*size[1] + idxBump [1]], mask_curr);
	//}

	//__syncthreads ();
	//if ((old_mask < mask_curr && Mask_dev[idx_prim] == mask_curr)) {

		Bump_dev[3*idx_prim] = unsigned short(__float2int_rn(shift[0]*60000.0)); 
		Bump_dev[3*idx_prim+1] = unsigned short(__float2int_rn(shift[1]*60000.0)); 
		Bump_dev[3*idx_prim+2] = unsigned short(__float2int_rn(((error_dist+15.0))*2000.0)); 
		RGB_dev[3*idx_prim] = unsigned char(__float2int_rn(RGB[4*idx]*255.0));
		RGB_dev[3*idx_prim+1] = unsigned char(__float2int_rn(RGB[4*idx+1]*255.0));
		RGB_dev[3*idx_prim+2] = unsigned char(__float2int_rn(RGB[4*idx+2]*255.0));
		Mask_dev[idx_prim] = mask_curr;
	//}
}
__global__ void UpdateBumpKernel(unsigned short *Bump_dev, unsigned char *RGB_dev, unsigned char *Mask_dev, float *VMap, float *NMap, float *RGB, float *Mask, 
													float *param, float *pose, int N_prim, int M_prim, int n, int m)
{
	UpdateBumpProcess(Bump_dev, RGB_dev, Mask_dev, VMap, NMap, RGB, Mask, param, pose, N_prim, M_prim, n, m);

}

__device__ __forceinline__ void BBOXProcess(unsigned char *Mask, int *BBox, int n, int m) {
	 // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1) 
		return;

	if (Mask[idx] < 11)
		return;

	if (i < BBox[0])
		atomicMin(&BBox[0], i);
	if (i > BBox[1])
		atomicMax(&BBox[1], i);
	if (j < BBox[2])
		atomicMin(&BBox[2], j);
	if (j > BBox[3])
		atomicMax(&BBox[3], j);
}
__global__ void BBOXKernel(unsigned char *Mask, int *BBox, int n, int m) {
	BBOXProcess(Mask, BBox, n, m);
}

__device__ __forceinline__ void CountProcess(unsigned char *Mask, unsigned int *res, int n, int m) {
	 // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1) 
		return;

	if (Mask[idx] < 11)
		return;

	atomicAdd(&res[0], 1);
}
__global__ void CountKernel(unsigned char *Mask, unsigned int *res, int n, int m) {
	CountProcess(Mask, res, n, m);
}

__device__ __forceinline__ void OverlapProcess(float *VMap, float *param, unsigned int *counter, int n, int m, int N_prim, int M_prim) {
	 // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1) 
		return;

	float pt [3];
	pt[0] = VMap[3*idx];
	pt[1] = VMap[3*idx+1];
	pt[2] = VMap[3*idx+2];

	if (pt[0] == 0.0 && pt[1] == 0.0 && pt[2] == 0.0)
		return;
	
	atomicAdd(&counter[1], 1);

	// Project on parameters
	float error_dist = (pt[0]*param[0] + pt[1]*param[1] + pt[2]*param[2]) - param[3];

	if (fabs(error_dist) > EPSILON) {
		return;
	}

	float proj_a = pt[0]*param[8] + pt[1]*param[9] + pt[2]*param[10]; // e1
	float proj_b = pt[0]*param[11] + pt[1]*param[12] + pt[2]*param[13]; // e2

	proj_a = (proj_a-param[6])*param[4]/2.0; //shift[0]
	proj_b = (proj_b-param[7])*param[5]/2.0; //Shift[1];

	int ind_i = __float2int_rd(proj_a);
	int ind_j = __float2int_rd(proj_b);

	if (ind_i > N_prim-1 || ind_j > M_prim-1 || ind_i  < 0 || ind_j  < 0 ) 
		return;

	atomicAdd(&counter[0], 1);
}
__global__ void OverlapKernel(float *VMap, float *param, unsigned int *counter, int n, int m, int N_prim, int M_prim) {
	OverlapProcess(VMap, param, counter, n, m, N_prim, M_prim);
}

__device__ __forceinline__ void SetRGBProcess(unsigned char *RGB, int n, int m) {
	 // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1) 
		return;

	float4 RGBD_v = tex2D(texRef, (float)i+0.5f,  (float)j+0.5f);

	RGB[3*idx] = unsigned char(__float2int_rn(RGBD_v.x*255.0));
	RGB[3*idx+1] = unsigned char(__float2int_rn(RGBD_v.y*255.0));
	RGB[3*idx+2] = unsigned char(__float2int_rn(RGBD_v.z*255.0));
}
__global__ void SetRGBKernel(unsigned char *RGB, int n, int m) {
	SetRGBProcess(RGB, n, m);
}

__device__ __forceinline__ void ReadVtxMaskPrimProcess(float *param, unsigned short *Bump, unsigned char *Mask, int n, int m) {
	 // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1) 
		return;

	float4 RGBD_v = tex2D(texRef, (float)i+0.5f,  (float)j+0.5f);
	if (RGBD_v.x == 0.0 && RGBD_v.y == 0.0 && RGBD_v.z == 0.0) {
		Mask[idx] = 10;
		Bump[3*idx] = 0;
		Bump[3*idx+1] = 0;
		Bump[3*idx+2] = 0;	
		return;
	}

	float pt [3];
	pt[0] = (RGBD_v.x*20.0)-10.0;
	pt[1] = (RGBD_v.y*20.0)-10.0;
	pt[2] = (RGBD_v.z*20.0)-10.0;

	float error_dist = (pt[0])*param[0] + (pt[1])*param[1] + (pt[2])*param[2] - param[3];

	if (fabs(error_dist) > EPSILON)
		return;

	float a,b;
	a = pt[0]*param[8] + pt[1]*param[9] + pt[2]*param[10];
    b = pt[0]*param[11] + pt[1]*param[12] + pt[2]*param[13];

	a = (a-param[6])*param[4]/2.0;
	b = (b-param[7])*param[5]/2.0; 

	int ind_i = __float2int_rd(a);
	int ind_j = __float2int_rd(b);

	if (ind_i > n-1 || ind_j > m-1 || ind_i  < 0 || ind_j  < 0 ) 
		return;

	float shift [2];
	shift[0] = a - float(ind_i);
	shift[1] = b - float(ind_j);

	int idx_prim = ind_i*m + ind_j;

	/*bool test = idx_prim == idx;
	Mask[idx_prim] = unsigned char(test);*/

	Bump[3*idx_prim] = unsigned short(__float2int_rn(shift[0]*60000.0)); 
	Bump[3*idx_prim+1] = unsigned short(__float2int_rn(shift[1]*60000.0)); 
	Bump[3*idx_prim+2] = unsigned short(__float2int_rn(((error_dist+15.0))*2000.0)); 
	Mask[idx_prim] = unsigned char(__float2int_rn(RGBD_v.w*255.0));

}
__global__ void ReadVtxMaskPrimkKernel(float *param, unsigned short *Bump, unsigned char *Mask, int n, int m) {
	ReadVtxMaskPrimProcess(param, Bump, Mask, n, m);
}


///**** Function definitions ****/
void VertexFromBump(float *VMap_VBO, unsigned short *Bump_dev, unsigned char *RGB_bump_dev, unsigned short *Mask_dev, float *param_dev, int n, int m, int lvl)  {
    	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n/lvl, dimBlock.x);
	dimGrid.y = divUp (m/lvl, dimBlock.y);

    VertexFromBumpKernel<<<dimGrid, dimBlock>>>(VMap_VBO, Bump_dev, RGB_bump_dev, Mask_dev, param_dev, n, m, lvl);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return;
}

void VertexOnlyFromBump(float *VMap_VBO, unsigned short *Bump_dev, unsigned short *Mask_dev, float *param_dev, int n, int m, int lvl)  {
    	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n/lvl, dimBlock.x);
	dimGrid.y = divUp (m/lvl, dimBlock.y);

    VertexOnlyFromBumpKernel<<<dimGrid, dimBlock>>>(VMap_VBO, Bump_dev, Mask_dev, param_dev, n, m, lvl);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return;
}

void VertexRGBFromBump(float *VMap_VBO, float *RGB_VBO, unsigned short *Bump_dev, unsigned char *RGB_bump_dev, unsigned char *Mask_dev, float *param_dev, int n, int m, int lvl) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	VertexRGBFromBumpKernel<<<dimGrid, dimBlock>>>(VMap_VBO, RGB_VBO, Bump_dev, RGB_bump_dev, Mask_dev, param_dev, n, m, lvl);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return;
}
void ComputeNormal(float *NMap, float *VMap, int n, int m, bool inverse) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	NormalKernel<<<dimGrid, dimBlock>>>(NMap, VMap, n, m, inverse);

	checkCudaErrors( cudaDeviceSynchronize() );
    
	return;
}

void QuadTrim(float *VMap, unsigned char *Mask, unsigned int *indices_dev, unsigned short min_conf, int n, int m, float thresh, int lvl) {

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n/lvl-1, dimBlock.x);
	dimGrid.y = divUp (m/lvl-1, dimBlock.y);

	QuadTrimKernel<<<dimGrid, dimBlock>>>(indices_dev, VMap, Mask, min_conf, n, m, thresh, lvl);

	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void UpdateBump_cu(unsigned short *Bump_dev, unsigned char *RGB_dev, unsigned char *Mask_dev, float *VMap, float *NMap, float *RGB, float *Mask, 
													float *param, int N_prim, int M_prim, int n, int m) {

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	UpdateBumpKernel<<<dimGrid, dimBlock>>>(Bump_dev, RGB_dev, Mask_dev, VMap, NMap, RGB, Mask, param, pose_prim, N_prim, M_prim, n, m);

	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;

}

void BBOX_cu(unsigned char *Mask, int *BBox, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	BBOXKernel<<<dimGrid, dimBlock>>>(Mask, BBox, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return;
}

unsigned int Count_cu(unsigned char *Mask, int n, int m) {
	checkCudaErrors( cudaMemset(res_dev,0,sizeof(unsigned int)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	CountKernel<<<dimGrid, dimBlock>>>(Mask, res_dev, n, m);

	unsigned int res;
	checkCudaErrors( cudaMemcpy(&res, res_dev, sizeof(unsigned int), cudaMemcpyDeviceToHost) );
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return res;
}


float Overlap_cu(float *VMap, float *param, int n, int m, int N_prim, int M_prim) {
	unsigned int *counter_dev;
	
	checkCudaErrors( cudaMalloc((void**)&counter_dev, 2*sizeof(unsigned int)) );
	checkCudaErrors( cudaMemset(counter_dev,0,2*sizeof(unsigned int)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	OverlapKernel<<<dimGrid, dimBlock>>>(VMap, param, counter_dev, n, m, N_prim, M_prim);

	unsigned int counter[2];
	checkCudaErrors( cudaMemcpy(&counter, counter_dev, 2*sizeof(unsigned int), cudaMemcpyDeviceToHost) );
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	checkCudaErrors( cudaFree(counter_dev) );

	if (counter[1] == 0)
		return 1.0;
	
	return float(counter[0])/float(counter[1]);
}

void SetRGBPrim_cu(unsigned char *RGB, cudaArray* Array, int n, int m) {
	
	struct cudaChannelFormatDesc desc;
	desc = cudaCreateChannelDesc<float4>();

	checkCudaErrors( cudaBindTextureToArray( &texRef, Array, & desc) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	SetRGBKernel<<<dimGrid, dimBlock>>>(RGB, n, m);
	
	checkCudaErrors( cudaUnbindTexture( &texRef) );
	checkCudaErrors( cudaDeviceSynchronize() );

}

void ReadVtxMask_cu(float *param, unsigned short *Bump, unsigned char *Mask, cudaArray* Array, int n, int m) {
	
	struct cudaChannelFormatDesc desc;
	desc = cudaCreateChannelDesc<float4>();

	checkCudaErrors( cudaBindTextureToArray( &texRef, Array, & desc) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	ReadVtxMaskPrimkKernel<<<dimGrid, dimBlock>>>(param, Bump, Mask, n, m);
	
	checkCudaErrors( cudaUnbindTexture( &texRef) );
	checkCudaErrors( cudaDeviceSynchronize() );
}
