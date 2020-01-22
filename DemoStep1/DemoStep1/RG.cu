#include "RG.cuh"


/******* Kernel definitions ******/

__device__ __forceinline__ void VoteSphereProcess(float *NMap, int *Sphere, float *AVG_NMLE, int dimAzimut, int dimElev, float step, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;
	
	float nmle [3];
	nmle [0] = NMap[3*idx];
	nmle [1] = NMap[3*idx+1];
	nmle [2] = NMap[3*idx+2];
	
	int idx_sphere;
	if (nmle [0] == 0.0 && nmle [1] == 0.0 && nmle [2] == 0.0) 
		return;

	float alpha = (acos(nmle[2]) / PI) * 180.0; // [0;180]

	float beta = (acos(nmle[0]/sin(alpha)) / PI) * 180.0; // [0;360]

	if (nmle[1] < 0.0)
		beta += 180.0;

	idx_sphere = int(alpha/step)*dimAzimut + int(beta/step);
		
	atomicAdd(&AVG_NMLE[3*idx_sphere], nmle [0]);
	atomicAdd(&AVG_NMLE[3*idx_sphere+1], nmle [1]);
	atomicAdd(&AVG_NMLE[3*idx_sphere+2], nmle [2]);
	atomicAdd(&Sphere[idx_sphere], 1);

}
__global__ void VoteSphereKernel(float *NMap, int *Sphere, float *AVG_NMLE, int dimAzimut, int dimElev, float step, int n, int m) {
	VoteSphereProcess(NMap, Sphere, AVG_NMLE, dimAzimut, dimElev, step, n, m);
}

__device__ __forceinline__ void VoteDistanceProcess(float *VMap, int *Indices, float *nmle, int *Space_count, float *Space, int dim, float epsilon, int ref_i, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;

	int idx_pt = Indices[idx];
	if (idx_pt != ref_i)
		return;
	
	float pt [3];
	pt [0] = VMap[3*idx];
	pt [1] = VMap[3*idx+1];
	pt [2] = VMap[3*idx+2];

	float scal = pt[0]*nmle[0] + pt[1]*nmle[1] + pt[2]*nmle[2];
	if (scal < -10.0 || scal >= 10.0)
		return;

	int idx_space = int((scal+10.0)/epsilon);
	atomicAdd(&Space[idx_space], scal);
	atomicAdd(&Space_count[idx_space], 1);
}
__global__ void VoteDistanceKernel(float *VMap, int *Indices, float *nmle, int *Space_count, float *Space, int dim, float epsilon, int ref_i, int n, int m) {
	VoteDistanceProcess(VMap, Indices, nmle, Space_count, Space, dim, epsilon, ref_i, n, m);
}

__device__ __forceinline__ void GetIndicesProcess(float *NMap, int *Indices_dev, float *Plan_dev, float alpha, int nbPlan, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	float nmle [3];
	nmle [0] = NMap[3*idx];
	nmle [1] = NMap[3*idx+1];
	nmle [2] = NMap[3*idx+2];
	
	int idx_sphere;
	if (nmle [0] == 0.0 && nmle [1] == 0.0 && nmle [2] == 0.0) {
		Indices_dev[idx] = -1;
		return;
	}

	float min_error = -2.0;
	int k = 0;
	int idx_plan = -1;
	float nmletmp [3];
	float error_alpha;
	for (int l = 0; l < nbPlan; l++) {
		nmletmp [0] = Plan_dev[3*l];
		nmletmp [1] = Plan_dev[3*l+1];
		nmletmp [2] = Plan_dev[3*l+2];

		error_alpha = fabs(nmle[0]*nmletmp[0] + nmle[1]*nmletmp[1] + nmle[2]*nmletmp[2]);
		if (error_alpha > min_error) {
			min_error = error_alpha; 
			idx_plan = k;
		}
		k++;
	}

	if (min_error > cos(alpha)) {
		Indices_dev[idx] = idx_plan;
	} else {
		Indices_dev[idx] = -1;
	}
}
__global__ void GetIndicesKernel(float *NMap, int *Indices_dev, float *Plan_dev, float alpha, int nbPlan, int n, int m) {
	GetIndicesProcess(NMap, Indices_dev, Plan_dev, alpha, nbPlan, n, m);
}

__device__ __forceinline__ void GetIndicesDistancesProcess(float *VMap, int *Indices_Final_dev, int *Indices_dev, float *Equations_dev, float epsilon, int nb_subPlan, int shift_eq, int ref_i, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;

	int idx_pt = Indices_dev[idx];
	if (idx_pt != ref_i) 
		return;
		
	float pt [3];
	pt [0] = VMap[3*idx];
	pt [1] = VMap[3*idx+1];
	pt [2] = VMap[3*idx+2];

	float min_error = 2.0;
	int k = 0;
	int idx_dist = -1;
	float equa [4];
	float error_dist;
	for (int l = 0; l < nb_subPlan; l++) {
		equa [0] = Equations_dev[4*l];
		equa [1] = Equations_dev[4*l+1];
		equa [2] = Equations_dev[4*l+2];
		equa [3] = Equations_dev[4*l+3];
		error_dist = fabs(pt[0]*equa[0] + pt[1]*equa[1] + pt[2]*equa[2] - equa[3]);
		if (error_dist < min_error) {
			min_error = error_dist;
			idx_dist = k;
		}
		k++;
	}

	if (min_error < epsilon) {
		Indices_Final_dev[idx] = idx_dist+shift_eq;
	} else {
		Indices_Final_dev[idx] = -1;
	}
}
__global__ void GetIndicesDistancesKernel(float *VMap, int *Indices_Final_dev, int *Indices_dev, float *Equations_dev, float epsilon, int nb_subPlan, int shift_eq, int ref_i, int n, int m) {
	GetIndicesDistancesProcess(VMap, Indices_Final_dev, Indices_dev, Equations_dev, epsilon, nb_subPlan, shift_eq, ref_i, n, m);
}

__device__ __forceinline__ void ProjectionProcess(float *Projected_dev, int *BBox, float *VMap, float *NMap, float *RGB, int *Indices_dev, float *Equations_dev, int nbPlans, float res, float epsilon, float alpha, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;

	/*int idx_pt = Indices_dev[idx];
	if (idx_pt < 0)
		return;*/

	float pt [3];
	pt [0] = VMap[3*idx];
	pt [1] = VMap[3*idx+1];
	pt [2] = VMap[3*idx+2];
	
	float npt [3];
	npt [0] = NMap[3*idx];
	npt [1] = NMap[3*idx+1];
	npt [2] = NMap[3*idx+2];

	if (npt [0] == 0.0 && npt [1] == 0.0 && npt [2] == 0.0)
		return;

	float min_val_dist = 1.0e10;
	int idx_pt = -1;
	float error_dist, error_alpha, a, b, scal, d;
	float proj [3];

	float nml[3], e1[3], e2[3];
	for (int count = 0; count < nbPlans; count++) {

		nml[0] = Equations_dev[10*count]; nml[1] = Equations_dev[10*count+1]; nml[2] = Equations_dev[10*count+2];
		e1[0] = Equations_dev[10*count+3]; e1[1] = Equations_dev[10*count+4]; e1[2] = Equations_dev[10*count+5];
		e2[0] = Equations_dev[10*count+6]; e2[1] = Equations_dev[10*count+7]; e2[2] = Equations_dev[10*count+8];
		d = Equations_dev[10*count+9];
		
		error_dist = (pt[0])*nml[0] + (pt[1])*nml[1] + (pt[2])*nml[2] - d;
        error_alpha = (npt[0]*nml[0] + npt[1]*nml[1] + npt[2]*nml[2]);

		if (fabs(error_dist) > epsilon || fabs(error_alpha) < alpha)
			continue;
		
		if (fabs(error_dist) < min_val_dist) {
			min_val_dist = fabs(error_dist);
			idx_pt = count;
		}
	}

	if (idx_pt == -1)
		return;

	
	float color [3];
	color [0] = RGB[4*idx];
	color [1] = RGB[4*idx+1];
	color [2] = RGB[4*idx+2];

	nml[0] = Equations_dev[10*idx_pt]; nml[1] = Equations_dev[10*idx_pt+1]; nml[2] = Equations_dev[10*idx_pt+2];
	e1[0] = Equations_dev[10*idx_pt+3]; e1[1] = Equations_dev[10*idx_pt+4]; e1[2] = Equations_dev[10*idx_pt+5];
	e2[0] = Equations_dev[10*idx_pt+6]; e2[1] = Equations_dev[10*idx_pt+7]; e2[2] = Equations_dev[10*idx_pt+8];
	d = Equations_dev[10*idx_pt+9];

	scal = (pt [0])*nml[0]+(pt [1])*nml[1]+(pt [2])*nml[2] - d;
    proj[0] = (pt [0]) - scal*nml[0];
    proj[1] = (pt [1]) - scal*nml[1];
    proj[2] = (pt [2]) - scal*nml[2];
        
    a = proj[0]*e1[0] + proj[1]*e1[1] + proj[2]*e1[2];
    b = proj[0]*e2[0] + proj[1]*e2[1] + proj[2]*e2[2];

	atomicMin(&BBox[4*idx_pt], int(a/res));
	atomicMax(&BBox[4*idx_pt+1], int(a/res));
	atomicMin(&BBox[4*idx_pt+2], int(b/res));
	atomicMax(&BBox[4*idx_pt+3], int(b/res));

	Projected_dev[6*idx] = a/res;
	Projected_dev[6*idx+1] = b/res;
	Projected_dev[6*idx+2] = scal;
	Projected_dev[6*idx+3] = color [0];
	Projected_dev[6*idx+4] = color [1];
	Projected_dev[6*idx+5] = color [2];
	Indices_dev[idx] = idx_pt;
}
__global__ void ProjectionKernel(float *Projected_dev, int *BBox, float *VMap, float *NMap, float *RGB, int *Indices_dev, float *Equations_dev, int nbPlans, float res, float epsilon, float alpha, int n, int m) {
	ProjectionProcess(Projected_dev, BBox, VMap, NMap, RGB, Indices_dev, Equations_dev, nbPlans, res, epsilon, alpha, n, m);
}

__device__ __forceinline__ void SegmentProcess(unsigned char *Label, float *VMap, float *NMap, float *pose, float *Equations, int nbPlans, float epsilon, float alpha, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;

	if (Label[idx] > 0)
		return;

	int s = 1;
	int lb = max(0, i-s);
	int ub = min(n, i+s+1);
	int lr = max(0, j-s);
	int ur = min(m, j+s+1);
	float depth = VMap[3*idx+2];
	float thresh_depth = 0.003;

	float pt_l [3];
	pt_l [0] = VMap[3*idx];
	pt_l [1] = VMap[3*idx+1];
	pt_l [2] = VMap[3*idx+2];

	float pt [3];
	pt [0] = pose[0]*pt_l[0] + pose[4]*pt_l[1] + pose[8]*pt_l[2] + pose[12];
	pt [1] = pose[1]*pt_l[0] + pose[5]*pt_l[1] + pose[9]*pt_l[2] + pose[13];
	pt [2] = pose[2]*pt_l[0] + pose[6]*pt_l[1] + pose[10]*pt_l[2] + pose[14];
	
	float npt_l [3];
	npt_l [0] = NMap[3*idx];
	npt_l [1] = NMap[3*idx+1];
	npt_l [2] = NMap[3*idx+2];
	
	float npt [3];
	npt [0] = pose[0]*npt_l[0] + pose[4]*npt_l[1] + pose[8]*npt_l[2];
	npt [1] = pose[1]*npt_l[0] + pose[5]*npt_l[1] + pose[9]*npt_l[2];
	npt [2] = pose[2]*npt_l[0] + pose[6]*npt_l[1] + pose[10]*npt_l[2];

	if (npt [0] == 0.0 && npt [1] == 0.0 && npt [2] == 0.0)
		return;

	float error_dist, error_alpha, a, b, scal, d;
	float nml[3], e1[3], e2[3];
	for (int ki = lb; ki < ub; ki++) {
		for (int kj = lr; kj < ur; kj++) {
			if (Label[ki*m + kj] > 0 && fabs(VMap[3*(ki*m + kj)+2]-depth) < thresh_depth) {
				int count = int(Label[ki*m + kj])-1;
				nml[0] = Equations[10*count]; nml[1] = Equations[10*count+1]; nml[2] = Equations[10*count+2];
				e1[0] = Equations[10*count+3]; e1[1] = Equations[10*count+4]; e1[2] = Equations[10*count+5];
				e2[0] = Equations[10*count+6]; e2[1] = Equations[10*count+7]; e2[2] = Equations[10*count+8];
				d = Equations[10*count+9];
		
				error_dist = (pt[0])*nml[0] + (pt[1])*nml[1] + (pt[2])*nml[2] - d;
				error_alpha = (npt[0]*nml[0] + npt[1]*nml[1] + npt[2]*nml[2]);

				if (fabs(error_dist) > epsilon || error_alpha < alpha)
					continue;

				Label[idx] = Label[ki*m + kj];
				return;
			}
		}
	}
	return;
}
__global__ void SegmentKernel(unsigned char *Label, float *VMap, float *NMap, float *pose, float *Equations, int nbPlans, float epsilon, float alpha, int n, int m) {
	SegmentProcess(Label, VMap, NMap, pose, Equations, nbPlans, epsilon, alpha, n, m);
}

__device__ __forceinline__ void InitFragsProcess(float *Projected, int *Indices, int *Size, float *center, unsigned short *TheBumps, unsigned char *TheRGBs, unsigned char *TheMasks, float *equation, int currj, float res, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
        return;

	int idx_pt = Indices[idx];
	if (currj != -1 && idx_pt != currj /* < 0*/)
		return;

	// The local origintransformed into the global coordinate system
	float origin [3];
	origin [0] = equation [0];
	origin [1] = equation [1];
	origin [2] = equation [2];

	// The viewing direction of the local camera in the global coordinate system
	float view_dir [3];
	view_dir [0] = equation [3];
	view_dir [1] = equation [4];
	view_dir [2] = equation [5];

	// The normal of the plane
	float nmle [3];
	nmle [0] = equation [6];
	nmle [1] = equation [7];
	nmle [2] = equation [8];

	float rgb [3];
	rgb [0] = Projected[6*idx+3];
	rgb [1] = Projected[6*idx+4];
	rgb [2] = Projected[6*idx+5];

	float scal, a, b, alpha, theta, d, x, y;
	
	a = Projected[6*idx];
	b = Projected[6*idx+1];
	scal = Projected[6*idx+2];
	
	x = a*res;
    y = b*res;

    d = equation [15] + scal;
	
	float pt [3];
	pt [0] = x*equation [9] + y*equation [12] + d*nmle[0];
    pt [1] = x*equation [10] + y*equation [13] + d*nmle[1];
    pt [2] = x*equation [11] + y*equation [14] + d*nmle[2];

	// The vector from the point to the origin
	float vect [3];
	vect [0] = origin [0] - pt [0];
	vect [1] = origin [1] - pt [1];
	vect [2] = origin [2] - pt [2];
	float nrm = sqrt(vect [0]*vect [0] + vect [1]*vect [1] + vect [2]*vect [2]);
	vect [0] = vect [0]/nrm;
	vect [1] = vect [1]/nrm;
	vect [2] = vect [2]/nrm;

	// Dot product between nmle and vector
	theta = nmle[0]*vect[0] + nmle[1]*vect[1] + nmle[2]*vect[2];
	alpha = view_dir[0]*vect[0] + view_dir[1]*vect[1] + view_dir[2]*vect[2];

	bool lockval = (theta > 0.8) && (alpha > 0.4);

	int idxBump [2];
	idxBump [0] = int(a -center[0]/res);
	idxBump [1] = int(b -center[1]/res);

	float shift [2];
	shift[0] = (a -center[0]/res) - float(idxBump [0]);
	shift[1] = (b -center[1]/res) - float(idxBump [1]);

	if (idxBump [0] < 0 || idxBump [0] > Size[0]-1 || idxBump [1] < 0 || idxBump [1] > Size[1]-1)
		return;
		
	int old_mask = TheMasks[idxBump [0]*Size[1] + idxBump [1]]; ///atomicExch(&TheMasks[idxBump [0]*Size[1] + idxBump [1]], 11);
	__syncthreads ();

	if (old_mask == 10) {
		TheBumps[3*(idxBump [0]*Size[1] + idxBump [1])] = unsigned short(shift[0]*60000.0);
		TheBumps[3*(idxBump [0]*Size[1] + idxBump [1])+1] = unsigned short(shift[1]*60000.0);
		TheBumps[3*(idxBump [0]*Size[1] + idxBump [1])+2] = unsigned short(((scal+15.0)/*/0.4*/)*2000.0);
		
		TheRGBs[3*(idxBump [0]*Size[1] + idxBump [1])] = unsigned char(rgb[0]*255.0);
		TheRGBs[3*(idxBump [0]*Size[1] + idxBump [1])+1] = unsigned char(rgb[1]*255.0);
		TheRGBs[3*(idxBump [0]*Size[1] + idxBump [1])+2] = unsigned char(rgb[2]*255.0);

		TheMasks[idxBump [0]*Size[1] + idxBump [1]] = 11;
	}

}
__global__ void InitFragsKernel(float *Projected, int *Indices, int *Size, float *center, unsigned short *TheBumps, unsigned char *TheRGBs, unsigned char *TheMasks, float *equation, int currj, float res, int n, int m) {
	InitFragsProcess(Projected, Indices, Size, center, TheBumps, TheRGBs, TheMasks, equation, currj, res, n, m);
}

__device__ __forceinline__ void InitValProcess(unsigned char *InOut, unsigned char val, int n, int m) {
	 // identifiant de thread a deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;

	 if (i > n-1 || j > m-1) 
        return;

	 InOut[idx] = val;
}
__global__ void InitValKernel(unsigned char *InOut, unsigned char val, int n, int m) {
	InitValProcess(InOut, val, n, m);
}

__device__ __forceinline__ void ImDilateProcess(bool *res, bool *Input, int n, int m, int size) {
	 // identifiant de thread ? deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;

	if (i > n-size-1 || j > m-size-1 || i < size || j < size)
		return;

	res[idx] = false;
	for (int k = -size; k < size+1; k++) {
		for (int l = -size; l < size+1; l++) {
			if (Input[(i+k)*m + j+l]) {
				res[idx] = true;
				return;
			}
		}
	}	
}
__global__ void ImDilateKernel(bool *res, bool *Input, int n, int m, int size) {
	ImDilateProcess(res, Input, n, m, size);
}

__device__ __forceinline__ void ImDilateProcess(bool *res, unsigned char *Input, int n, int m, int size) {
	 // identifiant de thread ? deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;

	if (i > n-size-1 || j > m-size-1 || i < size || j < size)
		return;

	res[idx] = false;
	for (int k = -size; k < size+1; k++) {
		for (int l = -size; l < size+1; l++) {
			if (Input[(i+k)*m + j+l] > 10) {
				res[idx] = true;
				return;
			}
		}
	}	
}
__global__ void ImDilateKernel(bool *res, unsigned char *Input, int n, int m, int size) {
	ImDilateProcess(res, Input, n, m, size);
}

__device__ __forceinline__ void ImErodeProcess(bool *res, bool *Input, int n, int m, int size) {
	 // identifiant de thread ? deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;

	res[idx] = true;
	for (int k = -size; k < size+1; k++) {
		for (int l = -size; l < size+1; l++) {
			if ((i+k) > n-1 || j+l > m-1 || (i+k) < 0 || j+l < 0)
				continue;

			if (!Input[(i+k)*m + j+l] ) {
				res[idx] = false;
				return;
			}
		}
	}	
	
}
__global__ void ImErodeKernel(bool *res, bool *Input, int n, int m, int size) {
	ImErodeProcess(res, Input, n, m, size);
}

__device__ __forceinline__ void VotePlanProcess(unsigned char *Label, float *VMap, float* centers, float *count, float *pose, int n, int m) {
	 // identifiant de thread ? deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;

	if (Label[idx] == 0)
		return;
	
	float pt_l [3];
	pt_l [0] = VMap[3*idx];
	pt_l [1] = VMap[3*idx+1];
	pt_l [2] = VMap[3*idx+2];

	float pt [3];
	pt [0] = pose[0]*pt_l[0] + pose[4]*pt_l[1] + pose[8]*pt_l[2] + pose[12];
	pt [1] = pose[1]*pt_l[0] + pose[5]*pt_l[1] + pose[9]*pt_l[2] + pose[13];
	pt [2] = pose[2]*pt_l[0] + pose[6]*pt_l[1] + pose[10]*pt_l[2] + pose[14];

	atomicAdd(&centers[3*(Label[idx]-1)], pt [0]);
	atomicAdd(&centers[3*(Label[idx]-1)+1], pt [1]);
	atomicAdd(&centers[3*(Label[idx]-1)+2], pt [2]);	
	atomicAdd(&count[Label[idx]-1], 1.0);
}
__global__ void VotePlanKernel(unsigned char *Label, float *VMap, float* centers, float *count, float *pose, int n, int m) {
	VotePlanProcess(Label, VMap, centers, count, pose, n, m);
}

__device__ __forceinline__ void AffectPlanProcess(unsigned char *Label, int *Buff, int n, int m) {
	 // identifiant de thread ? deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;

	if (Label[idx] == 0)
		return;

	Label[idx] = unsigned char(Buff[Label[idx]-1]);	
}
__global__ void AffectPlanKernel(unsigned char *Label, int *Buff, int n, int m) {
	AffectPlanProcess(Label, Buff, n, m);
}

__global__ void  AffectKernel(float **Tab, float *in, int indx){
	Tab[indx] = in;
}

__global__ void  AffectCharKernel(unsigned char **Tab, unsigned char *in, int indx){
	Tab[indx] = in;
}

__global__ void  AffectShortKernel(unsigned short **Tab, unsigned short *in, int indx){
	Tab[indx] = in;
}


///**** Function definitions ****/
vector<float *> DetectPlans_cu(float *VMap, float *NMap, int *Indices_Final_dev, float epsilon, float alpha, int n, int m) {

	int dimAzimut = int(360.0/alpha);
	int dimElev = int(180.0/alpha);

	int *Sphere = (int *) malloc(dimAzimut*dimElev*sizeof(int)); // 360/30 * 180/30
	float *Avg_NMLE = (float *) malloc(3*dimAzimut*dimElev*sizeof(float)); // 3*(360/30 * 180/30)
	int *Sphere_dev;
	float *Avg_NMLE_dev;
	
	checkCudaErrors( cudaMalloc((void **) &Sphere_dev, dimAzimut*dimElev*sizeof(int)) );
	checkCudaErrors( cudaMemset(Sphere_dev, 0, dimAzimut*dimElev*sizeof(int)) );
	checkCudaErrors( cudaMalloc((void **) &Avg_NMLE_dev, 3*dimAzimut*dimElev*sizeof(float)) );
	checkCudaErrors( cudaMemset(Avg_NMLE_dev, 0, 3*dimAzimut*dimElev*sizeof(float)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x); // !! ne pas inverser n et m !!
	dimGrid.y = divUp (m, dimBlock.y);

	VoteSphereKernel<<<dimGrid, dimBlock>>>(NMap, Sphere_dev, Avg_NMLE_dev, dimAzimut, dimElev, alpha, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );	

	checkCudaErrors( cudaMemcpy(Sphere, Sphere_dev, dimAzimut*dimElev*sizeof(int), cudaMemcpyDeviceToHost) );
	checkCudaErrors( cudaMemcpy(Avg_NMLE, Avg_NMLE_dev, 3*dimAzimut*dimElev*sizeof(float), cudaMemcpyDeviceToHost) );
	
	int count_pt;
	vector<float *> NMLES_buff;
	vector<int> NMLES_count_buff;
	for (int i = 0; i < dimAzimut*dimElev; i++) {
		count_pt = Sphere[i];

		if (count_pt > 1000) {
			float *nmletmp  = (float *) malloc(3*sizeof(float));
			nmletmp[0] = Avg_NMLE[3*i]/float(count_pt);
			nmletmp[1] = Avg_NMLE[3*i+1]/float(count_pt);
			nmletmp[2] = Avg_NMLE[3*i+2]/float(count_pt);
			float norm = sqrt(nmletmp[0]*nmletmp[0] + nmletmp[1]*nmletmp[1] + nmletmp[2]*nmletmp[2]);
			nmletmp[0] = nmletmp[0]/norm;
			nmletmp[1] = nmletmp[1]/norm;
			nmletmp[2] = nmletmp[2]/norm;

			NMLES_buff.push_back(nmletmp);
			NMLES_count_buff.push_back(count_pt);
		}		
	}

	int nbPlan = NMLES_buff.size();	

	///// Merge close enough clusters
	// build equivalence matrix;
	bool *equivalences = (bool *) malloc(nbPlan*nbPlan);
	memset(equivalences, 0, nbPlan*nbPlan*sizeof(bool));

	float error_alpha;
	for (int i = 0; i < nbPlan; i++) {
		float *nmlecurr = NMLES_buff[i];
		equivalences[nbPlan*i + i] = true;
		for (int j = i+1; j < nbPlan; j++) {
			float *nmletmp = NMLES_buff[j];
			error_alpha = fabs(nmlecurr[0]*nmletmp[0] + nmlecurr[1]*nmletmp[1] + nmlecurr[2]*nmletmp[2]);
			if (error_alpha > cos(10.0*PI/180.0)) {
				equivalences[nbPlan*i + j] = true;
				equivalences[nbPlan*j + i] = true;
			}
		}
	}

	// Transitive closure by Floyd-Warshall algorithm
	for (int i = 0; i < nbPlan; i++) {
		for (int j = 0; j < nbPlan; j++) {
			if (equivalences[nbPlan*i + j]) {
				for (int k = 0; k < nbPlan; k++) {
					equivalences[nbPlan*i + k] = equivalences[nbPlan*i + k] || equivalences[nbPlan*j + k];
				}
			}
		}
	}

	vector<float *> NMLES;
	vector<int> NMLES_count;
	for (int i = 0; i < nbPlan; i++) {
		if (equivalences[nbPlan*i + i]) {
			float *nmlecurr  = (float *) malloc(3*sizeof(float));
			nmlecurr[0] = nmlecurr[1] = nmlecurr[2] = 0.0;
			//int count_nmle = 0;
			int count = 0;
			for (int j = 0; j < nbPlan; j++) {
				if (equivalences[nbPlan*j + i]) {
					float *nmletmp = NMLES_buff[j];
					nmlecurr[0] = nmlecurr[0] + float(NMLES_count_buff[j])*nmletmp[0];
					nmlecurr[1] = nmlecurr[1] + float(NMLES_count_buff[j])*nmletmp[1];
					nmlecurr[2] = nmlecurr[2] + float(NMLES_count_buff[j])*nmletmp[2];
					//count_nmle ++;
					equivalences[nbPlan*j + j] = false;
					count += NMLES_count_buff[j];
				}
			}
			if (count < 3000) {
				free(nmlecurr);
				continue;
			}

			nmlecurr[0] = nmlecurr[0]/float(count);
			nmlecurr[1] = nmlecurr[1]/float(count);
			nmlecurr[2] = nmlecurr[2]/float(count);
			float norm = sqrt(nmlecurr[0]*nmlecurr[0] + nmlecurr[1]*nmlecurr[1] + nmlecurr[2]*nmlecurr[2]);
			nmlecurr[0] = nmlecurr[0]/norm;
			nmlecurr[1] = nmlecurr[1]/norm;
			nmlecurr[2] = nmlecurr[2]/norm;

			NMLES.push_back(nmlecurr);
			NMLES_count.push_back(count);
		}
	}
	
	for (vector<float *>::iterator it = NMLES_buff.begin(); it != NMLES_buff.end(); it++)
		free((*it));

	NMLES_buff.clear();
	NMLES_count_buff.clear();

	nbPlan = NMLES.size();

	int *Indices_dev;
	checkCudaErrors( cudaMalloc((void **) &Indices_dev,n*m*sizeof(int)) );
	checkCudaErrors( cudaMemset(Indices_dev, 0, n*m*sizeof(int)) );	
	
	float *Plan = (float *) malloc(3*nbPlan*sizeof(float));

	int nbPt = 0;
	for (int i = 0; i < nbPlan; i++) {
		Plan[3*i] = NMLES[i][0];
		Plan[3*i+1] = NMLES[i][1];
		Plan[3*i+2] = NMLES[i][2];
		nbPt += NMLES_count[i];
	}

	float *Plan_dev;
	checkCudaErrors( cudaMalloc((void **) &Plan_dev,3*nbPlan*sizeof(float)) );
	checkCudaErrors( cudaMemcpy(Plan_dev, Plan, 3*nbPlan*sizeof(float), cudaMemcpyHostToDevice) );
	
	GetIndicesKernel<<<dimGrid, dimBlock>>>(NMap, Indices_dev, Plan_dev, 0.8f*alpha*PI/180.0f, nbPlan, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );		

	//////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////// Cluster in distance //////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
		
	int dim = 2*(int(10.0/epsilon)+1);
	float *Space = (float *) malloc (dim*sizeof(float));
	int *Space_count = (int *) malloc (dim*sizeof(int));
	
	int *Space_count_dev;
	float *Space_dev;
	float *curr_nmle_dev;
	checkCudaErrors( cudaMalloc((void **) &Space_count_dev, dim*sizeof(int)) );
	checkCudaErrors( cudaMalloc((void **) &Space_dev, dim*sizeof(float)) );
	checkCudaErrors( cudaMalloc((void **) &curr_nmle_dev, 3*sizeof(float)) );
	
	vector<float *> EQUA_buff;
	int shift_eq = 0;

	for (int i = 0; i < nbPlan; i++) {
		
		checkCudaErrors( cudaMemcpy(curr_nmle_dev, NMLES[i], 3*sizeof(float), cudaMemcpyHostToDevice) );
		checkCudaErrors( cudaMemset(Space_count_dev, 0, dim*sizeof(int)) );
		checkCudaErrors( cudaMemset(Space_dev, 0, dim*sizeof(float)) );

		VoteDistanceKernel<<<dimGrid, dimBlock>>>(VMap, Indices_dev, curr_nmle_dev, Space_count_dev, Space_dev, dim, epsilon, i, n, m);
		checkCudaErrors( cudaDeviceSynchronize() );	

		checkCudaErrors( cudaMemcpy(Space_count, Space_count_dev, dim*sizeof(int), cudaMemcpyDeviceToHost) );
		checkCudaErrors( cudaMemcpy(Space, Space_dev, dim*sizeof(float), cudaMemcpyDeviceToHost) );
		
		vector<float *> EQUA_tmp;
		vector<int> EQUA_count;

		for (int j = 0; j < dim; j++) {
			count_pt = Space_count [j];

			if (count_pt > 1000) {
				float *equa = (float *) malloc(4*sizeof(float));
				equa[0] = NMLES[i][0];
				equa[1] = NMLES[i][1];
				equa[2] = NMLES[i][2];
				equa[3] = Space[j]/float(count_pt);

				EQUA_tmp.push_back(equa);
				EQUA_count.push_back(count_pt);
			}
			
		}

		int nb_subPlan = EQUA_tmp.size();

		///// Merge close enough clusters
		// build equivalence matrix;
		bool *equivalences_dist = (bool *) malloc(nb_subPlan*nb_subPlan);
		memset(equivalences_dist, 0, nb_subPlan*nb_subPlan*sizeof(bool));

		float error_alpha;
		for (int l = 0; l < nb_subPlan; l++) {
			float *equa = EQUA_tmp[l];
			equivalences_dist[nb_subPlan*l + l] = true;
			for (int j = l+1; j < nb_subPlan; j++) {
				float *equatmp = EQUA_tmp[j];
				error_alpha = fabs(equa[3] - equatmp[3]);
				if (error_alpha < 4.0*epsilon) {
					equivalences_dist[nb_subPlan*l + j] = true;
					equivalences_dist[nb_subPlan*j + l] = true;
				}
			}
		}

		// Transitive closure by Floyd-Warshall algorithm
		for (int l = 0; l < nb_subPlan; l++) {
			for (int j = 0; j < nb_subPlan; j++) {
				if (equivalences_dist[nb_subPlan*l + j]) {
					for (int k = 0; k < nb_subPlan; k++) {
						equivalences_dist[nb_subPlan*l + k] = equivalences_dist[nb_subPlan*l + k] || equivalences_dist[nb_subPlan*j + k];
					}
				}
			}
		}

		vector<float *> EQUA_LOC;
		for (int l = 0; l < nb_subPlan; l++) {
			if (equivalences_dist[nb_subPlan*l + l]) {
				float *equacurr  = (float *) malloc(4*sizeof(float));
				equacurr[0] = NMLES[i][0];
				equacurr[1] = NMLES[i][1];
				equacurr[2] = NMLES[i][2];
				equacurr[3] = 0.0; //NMLES[i][3]; //0.0;
				//int count_nmle = 0;
				int count = 0;
				for (int j = 0; j < nb_subPlan; j++) {
					if (equivalences_dist[nb_subPlan*j + l]) {
						float *equatmp = EQUA_tmp[j];
						equacurr[3] = equacurr[3] + float(EQUA_count[j])*equatmp[3];
						//count_nmle ++;
						equivalences_dist[nb_subPlan*j + j] = false;
						count += EQUA_count[j];
					}
				}
				if (count < 1000) {
					free(equacurr);
					continue;
				}
				equacurr[3] = equacurr[3]/float(count);

				EQUA_LOC.push_back(equacurr);
				EQUA_buff.push_back(equacurr);
			}
		}

		for (vector<float *>::iterator it = EQUA_tmp.begin(); it != EQUA_tmp.end(); it++) {
			free((*it));
		}

		EQUA_tmp.clear();
		EQUA_count.clear();
		free(equivalences_dist);

		nb_subPlan = EQUA_LOC.size();

		float *Equations = (float *) malloc(4*nb_subPlan*sizeof(float));

		for (int j = 0; j < nb_subPlan; j++) {
			Equations[4*j] = EQUA_LOC[j][0];
			Equations[4*j+1] = EQUA_LOC[j][1];
			Equations[4*j+2] = EQUA_LOC[j][2];
			Equations[4*j+3] = EQUA_LOC[j][3];
		}

		float *Equations_dev;
		checkCudaErrors( cudaMalloc((void **) &Equations_dev,4*nb_subPlan*sizeof(float)) );
		checkCudaErrors( cudaMemcpy(Equations_dev, Equations, 4*nb_subPlan*sizeof(float), cudaMemcpyHostToDevice) );
	
		cudaDeviceSynchronize();
		GetIndicesDistancesKernel<<<dimGrid, dimBlock>>>(VMap, Indices_Final_dev, Indices_dev, Equations_dev, 5.0*epsilon, nb_subPlan, shift_eq, i, n, m);
		checkCudaErrors( cudaDeviceSynchronize() );

		checkCudaErrors( cudaFree(Equations_dev) );
		free(Equations);

		shift_eq += nb_subPlan;

		EQUA_LOC.clear();
	}

	nbPlan = EQUA_buff.size();
		
	for (vector<float *>::iterator it = NMLES.begin(); it != NMLES.end(); it++)
		free(*it);

	NMLES_buff.clear();
	
	checkCudaErrors( cudaFree(Plan_dev) );
	checkCudaErrors( cudaFree(Indices_dev) );
	checkCudaErrors( cudaFree(Sphere_dev) );
	checkCudaErrors( cudaFree(Avg_NMLE_dev) );
	checkCudaErrors( cudaFree(Space_count_dev) );
	checkCudaErrors( cudaFree(Space_dev) );
	checkCudaErrors( cudaFree(curr_nmle_dev) );
	
	free(Sphere);
	free(Avg_NMLE);
	free(Plan);
	free(equivalences);
	free(Space);
	free(Space_count);

	return EQUA_buff;
}

void Project_on_primitives_cu(float *Projected_dev, int *BBox, float *VMap, float *NMap, float *RGB, int *Indices_Final_dev, float *Equations_dev, int nbPlans, float res, float epsilon, float alpha, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x); // !! ne pas inverser n et m !!
	dimGrid.y = divUp (m, dimBlock.y);

	ProjectionKernel<<<dimGrid, dimBlock>>>(Projected_dev, BBox, VMap, NMap, RGB, Indices_Final_dev, Equations_dev, nbPlans, res, epsilon, alpha, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );	

	return;
}

void Segment_cu(unsigned char *Label, float *VMap, float *NMap, float *pose, float *Equations, int nbPlans, float epsilon, float alpha, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x); // !! ne pas inverser n et m !!
	dimGrid.y = divUp (m, dimBlock.y);

	float *pose_dev;
	checkCudaErrors( cudaMalloc((void **) &pose_dev, 16*sizeof(float)) );
	checkCudaErrors( cudaMemcpy(pose_dev, pose,  16 * sizeof(float), cudaMemcpyHostToDevice) );

	for (int i = 0; i < 3; i++)
		SegmentKernel<<<dimGrid, dimBlock>>>(Label, VMap, NMap, pose_dev, Equations, nbPlans, epsilon, alpha, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );	

	checkCudaErrors( cudaFree(pose_dev) );	
}

void InitFrags_cu(float *Projected_dev, int *Indices, int *Size, float *center, unsigned short *TheBumps_dev, unsigned char *TheRGBs_dev, unsigned char *TheMasks_dev, float *equation, int j, float res, int n, int m) {

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x); // !! ne pas inverser n et m !!
	dimGrid.y = divUp (m, dimBlock.y);

	InitFragsKernel<<<dimGrid, dimBlock>>>(Projected_dev, Indices, Size, center, TheBumps_dev, TheRGBs_dev, TheMasks_dev, equation, j, res, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );	

	return;
}

void init_val(unsigned char *InOutput, unsigned char val, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	InitValKernel<<<dimGrid, dimBlock>>>(InOutput, val, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return;
}

bool *ImDilate(bool *Im, int n, int m, int size) {
	bool *res_dev;

	checkCudaErrors( cudaMalloc((void **) &res_dev, n*m*sizeof(bool)) );
	checkCudaErrors( cudaMemset(res_dev,0,n*m*sizeof(bool)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

    ImDilateKernel<<<dimGrid, dimBlock>>>(res_dev, Im, n, m, size);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return res_dev;
}

bool *ImDilate(unsigned char *Im, int n, int m, int size) {
	bool *res_dev;

	checkCudaErrors( cudaMalloc((void **) &res_dev, n*m*sizeof(bool)) );
	checkCudaErrors( cudaMemset(res_dev,0,n*m*sizeof(bool)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

    ImDilateKernel<<<dimGrid, dimBlock>>>(res_dev, Im, n, m, size);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return res_dev;
}

bool *ImErode(bool *Im, int n, int m, int size) {
	bool *res_dev;

	checkCudaErrors( cudaMalloc((void **) &res_dev, n*m*sizeof(bool)) );
	checkCudaErrors( cudaMemset(res_dev,0,n*m*sizeof(bool)) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	ImErodeKernel<<<dimGrid, dimBlock>>>(res_dev, Im, n, m, size);
	
	checkCudaErrors( cudaDeviceSynchronize() );

	return res_dev;
	
}

void VotePlan_cu(unsigned char *Label, float *VMap, float *centers, float *count, float *pose, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
	float *pose_dev;
	checkCudaErrors( cudaMalloc((void **) &pose_dev, 16*sizeof(float)) );
	checkCudaErrors( cudaMemcpy(pose_dev, pose,  16 * sizeof(float), cudaMemcpyHostToDevice) );

	VotePlanKernel<<<dimGrid, dimBlock>>>(Label, VMap, centers, count, pose_dev, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	checkCudaErrors( cudaFree(pose_dev) );	
}

void AffectPlan_cu(unsigned char *Label, int *Buff, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	AffectPlanKernel<<<dimGrid, dimBlock>>>(Label, Buff, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
}

void Affect(float **Tab, float *in, int indx) {
	dim3 dimBlock(1, 1);
	dim3 dimGrid (1, 1, 1);

	AffectKernel<<<dimGrid, dimBlock>>>(Tab, in, indx);
	checkCudaErrors( cudaDeviceSynchronize() );
}

void AffectChar(unsigned char **Tab, unsigned char *in, int indx) {
	dim3 dimBlock(1, 1);
	dim3 dimGrid (1, 1, 1);

	AffectCharKernel<<<dimGrid, dimBlock>>>(Tab, in, indx);
	checkCudaErrors( cudaDeviceSynchronize() );
}

void AffectShort(unsigned short **Tab, unsigned short *in, int indx) {
	dim3 dimBlock(1, 1);
	dim3 dimGrid (1, 1, 1);

	AffectShortKernel<<<dimGrid, dimBlock>>>(Tab, in, indx);
	checkCudaErrors( cudaDeviceSynchronize() );
}