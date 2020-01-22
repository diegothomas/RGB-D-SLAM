#include "stdafx.h"
#include "Utilities.h"


void 
quaternion2matrix(double *q, double m[3][3])
{
  double q00 = q[0]*q[0];
  double q11 = q[1]*q[1];
  double q22 = q[2]*q[2];
  double q33 = q[3]*q[3];
  double q03 = q[0]*q[3];
  double q13 = q[1]*q[3];
  double q23 = q[2]*q[3];
  double q02 = q[0]*q[2];
  double q12 = q[1]*q[2];
  double q01 = q[0]*q[1];
  m[0][0] = q00 + q11 - q22 - q33;
  m[1][1] = q00 - q11 + q22 - q33;
  m[2][2] = q00 - q11 - q22 + q33;
  m[0][1] = 2.0*(q12-q03);
  m[1][0] = 2.0*(q12+q03);
  m[0][2] = 2.0*(q13+q02);
  m[2][0] = 2.0*(q13-q02);
  m[1][2] = 2.0*(q23-q01);
  m[2][1] = 2.0*(q23+q01);
}

bool CheckOrthoBasis(float * e1, float *e2, float *e3) {

	float prod_scal = e1[0]*e2[0] + e1[1]*e2[1] + e1[2]*e2[2];
	if (fabs(prod_scal) > 1.0e-6) {
		cout << "basis not orthogonal (e1,e2)" << endl;
		return false;
	}

	prod_scal = e1[0]*e3[0] + e1[1]*e3[1] + e1[2]*e3[2];
	if (fabs(prod_scal) > 1.0e-6) {
		cout << "basis not orthogonal (e1,e3)" << endl;
		return false;
	}

	prod_scal = e3[0]*e2[0] + e3[1]*e2[1] + e3[2]*e2[2];
	if (fabs(prod_scal) > 1.0e-6) {
		cout << "basis not orthogonal (e3,e2)" << endl;
		return false;
	}

	prod_scal = e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2];
	if (fabs(prod_scal-1.0) > 1.0e-6) {
		cout << "basis not orthonormal (e1)" << endl;
		return false;
	}

	prod_scal = e2[0]*e2[0] + e2[1]*e2[1] + e2[2]*e2[2];
	if (fabs(prod_scal-1.0) > 1.0e-6) {
		cout << "basis not orthonormal (e2)" << endl;
		return false;
	}

	prod_scal = e3[0]*e3[0] + e3[1]*e3[1] + e3[2]*e3[2];
	if (fabs(prod_scal-1.0) > 1.0e-6) {
		cout << "basis not orthonormal (e3)" << endl;
		return false;
	}

	return true;
}

void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs)
{
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1); // weird it doesn't support CV_32S!

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < binary.rows; y++) {
        for(int x=0; x < binary.cols; x++) {
            if((int)label_image.at<float>(y,x) != 1) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if((int)label_image.at<float>(i,j) != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }
}

bool Estimate_transfo(float *points, Eigen::Matrix3f poseR, Eigen::Vector3f poset, float *Rotation, float *translation, int  nb_match) {
	
	int max_iter_inner = 100;
	float delta = 0.0001;
	bool inner_cvg = false;
	int inner_iter = 0;
	float lambda = 100.0;
	float v = 2.0;//1.5;
	Eigen::Matrix3f Rinc;
	Eigen::Vector3f tinc;
	Eigen::Matrix3f LMRtmp_Rx;
	Eigen::Vector3f LMttmp_Rx; 
	Eigen::Matrix3f LMRtmp_Ry;
	Eigen::Vector3f LMttmp_Ry; 
	Eigen::Matrix3f LMRtmp_Rz;
	Eigen::Vector3f LMttmp_Rz; 
	Eigen::Matrix3f LMRtmp_Tx;
	Eigen::Vector3f LMttmp_Tx; 
	Eigen::Matrix3f LMRtmp_Ty;
	Eigen::Vector3f LMttmp_Ty; 
	Eigen::Matrix3f LMRtmp_Tz;
	Eigen::Vector3f LMttmp_Tz; 

	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
	Eigen::Matrix<float, 6, 1> b;
				
	float *Jac_x = (float *) malloc(6*nb_match*sizeof(float));
	float *Jac_y = (float *) malloc(6*nb_match*sizeof(float));
	float *Jac_z = (float *) malloc(6*nb_match*sizeof(float));
			
	float *Right_x = (float *) malloc(nb_match*sizeof(float));
	float *Right_y = (float *) malloc(nb_match*sizeof(float));
	float *Right_z = (float *) malloc(nb_match*sizeof(float));
		
	Eigen::Matrix3f Rtmp = poseR;
	Eigen::Vector3f ttmp = poset;
	while (!inner_cvg) {
				
		Rinc = (Eigen::Matrix3f) Eigen::AngleAxisf (10.0*delta, Eigen::Vector3f::UnitX ());
		tinc = Eigen::Vector3f::Zero();
		LMttmp_Rx = Rinc * ttmp + tinc;
		LMRtmp_Rx = Rinc * Rtmp;

		Rinc = (Eigen::Matrix3f) Eigen::AngleAxisf (10.0*delta, Eigen::Vector3f::UnitY ());
		LMttmp_Ry = Rinc * ttmp + tinc;
		LMRtmp_Ry = Rinc * Rtmp;
				
		Rinc = (Eigen::Matrix3f) Eigen::AngleAxisf (10.0*delta, Eigen::Vector3f::UnitZ ());
		LMttmp_Rz = Rinc * ttmp + tinc;
		LMRtmp_Rz = Rinc * Rtmp;
				
		Rinc = Eigen::Matrix3f::Identity();
		tinc(0) = delta;
		LMttmp_Tx = ttmp + tinc;
		LMRtmp_Tx = Rtmp;

		tinc = Eigen::Vector3f::Zero();
		tinc(1) = delta;
		LMttmp_Ty = ttmp + tinc;
		LMRtmp_Ty = Rtmp;

		tinc = Eigen::Vector3f::Zero();
		tinc(2) = delta;
		LMttmp_Tz = ttmp + tinc;
		LMRtmp_Tz = Rtmp;
				
		// The small transfo should be the inverse
			
		for (int j = 0; j < nb_match; j++) {
				
			float p_curr_cp [3];
			p_curr_cp[0] = Rtmp(0,0)*points[6*j] + Rtmp(0,1)*points[6*j+1] + Rtmp(0,2)*points[6*j+2] + ttmp(0);
			p_curr_cp[1] = Rtmp(1,0)*points[6*j] + Rtmp(1,1)*points[6*j+1] + Rtmp(1,2)*points[6*j+2] + ttmp(1);
			p_curr_cp[2] = Rtmp(2,0)*points[6*j] + Rtmp(2,1)*points[6*j+1] + Rtmp(2,2)*points[6*j+2] + ttmp(2);
				
			float p_prev [3];
			p_prev[0] = points[6*j+3];
			p_prev[1] = points[6*j+4];
			p_prev[2] = points[6*j+5];
												
			Right_x[j] = p_prev[0] - p_curr_cp[0];
			Right_y[j] = p_prev[1] - p_curr_cp[1];
			Right_z[j] = p_prev[2] - p_curr_cp[2];

			// Jacobian in rotX
			float p_tmp_cp [3];
			p_tmp_cp[0] = LMRtmp_Rx(0,0)*points[6*j] + LMRtmp_Rx(0,1)*points[6*j+1] + LMRtmp_Rx(0,2)*points[6*j+2] + LMttmp_Rx(0);
			p_tmp_cp[1] = LMRtmp_Rx(1,0)*points[6*j] + LMRtmp_Rx(1,1)*points[6*j+1] + LMRtmp_Rx(1,2)*points[6*j+2] + LMttmp_Rx(1);
			p_tmp_cp[2] = LMRtmp_Rx(2,0)*points[6*j] + LMRtmp_Rx(2,1)*points[6*j+1] + LMRtmp_Rx(2,2)*points[6*j+2] + LMttmp_Rx(2);

			Jac_x[0*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/(10.0*delta);
			Jac_y[0*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/(10.0*delta);
			Jac_z[0*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/(10.0*delta);

			// Jacobian in rotY
			p_tmp_cp[0] = LMRtmp_Ry(0,0)*points[6*j] + LMRtmp_Ry(0,1)*points[6*j+1] + LMRtmp_Ry(0,2)*points[6*j+2] + LMttmp_Ry(0);
			p_tmp_cp[1] = LMRtmp_Ry(1,0)*points[6*j] + LMRtmp_Ry(1,1)*points[6*j+1] + LMRtmp_Ry(1,2)*points[6*j+2] + LMttmp_Ry(1);
			p_tmp_cp[2] = LMRtmp_Ry(2,0)*points[6*j] + LMRtmp_Ry(2,1)*points[6*j+1] + LMRtmp_Ry(2,2)*points[6*j+2] + LMttmp_Ry(2);

			Jac_x[1*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/(10.0*delta);
			Jac_y[1*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/(10.0*delta);
			Jac_z[1*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/(10.0*delta);
				
			// Jacobian in rotZ
			p_tmp_cp[0] = LMRtmp_Rz(0,0)*points[6*j] + LMRtmp_Rz(0,1)*points[6*j+1] + LMRtmp_Rz(0,2)*points[6*j+2] + LMttmp_Rz(0);
			p_tmp_cp[1] = LMRtmp_Rz(1,0)*points[6*j] + LMRtmp_Rz(1,1)*points[6*j+1] + LMRtmp_Rz(1,2)*points[6*j+2] + LMttmp_Rz(1);
			p_tmp_cp[2] = LMRtmp_Rz(2,0)*points[6*j] + LMRtmp_Rz(2,1)*points[6*j+1] + LMRtmp_Rz(2,2)*points[6*j+2] + LMttmp_Rz(2);

			Jac_x[2*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/(10.0*delta);
			Jac_y[2*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/(10.0*delta);
			Jac_z[2*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/(10.0*delta);
				
			// Jacobian in transX
			p_tmp_cp[0] = LMRtmp_Tx(0,0)*points[6*j] + LMRtmp_Tx(0,1)*points[6*j+1] + LMRtmp_Tx(0,2)*points[6*j+2] + LMttmp_Tx(0);
			p_tmp_cp[1] = LMRtmp_Tx(1,0)*points[6*j] + LMRtmp_Tx(1,1)*points[6*j+1] + LMRtmp_Tx(1,2)*points[6*j+2] + LMttmp_Tx(1);
			p_tmp_cp[2] = LMRtmp_Tx(2,0)*points[6*j] + LMRtmp_Tx(2,1)*points[6*j+1] + LMRtmp_Tx(2,2)*points[6*j+2] + LMttmp_Tx(2);

			Jac_x[3*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/delta;
			Jac_y[3*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/delta;
			Jac_z[3*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/delta;
				
			// Jacobian in transY
			p_tmp_cp[0] = LMRtmp_Ty(0,0)*points[6*j] + LMRtmp_Ty(0,1)*points[6*j+1] + LMRtmp_Ty(0,2)*points[6*j+2] + LMttmp_Ty(0);
			p_tmp_cp[1] = LMRtmp_Ty(1,0)*points[6*j] + LMRtmp_Ty(1,1)*points[6*j+1] + LMRtmp_Ty(1,2)*points[6*j+2] + LMttmp_Ty(1);
			p_tmp_cp[2] = LMRtmp_Ty(2,0)*points[6*j] + LMRtmp_Ty(2,1)*points[6*j+1] + LMRtmp_Ty(2,2)*points[6*j+2] + LMttmp_Ty(2);

			Jac_x[4*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/delta;
			Jac_y[4*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/delta;
			Jac_z[4*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/delta;
				
			// Jacobian in transZ
			p_tmp_cp[0] = LMRtmp_Tz(0,0)*points[6*j] + LMRtmp_Tz(0,1)*points[6*j+1] + LMRtmp_Tz(0,2)*points[6*j+2] + LMttmp_Tz(0);
			p_tmp_cp[1] = LMRtmp_Tz(1,0)*points[6*j] + LMRtmp_Tz(1,1)*points[6*j+1] + LMRtmp_Tz(1,2)*points[6*j+2] + LMttmp_Tz(1);
			p_tmp_cp[2] = LMRtmp_Tz(2,0)*points[6*j] + LMRtmp_Tz(2,1)*points[6*j+1] + LMRtmp_Tz(2,2)*points[6*j+2] + LMttmp_Tz(2);

			Jac_x[5*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/delta;
			Jac_y[5*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/delta;
			Jac_z[5*nb_match+j] = (p_tmp_cp[2]-p_curr_cp [2])/delta;
				
		}

		// Compute J^tJ and J^tb
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				A(i,j) = 0.0;
				for (int k = 0; k < nb_match; k++) {
					A(i,j) = A(i,j) + Jac_x[i*nb_match+k]*Jac_x[j*nb_match+k] + Jac_y[i*nb_match+k]*Jac_y[j*nb_match+k] + Jac_z[i*nb_match+k]*Jac_z[j*nb_match+k];
				}
			}

			b(i) = 0.0;
			for (int k = 0; k < nb_match; k++) {
				b(i) = b(i) + Jac_x[i*nb_match+k]*Right_x[k] + Jac_y[i*nb_match+k]*Right_y[k] + Jac_z[i*nb_match+k]*Right_z[k];
			}
		}

		/*for (int i = 0; i < 6; i++) {
			A(i,i) = A(i,i) + lambda*A(i,i);
		}*/

		//checking nullspace
		float det = A.determinant ();

		if (fabs (det) < 1e-15 /*||  isnan (det)*/)
		{
			//if (isnan (det)) cout << "qnan" << endl;
			//cout << "det null" << endl;
			free(Jac_x);
			free(Jac_y);
			free(Jac_z);
			free(Right_x);
			free(Right_y);
			free(Right_z);
			return false;
		}

		Eigen::Matrix<float, 6, 1> result = A.inverse() * b; // A.llt ().solve (b).cast<float>();
		float alpha = result (0);
		float beta  = result (1);
		float gamma = result (2);
				
		Rinc = (Eigen::Matrix3f) Eigen::AngleAxisf (gamma, Eigen::Vector3f::UnitZ ()) * Eigen::AngleAxisf (beta, Eigen::Vector3f::UnitY ()) * Eigen::AngleAxisf (alpha, Eigen::Vector3f::UnitX ());
		tinc = result.tail<3> ();

		ttmp = Rinc * ttmp + tinc;
		Rtmp = Rinc * Rtmp;

		lambda = lambda/v;

		if (/*final && */(inner_iter > max_iter_inner || ((Rinc-Eigen::Matrix3f::Identity()).norm() < 1.0e-8 && tinc.norm() < 1.0e-8))) {
			inner_cvg = true;
		}

		inner_iter++;
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rotation[j*3+i] = Rtmp(i,j);
		}
		translation[i] = ttmp(i);
	}
			
	////////////////////////// END LEvenberg Marquart Algorithm /////////////////////////////////////////////
				
	free(Jac_x);
	free(Jac_y);
	free(Jac_z);
	free(Right_x);
	free(Right_y);
	free(Right_z);
	return true;
}