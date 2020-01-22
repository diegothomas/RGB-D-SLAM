#include "stdafx.h"
#include "utilities.h"

using namespace std;
using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix3f;

void matrix2quaternion(Eigen::Matrix3f currR, float *res) {
	float t = currR(0,0)+currR(1,1)+currR(2,2);
	if (currR(0,0) > currR(1,1) && currR(0,0) > currR(2,2)) {
		float r = sqrt(1+currR(0,0)-currR(1,1)-currR(2,2));
		float s = 0.5/r;
		float w = (currR(2,1)-currR(1,2))*s;
		float x = 0.5*r;
		float y = (currR(0,1)+currR(1,0))*s;
		float z = (currR(2,0)+currR(0,2))*s;
		
		res[0] = w;
		res[1] = x;
		res[2] = y;
		res[3] = z;
		return;
	}

	if (currR(1,1) > currR(0,0) && currR(1,1) > currR(2,2)) {
		float r = sqrt(1+currR(1,1)-currR(0,0)-currR(2,2));
		float s = 0.5/r;
		float w = (currR(2,0)-currR(0,2))*s;
		float x = (currR(0,1)+currR(1,0))*s;
		float y = 0.5*r;
		float z = (currR(2,1)+currR(1,2))*s;
		
		res[0] = w;
		res[1] = x;
		res[2] = y;
		res[3] = z;
		return;
	}

	if (currR(2,2) > currR(0,0) && currR(2,2) > currR(1,1)) {
		float r = sqrt(1+currR(2,2)-currR(0,0)-currR(1,1));
		float s = 0.5/r;
		float w = (currR(1,0)-currR(0,1))*s;
		float x = (currR(0,2)+currR(2,0))*s;
		float y = (currR(2,1)+currR(1,2))*s;
		float z = 0.5*r;
		
		res[0] = w;
		res[1] = x;
		res[2] = y;
		res[3] = z;
		return;
	}


	float qw = sqrt(1.0 + currR(0,0) + currR(1,1) + currR(2,2)) / 2.0;
    float qx = (currR(2,1) - currR(1,2))/( 4.0 * qw);
    float qy = (currR(0,2) - currR(2,0))/( 4.0 * qw);
    float qz = (currR(1,0) - currR(0,1))/( 4.0 * qw);

	res[0] = qw;
	res[1] = qx;
	res[2] = qy;
	res[3] = qz;
}

void quaternion2matrix(float *q, double *res) {

    double nq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (nq < 1.0e-6){
        res[0] = 1.0; res[4] = 0.0; res[8] = 0.0; res[12] = 0.0;
        res[1] = 0.0; res[5] = 1.0; res[9] = 0.0; res[13] = 0.0;
        res[2] = 0.0; res[6] = 0.0; res[10] = 1.0; res[14] = 0.0;
        res[3] = 0.0; res[7] = 0.0; res[11] = 0.0; res[15] = 1.0;
        return;
	}
    
	for (int i = 0; i < 4; i++)
		q[i] = q[i] * sqrt(2.0 / nq); 
	
	double q_tmp[16];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			q_tmp[4*i+j] = q[i]*q[j];
		}
	}
    
    res[0] = 1.0-q_tmp[1*4+1]-q_tmp[2*4+2];		res[4] = q_tmp[0*4+1]-q_tmp[2*4+3];			res[8] = q_tmp[0*4+2]+q_tmp[1*4+3];			res[12] = q[4];
    res[1] = q_tmp[0*4+1]+q_tmp[2*4+3];			res[5] = 1.0-q_tmp[0*4+0]-q_tmp[2*4+2];		res[9] = q_tmp[1*4+2]-q_tmp[0*4+3];			res[13] = q[5];
    res[2] = q_tmp[0*4+2]-q_tmp[1*4+3];			res[6] = q_tmp[1*4+2]+q_tmp[0*4+3];			res[10] = 1.0-q_tmp[0*4+0]-q_tmp[1*4+1];	res[14] = q[6];
    res[3] = 0.0;								res[7] = 0.0;								res[11] = 0.0;								res[15] = 1.0;
	
}

void quaternion2matrix(double *q, double *res) {

    double nq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (nq < 1.0e-6){
        res[0] = 1.0; res[4] = 0.0; res[8] = 0.0; res[12] = 0.0;
        res[1] = 0.0; res[5] = 1.0; res[9] = 0.0; res[13] = 0.0;
        res[2] = 0.0; res[6] = 0.0; res[10] = 1.0; res[14] = 0.0;
        res[3] = 0.0; res[7] = 0.0; res[11] = 0.0; res[15] = 1.0;
        return;
	}
    
	for (int i = 0; i < 4; i++)
		q[i] = q[i] * sqrt(2.0 / nq); 
	
	double q_tmp[16];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			q_tmp[4*i+j] = q[i]*q[j];
		}
	}
    
    res[0] = 1.0-q_tmp[1*4+1]-q_tmp[2*4+2];		res[4] = q_tmp[0*4+1]-q_tmp[2*4+3];			res[8] = q_tmp[0*4+2]+q_tmp[1*4+3];			res[12] = q[4];
    res[1] = q_tmp[0*4+1]+q_tmp[2*4+3];			res[5] = 1.0-q_tmp[0*4+0]-q_tmp[2*4+2];		res[9] = q_tmp[1*4+2]-q_tmp[0*4+3];			res[13] = q[5];
    res[2] = q_tmp[0*4+2]-q_tmp[1*4+3];			res[6] = q_tmp[1*4+2]+q_tmp[0*4+3];			res[10] = 1.0-q_tmp[0*4+0]-q_tmp[1*4+1];	res[14] = q[6];
    res[3] = 0.0;								res[7] = 0.0;								res[11] = 0.0;								res[15] = 1.0;
	
}


void AlignReorganize(vector<double *> pose) {
	Eigen::Matrix3f Rot1;
	Eigen::Vector3f Trans1;

	double *curr = pose[0];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rot1(i,j) = float(curr[i+4*j]);
		}
		Trans1(i) = float(curr[i+12]);
	}

	Eigen::Matrix3f Rotinv = Rot1.inverse();
	Trans1 = - Rotinv * Trans1;

	Eigen::Matrix3f RotCurr;
	Eigen::Vector3f TransCurr;
	for (vector<double *>::iterator it = pose.begin(); it != pose.end(); it++) {
		curr = (*it);

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				RotCurr(i,j) = curr[i+4*j];
			}
			TransCurr(i) = curr[i+12];
		}

		RotCurr = Rotinv * RotCurr;
		TransCurr = Rotinv*TransCurr + Trans1;
		
		Eigen::Matrix3f Rottmp = RotCurr.inverse();
		TransCurr = - Rottmp * TransCurr;

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				curr[i+4*j] = Rottmp(i,j);
			}
			curr[i+12] = TransCurr(i);
		}
	}


}

void LoadTimeStamps(vector<string> *timestamp_depth, vector<string> *timestamp_color, char *path) {
	char tline[256];
	char str [256];
	int tmp1, tmp2;
	vector<double> timestamp_depth_tmp;
	vector<double> timestamp_color_tmp;
	vector<string> timestamp_depth_cpy;
	vector<string> timestamp_color_cpy;

	ifstream  filestr;

	filestr.open (string(path)+"//depth.txt", ifstream ::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open depth files timestamps" << endl;
		return;
	}
	
	// Read header
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	cout << tline << endl;
	filestr.getline (tline,256);
	
	// Read data
	filestr.getline (tline,256);
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %s", &tmp1, &tmp2, str);
		timestamp_depth_tmp.push_back(double(tmp1) + double(tmp2)/1000000.0);
		timestamp_depth_cpy.push_back(string(str));
		filestr.getline (tline,256);
	}

	filestr.close();

	// Same for color timestamp
	filestr.open (string(path)+"//rgb.txt", ifstream::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open rgb files timestamps" << endl;
		return;
	}

	// Read header
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	
	// Read data
	filestr.getline (tline,256);
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %s", &tmp1, &tmp2, str);
		timestamp_color_tmp.push_back(double(tmp1) + double(tmp2)/1000000.0);
		timestamp_color_cpy.push_back(string(str));
		filestr.getline (tline,256);
	}

	filestr.close();

	// Match all timestamps together.
	
	double val, val2, min_val;
	double max_difference_depth = 0.02;
	int k, position;
	int indx_color = -1;

	for (vector<double>::iterator it = timestamp_color_tmp.begin(); it != timestamp_color_tmp.end(); it++) {
		val = (*it);
		indx_color++;
		val = timestamp_color_tmp[indx_color];

		min_val = max_difference_depth;
		k = 0;
		position = 0;
		for (vector<double>::iterator it2 = timestamp_depth_tmp.begin(); it2 != timestamp_depth_tmp.end(); it2++) {
			val2 = (*it2);

			if ( val2 > (val + max_difference_depth) )
				break;

			if (fabs(val-val2) < min_val && fabs(val-val2) < max_difference_depth) {
				min_val = fabs(val-val2);
				position = k;
			}
			k++;
		}

		if (min_val == max_difference_depth)
			continue;

		timestamp_depth->push_back(timestamp_depth_cpy[position]);
		timestamp_color->push_back(timestamp_color_cpy[indx_color]);

		it = timestamp_color_tmp.erase(timestamp_color_tmp.begin() + indx_color);
		timestamp_depth_tmp.erase(timestamp_depth_tmp.begin() + position);
		timestamp_color_cpy.erase(timestamp_color_cpy.begin() + indx_color);
		timestamp_depth_cpy.erase(timestamp_depth_cpy.begin() + position);
		indx_color --;

	}
	
	timestamp_depth_tmp.clear();
	timestamp_color_tmp.clear();
	timestamp_depth_cpy.clear();
	timestamp_color_cpy.clear();
}

void LoadAssociatedTimeStamps(vector<string> *timestamp_depth, vector<string> *timestamp_color, char *path) {
	char tline[256];
	char str1 [256];
	char str2 [256];
	int tmp1, tmp2, tmp3, tmp4;

	ifstream  filestr;

	filestr.open (string(path)+"//associated.txt", ifstream ::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open depth files timestamps" << endl;
		return;
	}

	// Read data
	filestr.getline (tline,256);
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %s %d.%d %s", &tmp1, &tmp2, str1, &tmp3, &tmp4, str2);
		timestamp_color->push_back(string(str1));
		timestamp_depth->push_back(string(str2));
		filestr.getline (tline,256);
	}

	filestr.close();

}

void LoadGroundTruth(vector<string> *timestamp_depth, vector<string> *timestamp_color, vector<double *> *timestamp_pose, char *path) {
	char tline[256];
	char str [256];
	int tmp1, tmp2;
	vector<double> timestamp_depth_tmp;
	vector<double> timestamp_color_tmp;
	vector<double> timestamp_pose_tmp;
	vector<string> timestamp_depth_cpy;
	vector<string> timestamp_color_cpy;
	vector<double *> timestamp_pose_cpy;

	ifstream  filestr;

	filestr.open (string(path)+"//depth.txt", ifstream ::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open depth files timestamps" << endl;
		return;
	}
	
	// Read header
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	cout << tline << endl;
	filestr.getline (tline,256);
	
	// Read data
	filestr.getline (tline,256);
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %s", &tmp1, &tmp2, str);
		timestamp_depth_tmp.push_back(double(tmp1) + double(tmp2)/1000000.0);
		timestamp_depth_cpy.push_back(string(str));
		filestr.getline (tline,256);
	}

	filestr.close();

	// Same for color timestamp
	filestr.open (string(path)+"//rgb.txt", ifstream::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open rgb files timestamps" << endl;
		return;
	}

	// Read header
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	
	// Read data
	filestr.getline (tline,256);
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %s", &tmp1, &tmp2, str);
		timestamp_color_tmp.push_back(double(tmp1) + double(tmp2)/1000000.0);
		timestamp_color_cpy.push_back(string(str));
		filestr.getline (tline,256);
	}

	filestr.close();

	// Same for groundTruth timestamp
	filestr.open (string(path)+"//groundtruth.txt", ifstream::in);
	
	if (!filestr.is_open()) {
		cout << "Could not open groundtruth files timestamps" << endl;
		return;
	}

	// Read header
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	filestr.getline (tline,256);
	
	// Read data
	filestr.getline (tline,256);
	float quat_curr[7];
	while (!filestr.eof()) {
		sscanf(tline, "%d.%d %f %f %f %f %f %f %f", &tmp1, &tmp2, &quat_curr[4], &quat_curr[5], &quat_curr[6], &quat_curr[0], &quat_curr[1], &quat_curr[2], &quat_curr[3]);
		timestamp_pose_tmp.push_back(double(tmp1) + double(tmp2)/10000.0);
		double *curr_mat = new double [16];
		quaternion2matrix(quat_curr, curr_mat);
		timestamp_pose_cpy.push_back(curr_mat);
		filestr.getline (tline,256);
	}

	filestr.close();

	// Match all timestamps together.
	
	double val, val2, min_val;
	double max_difference_depth = 0.02;
	int k, position;
	int indx_color = -1;

	for (vector<double>::iterator it = timestamp_color_tmp.begin(); it != timestamp_color_tmp.end(); it++) {
		val = (*it);
		indx_color++;

		min_val = max_difference_depth;
		k = 0;
		position = 0;
		for (vector<double>::iterator it2 = timestamp_depth_tmp.begin(); it2 != timestamp_depth_tmp.end(); it2++) {
			val2 = (*it2);

			if ( val2 > (val + max_difference_depth) )
				break;

			if (fabs(val-val2) < min_val && fabs(val-val2) < max_difference_depth) {
				min_val = fabs(val-val2);
				position = k;
			}
			k++;
		}

		if (min_val == max_difference_depth)
			continue;

		min_val = max_difference_depth;
		k = 0;
		int position_pose = 0;
		for (vector<double>::iterator it2 = timestamp_pose_tmp.begin(); it2 != timestamp_pose_tmp.end(); it2++) {
			val2 = (*it2);

			if ( val2 > (val + max_difference_depth) )
				break;

			if (fabs(val-val2) < min_val && fabs(val-val2) < max_difference_depth) {
				min_val = fabs(val-val2);
				position_pose = k;
			}
			k++;
		}
		
		if (min_val == max_difference_depth)
			continue;

		timestamp_depth->push_back(timestamp_depth_cpy[position]);
		timestamp_color->push_back(timestamp_color_cpy[indx_color]);
		timestamp_pose->push_back(timestamp_pose_cpy[position_pose]);
	}
	
	timestamp_depth_tmp.clear();
	timestamp_color_tmp.clear();
	timestamp_depth_cpy.clear();
	timestamp_color_cpy.clear();
	timestamp_pose_tmp.clear();
	timestamp_pose_cpy.clear();

	AlignReorganize(*timestamp_pose);
}

void PrintImage(void *Image, int *size_I, int format, char *windowname) {

	//int i,j,k;
	//cv::Mat img(size_I[0], size_I[1], CV_16UC3);
	//for (i=0,k=size_I[0]-1;i<size_I[0];i++,k--) {
	//	for (j=0;j<size_I[1];j++) {
	//		switch (format) {
	//		case 0: // RGB Image
	//			img.at<cv::Vec3w>(k,j)[2] = (ushort) (65535.0/255.0) * ((unsigned char *) Image)[3*((int)i*size_I[1] + (int)j)];
	//			img.at<cv::Vec3w>(k,j)[1] = (ushort) (65535.0/255.0) * ((unsigned char *) Image)[3*((int)i*size_I[1] + (int)j)+1];
	//			img.at<cv::Vec3w>(k,j)[0] = (ushort) (65535.0/255.0) * ((unsigned char *) Image)[3*((int)i*size_I[1] + (int)j)+2];
	//			break;
	//		case 1: // Depth Image
	//			img.at<cv::Vec3w>(k,j)[2] = (ushort)  ((((short *) Image)[((int)i*size_I[1] + (int)j)]+3) * 1000);
	//			img.at<cv::Vec3w>(k,j)[1] = (ushort)  ((((short *) Image)[((int)i*size_I[1] + (int)j)]+3) * 1000);
	//			img.at<cv::Vec3w>(k,j)[0] = (ushort)  ((((short *) Image)[((int)i*size_I[1] + (int)j)]+3) * 1000);
	//			break;
	//		case 2: // Mask Image
	//			img.at<cv::Vec3w>(k,j)[2] = (ushort) 30000 * ((unsigned short *) Image)[((int)i*size_I[1] + (int)j)];
	//			img.at<cv::Vec3w>(k,j)[1] = (ushort) 30000 * ((unsigned short *) Image)[((int)i*size_I[1] + (int)j)];
	//			img.at<cv::Vec3w>(k,j)[0] = (ushort) 30000 * ((unsigned short *) Image)[((int)i*size_I[1] + (int)j)];
	//			break;
	//		case 3: // Bool Mask Image
	//			img.at<cv::Vec3w>(k,j)[2] = (ushort) 65535 * ((ushort) ((bool *) Image)[((int)i*size_I[1] + (int)j)]);
	//			img.at<cv::Vec3w>(k,j)[1] = (ushort) 65535 * ((ushort) ((bool *) Image)[((int)i*size_I[1] + (int)j)]);
	//			img.at<cv::Vec3w>(k,j)[0] = (ushort) 65535 * ((ushort) ((bool *) Image)[((int)i*size_I[1] + (int)j)]);
	//			break;
	//		case 4: // Float Mask Image
	//			img.at<cv::Vec3w>(k,j)[2] = (ushort) 65535 * (((float *) Image)[3*((int)i*size_I[1] + (int)j)]);
	//			img.at<cv::Vec3w>(k,j)[1] = (ushort) 65535 * (((float *) Image)[3*((int)i*size_I[1] + (int)j)+1]);
	//			img.at<cv::Vec3w>(k,j)[0] = (ushort) 65535 * (((float *) Image)[3*((int)i*size_I[1] + (int)j)+2]);
	//			break;
	//		default:
	//			return;
	//		}
	//	}
	//}
	//cv::namedWindow(windowname, CV_WINDOW_AUTOSIZE);
	//cv::imshow(windowname, img);
	//cv::imwrite("F:\\Projects\\RG\\RT-RG-CPU-V.5\\Res\\depth.png", img);
}

void SaveTrajectory(const char *filename, vector<Eigen::Matrix3f> PosesR, vector<Eigen::Vector3f> Posest, vector<string> timestamp) {
	//
    //  Open the input file in "read text" mode.
    //
	ofstream  filestr;

	filestr.open (filename, fstream::out);

	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}

	filestr << "# Etimated camera trajectory\n";

	Eigen::Matrix3f currR;
	Eigen::Vector3f currT;
	float q[4]; 
	string currTime;
	char tmp[18];

	vector<string>::iterator itTime = timestamp.begin();
	vector<Eigen::Matrix3f>::iterator itR = PosesR.begin();
	for (vector<Eigen::Vector3f>::iterator itT = Posest.begin(); itT != Posest.end(); itT++, itR++, itTime++) {
		currR = (*itR);
		currT = (*itT);
		currTime = (*itTime);

		if (currT(0) == 100.0)
			continue;

		Eigen::Matrix3f Rcurr_inv = currR; //currR;// * base_change; //currR.inverse();
	    Vector3f  tcurr_inv = currT ; //currT; //- Rcurr_inv * currT;

		int length = currTime.copy(tmp,17,6);
		tmp[length]='\0';

		float val = atof( currTime.c_str() );

		matrix2quaternion(Rcurr_inv, &q[0]);

		filestr << tmp << " " << tcurr_inv(0) << " " << tcurr_inv(1) << " " << tcurr_inv(2) <<
			" " << q[1] << " " << q[2] << " " << q[3] << " " << q[0] << endl;
	}
	
    filestr.close();
}

//Computes connected components with 8-neigborhood
int ConnectedComponents(bool *Input, int *BBox, int n, int m) {
	
	int *Labels = (int *) malloc(n*m*sizeof(int));
	memset(Labels, -1, n*m*sizeof(int));

	int max_nbl = 200;

	bool *Association_matrix = (bool *) malloc(max_nbl*max_nbl*sizeof(bool));
	memset(Association_matrix, 0, max_nbl*max_nbl*sizeof(bool));

	int nb_label = 0;
	for (int i = 0; i < n; i++)  {
		for (int j = 0; j < m; j++) {
			if (Input[i*m+j]) {
				
				int lb_i = max(i-1, 0);
				int ub_i = min(i+2, n);
				int lb_j = max(j-1, 0);
				int ub_j = min(j+2, m);

				vector<int> all_labels;
				for (int k = lb_i; k < ub_i; k++) {
					for (int l = lb_j; l < ub_j; l++) {
						if (Labels[k*m+l] != -1) {
							all_labels.push_back(Labels[k*m+l]);
						}
					}
				}

				if (all_labels.size() == 0) {
					Labels[i*m+j] = nb_label;
					if (nb_label < max_nbl) {
						nb_label++;
					} else {
						cout << "Warning nb of label greater than the 200 limit" << endl;
						free(Labels);
						free(Association_matrix);
						return 0;
					}
					continue;
				}

				int min_label = *std::min_element(&all_labels[0],&all_labels[all_labels.size()-1]);
				Labels[i*m+j] = min_label;

				for (vector<int>::iterator it = all_labels.begin(); it != all_labels.end(); it++) {
					for (vector<int>::iterator it2 = all_labels.begin(); it2 != all_labels.end(); it2++) {
						Association_matrix[(*it)*max_nbl + (*it2)] = true;
					}
				}

				all_labels.clear();
			}
		}
	}

	// Transitive closure by Floyd-Warshall algorithm
	for (int i = 0; i < nb_label; i++) {
		for (int j = 0; j < nb_label; j++) {
			if (Association_matrix[i*max_nbl + j]) {
				for (int k = 0; k < nb_label; k++) {
					Association_matrix[i*max_nbl + k] = Association_matrix[i*max_nbl + k] || Association_matrix[j*max_nbl + k];
				}
			}
		}
	}

	int *new_labels = (int *) malloc(nb_label*sizeof(int));
	memset(new_labels, -1, nb_label*sizeof(int));
	for (int i = 0; i < nb_label; i++) {
		for (int j = 0; j < i+1; j++) {
			if (Association_matrix[i*max_nbl + j]) {
				new_labels[i] = j;
				break;
			}
		}
	}

	vector<vector<int>> connected_i;
	vector<vector<int>> connected_j;
	connected_i.resize(nb_label);
	connected_j.resize(nb_label);

	for (int i = 0; i < n; i++)  {
		for (int j = 0; j < m; j++) {
			if (Input[i*m+j]) {
				Labels[i*m+j] = new_labels[Labels[i*m+j]];
				connected_i[new_labels[Labels[i*m+j]]].push_back(i);
				connected_j[new_labels[Labels[i*m+j]]].push_back(j);
			}
		}
	}

	int max_label = -1;
	int max_nb = 0;
	for (int i = 0; i < nb_label; i++) {
		if (connected_i[i].size() > max_nb) {
			max_nb = connected_i[i].size();
			max_label = i;
		}
	}

	if (max_label == -1) {
		//cout << "no component" << endl;
		free(Labels);
		free(new_labels);
		free(Association_matrix);
		return 0;
	}

	int min_i = *std::min_element(&connected_i[max_label][0],&connected_i[max_label][max_nb-1]);
	int max_i = *std::max_element(&connected_i[max_label][0],&connected_i[max_label][max_nb-1]);
	int min_j = *std::min_element(&connected_j[max_label][0],&connected_j[max_label][max_nb-1]);
	int max_j = *std::max_element(&connected_j[max_label][0],&connected_j[max_label][max_nb-1]);

	BBox[1] = BBox[0] + max_i;
	BBox[0] = BBox[0] + min_i;
	BBox[3] = BBox[2] + max_j;
	BBox[2] = BBox[2] + min_j;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			if (Labels[i*m + j] != max_label) {
				Input[i*m+j] = false;
			}
		}
	}
	
	connected_i.clear();
	connected_j.clear();

	free(Labels);
	free(new_labels);
	free(Association_matrix);

	return max_nb;
}

bool Estimate_transfo(float *points, Eigen::Matrix3f poseR, Eigen::Vector3f poset, float *Rotation, float *translation, int  nb_match) {
	
	int max_iter_inner = 100;
	float delta = 0.0001;
	bool inner_cvg = false;
	int inner_iter = 0;
	float lambda = 100.0;
	float v = 2.0;//1.5;
	Matrix3f Rinc;
	Vector3f tinc;
	Matrix3f LMRtmp_Rx;
	Vector3f LMttmp_Rx; 
	Matrix3f LMRtmp_Ry;
	Vector3f LMttmp_Ry; 
	Matrix3f LMRtmp_Rz;
	Vector3f LMttmp_Rz; 
	Matrix3f LMRtmp_Tx;
	Vector3f LMttmp_Tx; 
	Matrix3f LMRtmp_Ty;
	Vector3f LMttmp_Ty; 
	Matrix3f LMRtmp_Tz;
	Vector3f LMttmp_Tz; 

	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
	Eigen::Matrix<float, 6, 1> b;
				
	float *Jac_x = (float *) malloc(6*nb_match*sizeof(float));
	float *Jac_y = (float *) malloc(6*nb_match*sizeof(float));
	float *Jac_z = (float *) malloc(6*nb_match*sizeof(float));
			
	float *Right_x = (float *) malloc(nb_match*sizeof(float));
	float *Right_y = (float *) malloc(nb_match*sizeof(float));
	float *Right_z = (float *) malloc(nb_match*sizeof(float));
		
	Matrix3f Rtmp = poseR;
	Vector3f ttmp = poset;
	while (!inner_cvg) {
				
		Rinc = (Matrix3f)AngleAxisf (10.0*delta, Vector3f::UnitX ());
		tinc = Vector3f::Zero();
		LMttmp_Rx = Rinc * ttmp + tinc;
		LMRtmp_Rx = Rinc * Rtmp;

		Rinc = (Matrix3f)AngleAxisf (10.0*delta, Vector3f::UnitY ());
		LMttmp_Ry = Rinc * ttmp + tinc;
		LMRtmp_Ry = Rinc * Rtmp;
				
		Rinc = (Matrix3f)AngleAxisf (10.0*delta, Vector3f::UnitZ ());
		LMttmp_Rz = Rinc * ttmp + tinc;
		LMRtmp_Rz = Rinc * Rtmp;
				
		Rinc = Matrix3f::Identity();
		tinc(0) = delta;
		LMttmp_Tx = ttmp + tinc;
		LMRtmp_Tx = Rtmp;

		tinc = Vector3f::Zero();
		tinc(1) = delta;
		LMttmp_Ty = ttmp + tinc;
		LMRtmp_Ty = Rtmp;

		tinc = Vector3f::Zero();
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
				
		Rinc = (Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
		tinc = result.tail<3> ();

		ttmp = Rinc * ttmp + tinc;
		Rtmp = Rinc * Rtmp;

		lambda = lambda/v;

		if (/*final && */(inner_iter > max_iter_inner || ((Rinc-Matrix3f::Identity()).norm() < 1.0e-8 && tinc.norm() < 1.0e-8))) {
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


bool Estimate_transfo2D(float *points, float *Rotation, float *translation, int nb_match) {
	
	int max_iter_inner = 100;
	float delta = 0.0001;
	bool inner_cvg = false;
	int inner_iter = 0;
	float lambda = 100.0;
	float v = 2.0;//1.5;
	Matrix2f Rinc;
	Vector2f tinc;
	Matrix2f LMRtmp_R;
	Vector2f LMttmp_R; 
	Matrix2f LMRtmp_Tx;
	Vector2f LMttmp_Tx; 
	Matrix2f LMRtmp_Ty;
	Vector2f LMttmp_Ty; 
	
	Eigen::Matrix<float, 3, 3, Eigen::RowMajor> A;
	Eigen::Matrix<float, 3, 1> b;
				
	float *Jac_x = (float *) malloc(3*nb_match*sizeof(float));
	float *Jac_y = (float *) malloc(3*nb_match*sizeof(float));
			
	float *Right_x = (float *) malloc(nb_match*sizeof(float));
	float *Right_y = (float *) malloc(nb_match*sizeof(float));
		
	Matrix2f Rtmp = Matrix2f::Identity();
	Vector2f ttmp = Vector2f::Zero();
	while (!inner_cvg) {
				
		Rinc(0,0) = cos(10.0*delta); Rinc(0,1) = -sin(10.0*delta);	
		Rinc(1,0) = sin(10.0*delta); Rinc(1,1) = cos(10.0*delta);
		tinc = Vector2f::Zero();
		LMttmp_R = Rinc * ttmp + tinc;
		LMRtmp_R = Rinc * Rtmp;
				
		tinc(0) = delta;
		LMttmp_Tx = ttmp + tinc;
		LMRtmp_Tx = Rtmp;

		tinc = Vector2f::Zero();
		tinc(1) = delta;
		LMttmp_Ty = ttmp + tinc;
		LMRtmp_Ty = Rtmp;
				
		// The small transfo should be the inverse
			
		for (int j = 0; j < nb_match; j++) {
				
			float p_curr_cp [2];
			p_curr_cp[0] = Rtmp(0,0)*points[4*j] + Rtmp(0,1)*points[4*j+1] + ttmp(0);
			p_curr_cp[1] = Rtmp(1,0)*points[4*j] + Rtmp(1,1)*points[4*j+1] + ttmp(1);
				
			float p_prev [2];
			p_prev[0] = points[4*j+2];
			p_prev[1] = points[4*j+3];
												
			Right_x[j] = p_prev[0] - p_curr_cp[0];
			Right_y[j] = p_prev[1] - p_curr_cp[1];

			// Jacobian in rotX
			float p_tmp_cp [2];
			p_tmp_cp[0] = LMRtmp_R(0,0)*points[4*j] + LMRtmp_R(0,1)*points[4*j+1] + LMttmp_R(0);
			p_tmp_cp[1] = LMRtmp_R(1,0)*points[4*j] + LMRtmp_R(1,1)*points[4*j+1] + LMttmp_R(1);

			Jac_x[0*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/(10.0*delta);
			Jac_y[0*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/(10.0*delta);
				
			// Jacobian in transX
			p_tmp_cp[0] = LMRtmp_Tx(0,0)*points[4*j] + LMRtmp_Tx(0,1)*points[4*j+1] + LMttmp_Tx(0);
			p_tmp_cp[1] = LMRtmp_Tx(1,0)*points[4*j] + LMRtmp_Tx(1,1)*points[4*j+1] + LMttmp_Tx(1);

			Jac_x[1*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/delta;
			Jac_y[1*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/delta;
				
			// Jacobian in transY
			p_tmp_cp[0] = LMRtmp_Ty(0,0)*points[4*j] + LMRtmp_Ty(0,1)*points[4*j+1] + LMttmp_Ty(0);
			p_tmp_cp[1] = LMRtmp_Ty(1,0)*points[4*j] + LMRtmp_Ty(1,1)*points[4*j+1] + LMttmp_Ty(1);

			Jac_x[2*nb_match+j] = (p_tmp_cp[0]-p_curr_cp [0])/delta;
			Jac_y[2*nb_match+j] = (p_tmp_cp[1]-p_curr_cp [1])/delta;
				
		}

		// Compute J^tJ and J^tb
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				A(i,j) = 0.0;
				for (int k = 0; k < nb_match; k++) {
					A(i,j) = A(i,j) + Jac_x[i*nb_match+k]*Jac_x[j*nb_match+k] + Jac_y[i*nb_match+k]*Jac_y[j*nb_match+k];
				}
			}

			b(i) = 0.0;
			for (int k = 0; k < nb_match; k++) {
				b(i) = b(i) + Jac_x[i*nb_match+k]*Right_x[k] + Jac_y[i*nb_match+k]*Right_y[k];
			}
		}

		/*for (int i = 0; i < 6; i++) {
			A(i,i) = A(i,i) + lambda*A(i,i);
		}*/

		//checking nullspace
		float det = A.determinant ();

		if (fabs (det) < 1e-15)
		{
			free(Jac_x);
			free(Jac_y);
			free(Right_x);
			free(Right_y);
			return false;
		}

		Eigen::Matrix<float, 3, 1> result = A.inverse() * b; // A.llt ().solve (b).cast<float>();
		float alpha = result (0);

		Rinc(0,0) = cos(alpha); Rinc(0,1) = -sin(alpha);	
		Rinc(1,0) = sin(alpha); Rinc(1,1) = cos(alpha);
		tinc = result.tail<2> ();

		ttmp = Rinc * ttmp + tinc;
		Rtmp = Rinc * Rtmp;

		lambda = lambda/v;

		if (/*final && */(inner_iter > max_iter_inner || ((Rinc-Matrix2f::Identity()).norm() < 1.0e-8 && tinc.norm() < 1.0e-8))) {
			inner_cvg = true;
		}

		inner_iter++;
	}

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			Rotation[j*2+i] = Rtmp(i,j);
		}
		translation[i] = ttmp(i);
	}
			
	////////////////////////// END LEvenberg Marquart Algorithm /////////////////////////////////////////////
				
	free(Jac_x);
	free(Jac_y);
	free(Right_x);
	free(Right_y);
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


bool Horn_align(float *p, float *Rotation, float *translation, int  nb_match) {
	// calculate the best rigid motion for alignment 
	double q[7];
	if (!horn_align(p, nb_match, q))
		return false;

	double phi = 2 * acos(q[0]);
      cout << "rotation: " << 
        phi/PI*180 << "\t" <<
        q[1]/sin(phi/2) << "\t" <<
        q[2]/sin(phi/2) << "\t" <<
        q[3]/sin(phi/2) << "\t" << endl;

	  cout << "translation: " << 
			q[4]<< "\t" <<
			q[5]<< "\t" <<
			q[6]<< "\t" << endl;

	double matr[16];
	quaternion2matrix(q, matr);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rotation[3*i+j] = matr[4*i+j];
		}
	}
	
	for (int j = 0; j < 3; j++) {
		translation[j] = matr[12+j];
	}

	return (Rotation[0] == Rotation[0]);
}



#ifdef WIN32
#define cbrt(r)  (pow((r), 1./3))
#endif

#define SHOW(X) cout << #X " = " << X << endl

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////              Horn's method starts              ////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


// transform a quaternion to a rotation matrix
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


// find the coefficients of the characteristic eqn.
// l^4 + a l^3 + b l^2 + c l + d = 0
// for a symmetric 4x4 matrix
static void
characteristicPol(double Q[4][4], double c[4])
{
  // squares
  double q01_2 = Q[0][1] * Q[0][1];
  double q02_2 = Q[0][2] * Q[0][2];
  double q03_2 = Q[0][3] * Q[0][3];
  double q12_2 = Q[1][2] * Q[1][2];
  double q13_2 = Q[1][3] * Q[1][3];
  double q23_2 = Q[2][3] * Q[2][3];

  // other factors
  double q0011 = Q[0][0] * Q[1][1];
  double q0022 = Q[0][0] * Q[2][2];
  double q0033 = Q[0][0] * Q[3][3];
  double q0102 = Q[0][1] * Q[0][2];
  double q0103 = Q[0][1] * Q[0][3];
  double q0223 = Q[0][2] * Q[2][3];
  double q1122 = Q[1][1] * Q[2][2];
  double q1133 = Q[1][1] * Q[3][3];
  double q1223 = Q[1][2] * Q[2][3];
  double q2233 = Q[2][2] * Q[3][3];

  // a
  c[0] = -Q[0][0] - Q[1][1] - Q[2][2] - Q[3][3];

  // b
  c[1] = - q01_2 - q02_2 - q03_2 + q0011 - q12_2 - 
    q13_2 + q0022 + q1122 - q23_2 + q0033 + q1133 + 
    q2233;

  // c
  c[2] = (q02_2 + q03_2 + q23_2)*Q[1][1] - 2*q0102*Q[1][2] + 
    (q12_2 + q13_2 + q23_2)*Q[0][0] +
    (q01_2 + q03_2 - q0011 + q13_2 - q1133)*Q[2][2] - 
    2*Q[0][3]*q0223 - 2*(q0103 + q1223)*Q[1][3] + 
    (q01_2 + q02_2 - q0011 + q12_2 - q0022)*Q[3][3];
    
  // d
  c[3] = 2*(-Q[0][2]*Q[0][3]*Q[1][2] + q0103*Q[2][2] -
	    Q[0][1]*q0223 + Q[0][0]*q1223)*Q[1][3] +
    q02_2*q13_2 - q03_2*q1122 - q13_2*q0022 + 
    2*Q[0][3]*Q[1][1]*q0223 - 2*q0103*q1223 + q01_2*q23_2 - 
    q0011*q23_2 - q02_2*q1133 + q03_2*q12_2 +
    2*q0102*Q[1][2]*Q[3][3] - q12_2*q0033 - q01_2*q2233 + 
    q0011*q2233;
}



static int ferrari(double a, double b, double c, double d,
	  double rts[4]);

// calculate the maximum eigenvector of a symmetric
// 4x4 matrix
// from B. Horn 1987 Closed-form solution of absolute
// orientation using unit quaternions (J.Opt.Soc.Am.A)
bool
maxEigenVector(double Q[4][4], double ev[4])
{
  
  //Eigen::Matrix4f N;
  Eigen::Matrix<double, 4, 4> N;
  double rts[4];
  double c[4];
  // find the coeffs for the characteristic eqn.
  characteristicPol(Q, c);
  // find roots
  ferrari(c[0], c[1], c[2], c[3], rts);
  // find maximum root = maximum eigenvalue
  double l = rts[0];
  if (rts[1] > l) l = rts[1];
  if (rts[2] > l) l = rts[2];
  if (rts[3] > l) l = rts[3];

  /*
  SHOW(l);
  SHOW(rts[0]);
  SHOW(rts[1]);
  SHOW(rts[2]);
  SHOW(rts[3]);
  */

  // create the Q - l*I matrix
  N(0,0)=Q[0][0]-l;N(0,1)=Q[0][1] ;N(0,2)=Q[0][2]; N(0,3)=Q[0][3];
  N(1,0)=Q[1][0]; N(1,1)=Q[1][1]-l;N(1,2)=Q[1][2]; N(1,3)=Q[1][3];
  N(2,0)=Q[2][0]; N(2,1)=Q[2][1] ;N(2,2)=Q[2][2]-l;N(2,3)=Q[2][3];
  N(3,0)=Q[3][0]; N(3,1)=Q[3][1] ;N(3,2)=Q[3][2];N(3,3)=Q[3][3]-l;

  double det = N.determinant ();
  if (fabs (det) < 1e-15 || det != det)
  {
		if (det != det) cout << "qnan" << endl;
		cout << "det null" << endl;
		return false;
  }

  // the columns of the inverted matrix should be multiples of
  // the eigenvector, pick the largest
  Eigen::Matrix<double, 4, 1> input;// = VectorXf::Random(20);
  Eigen::Matrix<double, 4, 1> best;
  Eigen::Matrix<double, 4, 1> curr;
  //static Vector<int> ipiv(4);
  //static Vector<double> best(4),curr(4);
 
  //Eigen::LU<MatrixXf> luOfN(N);
  //if (LU_factor(N, ipiv)) {
  //  //SHOW(Q[0][0]);
  //  cerr << "maxEigenVector():" << endl;
  //  cerr << "LU_factor failed!" << endl;
  //  cerr << "return identity quaternion" << endl;
  //  //cerr << N << endl;
  //  ev[0] = 1.0;
  //  ev[1] = ev[2] = ev[3] = 0.0;
  //  return;
  //}
  input(0) = 1.0; input(1) = 0.0; input(2) = 0.0; input(3) = 0.0;
  best = N.lu().solve(input);
  //LU_solve(N, ipiv, best);
  double len = 
    best(0)*best(0) + best(1)*best(1) + 
    best(2)*best(2) + best(3)*best(3);
  for (int i=1; i<4; i++) {
	input(0) = 0.0; input(1) = 0.0; input(2) = 0.0; input(3) = 0.0;
    input(i) = 1;
	curr = N.lu().solve(input);
    //LU_solve(N, ipiv, curr);
    double tlen = 
      curr(0)*curr(0) + curr(1)*curr(1) + 
      curr(2)*curr(2) + curr(3)*curr(3);
    if (tlen > len) { len = tlen; best = curr; }
  }
  // normalize the result
  len = 1.0/sqrt(len);
  ev[0] = best(0)*len;
  ev[1] = best(1)*len;
  ev[2] = best(2)*len;
  ev[3] = best(3)*len;

  return true;
}


//double 
//dist_2(MyPoint3D *p,      // the model points  (source)
//	   MyPoint3D *x,      // the measured points (destination)
//	   double q[7])
//{
//	Xform<double> xf;
//	xf.identity();
//	xf.addQuaternion(q, 7);
//	MyPoint3D *tmp = new MyPoint3D();
//	xf.apply(p, tmp);
//	return dist(tmp, x);
//}


// find the transformation that aligns measurements to
// model
//void
//weigh_least_square(float **p,      // the model points  (source)
//	   float **x,      // the measured points (destination)
//	   int n,        // how many pairs
//	   double q[7],  // the reg. vector 0..3 rot, 4..6 trans
//	   double d)
//{
//	q[0]=1;
//	q[1]=0;
//	q[2]=0;
//	q[3]=0;
//
//	
//	q[4]=0;
//	q[5]=0;
//	q[6]=0;
//
//	int it = 30;
//	double l = 0.75;
//	double gamma = 0.125;
//	double B0 = 0.1/d;
//	double Bf = 4000/d;
//	double Br = 1.1;
//	double thresh = 0.001;
//	double var = 1;
//	std::vector<double> mi;
//	std::vector<double> qi;
//	for (int i = 0; i < n; i++)
//	{
//		mi.push_back(1/static_cast<double>(n));
//		qi.push_back(0);
//	}
//
//	while (B0 <= Bf)
//	{
//		while (var > thresh && it > 0)
//		{
//			it--;
//			double sum = 0;
//			int i = 0;
//			std::vector<double>::iterator itm = mi.begin();
//			std::vector<double>::iterator itq = qi.begin();
//			for ( ; itm != mi.end(); itm++, itq++)
//			{
//				(*itq) = dist_2(p[i], x[i], q);
//				(*itm) = pow(1+(B0*(*itm)*l*(*itq)),-1/l);
//				sum += (*itm);
//			}
//
//			//impose the two way constraint
//
//
//			//Update rotation and translation
//
//			  if (n<3) {
//				cerr << "align() was given " << n << " pairs," 
//				 << " while at least 3 are required" << endl;
//				return;
//			  }
//
//			  MyPoint3D *p_mean = new MyPoint3D();
//			  MyPoint3D *x_mean = new MyPoint3D();
//			  double S[3][3];
//			  double Q[4][4];
//			  int j;
//
//			  // calculate the centers of mass
//			  for (i=0; i<n; i++) {
//				  p_mean->_x += mi[i]*p[i]->_x;
//				  p_mean->_y += mi[i]*p[i]->_y;
//				  p_mean->_z += mi[i]*p[i]->_z;
//				  x_mean->_x += mi[i]*x[i]->_x;
//				  x_mean->_y += mi[i]*x[i]->_y;
//				  x_mean->_z += mi[i]*x[i]->_z;
//			  }
//			  p_mean->_x = p_mean->_x/sum;
//			  p_mean->_y = p_mean->_y/sum;
//			  p_mean->_z = p_mean->_z/sum;
//			  x_mean->_x = x_mean->_x/sum;	
//			  x_mean->_y = x_mean->_y/sum;
//			  x_mean->_z = x_mean->_z/sum;
//
//			 /* p_mean /= static_cast<double>(n);
//			  x_mean /= static_cast<double>(n);*/
//			  // calculate the cross covariance matrix
//			  for (i=0; i<3; i++)
//				for (j=0; j<3; j++)
//				  S[i][j] = 0;
//			  for (i=0; i<n; i++) {
//				  S[0][0] += mi[i]*((p[i]->_x-p_mean->_x)*(x[i]->_x-x_mean->_x));
//				  S[0][1] += mi[i]*((p[i]->_x-p_mean->_x)*(x[i]->_y-x_mean->_y));
//				  S[0][2] += mi[i]*((p[i]->_x-p_mean->_x)*(x[i]->_z-x_mean->_z));
//				  S[1][0] += mi[i]*((p[i]->_y-p_mean->_y)*(x[i]->_x-x_mean->_x));
//				  S[1][1] += mi[i]*((p[i]->_y-p_mean->_y)*(x[i]->_y-x_mean->_y));
//				  S[1][2] += mi[i]*((p[i]->_y-p_mean->_y)*(x[i]->_z-x_mean->_z));
//				  S[2][0] += mi[i]*((p[i]->_z-p_mean->_z)*(x[i]->_x-x_mean->_x));
//				  S[2][1] += mi[i]*((p[i]->_z-p_mean->_z)*(x[i]->_y-x_mean->_y));
//				  S[2][2] += mi[i]*((p[i]->_z-p_mean->_z)*(x[i]->_z-x_mean->_z));
//			  }
//			  /*double fact = 1/double(n);
//			  for (i=0; i<3; i++)
//				for (j=0; j<3; j++)
//				  S[i][j] *= fact;
//			  S[0][0] -= p_mean[0]*x_mean[0];
//			  S[0][1] -= p_mean[0]*x_mean[1];
//			  S[0][2] -= p_mean[0]*x_mean[2];
//			  S[1][0] -= p_mean[1]*x_mean[0];
//			  S[1][1] -= p_mean[1]*x_mean[1];
//			  S[1][2] -= p_mean[1]*x_mean[2];
//			  S[2][0] -= p_mean[2]*x_mean[0];
//			  S[2][1] -= p_mean[2]*x_mean[1];
//			  S[2][2] -= p_mean[2]*x_mean[2];*/
//			  // calculate the 4x4 symmetric matrix Q
//			  double trace = S[0][0] + S[1][1] + S[2][2];
//			  double A23 = S[1][2] - S[2][1];
//			  double A31 = S[2][0] - S[0][2];
//			  double A12 = S[0][1] - S[1][0];
//
//			  Q[0][0] = trace;
//			  Q[0][1] = Q[1][0] = A23;
//			  Q[0][2] = Q[2][0] = A31;
//			  Q[0][3] = Q[3][0] = A12;
//			  for (i=0; i<3; i++)
//				for (j=0; j<3; j++)
//				  Q[i+1][j+1] = S[i][j]+S[j][i]-(i==j ? trace : 0);
//			  
//			  maxEigenVector(Q, q);
//
//			  // calculate the rotation matrix
//			  double m[3][3]; // rot matrix
//			  double Ttmp[3];
//			  quaternion2matrix(q, m);
//			  // calculate the translation vector, put it into q[4..6]
//			  Ttmp[0] = q[4];
//			  Ttmp[1] = q[5];
//			  Ttmp[2] = q[6];
//			  q[4] = x_mean->_x - m[0][0]*p_mean->_x - 
//				  m[0][1]*p_mean->_y - m[0][2]*p_mean->_z;
//			  q[5] = x_mean->_y - m[1][0]*p_mean->_x - 
//				  m[1][1]*p_mean->_y - m[1][2]*p_mean->_z;
//			  q[6] = x_mean->_z - m[2][0]*p_mean->_x - 
//				  m[2][1]*p_mean->_y - m[2][2]*p_mean->_z;
//
//			  var = sqrt((Ttmp[0]-q[4])*(Ttmp[0]-q[4]) + (Ttmp[1]-q[5])*(Ttmp[1]-q[5]) + (Ttmp[2]-q[6])*(Ttmp[2]-q[6]));
//
//			for (int i = 0; i < n; i++)
//			{
//				mi[i] = mi[i]/sum;
//				mi[i] = pow(gamma*mi[i], gamma-1);
//			}
//		}
//
//		B0 = Br*B0;
//	}
//
//}


// find the transformation that aligns measurements to
// model
bool
horn_align(float *p,      // the macthed points  (source)
	   int n,        // how many pairs
	   double q[7])  // the reg. vector 0..3 rot, 4..6 trans
{
  if (n<3) {
    cerr << "horn_align() was given " << n << " pairs," 
	 << " while at least 3 are required" << endl;
    return false;
  }

  double p_mean[3];
  double x_mean[3];

  double S[3][3];
  double Q[4][4];
  int i,j;

  // calculate the centers of mass
  for (i=0; i<n; i++) {
	   p_mean[0] = p_mean[0] + static_cast<double>(p[6*i]);
	   p_mean[1] = p_mean[1] + static_cast<double>(p[6*i+1]);
	   p_mean[2] = p_mean[2] + static_cast<double>(p[6*i+2]);
	   x_mean[0] = x_mean[0] + static_cast<double>(p[6*i+3]);
	   x_mean[1] = x_mean[1] + static_cast<double>(p[6*i+4]);
	   x_mean[2] = x_mean[2] + static_cast<double>(p[6*i+5]);
  }
  p_mean[0] = p_mean[0] /static_cast<double>(n);
  p_mean[1] = p_mean[1] /static_cast<double>(n);
  p_mean[2] = p_mean[2] /static_cast<double>(n);
  x_mean[0] = x_mean[0] /static_cast<double>(n);
  x_mean[1] = x_mean[1] /static_cast<double>(n);
  x_mean[2] = x_mean[2] /static_cast<double>(n);
  // calculate the cross covariance matrix
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      S[i][j] = 0;
  for (i=0; i<n; i++) {
    S[0][0] += static_cast<double>(p[6*i]*p[6*i+3]);
    S[0][1] += static_cast<double>(p[6*i]*p[6*i+4]);
	S[0][2] += static_cast<double>(p[6*i]*p[6*i+5]);
    S[1][0] += static_cast<double>(p[6*i+1]*p[6*i+3]);
    S[1][1] += static_cast<double>(p[6*i+1]*p[6*i+4]);
    S[1][2] += static_cast<double>(p[6*i+1]*p[6*i+5]);
    S[2][0] += static_cast<double>(p[6*i+2]*p[6*i+3]);
    S[2][1] += static_cast<double>(p[6*i+2]*p[6*i+4]);
    S[2][2] += static_cast<double>(p[6*i+2]*p[6*i+5]);
  }
  double fact = 1/double(n);
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      S[i][j] *= fact;
  S[0][0] -= p_mean[0]*x_mean[0];
  S[0][1] -= p_mean[0]*x_mean[1];
  S[0][2] -= p_mean[0]*x_mean[2];
  S[1][0] -= p_mean[1]*x_mean[0];
  S[1][1] -= p_mean[1]*x_mean[1];
  S[1][2] -= p_mean[1]*x_mean[2];
  S[2][0] -= p_mean[2]*x_mean[0];
  S[2][1] -= p_mean[2]*x_mean[1];
  S[2][2] -= p_mean[2]*x_mean[2];
  // calculate the 4x4 symmetric matrix Q
  double trace = S[0][0] + S[1][1] + S[2][2];
  double A23 = S[1][2] - S[2][1];
  double A31 = S[2][0] - S[0][2];
  double A12 = S[0][1] - S[1][0];

  Q[0][0] = trace;
  Q[0][1] = Q[1][0] = A23;
  Q[0][2] = Q[2][0] = A31;
  Q[0][3] = Q[3][0] = A12;
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      Q[i+1][j+1] = S[i][j]+S[j][i]-(i==j ? trace : 0);
  
  if (!maxEigenVector(Q, q))
	  return false;

  // calculate the rotation matrix
  double m[3][3]; // rot matrix
  quaternion2matrix(q, m);
  // calculate the translation vector, put it into q[4..6]
  q[4] = x_mean[0]- m[0][0]*p_mean[0] - 
    m[0][1]*p_mean[1] - m[0][2]*p_mean[2];
  q[5] = x_mean[1] - m[1][0]*p_mean[0] - 
    m[1][1]*p_mean[1] - m[1][2]*p_mean[2];
  q[6] = x_mean[2] - m[2][0]*p_mean[0] - 
    m[2][1]*p_mean[1] - m[2][2]*p_mean[2];

  return true;
}


/**************************************************/
static int 
qudrtc(double b, double c, double rts[4])
/* 
     solve the quadratic equation - 

         x**2+b*x+c = 0 

*/
{
  int nquad;
  double rtdis;
  double dis = b*b-4.0*c;
  
  if (dis >= 0.0) {
    nquad = 2;
    rtdis = sqrt(dis);
    if (b > 0.0) rts[0] = ( -b - rtdis)*.5;
    else         rts[0] = ( -b + rtdis)*.5;
    if (rts[0] == 0.0) rts[1] = -b;
    else               rts[1] = c/rts[0];
  } else {
    nquad = 0;
    rts[0] = 0.0;
    rts[1] = 0.0;
  }
  return nquad;
} /* qudrtc */
/**************************************************/

static double 
cubic(double p, double q, double r)
/* 
     find the lowest real root of the cubic - 
       x**3 + p*x**2 + q*x + r = 0 

   input parameters - 
     p,q,r - coeffs of cubic equation. 

   output- 
     cubic - a real root. 

   method - 
     see D.E. Littlewood, "A University Algebra" pp.173 - 6 

     Charles Prineas   April 1981 

*/
{
  //int nrts;
  double po3,po3sq,qo3;
  double uo3,u2o3,uo3sq4,uo3cu4;
  double v,vsq,wsq;
  double m,mcube,n;
  double muo3,s,scube,t,cosk,sinsqk;
  double root;
  
  double doubmax = sqrt(DBL_MAX);
  
  m = 0.0;
  //nrts =0;
  if        ((p > doubmax) || (p <  -doubmax)) {
    root = -p;
  } else if ((q > doubmax) || (q <  -doubmax)) {
    if (q > 0.0) root = -r/q;
    else         root = -sqrt(-q);
  } else if ((r > doubmax)|| (r <  -doubmax)) {
    root =  -cbrt(r);
  } else {
    po3 = p/3.0;
    po3sq = po3*po3;
    if (po3sq > doubmax) root =  -p;
    else {
      v = r + po3*(po3sq + po3sq - q);
      if ((v > doubmax) || (v < -doubmax)) 
	root = -p;
      else {
	vsq = v*v;
	qo3 = q/3.0;
	uo3 = qo3 - po3sq;
	u2o3 = uo3 + uo3;
	if ((u2o3 > doubmax) || (u2o3 < -doubmax)) {
	  if (p == 0.0) {
	    if (q > 0.0) root =  -r/q;
	    else         root =  -sqrt(-q);
	  } else         root =  -q/p;
	}
	uo3sq4 = u2o3*u2o3;
	if (uo3sq4 > doubmax) {
	  if (p == 0.0) {
	    if (q > 0.0) root = -r/q;
	    else         root = -sqrt(fabs(q));
	  } else         root = -q/p;
	}
	uo3cu4 = uo3sq4*uo3;
	wsq = uo3cu4 + vsq;
	if (wsq >= 0.0) {
	  //
	  // cubic has one real root 
	  //
	  //nrts = 1;
	  if (v <= 0.0) mcube = ( -v + sqrt(wsq))*.5;
	  if (v  > 0.0) mcube = ( -v - sqrt(wsq))*.5;
	  m = cbrt(mcube);
	  if (m != 0.0) n = -uo3/m;
	  else          n = 0.0;
	  root = m + n - po3;
	} else {
	  //nrts = 3;
	  //
	  // cubic has three real roots 
	  //
	  if (uo3 < 0.0) {
	    muo3 = -uo3;
	    s = sqrt(muo3);
	    scube = s*muo3;
	    t =  -v/(scube+scube);
	    cosk = cos(acos(t)/3.0);
	    if (po3 < 0.0)
	      root = (s+s)*cosk - po3;
	    else {
	      sinsqk = 1.0 - cosk*cosk;
	      if (sinsqk < 0.0) sinsqk = 0.0;
	      root = s*( -cosk - sqrt(3*sinsqk)) - po3;
	    }
	  } else
	    //
	    // cubic has multiple root -  
	    //
	    root = cbrt(v) - po3;
	}
      }
    }
  }
  return root;
} /* cubic */
/***************************************/


static int 
ferrari(double a, double b, double c, double d,	double rts[4])
/* 
     solve the quartic equation - 

   x**4 + a*x**3 + b*x**2 + c*x + d = 0 

     input - 
   a,b,c,e - coeffs of equation. 

     output - 
   nquar - number of real roots. 
   rts - array of root values. 

     method :  Ferrari - Lagrange
     Theory of Equations, H.W. Turnbull p. 140 (1947)

     calls  cubic, qudrtc 
*/
{
  int nquar,n1,n2;
  double asq,ainv2;
  double v1[4],v2[4];
  double p,q,r;
  double y;
  double e,f,esq,fsq,ef;
  double g,gg,h,hh;

  asq = a*a;

  p = b;
  q = a*c-4.0*d;
  r = (asq - 4.0*b)*d + c*c;
  y = cubic(p,q,r);

  esq = .25*asq - b - y;
  if (esq < 0.0) return(0);
  else {
    fsq = .25*y*y - d;
    if (fsq < 0.0) return(0);
    else {
      ef = -(.25*a*y + .5*c);
      if ( ((a > 0.0)&&(y > 0.0)&&(c > 0.0))
	   || ((a > 0.0)&&(y < 0.0)&&(c < 0.0))
	   || ((a < 0.0)&&(y > 0.0)&&(c < 0.0))
	   || ((a < 0.0)&&(y < 0.0)&&(c > 0.0))
	   ||  (a == 0.0)||(y == 0.0)||(c == 0.0)
	   ) {
	/* use ef - */
	if ((b < 0.0)&&(y < 0.0)&&(esq > 0.0)) {
	  e = sqrt(esq);
	  f = ef/e;
	} else if ((d < 0.0) && (fsq > 0.0)) {
	  f = sqrt(fsq);
	  e = ef/f;
	} else {
	  e = sqrt(esq);
	  f = sqrt(fsq);
	  if (ef < 0.0) f = -f;
	}
      } else {
	e = sqrt(esq);
	f = sqrt(fsq);
	if (ef < 0.0) f = -f;
      }
      /* note that e >= 0.0 */
      ainv2 = a*.5;
      g = ainv2 - e;
      gg = ainv2 + e;
      if ( ((b > 0.0)&&(y > 0.0))
	   || ((b < 0.0)&&(y < 0.0)) ) {
	if (( a > 0.0) && (e != 0.0)) g = (b + y)/gg;
	else if (e != 0.0) gg = (b + y)/g;
      }
      if ((y == 0.0)&&(f == 0.0)) {
	h = 0.0;
	hh = 0.0;
      } else if ( ((f > 0.0)&&(y < 0.0))
		  || ((f < 0.0)&&(y > 0.0)) ) {
	hh = -.5*y + f;
	h = d/hh;
      } else {
	h = -.5*y - f;
	hh = d/h;
      }
      n1 = qudrtc(gg,hh,v1);
      n2 = qudrtc(g,h,v2);
      nquar = n1+n2;
      rts[0] = v1[0];
      rts[1] = v1[1];
      rts[n1+0] = v2[0];
      rts[n1+1] = v2[1];
      return nquar;
    }
  }
} /* ferrari */


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////        Chen & Medioni's method starts          ////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

static void
get_transform(double correction[6],
	      double m[3][3], double t[3])
{
  // produces a premult matrix: p' = M . p + t
  double sa = sin(correction[0]); 
  double ca = sqrt(1-sa*sa);
  double sb = sin(correction[1]); 
  double cb = sqrt(1-sb*sb);
  double sr = sin(correction[2]);
  double cr = sqrt(1-sr*sr);

  t[0] = correction[3];
  t[1] = correction[4];
  t[2] = correction[5];

  m[0][0] = cb*cr;
  m[0][1] = -cb*sr;
  m[0][2] = sb;

  m[1][0] = sa*sb*cr + ca*sr;
  m[1][1] = -sa*sb*sr + ca*cr;
  m[1][2] = -sa*cb;

  m[2][0] = -ca*sb*cr + sa*sr;
  m[2][1] = ca*sb*sr + sa*cr;
  m[2][2] = ca*cb;
}

// Solve x from Ax=b using Cholesky decomposition.
// This function changes the contents of A in the process.
static bool
cholesky_solve(double A[6][6], double b[6], double x[6])
{
  int i, j, k;
  double sum;

  for (i=0; i<6; i++) {

    for (sum=A[i][i], k=0; k<i; k++)
      sum -= A[i][k]*A[i][k];

      if (sum < 0.0) {
        cerr << "Cholesky: matrix not pos.semidef." << endl;
        SHOW(sum);
        return false;
      } else if (sum < 1.0e-7) {
        cerr << "Cholesky: matrix not pos.def." << endl;
        return false;
      } else {
        A[i][i] = sqrt(sum);
    }

    for (j=i+1; j<6; j++) {

      for (sum=A[i][j], k=0; k<i; k++)
        sum -= A[i][k]*A[j][k];

      A[j][i] = sum / A[i][i];
    }
  }

  for (i=0; i<6; i++) {               // Forward elimination;
    for (sum=b[i], j=0; j<i; j++)     // solve Ly=b, store y in x
      sum -= A[i][j]*x[j];
    x[i] = sum / A[i][i];
  }

  for (i=5; i>=0; i--) {              // Backward elimination;
    for (sum=x[i], j=i+1; j<6; j++)   // solve L'x = y
      sum -= A[j][i]*x[j];
    x[i] = sum / A[i][i];
  }

  return true;
}
