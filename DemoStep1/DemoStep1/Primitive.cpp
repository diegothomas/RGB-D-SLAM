#include "stdafx.h"
#include "Primitive.h"

bool Primitive::Clean() {

	int count = 0;
	int BBox [4];
	BBox[0] = 10000;
	BBox[1] = -10000;
	BBox[2] = 10000;
	BBox[3] = -10000;
	for (int i = 0; i < _Size[0]; i++) {
		for (int j = 0; j < _Size[1]; j++) {
			if (_Mask_char[i*_Size[1]+j] > 10) {
				BBox[0] = i < BBox[0] ? i : BBox[0];
				BBox[1] = i > BBox[1] ? i : BBox[1];
				BBox[2] = j < BBox[2] ? j : BBox[2];
				BBox[3] = j > BBox[3] ? j : BBox[3];
				count++;
			} else {
				_RGB[3*(i*_Size[1]+j)] = 0;
				_RGB[3*(i*_Size[1]+j)+1] = 0;
				_RGB[3*(i*_Size[1]+j)+2] = 0;
				_Bump[3*(i*_Size[1]+j)] = 0;
				_Bump[3*(i*_Size[1]+j)+1] = 0;
				_Bump[3*(i*_Size[1]+j)+2] = 0;
			}
		}
	}

	if (count < 10) {
		_Bump = NULL;
		_RGB = NULL;
		_Mask_char = NULL;
		return false;
	}

	int new_size [2];
	new_size [0] = BBox[1] - BBox[0] + 1;
	new_size [1] = BBox[3] - BBox[2] + 1;
	
	unsigned short *TheBump = (unsigned short *) malloc(3*new_size [0]*new_size [1] * sizeof(unsigned short));
	unsigned char *TheRGB = (unsigned char *) malloc(3*new_size [0]*new_size [1] * sizeof(unsigned char));
	unsigned char *TheMask = (unsigned char *) malloc(new_size [0]*new_size [1] * sizeof(unsigned char));
	
	memset(TheBump, 0, 3*new_size [0]*new_size [1] * sizeof(unsigned short));
	memset(TheRGB, 0, 3*new_size [0]*new_size [1] * sizeof(unsigned char));
	memset(TheMask, 0, new_size [0]*new_size [1] * sizeof(unsigned char));
	
	_Shift [0] = _Shift [0] + float(BBox[0])*RES_PLANE;
	_Shift [1] = _Shift [1] + float(BBox[2])*RES_PLANE;	

	/************************* Populate into new images *****************************/

	for (int i = 0; i < _Size[0]; i ++) {
		for (int j = 0; j < _Size[1]; j ++) {

			if (_Mask_char[i*_Size[1] + j] < 11)
				continue;

			int ind_i = i - BBox[0];
			int ind_j = j - BBox[2];

			if (ind_i > new_size[0]-1 || ind_j > new_size[1]-1 || ind_i  < 0 || ind_j  < 0 ) {
				cout << "error out of bbox" << endl;
				continue;
			}
			
			TheBump[3*(ind_i*new_size[1] + ind_j)] = _Bump[3*(i*_Size[1] + j)];
			TheBump[3*(ind_i*new_size[1] + ind_j)+1] = _Bump[3*(i*_Size[1] + j)+1];
			TheBump[3*(ind_i*new_size[1] + ind_j)+2] = _Bump[3*(i*_Size[1] + j)+2];
				
			TheRGB[3*(ind_i*new_size[1] + ind_j)] = _RGB[3*(i*_Size[1] + j)];
			TheRGB[3*(ind_i*new_size[1] + ind_j)+1] = _RGB[3*(i*_Size[1] + j)+1];
			TheRGB[3*(ind_i*new_size[1] + ind_j)+2] = _RGB[3*(i*_Size[1] + j)+2];				

			TheMask[ind_i*new_size[1] + ind_j] = _Mask_char[i*_Size[1] + j];
		}
	}
		
	_Size[0] = new_size [0];
	_Size[1] = new_size [1];
	
	_Bump = TheBump;
	_RGB = TheRGB;
	_Mask_char = TheMask;
	

	//Construct a buffer used by the pca analysis
 //   cv::Mat data_pts = cv::Mat(ind_i_list.size(), 2, CV_64FC1);
 //   for (int i = 0; i < data_pts.rows; ++i)
 //   {
 //       data_pts.at<double>(i, 0) = ind_i_list[i];
 //       data_pts.at<double>(i, 1) = ind_j_list[i];
 //   }
	//
 //   //Perform PCA analysis
 //   cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

	////Store the position of the object
 //   cv::Point pos = cv::Point(pca_analysis.mean.at<double>(0, 0),
 //                     pca_analysis.mean.at<double>(0, 1));

 //   //Store the eigenvalues and eigenvectors
 //   vector<cv::Point2d> eigen_vecs(2);
 //   vector<double> eigen_val(2);
 //   for (int i = 0; i < 2; ++i)
 //   {
 //       eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
 //                               pca_analysis.eigenvectors.at<double>(i, 1));

 //       eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
 //   }

	//cout << "mean : " << pos << endl;
	//cout << "eigen_vecs : " << eigen_vecs[0] << " " << eigen_vecs[1] <<  endl;
	//cout << "eigen_val : " << eigen_val[0] << " " << eigen_val[1] <<  endl;

	//cv::Mat proj_pts = cv::Mat(ind_i_list.size(), 2, CV_64FC1);
	//pca_analysis.project(data_pts, proj_pts);

	//memset(_Mask_char, 0, _Size[0]*_Size[1]*sizeof(unsigned char));
	//for (int i = 0; i < proj_pts.rows; ++i)
 //   {
	//	int indI = int(round(proj_pts.at<double>(i, 0) + pos.x));
	//	int indJ = int(round(proj_pts.at<double>(i, 1) + pos.y));
	//	if (indI < 0 || indI > _Size[0]-1 || indJ < 0 || indJ > _Size[1]-1 )
	//		continue;

	//	_Mask_char[indI*_Size[1] + indJ] = 255;
 //   }

	return true;
}

void Primitive::CpyToShortMask() {

	if (_Mask != NULL) {
		free(_Mask);
		_Mask = NULL;
	}

	_Mask = (unsigned short *) malloc(_Size[1]*_Size[0]*sizeof(unsigned short));

	for (int i = 0; i < _Size[0]; i++) {
		for (int j = 0; j < _Size[1]; j++) {
			_Mask[i*_Size[1] + j] = unsigned short(_Mask_char[i*_Size[1] +j]);
		}
	}
}

void Primitive::CpyToCharMask() {

	if (_Mask_char != NULL){
		free(_Mask_char);
		_Mask_char = NULL;
	}

	_Mask_char = (unsigned char *) malloc(_Size[1]*_Size[0]*sizeof(unsigned char));

	for (int i = 0; i < _Size[0]; i++) {
		for (int j = 0; j < _Size[1]; j++) {
			_Mask_char[i*_Size[1]+j] = unsigned char(min(unsigned short(255),_Mask[i*_Size[1]+j]));
		}
	}
}
