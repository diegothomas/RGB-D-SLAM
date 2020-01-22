#include "stdafx.h"
#include "KeyFrame.h"

double *KeyFrame::getQuaternion() {
	double *v = new double [7];

	v[0] = double(_pose(0,3));
	v[1] = double(_pose(1,3));
	v[2] = double(_pose(2,3));

	Eigen::Matrix3d Rot;
	Rot(0,0) = double(_pose(0,0)); Rot(0,1) = double(_pose(0,1)); Rot(0,2) = double(_pose(0,2));
	Rot(1,0) = double(_pose(1,0)); Rot(1,1) = double(_pose(1,1)); Rot(1,2) = double(_pose(1,2)); 
	Rot(2,0) = double(_pose(2,0)); Rot(2,1) = double(_pose(2,1)); Rot(2,2) = double(_pose(2,2));

	float *r = new float [4];
	Eigen::Quaterniond q((Eigen::Matrix3d) Rot);
	Eigen::Quaterniond normalize(q);
	v[3] = q.x(); v[4] = q.y(); v[5] = q.z(); v[6] = q.w();
	
	delete r;

	return v;
}

void KeyFrame::SetFromQuaternion(double *v) {

	Eigen::MatrixXf posePrev = _pose;

	//double Mat [3][3];
	double q[4];
	q[0] = v[6]; //qw
	q[1] = v[3]; //qx
	q[2] = v[4]; //qy
	q[3] = v[5]; //qz

	Eigen::Matrix3d Mat = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).toRotationMatrix();
	//quaternion2matrix(q, Mat);
	_pose(0,0) = float (Mat(0,0));
	_pose(1,0) = float (Mat(1,0));
	_pose(2,0) = float (Mat(2,0));

	_pose(0,1) = float (Mat(0,1));
	_pose(1,1) = float (Mat(1,1));
	_pose(2,1) = float (Mat(2,1));
	
	_pose(0,2) = float (Mat(0,2));
	_pose(1,2) = float (Mat(1,2));
	_pose(2,2) = float (Mat(2,2));
	
	_pose(0,3) = float (v[0]);
	_pose(1,3) = float (v[1]);
	_pose(2,3) = float (v[2]);

	Eigen::MatrixXf poseinc = _pose * posePrev.inverse();

	Transform(poseinc);

	return;
}

int KeyFrame::Align(KeyFrame *frame, float *Transfo) {

	/***** Do Surf coarse registration *****/

	// Compute matches between SURF descriptors
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	if (_descriptors.rows == 0 || frame->_descriptors.rows == 0) {
		cout << "no descriptors"  << endl;
		return 0;
	}

	matcher.match( _descriptors, frame->_descriptors, matches );
	
	cout << "nb matches: " << matches.size() << endl;

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < _descriptors.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	//-- Compute "good" matches (i.e. whose distance is less than 2*min_dist )
	//-- PS.- radiusMatch can also be used here.
	std::vector< cv::DMatch > good_matches;

	for( int i = 0; i < _descriptors.rows; i++ )
	{ 
		if (_keypoints[ matches[i].queryIdx ][0] == 0.0 && _keypoints[ matches[i].queryIdx ][1] == 0.0 && _keypoints[ matches[i].queryIdx ][2] == 0.0)
				continue;

		if (frame->_keypoints[ matches[i].trainIdx ][0] == 0.0 && frame->_keypoints[ matches[i].trainIdx ][1] == 0.0 && frame->_keypoints[ matches[i].trainIdx ][2] == 0.0)
				continue;

		if( matches[i].distance < 5*min_dist )
		{ 
			good_matches.push_back( matches[i]); 
		}
	}
	
	cout << "nb good_matches: " << good_matches.size() << endl;

	// If not enough matches then stop
	if (good_matches.size() < 20)
		return 0;

	/*** RANSAC ***/ 
	std::vector<float *> obj;
	std::vector<float *> scene;

	for( int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( _keypoints[ good_matches[i].queryIdx ] );
		scene.push_back( frame->_keypoints[ good_matches[i].trainIdx ] );
	}
		
	int max_ransac_iter = 1000;
	std::vector< cv::DMatch > best_matches;
	float max_nb_inliers = -1.0;
	//float thresh = 10;
	float thresh = 0.03;

	int num_match = obj.size();
	best_matches.clear();
	
	Eigen::Matrix3f poseR = Eigen::Matrix3f::Identity();
	Eigen::Vector3f poseT = Eigen::Vector3f::Zero();

	//#pragma omp parallel num_threads(NUM_THREADS)
	//{
		//#pragma omp for
		for (int i = 0; i < max_ransac_iter; i++) {
			//select 3 random matches
			int m [3];
			m[0] = int((num_match-1)*float(rand())/RAND_MAX);
			m[1] = int((num_match-1)*float(rand())/RAND_MAX);
			while (m[1] == m[0])
				m[1] = int((num_match-1)*float(rand())/RAND_MAX);
			m[2] = int((num_match-1)*float(rand())/RAND_MAX);
			while (m[2] == m[0] || m[2] == m[0])
				m[2] = int((num_match-1)*float(rand())/RAND_MAX);

			float points [18];
			for (int j = 0; j < 3; j++) {

				points [6*j] = obj[m[j]][0];
				points [6*j+1] = obj[m[j]][1];
				points [6*j+2] = obj[m[j]][2];
			
				points [6*j+3] = scene[m[j]][0];
				points [6*j+4] = scene[m[j]][1];
				points [6*j+5] = scene[m[j]][2];
			}
		
			// Compute best transformation for the current matches that transform obj into scene
			Eigen::Matrix3f Rotation;
			Eigen::Vector3f translation;

			if (!Estimate_transfo(points, poseR, poseT, Rotation.data(), translation.data(), 3))
				continue;

			// Compute score of current transfo 
			float nb_inliers = 0.0;
			std::vector< cv::DMatch > match_list;
			for (int k = 0; k < num_match; k++) {
				float p1[3];
				p1[0] = obj[k][0];
				p1[1] = obj[k][1];
				p1[2] = obj[k][2];
				
				float p2[3];
				p2[0] = scene[k][0];
				p2[1] = scene[k][1];
				p2[2] = scene[k][2];
				
				float p1_cp[3];
				p1_cp[0] = Rotation(0,0)*p1[0] + Rotation(0,1)*p1[1] + Rotation(0,2)*p1[2] + translation(0);
				p1_cp[1] = Rotation(1,0)*p1[0] + Rotation(1,1)*p1[1] + Rotation(1,2)*p1[2] + translation(1);
				p1_cp[2] = Rotation(2,0)*p1[0] + Rotation(2,1)*p1[1] + Rotation(2,2)*p1[2] + translation(2);

				float dist = sqrt((p1_cp[0]-p2[0])*(p1_cp[0]-p2[0]) + (p1_cp[1]-p2[1])*(p1_cp[1]-p2[1]) + (p1_cp[2]-p2[2])*(p1_cp[2]-p2[2]));
				
				if (dist < thresh) {
					nb_inliers += 1.0;
					match_list.push_back(good_matches[k]);
				}
			}

			//#pragma omp critical(maxcomp)
			//{
				if (nb_inliers > max_nb_inliers) {
					//cout << "nb_inliers: " << nb_inliers  << endl;
					max_nb_inliers = nb_inliers;
					best_matches = match_list;
					//cout << Rotation << endl;
					//cout << translation << endl;
				}
			//}
			match_list.clear();
		}
	//}

	cout << "nb matches: " << best_matches.size() << endl;

	if (best_matches.size() < 50) 
		return 0;

	cout << "nb matches: " << best_matches.size() << endl;

	// Refine best transfo
	num_match = best_matches.size();
	float *points = (float *) malloc(6*num_match*sizeof(float));
	for (int j = 0; j < num_match; j++) {
		points [6*j] = _keypoints[best_matches[j].queryIdx][0];
		points [6*j+1] = _keypoints[best_matches[j].queryIdx][1];
		points [6*j+2] = _keypoints[best_matches[j].queryIdx][2];
			
		points [6*j+3] = frame->_keypoints[best_matches[j].trainIdx][0];
		points [6*j+4] = frame->_keypoints[best_matches[j].trainIdx][1];
		points [6*j+5] = frame->_keypoints[best_matches[j].trainIdx][2];
	}

	Eigen::Matrix3f Rotation;
	Eigen::Vector3f translation;
	Estimate_transfo(points, poseR, poseT, Rotation.data(), translation.data(), num_match);

	free(points);

	//Affect best transfo

	Transfo[0] = Rotation(0,0); Transfo[4] = Rotation(0,1); Transfo[8] = Rotation(0,2); Transfo[12] = translation(0);
	Transfo[1] = Rotation(1,0); Transfo[5] = Rotation(1,1); Transfo[9] = Rotation(1,2); Transfo[13] = translation(1);
	Transfo[2] = Rotation(2,0); Transfo[6] = Rotation(2,1); Transfo[10] = Rotation(2,2); Transfo[14] = translation(2);
	Transfo[3] = 0.0; Transfo[7] = 0.0; Transfo[11] = 0.0; Transfo[15] = 1.0;
	
	Eigen::MatrixXf ThePose = Eigen::Matrix4f::Identity();
	return num_match;
}

void KeyFrame::Transform(Eigen::MatrixXf ThePose) {

	float *point;
	for (vector<float *>::iterator it = _keypoints.begin(); it != _keypoints.end(); it++) {
		point = (*it);

		if (point[0] == 0.0 && point[1] == 0.0 && point[2] == 0.0)
			continue;

		float pt [3];
		pt[0] = ThePose(0,0)*point[0] + ThePose(0,1)*point[1] + ThePose(0,2)*point[2] + ThePose(0,3);
		pt[1] = ThePose(1,0)*point[0] + ThePose(1,1)*point[1] + ThePose(1,2)*point[2] + ThePose(1,3);
		pt[2] = ThePose(2,0)*point[0] + ThePose(2,1)*point[1] + ThePose(2,2)*point[2] + ThePose(2,3);

		point[0] = pt[0];
		point[1] = pt[1];
		point[2] = pt[2];
	}
}


bool IsNeighboor(KeyFrame *F1, KeyFrame *F2) {
	float dist = sqrt((F1->_pose(0,3)-F2->_pose(0,3))*(F1->_pose(0,3)-F2->_pose(0,3)) +
					  (F1->_pose(1,3)-F2->_pose(1,3))*(F1->_pose(1,3)-F2->_pose(1,3)) +
					  (F1->_pose(2,3)-F2->_pose(2,3))*(F1->_pose(2,3)-F2->_pose(2,3)));

	if (dist > 3.0)
		return false;

	float angle = acos(F1->_pose(0,2)*F2->_pose(0,2) +
					   F1->_pose(1,2)*F2->_pose(1,2) +
					   F1->_pose(2,2)*F2->_pose(2,2));
		
	if (dist > PI/4.0)
		return false;
	
	return true;
}