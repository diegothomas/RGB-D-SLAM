#ifndef __UTILITIES_H
#define __UTILITIES_H

#pragma once

/*** Include files for CGAL ***/
#ifndef _DLL
#define _DLL
#endif

//#include <CGAL/trace.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/grid_simplify_point_set.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/IO/Polyhedron_iostream.h>
//#include <CGAL/Surface_mesh_default_triangulation_3.h>
//#include <CGAL/make_surface_mesh.h>
//#include <CGAL/Implicit_surface_3.h>
//#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
//#include <CGAL/Poisson_reconstruction_function.h>
//#include <CGAL/Point_with_normal_3.h>
//#include <CGAL/property_map.h>
//#include <CGAL/IO/read_xyz_points.h>
//#include <CGAL/compute_average_spacing.h>

#include <CGAL/Triangulation_euclidean_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_3.h>

/*** Include files for OpenGL to work ***/
#include <GL\glew.h>
#include <GL\glut.h>

#include "Maths/Maths.h"

/*** Standard include files for manipulating vectors, files etc... ***/
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <conio.h>
#include <algorithm>
#include <time.h>
#include <numeric>
#include <iterator>

//#include <boost/make_shared.hpp>

/*** Include files to manipulate matrices ***/
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

/*** Include files for gpu operations on vectors ***/
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/remove.h>
#include <thrust/generate.h>
#include <thrust/detail/type_traits.h>

/*** Include files for CUDA to work ***/
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h
// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop

/*** Include files for Opencv to work ***/
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/nonfree/gpu.hpp>

/*** Include files for g2o to work ***/
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_euclidean_traits_xy_3<K> Gt;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef CGAL::Triangulation_3<K>      Triangulation;
typedef K::Point_3 Point3D;
typedef K::Point_2 Point2D;

typedef Triangulation::Cell_handle    Cell_handle;
typedef Triangulation::Vertex_handle  Vertex_handle;
typedef Triangulation::Locate_type    Locate_type;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;


#include "Primitive.cuh"

const int width   = 640; 
const int height  = 400;//480;  

#define RES_PLANE 0.004
#define MASKTHRESH 10

using namespace std;
using namespace g2o;

void quaternion2matrix(double *q, double m[3][3]);

bool CheckOrthoBasis(float * e1, float *e2, float *e3);

void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);

bool Estimate_transfo(float *points, Eigen::Matrix3f poseR, Eigen::Vector3f poset, float *Rotation, float *translation, int nb_match);

#endif