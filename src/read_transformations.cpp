#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "read_matrices_pose.h"
#include "read_velo_to_cam.h"
#include "read_transformations.h"

#include <vector>


using namespace std;

// int main(int argc, char const *argv[])
vector<Eigen::Matrix4d> read_transformations()
{
	// read poses of the car (left camera pose)
	map<int, Eigen::Matrix4d> matrices;
	read_matrices_pose(matrices);

	// transformation from the velodyne to the left camera
	Eigen::Affine3d transform;
	read_velo_to_cam(transform);

	// convert the tranform to a 4x4 matrix 
	Eigen::Matrix4d transform4x4;
	transform4x4.setIdentity();
	transform4x4.block<3,3>(0,0)= transform.rotation();
	transform4x4.block<3,1>(0,3)= transform.translation();

	// save the total tranformation from the velo to the navigational frame in T vector
	vector<Eigen::Matrix4d> T;
	for (int i = 0; i < matrices.size(); ++i)
	{
		T.push_back( matrices[i]*transform4x4 );
	}

	cout<<"size of the trasformation vector is "<< T.size()<< endl;

	return T;
}