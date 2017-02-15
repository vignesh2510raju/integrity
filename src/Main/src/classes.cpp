
#include "../header/classes.h"

Parameters::Parameters()
{
	numFrames= 5;
	min_cluster_size= 25;
	min_plane_size= 100;
	min_density= 200;
	min_cluster_distance= 0.3;
	xlim= 10;
	ylim= 10;
	zlim= 0.5;
	nd_weight= 0.1;
	cylinder_max_radius= 0.12;
	options_flag= false;
	SWITH_VOXEL_GRID= true;      // Downsample the point cloud
	SWITH_WRITE_CLUSTERS= false; // Write the clusters to disk 
	plane_distance_threshold= 0.1;
	plane_normal_distance_weight= 0.05;
	plane_RANSAC_max_iter= 1000;
	normal_radius_search= 0.1;
	min_cylinder_size= 25;
	min_association_distance= 2;
	rlim= 10;
}

Cylinder::Cylinder()
{}

Frame::Frame()
{}

Landmark::Landmark()
{}

