
/* This file defines the constructor for the structures declare in "classes.h". */

#include "../header/classes.h"

Parameters::Parameters()
{
	numFrames= 5; // number of pcd files to read
	options_flag= false;
	SWITH_VOXEL_GRID= true;      // Downsample the point cloud using a voxel grid
	SWITH_WRITE_CLUSTERS= false; // Write the clusters to disk 
	xlim= 10; // points with x coordinate farther than this are removed
	ylim= 10; // points with y coordinate farther than this are removed
	zlim= 0.5; // points lower than 0.5m from the LIDAR height are removed (remove the ground points)
	rlim= 10; // points farther than this distance from the LIDAR are removed
	
	min_cluster_distance= 0.2; // when clustering, this is the minimum distance in between two clusters. If the distance is less than this, they won't be divide into two clusters
	min_cluster_size= 25; //min # of points of each cluster. If less than this -> remove cluster
	
	plane_distance_threshold= 0.1; // RANSAC parameter. Max distance from a point to a plane so that such point is considered part of the plane
	plane_RANSAC_max_iter= 250; // Max # of iterations for the plane extraction. RANSAC parameter.
	plane_normal_distance_weight= 0.05; // RANSAC parameter. See documentation
	normal_radius_search= 0.1; // Create normals using points closer than this distance
	
	min_plane_size= 100; // min # of points of each plane. If less than this -> do not extract plane
	min_density= 200; // min density of points of each cluster. If less dense than this -> remove cluster
	cylinder_normal_distance_weight= 0.1; // RANSAC parameter. See documentation
	cylinder_max_radius= 0.12; // Extract cylinder with radius less than this only
	min_cylinder_size= 25; // extract cylinders with at least this # of points
	min_association_distance= 1; // a cylinder is associated to a landmark in the map if is closer than this distance
}

Cylinder::Cylinder()
{}

Frame::Frame()
{}

Landmark::Landmark()
{}

