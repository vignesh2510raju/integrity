#include <iostream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <string>
#include <Eigen/Dense>

int read_matrices_pose(std::map<int, Eigen::Matrix4d>& matrices);