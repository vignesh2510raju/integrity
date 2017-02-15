#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int user_input(bool options_flag, 
                  int c,
                  std::string filename, 
                  float max_radius, float nd_weight,float min_cluster_distance,float min_density,float xlim,float ylim,float zlim, 
                  int min_cluster_size,int min_plane_size,int min_cylinder_size,
                  int argc);