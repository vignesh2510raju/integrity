#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "../header/visualize.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

pcl::visualization::PCLVisualizer visualize (pcl::visualization::PCLVisualizer viewer, 
                                                            std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > &newClouds,
                                                            std::string cloud_initial_name,
                                                            bool OPT_RED,
                                                            bool OPT_BIG_POINTS)
{

  int R, G, B;

  int j= 0;
  while(j < newClouds.size())
  {
    if (newClouds[j] != NULL)
    {
      R= 40 + std::rand() * (255-40);
      G= 40 + std::rand() * (255-40);
      B= 40 + std::rand() * (255-40);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> random_color(newClouds[j], R, G, B);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color   (newClouds[j], 255, 0, 0);

      // generate name of the cluster
      std::string cloud_name= cloud_initial_name + patch::to_string(j);


      if (OPT_RED)
      {
        viewer.addPointCloud<pcl::PointXYZ> (newClouds[j], red_color, cloud_name);
        //viewer.setFullscreen(true); 
        //viewer.createInteractor();
      }
      else
      {
        viewer.addPointCloud<pcl::PointXYZ> (newClouds[j], red_color, cloud_name);
      }
      if (OPT_BIG_POINTS){
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name);
      }

      //cout<< "Visualization - Adding point cloud: " << cloud_name<< endl;
    }
    j++;
  }
  
  return(viewer);
}