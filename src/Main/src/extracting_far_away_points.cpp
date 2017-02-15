#include "../header/extracting_far_away_points.h"

void extracting_far_away_points (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                  Parameters p)
{ 

  // eliminate far-away x points
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-p.xlim, p.xlim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // eliminate far-away y points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-p.ylim, p.ylim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  // eliminate far-away z points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-p.zlim, 200.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);





  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  float radius = p.rlim;

  kdtree.setInputCloud (cloud);
  pcl::PointXYZ searchPoint;
  searchPoint.x= 0;
  searchPoint.y= 0;
  searchPoint.z= 0;

  std::vector<int> idx;
  std::vector<float> pointRadiusSquaredDistance;
  if ( kdtree.radiusSearch (searchPoint, radius, idx, pointRadiusSquaredDistance) > 0 )
  {
    newCloud->points.resize( idx.size() );
    for (size_t i = 0; i < idx.size (); ++i)
    {
      newCloud->points[i]= cloud->points[ idx[i] ];
    }
  }

  cloud= newCloud;
  
  cout<< "Cloud filtered"<< endl;
}