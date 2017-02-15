#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

using namespace std;
using namespace pcl;

int main (int argc, char* argv[])
{
  // User input
  string filename= argv[1];
  float leaf_size= atof(argv[2]);			 // 0.4f
  float max_window_size= atof(argv[3]);  // 30
  float slope= atof(argv[4]);            // 1.0f
  float initial_distance= atof(argv[5]); // 0.1
  float max_distance= atof(argv[6]);     // 2
  float scale1= atof(argv[7]);
  float scale2= atof(argv[8]);
  float threshold= atof(argv[9]);
  float set_radius= atof(argv[10]);



  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (filename, *cloud);
  std::cout << "Cloud before filtering: " << cloud->size() <<  std::endl;

  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloud_filtered);
  cloud= cloud_filtered;                                 // Try changing just the pointer -- faster!!
  std::cout << "Cloud after filtering: " << cloud->size() <<  std::endl;

  // Create the ground filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (max_window_size); 
  pmf.setSlope (slope); 
  pmf.setInitialDistance (initial_distance); 
  pmf.setMaxDistance (max_distance); 
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
  cloud= cloud_filtered;                                 // Try changing just the pointer -- faster!!
  std::cout << "Ground cloud after extracting the ground: " << cloud->size() << std::endl;

  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZ>::Ptr tree;
  tree.reset (new pcl::search::KdTree<PointXYZ> (false));

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
  ne.setInputCloud (cloud);   //   Try using the complete cloud to estimate the normal better!!
  ne.setSearchMethod (tree);
  ne.setViewPoint (0.0, 0.0, 0.0);

  // calculate normals with the SMALL scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the LARGE scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud); // copy the points XYZ
  cout << "Calculating DoN... " << endl;

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);
  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
             new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));

  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  // Apply filter
  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);
  condrem.filter (*doncloud_filtered);
  doncloud = doncloud_filtered;
  cout<< "number of points in doncloud after don filtering: "<< doncloud->size() << endl;

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> red_color (doncloud, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud <pcl::PointXYZ>(cloud, white_color, "cloud");
  viewer.addPointCloud <pcl::PointNormal>(doncloud, red_color, "doncloud");

  // Display the result
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);


}