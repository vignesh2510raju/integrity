
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/don.h>

using namespace pcl;
using namespace std;

// To string function
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


int main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  double scale1;

  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;

  if (argc < 6)
  {
    cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << endl;
    exit (EXIT_FAILURE);
  }

  /// the file to read from.
  string infile = argv[1];
  /// small scale
  istringstream (argv[2]) >> scale1;
  /// large scale
  istringstream (argv[3]) >> scale2;
  istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  istringstream (argv[5]) >> segradius;   // threshold for radius segmentation

  // Load cloud in blob format
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile.c_str (), blob);
  pcl::PointCloud<PointXYZI>::Ptr cloud (new pcl::PointCloud<PointXYZI>);
  pcl::fromPCLPointCloud2 (blob, *cloud);

  // Create visualizer and color for the clusters
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white_color (cloud, 255, 255, 255);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> intensity_distribution(cloud, "intensity");   
  // viewer.addPointCloud <pcl::PointXYZ>(cloud, white_color, "cloud");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );

  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZI>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZI> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZI> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZI, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  // ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  ne.setViewPoint (0.0, 0.0, 0.0);

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZI, PointNormal>(*cloud, *doncloud); // uninitialized vector field for our point cloud

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZI, PointNormal, PointNormal> don;
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
  cout<< "number of points in doncloud before filtering: "<< doncloud->size() << endl;

  // Save DoN features
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
             new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));

  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;
  cout<< "number of points in doncloud after filtering: "<< doncloud->size() << endl;

  // writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  // Create a map of pointClouds to store the clusters
  std::map<int, pcl::PointCloud<PointXYZI>::Ptr > cloud_clusters_store;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<PointXYZI>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
      cloud_cluster->points.push_back (cloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    cloud_cluster->width = int (cloud_cluster->points.size ());
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //Save cluster
    cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    // writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    cloud_clusters_store[j]= cloud_cluster;

  }

  string id;
  for (int i = 0; i < j; ++i)
  {
    id= "cluster" + patch::to_string(i);
    viewer.addPointCloud<pcl::PointXYZI> (cloud_clusters_store[i], id); 
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
      ((double) std::rand() / RAND_MAX), ((double) std::rand() / RAND_MAX), ((double) std::rand() / RAND_MAX), id); 
  }


  // Display the result
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }


}