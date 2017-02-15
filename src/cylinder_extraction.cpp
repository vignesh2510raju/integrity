#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// Adds a points cloud with a random color - To add clusters
pcl::visualization::PCLVisualizer addRandomColorPointCloud (pcl::visualization::PCLVisualizer viewer, 
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
      }
      else
      {
        viewer.addPointCloud<pcl::PointXYZ> (newClouds[j], random_color, cloud_name);
      }
      if (OPT_BIG_POINTS){
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name);
      }

      cout<< "Visualization - Adding point cloud: " << cloud_name<< endl;
    }
    j++;
  }
  
  return(viewer);
}


// ---------------------------------------------------- MAIN ----------------------------------------------------
int 
main (int argc, char** argv)
{

  // User input
  std::string filename= argv[1];
  float max_radius= atof(argv[2]);      // 0.15
  float nd_weight= atof(argv[3]);       // 0.1
  // float slope= atof(argv[4]);            // 1.0f
  // float initial_distance= atof(argv[5]); // 0.1
  // float max_distance= atof(argv[6]);     // 2
  // float scale1= atof(argv[7]);
  // float scale2= atof(argv[8]);
  // float threshold= atof(argv[9]);
  // float set_radius= atof(argv[10]);



  // Parameters
  int min_cluster_size= 30;
  int min_plane_size= 200;
  float min_cluster_distance= 0.3;
  float min_density= 400;
  float min_cylinder= 30;

  bool SWITH_VOXEL_GRID= false;
  bool SWITH_WRITE_CLUSTERS= false;



  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("../pcl_files/" + filename + ".pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // eliminate far-away points
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-20.0, 20.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.5, 50.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-20, 20.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  if (SWITH_VOXEL_GRID)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  }
  else
  {
    cloud_filtered= cloud;
    std::cout << "PointCloud not filted - same as original cloud" << std::endl; 
  }
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (min_cluster_distance); // 0.3
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (250000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // clusters contains all clusters in a map
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "Cluster " <<j << "has   " << cloud_cluster->points.size () << " points." << std::endl;

    if (SWITH_WRITE_CLUSTERS)
    {
      pcl::PCDWriter writer;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    }

    // save the cluster in the map
    clusters[j] = cloud_cluster;

    j++;
  }

  cout<< "map size before eliminating the planes is: "<< clusters.size()<< endl;




  // Extract Planes from clusters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);

  j= 0;
  while(j < clusters.size())
  {
    while (clusters[j]->points.size () > min_cluster_size)
    {
      // Segment the largest planar component from the remaining cluster cloud
      seg.setInputCloud (clusters[j]);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        // std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (clusters[j]);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);

      // Extract plane
      if (cloud_plane->points.size () > min_plane_size)
      {
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*clusters[j]);
        std::cout << "PointCloud representing the planar component from cluster "<< j <<" has: " << cloud_plane->points.size () << " data points." << std::endl;
        // If the remainder is small -> remove this cluster b/c there is no cylinder
        if (clusters[j]->points.size() < min_cluster_size)
        {
          clusters[j].reset();
          break; // out of the current cluster loop
        }
      }
      else
      {
        // cout<< "The plane has too few points"<< endl;
        break;
      } 
    }

    j++;
  }

  cout<< "map size after eliminating the small clusters is: "<< clusters.size()<< endl;

  // Find maximun segment in each point cloud
  pcl::PointXYZ pmin, pmax;
  double maxDistance, area, density;

  j= 0;
  while(j < clusters.size())
  {
    // maxDistance= pcl::getMaxSegment(*clusters[j], pmin, pmax);
    area= pcl::calculatePolygonArea (*clusters[j]);
    density= clusters[j]->points.size() / area;

    std::cout<<"Cluster " << j<< "   #points: "<<clusters[j]->points.size()<< "     Density: "<< density<< "     Area: "<< area<< endl;

    if (density < min_density)
    {
      clusters[j].reset();
    }
    j++;
  }


  // Fit cylinders to the remainding clusters
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod (tree);


  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg_normals.setOptimizeCoefficients (true);
  seg_normals.setModelType (pcl::SACMODEL_CYLINDER);
  seg_normals.setMethodType (pcl::SAC_RANSAC);
  seg_normals.setNormalDistanceWeight (nd_weight); // 0.1
  seg_normals.setMaxIterations (10000);
  seg_normals.setDistanceThreshold (0.2); // 0.2
  seg_normals.setRadiusLimits (0.0, max_radius); // 0.1
  seg_normals.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg_normals.setEpsAngle (pcl::deg2rad (15.0f));

  // Obtain the cylinder inliers and coefficients for each cluster (just one cylinder per cluster)
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;

  j= 0;
  while(j < clusters.size() )
  {
    // allocate new memory to the cloud cylinder, the one before has been saved to cloud cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());

    if (clusters[j] != NULL)
    {
      // compute normals
      ne.setInputCloud (clusters[j]);
      ne.setKSearch (20); // 50
      ne.compute (*cloud_normals);

      // Segment cylinder
      seg_normals.setInputCloud (clusters[j]);
      seg_normals.setInputNormals (cloud_normals);
      seg_normals.segment (*inliers_cylinder, *coefficients_cylinder);

      // Extract indices
      extract.setInputCloud (clusters[j]);
      extract.setIndices (inliers_cylinder);
      extract.setNegative (false);
      extract.filter (*cloud_cylinder);
      if (cloud_cylinder->points.empty ()) 
      {
        std::cerr << "Can't find the cylindrical component." << std::endl;
        cylinders[j].reset();
      }
      else
      {
        std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        // std::cout<< "Coefficients of the cylinder: "<< *coefficients_cylinder << endl;
        if (cloud_cylinder->points.size() > min_cylinder)
        {
          cylinders[j]= cloud_cylinder;
        }
        else
        {
          cylinders[j].reset();
        }
      }
    }
    j++;
  }


  // visualization
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  viewer.addPointCloud <pcl::PointXYZ>(cloud_filtered, white_color, "cloud");
  // viewer= addRandomColorPointCloud(viewer, clusters, "clusters", false, false);
  viewer= addRandomColorPointCloud(viewer, cylinders, "cylinders", true, true);

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
