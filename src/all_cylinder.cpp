#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "read_matrices_pose.h"
#include "read_velo_to_cam.h"
#include "read_transformations.h"
#include <vector>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// Adds a points cloud with a random color - To add clusters and cylinders
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

// ---------------------------------------------------- MAIN ----------------------------------------------------
int 
main (int argc, char** argv)
{
  
  int counter= 3; //depends on the number of .pcd files in the folder
  
  float imagearray[counter][50]; //creating a 2D array for storing coordinates of all the cylinders per PCD file
  int countforcyl[100]; // counter for counting number of cylinders per PCD file
  float accuscore[counter];
  float match[counter];
  double tran[4][4];
  match[0] = 0;

  // User input
  int c; bool options_flag= false;
  std::string filename;
  float max_radius, nd_weight, min_cluster_distance, min_density, xlim, ylim, zlim;  
  int min_cluster_size, min_plane_size, min_cylinder_size;

  // This introduces the options from the terminal so that you don't have to compile the script every time you want to change something
  while (!options_flag && (c= getopt(argc,argv,"a:b:c:d:e:f:g:h:")) != -1 )
  {
    switch (c)
    {
      case 'a':
        filename= optarg;
        cout<< "Filename is: "<< optarg<< endl;
        break;
      case 'b':
        max_radius= atof(optarg);    // 0.15
        cout<< "Maximum radius is: "<< optarg<< endl;
        break;
      case 'c':
        nd_weight= atof(optarg);     // 0.1
        cout<< "Normal-distance weight is: "<< optarg<< endl;
        break;
      case 'd':
        min_cluster_size= atoi(optarg);
        cout<< "Minimum cluster size is: "<< optarg<< endl;
        break;
      case 'e':
        min_plane_size= atoi(optarg);
        cout<< "Minimum plane size is: "<< optarg<< endl;
        break;
      case 'f':
        min_cluster_distance= atof(optarg);
        cout<< "Minimum cluster distance is: "<< optarg<< endl;
        break;
      case 'g':
        min_density= atof(optarg);
        cout<< "Minimum density of a cluster is: "<< optarg<< endl;
        break;
      case 'h':
        min_cylinder_size= atoi(optarg);
        cout<< "Minimum cylinder size is: "<< optarg<< endl;
        break;
      case 'x':
        xlim= atof(optarg);
        cout<< "X limits are: [-"<< xlim<< ",+"<< xlim<< "]"<< endl;
        break;
      case 'y':
        ylim= atof(optarg);
        cout<< "Y limits are: [-"<< ylim<< ",+"<< ylim<< "]"<< endl;
        break;
      case 'z':
        zlim= atof(optarg);
        cout<< "Z limits are: [-"<< zlim<< ",+ inf]"<< endl;
        break;

      // If you don't want to set all the options just run the code with an option that is not here and it will take the defaults
      case '?':
        cout<< "invalid argumnet, using default options"<< endl;
        options_flag= true;           
        //filename= "000009";           // a)
        max_radius= 0.12f;            // b)
        nd_weight= 0.1f;              // c)
        min_cluster_size= 50;         // d)
        min_plane_size= 250;          // e) 
        min_cluster_distance= 0.1;    // f)
        min_density= 400;             // g)
        min_cylinder_size= 30;        // h)
        xlim= 14;                     // x)
        ylim= 14;                     // y)
        zlim= 0.5;                    // z)
        break;

      default:
        cout<<" Default reached"<<endl;
        return (1);
    }
  }

  // Parameters
  bool SWITH_VOXEL_GRID= false; // downsample the point cloud
  bool SWITH_WRITE_CLUSTERS= false; // write the clusters to disk 

  // To get the pose
  std::vector<Eigen::Matrix4d> T;
  T = read_transformations();  
  
  //std::cout << "Trying to do a cout here : " << T[0, 0] << std::endl;

  for (int y = 0; y < 5; y++)
  {
    std::cout << "Transformation matrix is : " << T[y] << std::endl << std::endl;
  }
  
 /* T.setIdentity();
  T.block<3,3>(0,0)= transform.rotation();
  T.block<3,1>(0,3)= transform.translation();*/

  // tran[0] = T[0];

 // tran[0][0] = T[0*3 + 0];

 int fi= 0; //starting the counter for the first .pcd file
 for (int fi= 0; fi <= counter; ++fi)
 {
  
  std::string filename2; 
  std::stringstream sa;
  sa << setw(6) << setfill('0') << fi;
  filename2= sa.str();

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "File currently being used :"<< filename2 << ".pcd" << endl;
  reader.read ("../pcl_files/" + filename2 + ".pcd", *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // eliminate far-away x points
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-xlim, xlim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // eliminate far-away y points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-ylim, ylim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  // eliminate far-away z points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-zlim, 14.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  if (SWITH_VOXEL_GRID)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.075f, 0.075f, 0.075f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  }
  else
  {
    cloud_filtered= cloud;
    std::cout << "PointCloud not filted - same as original cloud" << std::endl; 
  }
  
  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (min_cluster_distance);
  ec.setMinClusterSize (min_cluster_size);
  // ec.setMaxClusterSize (250000);  // very high value
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

    // If we want to save to clusters to disk - false default
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
   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1;
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
   seg1.setOptimizeCoefficients (true);
   seg1.setModelType (pcl::SACMODEL_NORMAL_PLANE);
   seg1.setMethodType (pcl::SAC_RANSAC);
   seg1.setMaxIterations (1000);
   seg1.setDistanceThreshold (0.01);
   seg1.setNormalDistanceWeight (0.1);

   j= 0;
   while(j < clusters.size())
   {
     while (clusters[j]->points.size () > min_cluster_size)
     {


        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        //pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
        //pcl::ExtractIndices<pcl::PointXYZ> extract;

        ne.setInputCloud (clusters[j]);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);

      // Segment the largest planar component from the remaining cluster cloud
      seg1.setInputCloud (clusters[j]);
      seg1.setInputNormals (cloud_normals);

      seg1.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
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
          //clusters[j].reset();
          break; // out of the current cluster loop
        }
      }
      else
      {
        //cout<< "The plane has too few points"<< endl;
        break;
      } 
     }

    j++;
   }

  cout<< "map size after eliminating the small clusters is: "<< clusters.size()<< endl;

  // Check the density of clustes, if it's too low -> eliminate the cluster
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
  seg_normals.setDistanceThreshold (0.15); // 0.2
  seg_normals.setRadiusLimits (0.0, max_radius); // 0.1
  seg_normals.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg_normals.setEpsAngle (pcl::deg2rad (15.0f));

  // Obtain the cylinder inliers and coefficients for each cluster (just one cylinder per cluster)
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;

  int k = 0;

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
        //std::cerr << "Can't find the cylindrical component." << std::endl;
        //cylinders[j].reset();
      }
      else
      {
        std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        // std::cout<< "Coefficients of the cylinder: "<< *coefficients_cylinder << endl;
        if (cloud_cylinder->points.size() > min_cylinder_size)
        {
          cylinders[j]= cloud_cylinder;

          // pcl::SampleConsensusModelCylinder<pcl::PointXYZ>::Ptr model_c (new pcl::PointCloud<pcl::PointXYZ> ());
          // model_c->setInputCloud(cloud_cylinder); 
          // model_c->setInputNormals(cloud_normals); 
  
          // std::vector<int> inliers; 
          // pcl::RandomSampleConsensus<PointXYZ> Redef (model_c, 0.03); 
          // Redef->setSampleConsensusModel(model_c);
          // Eigen::VectorXf coef; 

          // Redef.setDistanceThreshold (.01); 
          // Redef.setMaxIterations(10000); 
          // Redef.computeModel(); 
          // Redef.getInliers(inliers); 
          // Redef.getModelCoefficients(coef); 

          //std::cout << coef.radius << "heehaw!!!!" << std::endl;

          //std::cout << cloud_cylinder->center << std::endl;

          pcl::CentroidPoint<pcl::PointXYZ> centroid;

          for (int z = 0; z < min_cylinder_size ; z++)
          {
              centroid.add (pcl::PointXYZ ( cloud_cylinder->points[ j + z ].x , cloud_cylinder->points[ j + z ].y , cloud_cylinder->points[ j + z ].z ));              
          }
          pcl::PointXYZ cl;
          centroid.get (cl);
          
          //Storing the values of the Centroid into the Image Array Matrix
          imagearray[fi][k] = ((int)(cl.x * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place
          imagearray[fi][k+1] = ((int)(cl.y * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place

          std::cout << "Centroid is : " << imagearray[fi][k] << ',' << imagearray[fi][k+1] << ' ' << std::endl;

          k = k+2;

          double dist;
          pcl::PointXYZ dist2;
          dist2.x = 0.0;
          dist2.y = 0.0;
          dist2.z = 0.0;
          dist = pcl::euclideanDistance ( cl , dist2 );
          dist = ((int)(dist * 10 + 0.5) / 10.0 ); 
          std::cout << "Distance from the car - " << dist << std::endl;
        }
        else
        {
          //cylinders[j].reset();
        }
      }
    }

    j++;
  }

  countforcyl[fi] = (k-2)/2 + 1; // storing the number of cyliner found in that PCD file into a separate array


  std::cout << "Number of cylinders found in the " << filename2 <<".pcd file is : "<< countforcyl[fi] << std::endl;

  // visualization
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  //viewer.setFullScreen(true); 
  //viewer.createInteractor();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  viewer.addPointCloud <pcl::PointXYZ>(cloud_filtered, white_color, "cloud");
  //viewer= addRandomColorPointCloud(viewer, clusters, "clusters", false, false); // This part of the code was meant to plot the clusters after the planar extraction
  // and then the next line would plot the cylinders with a bigger point size to make them clear, but it only plots the clusters (not the cylinders) if I uncomment this line.
  viewer= addRandomColorPointCloud(viewer, cylinders, "cylinders", true, true);
  
  // Run the viewer
  //while (!viewer.wasStopped ())
 // {
     viewer.spinOnce (0.001);
 // }

 }
   
  // for (int score1 = 1; score1 <= counter; score1++)
  // {
  //   for (int score2 = 0; score2 < countforcyl[score1]; score2+=2)
  //   {
      
  //     for (int p = 0; p < countforcyl[score1]; p+=2)
  //     {
  //       float temp1 = imagearray[score1-1][p];
  //       float temp3 = imagearray[score1-1][p+1];
        
  //       for (float temp2 = -0.8; temp2 < 0.8 ; temp2 = temp2 + 0.1)
  //       {
              
  //         if ( imagearray[score1][score2] == (temp1+temp2) && imagearray[score1][score2+1] == (temp3+temp2) )
  //         {
  //           match[score1-1]++;
  //         } 
  //       }
  //     }
  //   }

  //   std::cout << "Match for the "<< score1<< "cylinder is : " << match[score1-1] << std::endl;
  // }

 for (int score1 = 1; score1 < counter-1; score1++)
  {
    match[score1-1] = 0;
    for (int score2 = 0; score2 < countforcyl[score1]-1; score2++)
    {
      
      for (int p = 0; p < countforcyl[score1-1]; p++)
      {
        float temp1 = imagearray[score1-1][p];
        
        for (float temp2 = -0.7; temp2 < 0.7 ; temp2 = temp2 + 0.1)
        {
              
          if ( imagearray[score1][score2] == (temp1+temp2) )
          {
            match[score1-1]++;
          } 
        }
      }
    }

    std::cout << "Match for the "<< score1<< " cylinder is : " << match[score1-1] << std::endl;

    accuscore[score1-1] = match[score1-1] / countforcyl[score1-1] * 100;

    //std::cout << "Repeatability for consecutive PCD Files is : " << accuscore[score1-1] << std::endl; 

    // Code for finding the accuracy

  }
  
    ofstream arrayData("C:\\array123.txt"); // File Creation and storing the value

    for(int i=0;i<counter;i++)
    {
        arrayData<<accuscore[i]<<endl; //Outputs array to txtFile
        //std::cout << "Data Wrote !!" << std::endl; 
    }
    
  return (0);
}