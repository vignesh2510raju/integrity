
#include "../header/plane_from_cluster_2.h"


void plane_from_cluster_2 
        (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > &clusters, 
         Parameters p)
{

// Create the normal estimation class, and pass the input dataset to it
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud (clusters[0]);

// Create an empty kdtree representation, and pass it to the normal estimation object.
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree 
                              (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod (tree);

// Output datasets
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals 
                              (new pcl::PointCloud<pcl::Normal>);

// Use all neighbors in a sphere of radius 3cm
ne.setRadiusSearch (p.plane_radius_search);

// Compute the features
ne.compute (*cloud_normals);

// Extract Planes from clusters
pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZ, pcl::Normal, pcl::Label > mps;
mps.setMinInliers (250);
mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
mps.setDistanceThreshold (0.02); // 2cm
mps.setInputNormals (cloud_normals);
mps.setInputCloud (clusters[0]);
std::vector< pcl::PlanarRegion< pcl::PointXYZ > > regions;



std::vector<pcl::ModelCoefficients> coefficients;
std::vector<pcl::PointIndices> inliers;

mps.segment (coefficients, inliers);

std::cout<< coefficients.size()<< endl;

// for (size_t i = 0; i < regions.size (); i++)
// {
//   Eigen::Vector3f centroid = regions[i].getCentroid ();
//   Eigen::Vector4f model = regions[i].getCoefficients ();
//   pcl::PointCloud boundary_cloud;
//   boundary_cloud.points = regions[i].getContour ();
//   printf ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
//           centroid[0], centroid[1], centroid[2],
//           model[0], model[1], model[2], model[3],
//           boundary_cloud.points.size ());
//  }


}










// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane 
//                                   (new pcl::PointCloud<pcl::PointXYZ> ());
// seg1.setOptimizeCoefficients (true);
// seg1.setModelType (pcl::SACMODEL_NORMAL_PLANE);
// seg1.setMethodType (pcl::SAC_RANSAC);
// seg1.setMaxIterations (1000);
// seg1.setDistanceThreshold (p.plane_distance_threshold);
// seg1.setNormalDistanceWeight (p.plane_normal_distance_weight);


// cout<< "# Features before extracting the planes from clusters is: "
//     << clusters.size()<< endl;

// int j= 0;
// while(j < clusters.size())
// {
//      while (clusters[j]->points.size () > p.min_cluster_size)
//      {
//         ne.setInputCloud (clusters[j]);

//         // Create an empty kdtree representation, and pass it to the normal estimation object.
//         // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree 
//                                       (new pcl::search::KdTree<pcl::PointXYZ> ());
//         ne.setSearchMethod (tree);

//         // Output datasets
//         pcl::PointCloud<pcl::Normal>::Ptr cloud_normals 
//                                       (new pcl::PointCloud<pcl::Normal>);

//         // Use all neighbors in a sphere of radius 3cm
//         ne.setRadiusSearch (p.plane_radius_search);

//         // Compute the features
//         ne.compute (*cloud_normals);

//         // Segment the largest planar component from the remaining cluster cloud
//         seg1.setInputCloud (clusters[j]);
//         seg1.setInputNormals (cloud_normals);

//         seg1.segment (*inliers, *coefficients);
        
//         if (inliers->indices.size () == 0)
//         {
//           std::cout << "Could not estimate a planar model for the given dataset." 
//                     << std::endl;
//           break;
//         }

//         // Extract the planar inliers from the input cloud
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
        
//         extract.setInputCloud (clusters[j]);
//         extract.setIndices (inliers);
//         extract.setNegative (false);

//         // Get the points associated with the planar surface
//         extract.filter (*cloud_plane);

//         // Extract plane
//         if (cloud_plane->points.size () > p.min_plane_size)
//         {
//           // Remove the planar inliers, extract the rest
//           extract.setNegative (true);
//           extract.filter (*clusters[j]);
//           std::cout << "PointCloud representing the planar component from cluster "
//                     << j <<" has: " << cloud_plane->points.size () 
//                     << " data points." << std::endl;
//           // If the remainder is small -> remove this cluster b/c there is no cylinder
//           if (clusters[j]->points.size() < p.min_cluster_size)
//           {
//             clusters[j].reset();
//             break; // out of the current cluster loop
//           }
//         }
//         else
//         {
//           //cout<< "The plane has too few points"<< endl;
//           break;
//         } 
//       }

//       // Next cluster
//       std::cout<< "Next cluster"<< '\n';
//       j++;
// }  

// cout<< "# Features after extracting the planes from clusters is: "
//     << clusters.size()<< endl;

// // Check the density of clustes, if it's too low -> eliminate the cluster
// double area, density;

// j= 0;
// while(j < clusters.size())
// {
//   area= pcl::calculatePolygonArea (*clusters[j]);
//   density= clusters[j]->points.size() / area;

//   std::cout<<"Cluster " << j<< "   #points: "
//            <<clusters[j]->points.size()<< "     Density: "
//            << density<< "     Area: "<< area<< endl;

//   if (density < p.min_density)
//   { 
//     clusters[j].reset();
//   }
//   j++;
// }
// }