
/*  This function removes the points conforming planes from the clusters. Then
those clusters with low density or low number of points are removed so that only
remain clusters that can form robust cylinders. */

#include "../header/plane_from_cluster.h"


void plane_from_cluster (
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clusters, 
            Parameters p)
{

double area, density;

// Extract Planes from clusters
pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_planes;
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane 
                                  (new pcl::PointCloud<pcl::PointXYZ> ());
seg_planes.setOptimizeCoefficients (false);
seg_planes.setModelType (pcl::SACMODEL_NORMAL_PLANE);
seg_planes.setMethodType (pcl::SAC_RANSAC);
seg_planes.setDistanceThreshold (p.plane_distance_threshold);
seg_planes.setMaxIterations (p.plane_RANSAC_max_iter);
seg_planes.setNormalDistanceWeight (p.plane_normal_distance_weight);

// Create an empty kdtree representation
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree 
                              (new pcl::search::KdTree<pcl::PointXYZ> ());

// Create the normal estimation class, and pass the input dataset to it
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setSearchMethod (tree);
//ne.setKSearch (5);
ne.setRadiusSearch (p.normal_radius_search); // Use neighbors in a sphere of radius 3cm

// Normals 
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals 
                           (new pcl::PointCloud<pcl::Normal>);

int j= 0;
while(j < clusters.size())
{
   while (clusters[j]->points.size () > p.min_cluster_size)
   {
      ne.setInputCloud (clusters[j]);

      // Compute the features
      ne.compute (*cloud_normals);

      // Segment the largest planar component from the remaining cluster cloud
      seg_planes.setInputCloud (clusters[j]);
      seg_planes.setInputNormals (cloud_normals);

      seg_planes.segment (*inliers, *coefficients);
      
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

      // Extract plane if big enough
      if (cloud_plane->points.size () > p.min_plane_size)
      {
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*clusters[j]);
        // std::cout << "Plane from cluster "
        //           << j <<" has: " << cloud_plane->points.size () 
        //           << " data points." << std::endl;
      }
      else // if the plane is small -> stop looking for planes
      {
        // cout<< "The plane has too few points"<< endl;
        break; // next cluster, no planes in this one
      } 
    }
    // std::cout<< "Next cluster"<< '\n';
    j++; // Next cluster
}  

// Check the density of clustes, if it's too low -> eliminate the cluster
for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator 
                          it= clusters.begin(); it != clusters.end(); )
{

  area= pcl::calculatePolygonArea (**it);
  density= (*it)->points.size() / area;  
  // std::printf("Cluster: %ld \t#Points: %lu \tDensity: %.2f", 
                // std::distance(clusters.begin(), it), (*it)->points.size(),
                // density);

  // First, check number of points
  if ((*it)->points.size() < p.min_cluster_size)
  {
    clusters.erase(it);
    //cout<< endl;
  }
  // Second, check density
  else
  {
    area= pcl::calculatePolygonArea (**it);
    density= (*it)->points.size() / area;

    if (density < p.min_density) // eliminate cluster
    { 
      // std::cout<< "Eliminating cluster "<< it->first << endl;
      clusters.erase(it);
      //cout<< endl;
    }
    else // Good cluster -> check next cluster
    {
      // cout<< " <--- KEEP"<< endl;
      it++;
    }
  }
}


// cout<< "# Features after extracting the planes from clusters is: "
//     << clusters.size()<< endl;
}