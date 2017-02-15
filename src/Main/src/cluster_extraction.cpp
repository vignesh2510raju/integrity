/* This function extract the clusters from the cloud. For two grounps of points 
to form two clusters, the minimum distance between any point in one cluster and 
another point in the other cluster must be less than some specified value. */


#include "../header/cluster_extraction.h"

void cluster_extraction 
        (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clusters,
         Parameters p)
{   

  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (p.min_cluster_distance);
  ec.setMinClusterSize (p.min_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  // cluster_temp contains the cluster that we input into clusters
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_temp;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
                                                 it != cluster_indices.end (); ++it)
  {
    cluster_temp.reset (new pcl::PointCloud<pcl::PointXYZ>());

    for (std::vector<int>::const_iterator pit = it->indices.begin ();
                                          pit != it->indices.end (); ++pit)
    {
      cluster_temp->points.push_back (cloud->points[*pit]); 
    }
    if (cluster_temp->points.size () < p.min_cluster_size)
      break;

    cluster_temp->width = cluster_temp->points.size ();
    cluster_temp->height = 1;
    cluster_temp->is_dense = true;

    // Add cluster
    clusters.push_back(cluster_temp);

    // std::cout << "Cluster " << clusters.size() - 1 << " has " << clusters.back()->points.size () 
    //           << " points." << std::endl;

    // If we want to save to clusters to disk - false default
    if (p.SWITH_WRITE_CLUSTERS)
    {
      pcl::PCDWriter writer;
      std::stringstream ss;
      ss << "cloud_cluster_" << clusters.size() - 1 << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *clusters.back(), false); 
    }
  } 

  // cout<< "# Features extracted before eliminating the planes is: "
  //     << clusters.size()<< endl;
}