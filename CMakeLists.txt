cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(narf_keypoint_extraction)

find_package(PCL 1.2 REQUIRED)

include_directories(${PROYECT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## catkin_make -DCMAKE_BUILD_TYPE=Debug

add_executable (main 	src/Main/src/MAIN.cpp
						src/Main/src/classes.cpp
						src/Main/src/visualize.cpp 
						src/Main/src/user_input.cpp 
						src/Main/src/extracting_far_away_points.cpp 
						src/Main/src/cluster_extraction.cpp 
						src/Main/src/plane_from_cluster.cpp 
						src/Main/src/cylinder_segmentation.cpp 
						src/Main/src/extracting_voxel_grid.cpp 
						src/Main/src/read_transformations.cpp 
						src/Main/src/read_matrices_pose.cpp 
						##src/Main/src/read_velo_to_cam.cpp
						src/Main/src/Inputs.cpp
						src/Main/src/UpdateMAP_saveFrame.cpp
						)


target_link_libraries (main ${PCL_LIBRARIES})






##add_executable (pcd_write src/pcd_write.cpp)
##target_link_libraries (pcd_write ${PCL_LIBRARIES})

##add_executable (pcd_viewer src/pcd_viewer.cpp)
##target_link_libraries (pcd_viewer ${PCL_LIBRARIES})

##add_executable (range_image_visualization src/range_image_visualization.cpp)
##target_link_libraries (range_image_visualization ${PCL_LIBRARIES})

##add_executable (pcd_read src/pcd_read.cpp)
##target_link_libraries (pcd_read ${PCL_LIBRARIES})

##add_executable (random_sample_consensus src/random_sample_consensus.cpp)
##target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

##add_executable (cylinder_segmentation src/cylinder_segmentation.cpp)
##target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

##add_executable (planar_segmentation src/planar_segmentation.cpp)
##target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

##add_executable (cylinder_extraction src/cylinder_extraction.cpp)
##target_link_libraries (cylinder_extraction ${PCL_LIBRARIES})

##add_executable (narf_keypoint_extraction src/narf_keypoint_extraction.cpp)
##target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})

##add_executable (normal_estimation src/normal_estimation.cpp)
##target_link_libraries (normal_estimation ${PCL_LIBRARIES})

##add_executable (voxel_grid src/voxel_grid.cpp)
##target_link_libraries (voxel_grid ${PCL_LIBRARIES})

##add_executable (range_image_creation src/range_image_creation.cpp)
##target_link_libraries (range_image_creation ${PCL_LIBRARIES})

##add_executable (cloud_viewer src/cloud_viewer.cpp)
##target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

##add_executable (matrix_transform src/matrix_transform.cpp)
##target_link_libraries (matrix_transform ${PCL_LIBRARIES})

##add_executable (range_image_border_extraction src/range_image_border_extraction.cpp)
##target_link_libraries (range_image_border_extraction ${PCL_LIBRARIES})

##add_executable (bare_earth src/bare_earth.cpp)
##target_link_libraries (bare_earth ${PCL_LIBRARIES})

##add_executable (don_segmentation src/don_segmentation.cpp)
##target_link_libraries (don_segmentation ${PCL_LIBRARIES})

##add_executable (extract_ground_and_don src/extract_ground_and_don.cpp)
##target_link_libraries (extract_ground_and_don ${PCL_LIBRARIES})
