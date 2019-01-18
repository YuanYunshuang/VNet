#ifndef __SUPERVOXEL_CLUSTERING_H__
#define __SUPERVOXEL_CLUSTERING_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <utility>
// PCL specific includes
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/common/colors.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

using namespace std;
// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

class SVSC{ //SuperVoxel Spectral Clustering
public:
   SVSC();
   ~SVSC();

   bool setOptions(int argc, char** argv);
   void process(PointCloudT::Ptr cloud);
   void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                          PointCloudT &adjacent_supervoxel_centers,
                                          std::string supervoxel_name,
                                          boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);



private:
	bool _disable_transform;
   float _voxel_resolution;
   bool _voxel_res_specified;
   float _seed_resolution;
   bool _seed_res_specified;
   float _color_importance;
   float _spatial_importance;
   float _normal_importance;
};


#endif
