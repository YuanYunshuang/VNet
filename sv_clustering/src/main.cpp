/* This programm read the pointclouds which contains more than one object from
   rosbags and extract each object, and transform and write it to an txt file.
   The input should be sting of an directory contains rosbags.
*/

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>



#include <boost/foreach.hpp>


#include <sys/types.h>
#include <cstdlib>
#include <dirent.h>
#include <vector>
#include <string>

#include "data_transform.h"
#include "sv_clustering.h"

using namespace std;

#define foreach BOOST_FOREACH


int main (int argc, char** argv)
{
   string root_dir = "/home/ophelia/data/rosbags_30cm/white_all";
   string savepath = "/home/ophelia/test_data";

   // make save directories
   const int dir_err = system(("mkdir -p " + savepath).c_str());
   if (-1 == dir_err)
   {
   printf("Error creating directory!\n");
   exit(1);
   }

   std::vector<std::string> files;
   read_directory(root_dir, files);

   string tmp = "bag";
   for(size_t j=0; j<files.size(); j++){
      if(files[j].find(tmp) != string::npos){
         string filename = root_dir + "/" + files[j];
         // Read rosbag and write ply and pcd
         rosbag::Bag bag;
         bag.open(filename, rosbag::bagmode::Read);

         std::vector<std::string> topics;
         topics.push_back(std::string("/camera/depth_registered/points"));
         topics.push_back(std::string("/camera/color/image_raw"));

         rosbag::View view(bag, rosbag::TopicQuery(topics));
         cout<<"processing file: "<<files[j]<<"..."<<endl;
         foreach(rosbag::MessageInstance const m, view)
         {
         	sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
         	if (cloud_msg != NULL){
         		// Container for original & filtered data
         		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
         		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
         		pcl::PCLPointCloud2 cloud_filtered;

         		// Convert to PCL data type
         		pcl_conversions::toPCL(*cloud_msg, *cloud);

         		// Perform the actual filtering
         		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
         		sor.setInputCloud (cloudPtr);
         		sor.setLeafSize (0.002, 0.002, 0.002);
         		sor.filter (cloud_filtered);
         		std::cout << "cloud_filtered:" << cloud_filtered.data.size()<< std::endl;

         		// Convert from pointcloud2 to PointT
         		pcl::PointCloud<PointT>::Ptr rgb_cloud(new pcl::PointCloud<PointT>);
         		pcl::fromPCLPointCloud2(cloud_filtered,*rgb_cloud);
         		std::cout << "rgb_cloud:" << rgb_cloud->points.size()<< std::endl;

               // for(size_t c=0; c<50; c++){
               //    cout<<rgb_cloud->points[c].x<<","<<rgb_cloud->points[c].y<<","<<rgb_cloud->points[c].z<<endl;
               // }

         		if(rgb_cloud->points.size() > 500){
         			// Preprocessing the data
                  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
						pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>);

                  if(getObjects(rgb_cloud,cloud_plane,cloud_outliers)){
                     // for(size_t c=0; c<50; c++){
                     //    cout<<cloud_outliers->points[c].x<<","<<cloud_outliers->points[c].y<<","<<cloud_outliers->points[c].z<<endl;
                     // }
                     pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
                     filter(cloud_outliers,filtered_cloud);
                     viewPointCloud(filtered_cloud);
                     cout<<filtered_cloud->points.size()<<endl;
                     // Clustering the point cloud to extract objects
                     //pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
                     //cloud_in = pcl::copyPointCloud(cloud_in,filtered_cloud);
                     SVSC svsc;
                     svsc.setOptions(argc, argv);
                     svsc.process(filtered_cloud);




                     goto LABEL;
                  } //ransac to extract the plane and objects

         		}
         	}

         	sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
         	if (img != NULL){
         		//std::cout << "image TODO" << std::endl;

         	}
         }//BOOST_FOREACH
LABEL:
            ;

         bag.close();
      }
   }


}
