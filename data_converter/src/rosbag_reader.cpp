#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sys/types.h>
#include <cstdlib>
#include <dirent.h>
#include <vector>
#include <string>

using namespace std;
typedef pcl::PointXYZRGB PointT;

void read_directory(const std::string& name, std::vector<std::string>& v)
{
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL ) {
    	if(!(!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, ".."))){
        	v.push_back(dp->d_name);
        	//std::cout << dp->d_name << std::endl;
        }
    }
    closedir(dirp);
}



int main (int argc, char** argv)
{
	string root_dir = "/home/ophelia/data/rosbags_30cm";
	string savepath = "/home/ophelia/data";
	std::vector<std::string> sub_dirs;
	read_directory(root_dir, sub_dirs);

	for(int i=0; i<sub_dirs.size(); i++){
		string path = root_dir + "/" + sub_dirs[i];
		std::vector<std::string> files;
		read_directory(path, files);

		// make save directories
		string pcdpath = savepath + "/" + "PCDs" + "/" + sub_dirs[i];
		string plypath = savepath + "/" + "PLYs" + "/" + sub_dirs[i];
		const int dir_err1 = system(("mkdir -p " + pcdpath).c_str());
		if (-1 == dir_err1)
		{
		printf("Error creating pcd directory!\n");
		exit(1);
		}
		const int dir_err2 = system(("mkdir -p " + plypath).c_str());
		if (-1 == dir_err2)
		{
		printf("Error creating ply directory!\n");
		exit(1);
		}

		string tmp = "bag";
		for(int j=0; j<files.size(); j++){
			if(files[j].find(tmp) != string::npos){
				string filename = path + "/" + files[j];
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

						if(rgb_cloud->points.size() > 500){
							// Write point cloud to pcd file

							string pcdname = pcdpath + "/pc" + to_string(j) + ".pcd";
							string plyname = plypath + "/pc" + to_string(j) + ".ply";
							pcl::io::savePCDFileASCII (pcdname, *rgb_cloud);
							pcl::io::savePLYFileBinary(plyname, *rgb_cloud);
							break;
						}
					}


					sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
					if (img != NULL){
						//std::cout << "image TODO" << std::endl;

					}
				}

				bag.close();
			}
		}
	}

}
