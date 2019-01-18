#ifndef __UTILS_H__
#define __UTILS_H__


#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <cstdlib>
#include <dirent.h>
#include <vector>
#include <string>
#include <math.h>
#include <map>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
// For RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;

std::map<string, int> OBJ_MAP = {

	{"Axis_H", 1 },
	{"Axis_V", 2 },
	{"Bearing_Box_H", 3 },
	{"Bearing_Box_V", 4 },
	{"Bearing_H", 5 },
	{"Bearing_V", 6 },
	{"Distance_Tube_H", 7 },
	{"Distance_Tube_V", 8 },
	{"F20_20_B_H", 9 },
	{"F20_20_B_V", 10 },
	{"F20_20_G_H", 11 },
	{"F20_20_G_V", 12 },
	{"F40_40_B_H", 13 },
	{"F40_40_B_V", 14 },
	{"F40_40_G_H", 15 },
	{"F40_40_G_V", 16 },
	{"M20_100_H", 17 },
	{"M20_100_V", 18 },
	{"M20_H", 19 },
	{"M20_V", 20 },
	{"M30_H", 21 },
	{"M30_V", 22 },
	{"Motor_H", 23 },
	{"Motor_V", 24 },
	{"R20_H", 25 },
	{"R20_V", 26 }

};

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

void calTransMatrix(pcl::ModelCoefficients coefficients, Eigen::Matrix4f& trans){
	//assume the input trans is a identity matrix
	auto a = -coefficients.values[0];
	auto b = -coefficients.values[1];
	auto c = -coefficients.values[2];
	auto d = -coefficients.values[3];
	auto root = sqrt(a*a+b*b+c*c);
	auto ctheta = c/root; //cos(theta)
	auto stheta = sqrt(a*a+b*b)/root; //sin(theta)
	auto u1 = b/root;
	auto u2 = -a/root;
	trans(0,0) = ctheta+u1*u1*(1-ctheta);
	trans(0,1) = u1*u2*(1-ctheta);
	trans(0,2) = u2*stheta;
	trans(1,0) = u1*u2*(1-ctheta);
	trans(1,1) = ctheta+u2*u2*(1-ctheta);
	trans(1,2) = -u1*stheta;
	trans(2,0) = -u2*stheta;
	trans(2,1) = u1*stheta;
	trans(2,2) = ctheta;
	trans(2,3) = -d/c;
}

void calTransMatrix(Eigen::Matrix3f m, Eigen::Vector4f t, Eigen::Matrix4f& trans){
	//assume the input trans is a identity matrix
	auto a = m(0,0);
	auto b = m(1,0);
	auto c = 0.0;
	auto root = sqrt(a*a+b*b+c*c);
	auto ctheta = a/root; //cos(theta)
	auto stheta = sqrt(c*c+b*b)/root; //sin(theta)

	trans(0,0) = ctheta;
	trans(0,1) = -stheta;
	trans(1,0) = stheta;
	trans(1,1) = ctheta;
	//trans(0,3) = -t(0);
	//trans(1,3) = -t(1);
}

void viewPointCloud(pcl::PointCloud<PointT>::Ptr transformed_plane,
	pcl::PointCloud<PointT>::Ptr transformed_object, Eigen::Vector4f meanvalues=Eigen::Vector4f::Zero()){

	pcl::visualization::PCLVisualizer viewer ("Point clouds comparison");
	if(meanvalues!=Eigen::Vector4f::Zero()){
		pcl::ModelCoefficients coeff;
		coeff.values.resize (4);
		coeff.values[0] = meanvalues[0];
		coeff.values[1] = meanvalues[1];
		coeff.values[2] = meanvalues[2];
		coeff.values[3] = 0.003;
		viewer.addSphere (coeff);
	}

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<PointT>
	plane_color_handler (transformed_plane, 20, 230, 20); //green
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud (transformed_plane, plane_color_handler, "transformed_plane");

	pcl::visualization::PointCloudColorHandlerCustom<PointT>
	object_color_handler (transformed_object, 230, 20, 20); // Red
	viewer.addPointCloud (transformed_object, object_color_handler, "transformed_object");

	viewer.addCoordinateSystem (0.1, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_plane");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_object");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
	viewer.close();
}

void viewPointCloud(const pcl::PointCloud<PointT>::Ptr cloud){

	pcl::visualization::PCLVisualizer viewer ("Single Point cloud");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<PointT>
	plane_color_handler (cloud, 20, 230, 20); //green
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud (cloud, plane_color_handler, "cloud");


	viewer.addCoordinateSystem (0.1, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
	viewer.close();
}

void write2txt(const char* filename, pcl::PointCloud<PointT>::Ptr cloud) {

	ofstream output(filename);
	cout << "Writing txt: " << filename << endl;
	for (int i = 0; i < cloud->points.size(); i++)
	{

		// unpack rgb into r/g/b
		uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
		uint8_t r = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8) & 0x0000ff;
		uint8_t b = (rgb) & 0x0000ff;
		output << std::fixed << std::setprecision(6) << cloud->points[i].x << " ";
		output << std::fixed << std::setprecision(6) << cloud->points[i].y << " ";
		output << std::fixed << std::setprecision(6) << cloud->points[i].z << " ";
		output << (int)r << " " << (int)g << " " << (int)b << endl;
	}
	output.close();
}


#endif
