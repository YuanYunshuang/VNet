
#include "utils.h"

using namespace std;




int main (int argc, char** argv)
{
	string root_dir = "/home/ophelia/data/tmp";
	string savepath = "/home/ophelia/data/txt";
	std::vector<std::string> sub_dirs;
	read_directory(root_dir, sub_dirs);

	for(size_t i=0; i<sub_dirs.size(); i++){
		if(sub_dirs[i].find("all") == string::npos){ //only considering one object scene now!
			string path = root_dir + "/" + sub_dirs[i];
			std::vector<std::string> files;
			read_directory(path, files);

			// make save directories
			/*string pcdpath = savepath + "/" + "PCD_objects" + "/" + sub_dirs[i];
			const int dir_err = system(("mkdir -p " + pcdpath).c_str());
			if (-1 == dir_err)
			{
			printf("Error creating pcd directory!\n");
			exit(1);
			}*/


			for(size_t j=0; j<files.size(); j++){
				if(files[j].find("pcd") != string::npos){
					string filename = path + "/" + files[j];
					cout<<"Processing "<<filename<<endl;
					// Read pcd and extract object
					pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

					if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
					{
						PCL_ERROR ("Couldn't read file. \n");
						return (-1);
					}
					else{
						// for(size_t c=0; c<50; c++){
				      //    cout<<cloud->points[c].x<<","<<cloud->points[c].y<<","<<cloud->points[c].z<<endl;
				      // }
						// cout<<"==================================="<<endl;
						// DO RANSAC
						pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
						pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
						// Create the segmentation object
						pcl::SACSegmentation<PointT> seg;
						// Optional
						seg.setOptimizeCoefficients (true);
						// Mandatory
						seg.setModelType (pcl::SACMODEL_PLANE);
						seg.setMethodType (pcl::SAC_RANSAC);
						seg.setDistanceThreshold (0.003);

						seg.setInputCloud (cloud);
						seg.segment (*inliers, *coefficients);

						if (inliers->indices.size () == 0)
						{
						PCL_ERROR ("Could not estimate a planar model for the given dataset.");
						return (-1);
						}//RANSAC end

						pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
						pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>);
						// Extract the planar inliers from the input cloud
						pcl::ExtractIndices<PointT> extract;
						extract.setInputCloud(cloud);
						extract.setIndices(inliers);
						// Get the points associated with the planar surface
						extract.filter(*cloud_plane);
						extract.setNegative(true);
						// Get the points outside plane
						extract.filter(*cloud_outliers);
						// for(size_t c=0; c<50; c++){
				      //    cout<<cloud_outliers->points[c].x<<","<<cloud_outliers->points[c].y<<","<<cloud_outliers->points[c].z<<endl;
				      // }
						if(cloud_outliers->points.size()<5)
							continue;
						// Transform the point cloud so that the workstation plane lies on the xy plane
						//Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();
						//shift(2,3) = coefficients->values[3]/coefficients->values[2];
						//cout<<shift(2,3)<<endl;
						Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
						calTransMatrix(*coefficients, trans);
						// Executing the transformation
						pcl::PointCloud<PointT>::Ptr transformed_plane (new pcl::PointCloud<PointT> ());
						pcl::PointCloud<PointT>::Ptr transformed_object (new pcl::PointCloud<PointT> ());
						// You can either apply transform_1 or transform_2; they are the same
						//pcl::transformPointCloud (*cloud, *transformed_cloud, shift);
						pcl::transformPointCloud (*cloud_plane, *transformed_plane, trans);
						pcl::transformPointCloud (*cloud_outliers, *transformed_object, trans);
						// for(size_t d=0; d<50; d++){
				      //    cout<<transformed_object->points[d].x<<","<<transformed_object->points[d].y<<","<<transformed_object->points[d].z<<endl;
				      // }
						viewPointCloud(transformed_plane,transformed_object);

						// Create the filtering object
						pcl::PassThrough<PointT> pass;
						pass.setInputCloud (transformed_object);
						pass.setFilterFieldName ("z");
						pass.setFilterLimits (0.0, 0.12);
						//pass.setFilterLimitsNegative (true);
						pass.filter (*transformed_object);
						// filter sparse outlier
						pcl::StatisticalOutlierRemoval<PointT> sor;
						sor.setInputCloud (transformed_object);
						sor.setMeanK (50);
						sor.setStddevMulThresh (0.12);
						sor.filter (*transformed_object);

						pcl::PointCloud<PointT>::Ptr cloudxy (new pcl::PointCloud<PointT>);
						//*cloudxy = *transformed_object;
						cloudxy->width = transformed_object->points.size();
						cloudxy->height = 1;
						cloudxy->points.resize (cloudxy->width * cloudxy->height);
						for (size_t k = 0; k < transformed_object->points.size(); k++) {
							cloudxy->points[k].x = transformed_object->points[k].x;
							cloudxy->points[k].y = transformed_object->points[k].y;
							cloudxy->points[k].z = 0.0;
						}

						// Compute principal directions

						/*Eigen::Vector4f pcaCentroid;
						pcl::compute3DCentroid(*cloudxy, pcaCentroid);
						Eigen::Matrix3f covariance;
						pcl::computeCovarianceMatrix(*cloudxy, pcaCentroid, covariance);
						Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
						Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
						cout<<eigenVectorsPCA<<endl;
						Eigen::Matrix4f trsf = Eigen::Matrix4f::Identity();
						calTransMatrix(eigenVectorsPCA, pcaCentroid, trsf);
						pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT> ());
						pcl::transformPointCloud (*transformed_object,*object,  trans);*/


						pcl::PCA<PointT> pca;
						pca.setInputCloud(cloudxy);
						//pca.project(*cloudSegmented, *cloudPCAprojection);
						Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
						//cout<<eigenVectorsPCA<<endl;
						Eigen::Vector4f meanvalues = pca.getMean();
						//cout<<meanvalues<<endl;

						pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT> ());
						Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();
						shift(0,3) = -meanvalues(0);
						shift(1,3) = -meanvalues(1);
						pcl::transformPointCloud (*transformed_object,*object,  shift);
						Eigen::Matrix4f trsf = Eigen::Matrix4f::Identity();
						calTransMatrix(eigenVectorsPCA, meanvalues, trsf);
						pcl::transformPointCloud (*object,*object,  trsf);
						viewPointCloud(object,transformed_object, meanvalues);

						std::stringstream ss;
						ss << std::setw(2) << std::setfill('0') << OBJ_MAP[sub_dirs[i]];
						string name = files[j];
						name = savepath + "/" + ss.str() + "_" +
									name.substr(0,name.size() - 4) + ".txt";
						write2txt(name.c_str(), object);

					}
				}
			}
		}
	}

}
