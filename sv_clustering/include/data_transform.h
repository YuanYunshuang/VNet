#ifndef __DATA_TRAMSFORM_H__
#define __DATA_TRAMSFORM_H__

#include "utils.h"

using namespace std;




bool getObjects(const pcl::PointCloud<PointT>::Ptr cloud,
              pcl::PointCloud<PointT>::Ptr &cloud_plane,
              pcl::PointCloud<PointT>::Ptr &cloud_outliers){
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

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			// Get the points associated with the planar surface
			extract.filter(*cloud_plane);
			extract.setNegative(true);
			// Get the points outside plane
			extract.filter(*cloud_outliers);

         Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
         calTransMatrix(*coefficients, trans);
         // Executing the transformation
         // pcl::PointCloud<PointT>::Ptr transformed_plane (new pcl::PointCloud<PointT> ());
         // pcl::PointCloud<PointT>::Ptr transformed_objects (new pcl::PointCloud<PointT> ());
         // You can either apply transform_1 or transform_2; they are the same
         //pcl::transformPointCloud (*cloud, *transformed_cloud, shift);
         pcl::transformPointCloud (*cloud_plane, *cloud_plane, trans);
         pcl::transformPointCloud (*cloud_outliers, *cloud_outliers, trans);

         // for(size_t i=0; i<50; i++){
         //    cout<<cloud_outliers->points[i].x<<","<<cloud_outliers->points[i].y<<","<<cloud_outliers->points[i].z<<endl;
         // }
         viewPointCloud(cloud_plane,cloud_outliers);
			if(cloud_outliers->points.size()<5)
				return false;

         cout<<"Object extracted. Point size Objects: "<<cloud_outliers->points.size()<<endl;
      return true;

}

void filter(const pcl::PointCloud<PointT>::Ptr &cloud_in,
            pcl::PointCloud<PointT>::Ptr &cloud_out){
      cout<<"Before filtering: "<<cloud_in->points.size()<<endl;
      // for(size_t i=0; i<cloud_in->points.size(); i++){
      //    cout<<cloud_in->points[i].x<<","<<cloud_in->points[i].y<<","<<cloud_in->points[i].z<<endl;
      // }
      // Create the filtering object
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud (cloud_in);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 0.12);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_out);
      cout<<"After PassThrough: "<<cloud_out->points.size()<<endl;
      // filter sparse outlier
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud (cloud_out);
      sor.setMeanK (50);
      sor.setStddevMulThresh (0.12);
      sor.filter (*cloud_out);
      cout<<"Noise removed. filtered size: "<<cloud_out->points.size()<<endl;
}


void ComposedTransform(pcl::PointCloud<PointT>::Ptr &cloud_in,
                        pcl::PointCloud<PointT>::Ptr &cloud_out){

      pcl::PointCloud<PointT>::Ptr cloudxy (new pcl::PointCloud<PointT>);
      //*cloudxy = *transformed_object;
      cloudxy->width = cloud_in->points.size();
      cloudxy->height = 1;
      cloudxy->points.resize (cloudxy->width * cloudxy->height);
      for (size_t k = 0; k < cloud_in->points.size(); k++) {
            cloudxy->points[k].x = cloud_in->points[k].x;
            cloudxy->points[k].y = cloud_in->points[k].y;
            cloudxy->points[k].z = 0.0;
      }
      // Compute principal directions
      pcl::PCA<PointT> pca;
      pca.setInputCloud(cloudxy);
      //pca.project(*cloudSegmented, *cloudPCAprojection);
      Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
      //cout<<eigenVectorsPCA<<endl;
      Eigen::Vector4f meanvalues = pca.getMean();
      //cout<<meanvalues<<endl;

      Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();
      shift(0,3) = -meanvalues(0);
      shift(1,3) = -meanvalues(1);
      pcl::transformPointCloud (*cloud_in,*cloud_out,  shift);
      Eigen::Matrix4f trsf = Eigen::Matrix4f::Identity();
      calTransMatrix(eigenVectorsPCA, meanvalues, trsf);
      pcl::transformPointCloud (*cloud_out,*cloud_out,  trsf);
}

#endif
