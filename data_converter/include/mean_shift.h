#ifndef __MEAN_SHIFT_H__
#define __MEAN_SHIFT_H__

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


using namespace std;
typedef pcl::PointXYZRGB PointT;

struct PointRGB{
   float x, y, z;
   uint8_t r,g,b;
};

class MeanShift{
public:

   typedef std::vector<PointRGB> VecPoint;
   typedef std::vector<VecPoint> VecVecPoint;
	float EPSILPON;
   float MERGE_THR;
	float C;  //Const for Gauss Kernel Function

   MeanShift();
   MeanShift(double (*_kernel_func)(double,double));
   ~MeanShift();

	 /** \brief set input point cloud
	   * \param[in]  cloud input cloud
	   */
	bool setInputCloud(const pcl::PointCloud<PointT>::Ptr );

	 /** \brief cloud after spectral clustering  */
	VecVecPoint & getOutputCloud()
	{
		return vv_pnt;
	}

	/** \brief set kNN Radius
	   * \param[in]  radius
	   */
	bool setKNNRadius(const float radius)
	{
		R = radius;
        return true;
	}

   float getPointsDist(PointRGB p1, PointRGB p2){
      float tmp;
      tmp = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
      return tmp;
   }

   std::vector<pcl::PointCloud<PointT>::Ptr> getOutputCloudPCL();

	 /** \brief do mean shift	   */
	bool process();


   void set_kernel( double (*_kernel_func)(double,double) );

private:
	size_t m_size;  //!the number of points need to be processed
	pcl::PointCloud<PointT>::Ptr mp_pointcloud;
	VecPoint mv_pntcld;
	VecPoint mv_local_mode;
	VecVecPoint vv_pnt;  //!point cloud after spectral clustering
   double (*kernel_func)(double,double);

	float R;  //!radius for kNN

	 /** \brief implement mean shift for every point
	   * \param[in]  in_pnt
	   * \param[out] out_pnt
	   */
	inline bool execMeanShiftEachPoint(const PointT &in_pnt, PointRGB &out_pnt);

	 /** \brief merge the points with similar local mode
	   * \param[in] v_localmode local mode of each point
	   * \param[out] vv_pnt points merged
	   */
	bool mergeSameLocalModePoint(const VecPoint &v_localmode, VecVecPoint &vv_pnt);

	inline float gauss(float x)
	{
		return C * sqrt(x) * exp(-0.5 * x);
	}


};




#endif
