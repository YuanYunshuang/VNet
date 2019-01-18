#include "mean_shift.h"

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

MeanShift::MeanShift() : m_size(0), R(0.0f){
   set_kernel(NULL);
   EPSILPON = 0.005;
   MERGE_THR = 0.005;
   C = 2.0;
}

MeanShift::MeanShift(double (*_kernel_func)(double,double)): m_size(0), R(0.0f){
   set_kernel(kernel_func);
   EPSILPON = 0.001;
   MERGE_THR = 0.001;
   C = 2.0;
}


MeanShift::~MeanShift()
{
}


void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;
    }
}

bool MeanShift::setInputCloud(const pcl::PointCloud<PointT>::Ptr pPntCloud)
{
	m_size = pPntCloud->points.size();
	mv_pntcld.resize(m_size);
	mv_local_mode.resize(m_size);

	mp_pointcloud = pPntCloud;

	for (size_t i = 0; i < m_size; ++i)
	{
		mv_pntcld[i].x = pPntCloud->points[i].x;
		mv_pntcld[i].y = pPntCloud->points[i].y;
		mv_pntcld[i].z = pPntCloud->points[i].z;
      mv_pntcld[i].r = pPntCloud->points[i].r;
		mv_pntcld[i].g = pPntCloud->points[i].g;
		mv_pntcld[i].b = pPntCloud->points[i].b;
	}

	return true;
}

inline bool MeanShift::execMeanShiftEachPoint(const PointT &in_pnt, PointRGB &out_pnt)
{
	// Set up KDTree
	pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
	tree->setInputCloud (mp_pointcloud);

	// Main Loop
	PointT pnt = in_pnt;

	while (1)
	{
		// Neighbors containers
		std::vector<int> k_indices;
		std::vector<float> k_distances;

		float sum_weigh = 0.0;
		float x = 0.0f, y = 0.0f, z = 0.0f;
		float dist_pnts = 0.0f;

		tree->radiusSearch (pnt, R, k_indices, k_distances);

		for (size_t i = 0; i < k_indices.size(); i++)
		{
			size_t index = k_indices[i];
			PointT &nbhd_pnt = mp_pointcloud->points[index];
			float sqr_dist = k_distances[i];
			float gauss_param = sqr_dist / (R * R);
			float w = gauss(gauss_param);

			x += nbhd_pnt.x * w;
			y += nbhd_pnt.y * w;
			z += nbhd_pnt.z * w;
			sum_weigh += w;
		}
		x = x / sum_weigh;
		y = y / sum_weigh;
		z = z / sum_weigh;

		float diff_x = x - pnt.x, diff_y = y - pnt.y, diff_z = z - pnt.z;

		dist_pnts = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

		if (dist_pnts <= MeanShift::EPSILPON)
		{
			break;
		}
		pnt.x = x;
		pnt.y = y;
		pnt.z = z;
	};

	out_pnt.x = pnt.x;
	out_pnt.y = pnt.y;
	out_pnt.z = pnt.z;

	return true;
}

bool MeanShift::mergeSameLocalModePoint(const VecPoint &v_localmode, VecVecPoint &vv_pnt)
{
	assert(v_localmode.size() == m_size);
   std::pair<bool, int> m(false,-1);
	std::vector<std::pair<bool,int>> v_iscluster(m_size,m);

	for (size_t i = 0; i < m_size; i++)
	{
		for (size_t j = i + 1; j < m_size; j++)
		{
			PointRGB  lmpnt1 = v_localmode[i];
			PointRGB  lmpnt2 = v_localmode[j];
			PointRGB  pnt1= {mp_pointcloud->points[i].x, mp_pointcloud->points[i].y, mp_pointcloud->points[i].z,
            mp_pointcloud->points[i].r, mp_pointcloud->points[i].g, mp_pointcloud->points[i].b};
			PointRGB  pnt2= {mp_pointcloud->points[j].x, mp_pointcloud->points[j].y, mp_pointcloud->points[j].z,
            mp_pointcloud->points[j].r, mp_pointcloud->points[j].g, mp_pointcloud->points[j].b};
			float dist = 0.0f;

			dist = getPointsDist(lmpnt1, lmpnt2);
			if (dist <= MeanShift::MERGE_THR)
			{
				//mode of 2 points are very near to each other
				VecPoint v_pnt;

				if ( !v_iscluster[i].first &&  !v_iscluster[j].first)
				{
					v_iscluster[i] = {true, (int)vv_pnt.size()};
					v_pnt.push_back(pnt1);

					v_iscluster[j] = {true, (int)vv_pnt.size()};
					v_pnt.push_back(pnt2);

					vv_pnt.push_back(v_pnt);
				}
            // pnt1 is clustered, pnt2 not, put pnt2 to the pnt1's cluster
				else if ( v_iscluster[i].first &&  !v_iscluster[j].first)
				{
					vv_pnt[v_iscluster[i].second].push_back(pnt2);
               v_iscluster[j] = {true, v_iscluster[i].second};
				}
           // // pnt2 is clustered, pnt1 not, put pnt1 to the pnt2's cluster
				else if ( !v_iscluster[i].first &&  v_iscluster[j].first)
				{
               vv_pnt[v_iscluster[j].second].push_back(pnt1);
               v_iscluster[i] = {true, v_iscluster[j].second};
				}
				else
				{
					//both are clustered, pass
				}
			}  //  if (dist <= NEAREST_ZERO)

		}  //  for j
	}  //  for i

	for (size_t i = 0; i < m_size; i++)
	{
		if (!v_iscluster[i].first)
		{
			PointRGB  pnt = {mp_pointcloud->points[i].x, mp_pointcloud->points[i].y, mp_pointcloud->points[i].z,
            mp_pointcloud->points[i].r, mp_pointcloud->points[i].g, mp_pointcloud->points[i].b};
			VecPoint v_pnt;

			v_iscluster[i].first = true;
			v_pnt.push_back(pnt);
			vv_pnt.push_back(v_pnt);
		}
	}

	return true;
}

bool MeanShift::process()
{
	for (size_t i = 0; i < m_size; ++i)
	{
		const PointT &pnt = mp_pointcloud->points[i];
		execMeanShiftEachPoint(pnt, mv_local_mode[i]);
	}

	mergeSameLocalModePoint(mv_local_mode, vv_pnt);

	return true;
}


std::vector<pcl::PointCloud<PointT>::Ptr> MeanShift::getOutputCloudPCL(){
   std::vector<pcl::PointCloud<PointT>::Ptr> list_pointcloud;
   for(size_t i=0; i<vv_pnt.size(); i++){
      pcl::PointCloud<PointT>::Ptr cld(new pcl::PointCloud<PointT>);
      for (size_t j = 0; j < vv_pnt[i].size(); j++) {
         PointT point;
         point.x = vv_pnt[i][j].x;
         point.y = vv_pnt[i][j].y;
         point.z = vv_pnt[i][j].z;
         point.r = vv_pnt[i][j].r;
         point.g = vv_pnt[i][j].g;
         point.b = vv_pnt[i][j].b;
         cld->points.push_back(point);
      }
      list_pointcloud.push_back(cld);
   }

   return list_pointcloud;
}
