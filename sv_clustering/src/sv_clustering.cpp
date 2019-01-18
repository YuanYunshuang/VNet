#include "sv_clustering.h"

SVSC::SVSC(){
   // Set default
   _disable_transform = true;
   _voxel_resolution = 0.004f;
   _seed_resolution = 0.04f;
   _color_importance = 0.2f;
   _spatial_importance = 0.4f;
   _normal_importance = 1.0f;
}

SVSC::~SVSC(){

}


bool SVSC::setOptions(int argc, char** argv){
   if (argc < 2)
   {
     pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
                                 "--NT Dsables the single cloud transform \n"
                                 "-v <voxel resolution>\n-s <seed resolution>\n"
                                 "-c <color weight> \n-z <spatial weight> \n"
                                 "-n <normal_weight>\n", argv[0]);
     return (0);
   }


   _disable_transform = pcl::console::find_switch (argc, argv, "--NT");


   _voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
   if (_voxel_res_specified)
     pcl::console::parse (argc, argv, "-v", _voxel_resolution);

   _seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
   if (_seed_res_specified)
     pcl::console::parse (argc, argv, "-s", _seed_resolution);

   if (pcl::console::find_switch (argc, argv, "-c"))
     pcl::console::parse (argc, argv, "-c", _color_importance);

   if (pcl::console::find_switch (argc, argv, "-z"))
     pcl::console::parse (argc, argv, "-z", _spatial_importance);

   if (pcl::console::find_switch (argc, argv, "-n"))
     pcl::console::parse (argc, argv, "-n", _normal_importance);

   return 1;
}


  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////
void SVSC::process(PointCloudT::Ptr cloud){
  pcl::SupervoxelClustering<PointT> super(_voxel_resolution, _seed_resolution);
  if (_disable_transform)
    //super.setUseSingleCameraTransform(0);

  super.setInputCloud(cloud);
  super.setColorImportance(_color_importance);
  super.setSpatialImportance(_spatial_importance);
  super.setNormalImportance(_normal_importance);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight("Extracting supervoxels!\n");
  super.extract(supervoxel_clusters);
  pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size ());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
  viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
  viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  pcl::console::print_highlight("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency(supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
  for ( ; label_itr != supervoxel_adjacency.end(); )
  {
    //First get the label
    uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
    for ( ; adjacent_itr!=supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
      adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
  }

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
  }
}


void SVSC::addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}
