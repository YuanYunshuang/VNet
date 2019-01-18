// This file cannot be compiled, its just an example of using mean shift class



   MeanShift ms;
   bool in = ms.setInputCloud(filtered_cloud);
   ms.setKNNRadius(0.05);
   ms.process();
   auto objects = ms.getOutputCloudPCL();
   for(auto obj:objects){
      cout<<obj->points.size()<<endl;
      viewPointCloud(obj);
   }
