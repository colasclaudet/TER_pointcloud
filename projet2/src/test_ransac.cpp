#include <iostream>
#include <thread>

 #include <pcl/point_types.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
 //tenter un exemple sur un echantillon en fct des voisins
int nb_cloud = 0;
using namespace std::chrono_literals;
// This function displays the help
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
} 
//dont use it
pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
//new
//return viewer with a color for the cloud
// but dont use it
pcl::visualization::PCLVisualizer::Ptr
simpleVisCol (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
	single_color)
{
  // -----------------------------------------------
  // -----Open 3D viewer and add color point cloud--
  // -----------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr addVisualiser()
{
  // ---------------------
  // -----set 3D viewer --
  // ---------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr
finaliseVis (pcl::visualization::PCLVisualizer::Ptr viewer)
{
  for(int i=0;i<nb_cloud;i++)
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"+i);
  
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
pcl::visualization::PCLVisualizer::Ptr
addPtsCloudColor (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
	single_color)
{
  // -----------------------------------------------
  // -----Open 3D viewer and add color point cloud--
  // -----------------------------------------------
  
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+nb_cloud);
  nb_cloud = nb_cloud +1;
  return (viewer);
}

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer::Ptr viewer;
  //load cloud
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }
  
  //new
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
	single_color (cloud, 0, 0, 255);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy = cloud;
  viewer = addVisualiser();
  //viewer = 
  addPtsCloudColor(viewer, cloud, single_color);
  //endnew
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
  //////////////////////////////
  //for(int j=0;j<10;j++)
  int j=0;
  while(cloud->size()>100)
  {
  j++;
  std::cout<<cloud->size()<<endl;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::vector<int> inliers;
  pcl::PointIndices::Ptr suppression_inliers(new pcl::PointIndices());
  
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    std::cout << "Plan" << endl;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    std::cout << "Plan 1" << endl;
    ransac.setDistanceThreshold (20);
    ransac.setProbability(0.05);

    std::cout << "Plan 2" << endl;
    ransac.computeModel();
    std::cout << "Plan 3" << endl;

    ransac.getInliers(inliers);
    
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    std::cout << "Sphere" << endl;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.setProbability(0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  std::cout << "Copy" << endl;
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
  /*std::for_each(pcl::PointXYZ p, final)
  {
	std::cout<<"ok "<<endl;
  }*/
  for(int i = 0; i < final->size(); i++)
  {
	  suppression_inliers->indices.push_back(i);
  }
  
  
  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  std::cout << "Viewer" << endl;
  
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
  { 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
	single_color (cloud, 255, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"); //nope
	//viewer = simpleVis(final);
	
	//viewer = simpleVisCol(final, single_color);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
	single_color_gen (final, (j*100)%255, (j*20)%255, (j*30)%255);
	//viewer = addPtsCloudColor(viewer,cloud,single_color_gen);
	addPtsCloudColor(viewer,final,single_color_gen);
	
	extract.setInputCloud(cloud);
    extract.setIndices(suppression_inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
	
    
  }
  else
    viewer = simpleVis(cloud);
  }
  finaliseVis(viewer);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return 0;
 }
