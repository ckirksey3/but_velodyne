#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/Header.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Calibration.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include <but_calibration_camera_velodyne/Image.h>

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;

// marker properties:
double STRAIGHT_DISTANCE; // 23cm
double RADIUS; // 8.25cm

ros::Publisher transformed_publisher;
ros::Publisher plane_publisher;
ros::Publisher circle_image_publisher;

Mat projection_matrix;
Mat frame_rgb;
Velodyne::Velodyne pointcloud;
bool doRefinement = false;

bool writeAllInputs()
{
  bool result = true;
  ROS_INFO("Saving point cloud, image to files");
  pointcloud.save("velodyne_pc.pcd");
  cv::imwrite("frame_rgb.png", frame_rgb);
  cv::FileStorage fs_P("projection.yml", cv::FileStorage::WRITE);
  fs_P << "P" << projection_matrix;
  fs_P.release();

  return result;
}


void filterPCL_2(pcl::PointCloud<pcl::PointXYZI> &ipCloud, pcl::PointCloud<pcl::PointXYZI> &filteredCloud)
{
  ROS_INFO("Filtering point cloud");
  size_t npoints = ipCloud.points.size();
    //filteredCloud.points.resize(npoints);
  for(unsigned int k = 0; k < npoints; k++)
  {
    if((ipCloud.at(k).x < 20) && (ipCloud.at(k).x > 0) && (ipCloud.at(k).z < 1.5) && (ipCloud.at(k).z > -1.5))
    { 
      filteredCloud.points.push_back(ipCloud.at(k));
    }
  } 
    // pass along original time stamp and frame ID
    filteredCloud.header.stamp = ipCloud.header.stamp;
    filteredCloud.header.frame_id = ipCloud.header.frame_id;
}


Calibration6DoF calibration(bool doRefinement, std_msgs::Header header)
{
  Mat frame_gray;
  cvtColor(frame_rgb, frame_gray, CV_BGR2GRAY);

  // Marker detection:
  Calibration3DMarker marker(frame_gray, projection_matrix, pointcloud.getPointCloud(), STRAIGHT_DISTANCE, RADIUS);
  
  // marker.plane_ros.header.stamp = ros::Time::now();
  // marker.plane_ros.header.frame_id = "/velodyne";
  marker.plane_ros.header = header;
  plane_publisher.publish(marker.plane_ros); 
  
  
  vector<float> radii2D;
  vector<Point2f> centers2D;
  if (!marker.detectCirclesInImage(centers2D, radii2D))
  {
    ROS_INFO(" Circles not detected in Image ");
    circle_image_publisher.publish(marker.circles_img_msg.toImageMsg());
    return Calibration6DoF::wrong();
  }
  else
  {
    ROS_INFO(" Circles detected in Image ");
    circle_image_publisher.publish(marker.circles_img_msg.toImageMsg());
  }
  
  float radius2D = accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  
  
  vector<float> radii3D;
  vector<Point3f> centers3D;
  if (!marker.detectCirclesInPointCloud(centers3D, radii3D))
  {
    ROS_INFO(" Circles not detected in Pointcloud ");  
    return Calibration6DoF::wrong();
  }
  else
  {
    ROS_INFO(" Circles detected in Pointcloud ");
  }
  float radius3D = accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();
  


  // rough calibration
  Calibration6DoF translation = Calibration::findTranslation(centers2D, centers3D, projection_matrix, radius2D,
                                                             radius3D);

  if (doRefinement)
  {
    ROS_INFO("Coarse calibration:");
    translation.print();
    ROS_INFO("Refinement process started - this may take a minute.");
    size_t divisions = 5;
    float distance_transl = 0.02;
    float distance_rot = 0.01;
    Calibration6DoF best_calibration, avg_calibration;
    Calibration::calibrationRefinement(Image::Image(frame_gray), pointcloud, projection_matrix, translation.DoF[0],
                                       translation.DoF[1], translation.DoF[2], distance_transl, distance_rot, divisions,
                                       best_calibration, avg_calibration);
    return avg_calibration;
  }
  else
  {
    return translation;
  }
}

void callback(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& msg_info,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{

  ROS_INFO_STREAM("Image received at " << msg_img->header.stamp.toSec());
  ROS_INFO_STREAM( "Camera info received at " << msg_info->header.stamp.toSec());
  ROS_INFO_STREAM( "Velodyne scan received at " << msg_pc->header.stamp.toSec());
  
  // Loading camera image:
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;

  // Loading projection matrix:
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;
  }
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
  
  // string ros_message;
  // ros_message << "Projection matrix = "<< endl << " "  << projection_matrix << endl << endl;
  ROS_INFO("Projection matrix:");
  ROS_INFO_STREAM(projection_matrix);

  // Loading Velodyne point cloud
  ROS_INFO("Loading point cloud");
  PointCloud<Velodyne::Point> pc;
  
  pcl::PointCloud<pcl::PointXYZI> ipCloud, filteredCloud; 
  sensor_msgs::PointCloud2 filtered_ros_cloud;
  
  pcl::fromROSMsg (*msg_pc,ipCloud);
  filterPCL_2(ipCloud, filteredCloud);
  pcl::toROSMsg (filteredCloud, filtered_ros_cloud);
  
  ROS_INFO("Publishing point cloud");
  transformed_publisher.publish(filtered_ros_cloud);
     
  ROS_INFO("Convert new pc from ros messg");
  pcl::fromROSMsg(filtered_ros_cloud, pc);

  // ROS_INFO("Transform pc");
  // x := x, y := -z, z := y,
  pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, -M_PI/2, -M_PI/2, M_PI/2);
  // pointcloud = Velodyne::Velodyne(pc);
  
  // calibration:
  writeAllInputs();
  ROS_INFO("get calibration params");
  Calibration6DoF calibrationParams = calibration(doRefinement, msg_pc->header);
  if (calibrationParams.isGood())
  {
    ROS_INFO_STREAM("Calibration succeeded, found parameters:");
    calibrationParams.print();
    shutdown();
  }
  else
  {
    ROS_WARN("Calibration failed - trying again after 1s ...");
    ros::Duration(10).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");

  int c;
  while ((c = getopt(argc, argv, "r")) != -1)
  {
    switch (c)
    {
      case 'r':
        doRefinement = true;
        break;
      default:
        return EXIT_FAILURE;
    }
  }
  bool offline;
  
  ros::NodeHandle n;
  // n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
  // n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
  // n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
  // n.getParam("/but_calibration_camera_velodyne/marker/circles_distance", STRAIGHT_DISTANCE);
  // n.getParam("/but_calibration_camera_velodyne/marker/circles_radius", RADIUS);
  // n.getParam("/but_calibration_camera_velodyne/offline_calibration", offline);

  CAMERA_FRAME_TOPIC = "/sensors/camera/image_rect_color";
  CAMERA_INFO_TOPIC = "/sensors/camera/camera_info";
  //TODO adjust these
  STRAIGHT_DISTANCE = 0.45;
  RADIUS = 0.15;
  VELODYNE_TOPIC = "/sensors/velodyne_points";
  offline = false;
  
  transformed_publisher = n.advertise<sensor_msgs::PointCloud2>("/lidar_camera_calibration/point_cloud",1);  
  plane_publisher = n.advertise<sensor_msgs::PointCloud2>("/lidar_camera_calibration/plane",1);  
  circle_image_publisher = n.advertise<sensor_msgs::Image>("/lidar_camera_calibration/circles_image",1); 
    
  if (offline)
  {
    ROS_INFO("Offline Mode");
     // image_transport::ImageTransport it(n);
      //image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::io::loadPCDFile("velodyne_pc.pcd", pcl_msg);
    pcl_msg.header.stamp = ros::Time::now();
    pcl_msg.header.frame_id = "/velodyne";
    sensor_msgs::PointCloud2ConstPtr pcl_ptr = sensor_msgs::PointCloud2ConstPtr (&pcl_msg);
  
    cv::Mat image = cv::imread("frame_rgb.png", CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = "/camera";
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_msg.image = image;
    
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = img_msg.header;
    camera_info.height = 1448;
    camera_info.width = 1928;
    camera_info.distortion_model = "plumb_bob";
    double distortion[] = {-0.1743330744813683, 0.03844085133381333, -0.000440382483744044, 0.0001904752615392291, 0};
    std::vector<double> temp (distortion, distortion + sizeof(distortion)/sizeof(double));
    camera_info.D = temp;
    camera_info.K = {934.7580051348723, 0, 941.5324993504246, 0, 937.0009750408727, 721.1446607992325, 0, 0, 1};
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.P = {766.831298828125, 0, 936.3543676629779, 0, 0, 833.7805786132812, 718.78976712035, 0, 0, 0, 1, 0};
    sensor_msgs::CameraInfoConstPtr info_ptr = sensor_msgs::CameraInfoConstPtr (&camera_info);
            
    ros::Rate loop_rate(5);
    while(n.ok())
    {
    callback(img_msg.toImageMsg(), info_ptr, pcl_ptr);
    ros::spinOnce();
    loop_rate.sleep();
  }
  }
  else
  {   
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
 
  std::cout << "\n Waiting for Topics = ";
  ROS_INFO("TEST: Waiting for topics...");
  ros::spin();
  }
  
  return EXIT_SUCCESS;
}
