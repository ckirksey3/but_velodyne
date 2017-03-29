/*
 * Calibration3DMarker.cpp
 *
 *  Created on: 2.4.2014
 *      Author: ivelas
 */

#include "but_calibration_camera_velodyne/Calibration3DMarker.h"

#include <ros/assert.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;

namespace but_calibration_camera_velodyne {

Calibration3DMarker::Calibration3DMarker(cv::Mat _frame_gray, cv::Mat _P, ::PointCloud<Velodyne::Point> _pc,
                                         float _circ_distance, float _radius) :
    frame_gray(_frame_gray), P(_P), pc(_pc), circ_distance(_circ_distance), radius(_radius)
{

  // ---------------- GET PLANE ----------------

  ROS_INFO("Initializing calibration marker");
  Velodyne::Velodyne scan(pc);
  scan.getRings();
  scan.intensityByRangeDiff();
  PointCloud<Velodyne::Point> visible_cloud;
  scan.project(P, Rect(0, 0, 1928, 1448), &visible_cloud);

  Velodyne::Velodyne visible_scan(visible_cloud);
  visible_scan.normalizeIntensity();
  Velodyne::Velodyne thresholded_scan = visible_scan.threshold(0); //0.1

  ROS_INFO("Building xyz_cloud_ptr");
  PointCloud<PointXYZ>::Ptr xyz_cloud_ptr(thresholded_scan.toPointsXYZ());

  SampleConsensusModelPlane<PointXYZ>::Ptr model_p(
      new ::SampleConsensusModelPlane<PointXYZ>(xyz_cloud_ptr));
  RandomSampleConsensus<PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(0.1); //0.05
  ransac.computeModel();
  std::vector<int> inliers_indicies;
  ransac.getInliers(inliers_indicies);

  //Move indices into pcl object
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  inliers->indices = inliers_indicies;

  // Filter out largest plane
  ROS_INFO("Building cloud_filter");
  PointCloud<PointXYZ>::Ptr cloud_filter(thresholded_scan.toPointsXYZ());

  printf( "inlier size %d\n", inliers_indicies.size() );
  printf( "cloud_filter size %d\n", cloud_filter->size() );

  ROS_INFO("Building cloud_p");
  PointCloud<PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filter);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_p);

  // Filter out second largest plane
  SampleConsensusModelPlane<PointXYZ>::Ptr model_p2(
    new ::SampleConsensusModelPlane<PointXYZ>(cloud_p));
  RandomSampleConsensus<PointXYZ> ransac2(model_p2);
  ransac2.setDistanceThreshold(0.1); //0.05
  ransac2.computeModel();
  std::vector<int> inliers_indicies2;
  ransac2.getInliers(inliers_indicies2);

  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
  inliers2->indices = inliers_indicies2;

  printf( "inlier2 size %d\n", inliers_indicies2.size() );
  printf( "cloud_p size %d\n", cloud_p->size() );

  PointCloud<PointXYZ>::Ptr cloud_board(new pcl::PointCloud<pcl::PointXYZ>);
  ROS_INFO("Filtering out second plane");
  pcl::ExtractIndices<pcl::PointXYZ> extract_board;
  extract_board.setInputCloud (cloud_p);
  extract_board.setIndices (inliers2);
  extract_board.filter (*cloud_board);

  // Publish cloud without plane
  ros::Publisher filtered_plane_publisher;
  sensor_msgs::PointCloud2 filtered_plane_ros;
  ros::NodeHandle n;
  ROS_INFO("Publishing filtered point cloud");
  filtered_plane_publisher = n.advertise<sensor_msgs::PointCloud2>("/lidar_camera_calibration/filtered_plane",1);  
  pcl::toROSMsg(*cloud_filter, filtered_plane_ros);
  filtered_plane_publisher.publish(filtered_plane_ros); 

  pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients() );

  copyPointCloud<PointXYZ>(*xyz_cloud_ptr, inliers_indicies, plane);

  // ---------------- REMOVE LINES ----------------

  // for (int i = 0; i < 2; i++)
  // {
  //   PointCloud<PointXYZ>::Ptr plane_ptr(new PointCloud<PointXYZ>(plane));
  //   SampleConsensusModelLine<PointXYZ>::Ptr model_l(
  //       new SampleConsensusModelLine<PointXYZ>(plane_ptr));
  //   RandomSampleConsensus<PointXYZ> ransac_l(model_l);
  //   ransac_l.setDistanceThreshold(0.05); //0.02
  //   ransac_l.computeModel();
  //   vector<int> line_inliers;
  //   ransac_l.getInliers(line_inliers);
  //   if (line_inliers.empty())
  //   {
	//     ROS_INFO(" Lines not detected ");	
  //     continue;
  //   }
  //   PointCloud<PointXYZ> plane_no_line;
  //   remove_inliers(*plane_ptr, line_inliers, plane_no_line);
  //   plane = plane_no_line;
    
  // }
  // pcl::toROSMsg(plane,plane_ros);
  pcl::toROSMsg(*cloud_board,plane_ros);
}

bool Calibration3DMarker::detectCirclesInImage(vector<Point2f> &centers, vector<float> &radiuses)
{
  Image::Image img(frame_gray);
  Image::Image img_edge(img.computeEdgeImage());
  cv::Mat circle_img;
  bool retVal = img_edge.detect4Circles(Calibration3DMarker::CANNY_THRESH, Calibration3DMarker::CENTER_THRESH_DISTANCE,
                                 centers, radiuses, circle_img);
                                 
  circles_img_msg.header.stamp = ros::Time::now();
  circles_img_msg.header.frame_id = "/camera";
  circles_img_msg.encoding = sensor_msgs::image_encodings::BGR8;
  circles_img_msg.image = circle_img;
  cv::imwrite("frame_rgb_with_circles.png", circle_img);
  return retVal;                            
}


bool Calibration3DMarker::detectCirclesInPointCloud(vector<Point3f> &centers, vector<float> &radiuses)
{
  PointCloud<PointXYZ>::Ptr detection_cloud(new PointCloud<PointXYZ>);
  *detection_cloud += this->plane;

  float tolerance = 0.03; // 3cm
  int round = 1;
  vector<PointXYZ> spheres_centers;
  bool detected = false;
  std::cout << "\n Cloud Size = "<<detection_cloud->size();
  for (int iterations = 0; iterations < 64; iterations++)
  {
    /* cerr << endl << " =========== ROUND " << round++ << " =========== "
    << endl << endl;
    cerr << "detection_cloud size: " << detection_cloud->size() << endl;*/
    spheres_centers = detect4spheres(detection_cloud, radiuses);
    
    //ROS_INFO_STREAM("Spheres Detected = " << spheres_centers.size());
    if (spheres_centers.size() == 4)
    {
      order4spheres(spheres_centers);
      /*cerr << "ordered centers: " << endl;
       for (size_t i = 0; i < spheres_centers.size(); i++) {
       cerr << spheres_centers[i] << endl;
       }*/
      if (verify4spheres(spheres_centers, this->circ_distance, tolerance))
      {
        spheres_centers = refine4centers(spheres_centers, detection_cloud);
        detected = true;
        //ROS_INFO(" Spheres detected in Pointcloud ");
        break;
      }
    }
    vector<PointXYZ> possible_centers = generate_possible_centers(spheres_centers, this->circ_distance);
    generate_possible_points(this->plane, detection_cloud, possible_centers, this->circ_distance, 0.01);
  }

  if (!detected)
  {
    return false;
  }

  for (size_t i = 0; i < spheres_centers.size(); i++)
  {
    centers.push_back(Point3f(spheres_centers[i].x, spheres_centers[i].y, spheres_centers[i].z));
  }
  return true;
}

/*
bool Calibration3DMarker::detectCirclesInPointCloud(PointCloud<Velodyne::Point> ipCloud, vector<Point3f> &centers, vector<float> &radiuses)
{
  PointCloud<PointXYZ>::Ptr detection_cloud(new PointCloud<PointXYZ>);
  *detection_cloud += this->plane;

  float tolerance = 0.03; // 3cm
  vector<PointXYZ> spheres_centers;
  bool detected = false;
  int grid_dim = 500;
  float m_per_cell = 0.004;
  
  Mat depthImg  = Mat::zeros(grid_dim, grid_dim, CV_32FC1);
  
  std::cout << "\n Cloud Size = "<<ipCloud.size();
  for (unsigned int i = 0; i < ipCloud.size(); ++i) 
  {
    int x = round((grid_dim/2)-ipCloud.points[i].z/m_per_cell);
    int y = round((grid_dim/2)-ipCloud.points[i].y/m_per_cell);
    
    if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim) 
    {
		depthImg.at<float>(x,y) = ipCloud.points[i].x;
	}
  }
  
  double minVal;
  double maxVal;
  Point minLoc; 
  Point maxLoc;
  
  minMaxLoc(depthImg, &minVal, &maxVal, &minLoc, &maxLoc );
  std::cout <<"Min = "<<minVal << "Max = " << maxVal;
  //subtract(depthImg, minVal, OutputArray dst, InputArray mask=noArray(), int dtype=-1)
  divide(depthImg, maxVal, depthImg, 1, -1);

  Image::Image img(depthImg);
  Image::Image img_edge(img.computeEdgeImage());
  //img_edge.detect4Circles(Calibration3DMarker::CANNY_THRESH, Calibration3DMarker::CENTER_THRESH_DISTANCE, centers, radiuses);
  
 // namedWindow("Depth Image edges", CV_WINDOW_AUTOSIZE);
 // imshow("Depth Image edges", depthImg);
 // waitKey(0);
   
  detected = false;
  
  if (!detected)
  {
    return false;
  }

  for (size_t i = 0; i < spheres_centers.size(); i++)
  {
    centers.push_back(Point3f(spheres_centers[i].x, spheres_centers[i].y, spheres_centers[i].z));
  }
  return true;
}
*/


vector<PointXYZ> Calibration3DMarker::detect4spheres(PointCloud<PointXYZ>::Ptr plane,
                                                          vector<float> &radiuses)
{

  radiuses.clear();
  vector<PointXYZ> centers;
  std::vector<int> inliers_indicies;
  PointCloud<PointXYZ> *four_spheres = new PointCloud<PointXYZ>();
  float tolerance = 0.08;
  for (int i = 0; i < 4; i++)
  {
    SampleConsensusModelSphere<PointXYZ>::Ptr model_s(
        new SampleConsensusModelSphere<PointXYZ>(plane));
    // model_s->setRadiusLimits(0.29, 0.40);
    model_s->setRadiusLimits(0.1, 0.2);
    RandomSampleConsensus<PointXYZ> ransac_sphere(model_s);
    ransac_sphere.setDistanceThreshold(tolerance);
    ransac_sphere.computeModel();
    inliers_indicies.clear();
    ransac_sphere.getInliers(inliers_indicies);

    if (inliers_indicies.size() == 0)
    {
	    std::cout<<"\n Sphere not detected in iteration: "<<i;
      continue;
    }
    else
    {
		  std::cout<<"\n Sphere detected in iteration: "<<i;
	  }
    Eigen::VectorXf coeficients;
    ransac_sphere.getModelCoefficients(coeficients);
    //cerr << i + 1 << ". circle: " << coeficients << endl << endl;

    PointCloud<PointXYZ>::Ptr outliers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr inliers(new PointCloud<PointXYZ>);
    remove_inliers<PointXYZ>(*plane, inliers_indicies, *outliers);
    copyPointCloud<PointXYZ>(*plane, inliers_indicies, *inliers);
    plane = outliers;
    // view(plane);

    *four_spheres += *inliers;
    PointXYZ middle(coeficients(0), coeficients(1), coeficients(2));
    four_spheres->push_back(middle);
    centers.push_back(middle);

    float radius = coeficients(3);
    radiuses.push_back(radius);
  }
  PointCloud<PointXYZ>::Ptr four_spheres_ptr(four_spheres);
  return centers;
}

/*
 * Indexes of circles in marker:
 *
 * 0 1
 * 2 3
 */
bool orderX(PointXYZ p1, PointXYZ p2)
{
  return p1.x < p2.x;
}
bool orderY(PointXYZ p1, PointXYZ p2)
{
  return p1.y < p2.y;
}

void Calibration3DMarker::order4spheres(vector<PointXYZ> &spheres_centers)
{
  ROS_ASSERT(spheres_centers.size() == 4);
  sort(spheres_centers.begin(), spheres_centers.end(), orderY);
  sort(spheres_centers.begin(), spheres_centers.begin() + 2, orderX);
  sort(spheres_centers.begin() + 2, spheres_centers.begin() + 4, orderX);
}

float euclid_dist(const PointXYZ p1, const PointXYZ p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

bool Calibration3DMarker::verify4spheres(const vector<PointXYZ> &spheres_centers, float straight_distance,
                                         float delta)
{
  ROS_ASSERT(spheres_centers.size() == 4);

  vector<pair<int, int> > neighbour_indexes;
  neighbour_indexes.push_back(pair<int, int>(0, 1));
  neighbour_indexes.push_back(pair<int, int>(1, 3));
  neighbour_indexes.push_back(pair<int, int>(3, 2));
  neighbour_indexes.push_back(pair<int, int>(2, 0));

  bool res = true;
  for (vector<pair<int, int> >::iterator neighbors = neighbour_indexes.begin(); neighbors < neighbour_indexes.end();
      neighbors++)
  {
    float error = abs(
        euclid_dist(spheres_centers[neighbors->first], spheres_centers[neighbors->second]) - straight_distance);
    //cerr << "error: " << error << endl;
    if (error > delta)
      res = false;
  }
  return res;
}

/*
 * All points around the all found centers:
 * x x x
 * x   x
 * x x x
 */
vector<PointXYZ> Calibration3DMarker::generate_possible_centers(const vector<PointXYZ> &spheres_centers,
                                                                     float straight_distance)
{
  vector<PointXYZ> possible_centers;

  for (vector<PointXYZ>::const_iterator c = spheres_centers.begin(); c < spheres_centers.end(); c++)
  {
    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        if (dx || dy)
        { // omitting found center (may be false detection)
          PointXYZ new_center = *c;
          new_center.x += dx * straight_distance;
          new_center.y += dy * straight_distance;
          possible_centers.push_back(new_center);
        }
      }
    }
  }

  return possible_centers;
}

void Calibration3DMarker::generate_possible_points(PointCloud<PointXYZ> &plane,
                                                   PointCloud<PointXYZ>::Ptr detection_cloud,
                                                   const vector<PointXYZ> &possible_centers, float radius,
                                                   float tolerance)
{

  detection_cloud->clear();
  for (PointCloud<PointXYZ>::iterator pt = plane.begin(); pt < plane.end(); pt++)
  {
    int votes = 0;
    for (vector<PointXYZ>::const_iterator center = possible_centers.begin(); center < possible_centers.end();
        center++)
    {
      if (euclid_dist(*pt, *center) < radius + tolerance)
      {
        votes++;
      }
    }
    if (votes > 0)
    {
      detection_cloud->push_back(*pt);
    }
  }
}

vector<PointXYZ> Calibration3DMarker::refine4centers(vector<PointXYZ> centers,
                                                     PointCloud<PointXYZ>::Ptr detection_cloud)
{

  float z_coord = 0;
  for (PointCloud<PointXYZ>::iterator pt = detection_cloud->begin(); pt < detection_cloud->end(); pt++)
  {
    z_coord += pt->z;
  }
  z_coord /= detection_cloud->size();

  for (vector<PointXYZ>::iterator c = centers.begin(); c < centers.end(); c++)
  {
    c->z = z_coord;
  }

  return centers;
}

}
