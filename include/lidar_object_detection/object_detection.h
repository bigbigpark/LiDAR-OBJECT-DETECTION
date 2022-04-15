/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-15 10:51
 */
#pragma once

#include <iostream>
#include <ros/ros.h>

#include <lidar_msgs/SocketBox.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>

// PCL for transformation
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#include <vector>
#include <ctime>
#include <time.h>
#include <chrono>

// Linux socket communication
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>

#define BOX_SIZE        30
#define VEHICLE_HEIGHT  1.9

using namespace std;

#pragma(push, 1)
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
    // float yaw;
};
#pragma(pop)

struct Vect
{
    float   x;
    float   y;
    float   z;
    Vect(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

#pragma pack(push, 1)
typedef struct _tPointVector
{
    float   x_;
    float   y_;
    float   z_;
}PVector;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _tInfoBox
{
    Box              box_;
}BOXINFO;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _tBoundingBoxForUDP
{
    int              num_of_box_;
    BOXINFO BoxInfo_[BOX_SIZE];
}UDPBOX;
#pragma pack(pop)

class Point
{
public:
    Point(Box box, int id);
    Point(int id);

    void getTF(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);

public:
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker_;
};

class ObjectDetector
{
private:
  // Publisher
  ros::Publisher pub_object;
  ros::Publisher pub_marker;
  ros::Publisher pub_origin;
  ros::Publisher pub_udpBox;

  // Suscriber
  ros::Subscriber sub_cloud;
  
  // Node handler
  ros::NodeHandle nh_;

  // Message
  lidar_msgs::SocketBox SocketBox_msg;
  
  // Parameters for FilterCloud
  float kFilterResolution;
  Eigen::Vector4f kMinPoint;
  Eigen::Vector4f kMaxPoint;

  // Parameters for SegmentPlane
  int kMaxIterations = 300;         // 1000
  float kDistanceThreshold = 0.2;   // 0.2

  // Parameters for Clustering
  float kClusterTolerance = 0.75;   // 0.35
  int kMinSize = 12;                // 12
  int kMaxSize = 500;

  // Parameters for BBOX
  float kBBoxMinHeight;
  float kBBoxMaxHeight;
  float h;
  
  float kBBoxMinWidth;
  float kBBoxMaxWidth;
  float w;
  
  float kBBoxMinLength;
  float kBBoxMaxLength;
  float l;

  // For UDP topic
  UDPBOX udp_box_info;

  visualization_msgs::MarkerArray marker_array;

  // Now covert output back from PCL native type to ROS
  sensor_msgs::PointCloud2 ros_output;
  sensor_msgs::PointCloud2 ros_origin;
  sensor_msgs::PointCloud2 ros_object;

public:
  ObjectDetector();
  ~ObjectDetector();

  // Initialize parameter
  void init();

  // callback
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs);

  // func.
  Box BoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);
  pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr& inliers, const  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceThreshold);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize);
  void BBOX(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud);
};