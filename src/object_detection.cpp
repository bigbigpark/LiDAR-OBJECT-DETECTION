/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-15 10:58
 */
#include <lidar_object_detection/object_detection.h>

Point::Point(Box box, int id)
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker_.header.frame_id = "/os_sensor";
  marker_.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_.ns = "box_marker";
  marker_.id = id;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker_.pose.position.x = (box.x_max + box.x_min) / 2;
  marker_.pose.position.y = (box.y_max + box.y_min) / 2;
  marker_.pose.position.z = (box.z_max + box.z_min) / 2;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_.scale.x = fabs(box.x_max - box.x_min);
  marker_.scale.y = fabs(box.y_max - box.y_min);
  marker_.scale.z = fabs(box.z_max - box.z_min);

  // Set the color -- be sure to set alpha to something non-zero!
  marker_.color.r = 0.0f;
  marker_.color.g = 0.3f;
  marker_.color.b = 1.0f;
  marker_.color.a = 1.0;

  marker_.lifetime = ros::Duration();
}

Point::Point(int id)
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker_.header.frame_id = "/os_sensor";
  marker_.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_.ns = "box_marker";
  marker_.id = id;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_.action = visualization_msgs::Marker::ADD;

  // Set the color -- be sure to set alpha to something non-zero!
  marker_.color.r = 0.0f;
  marker_.color.g = 0.3f;
  marker_.color.b = 1.0f;
  marker_.color.a = 1.0;

  marker_.lifetime = ros::Duration();
}


inline void Point::getTF(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster)
{
  // For PCA
  pcl::PCA<pcl::PointXYZI> pca_cloud (new pcl::PCA<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  temp_cloud = cluster;

  // cout << "c size: " << cluster->size() << endl;
  cout << fixed << setprecision(2);
  for(int i = 0; i < temp_cloud->size(); i++)
  {
    // temp_cloud->points[i].z = 0.0;
    cout << "x,y,z : " << temp_cloud->points[i].x << " " << temp_cloud->points[i].y << " " << temp_cloud->points[i].z << endl;
  }
  cout << "===" << endl;
  pca_cloud.setInputCloud(temp_cloud);
  Eigen::Vector4f pca_mean = pca_cloud.getMean();
  Eigen::Matrix3f pca_vector = pca_cloud.getEigenVectors();
  // pca_vector(0,0) = fabs(pca_vector(0,0));
  // pca_vector(1,1) = fabs(pca_vector(1,1));
  // pca_vector(2,2) = fabs(pca_vector(2,2));
  Eigen::Vector3f pca_value = pca_cloud.getEigenValues();

  // cout << "m_x, m_y: " << pca_mean(0,0) << " " << pca_mea<n(1,0) << endl;
  // cout << "mean: \n" << pca_mean << endl;
  // cout << "vector: \n" << pca_vector << endl;
  // cout << "value: \n" << pca_value << endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform (Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = pca_vector.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pca_mean.head<3>());
  // cout << "transform: \n" << projectionTransform << endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*temp_cloud, *cloudPointsProjected, projectionTransform);
  
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZI minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  // cout << "mean diagonal: \n" << meanDiagonal << endl;

  // Final transform

  const Eigen::Quaternionf bboxQuaternion(pca_vector); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = pca_vector * meanDiagonal + pca_mean.head<3>();
  // cout << "Orientation: \n" << bboxQuaternion.x() << "\n" << bboxQuaternion.y() << "\n" << bboxQuaternion.z() << "\n" << bboxQuaternion.w() << endl;
  // cout << "Translation: \n" << bboxTransform << endl;

  // cout << "===" << endl;
  // ===================

  // MARKER
  // Set 6DOF pose of the marker
  marker_.pose.position.x = bboxTransform(0,0);
  marker_.pose.position.y = bboxTransform(1,0);
  marker_.pose.position.z = bboxTransform(2,0);
  // marker_.pose.orientation.x = bboxQuaternion.x();
  // marker_.pose.orientation.y = bboxQuaternion.y();
  // marker_.pose.orientation.z = bboxQuaternion.z();
  // marker_.pose.orientation.w = bboxQuaternion.w();

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_.scale.x = fabs(maxPoint.x - minPoint.x);
  marker_.scale.y = fabs(maxPoint.y - minPoint.y);
  marker_.scale.z = fabs(maxPoint.z - minPoint.z);
}

ObjectDetector::ObjectDetector()
{

}
ObjectDetector::~ObjectDetector()
{

}

void ObjectDetector::init()
{
  // Publisher
  pub_object = nh_.advertise<sensor_msgs::PointCloud2>("/os1_object", 10);
  pub_marker = nh_.advertise<visualization_msgs::MarkerArray>("/os1_box", 10);
  pub_origin = nh_.advertise<sensor_msgs::PointCloud2>("/os_origin", 10);
  // pub_udpBox = nh_.advertise<lidar_msgs::SocketBox>("/udp_box", 10);

  // Suscriber
  sub_cloud = nh_.subscribe("/input", 10, &ObjectDetector::cloudCallback, this);

  // Parameters for FilterCloud
  kFilterResolution = 0.2;    // 0.2
  kMinPoint << -30, -8,  0.35, 1;     // -50 -6 -3 1
  kMaxPoint << 80,  8,   2.5, 1;     // 60 6.5 4 1

  // Parameters for SegmentPlane
  kMaxIterations = 300;         // 1000
  kDistanceThreshold = 0.2;  // 0.2

  // Parameters for Clustering
  kClusterTolerance = 0.75; // 0.35
  kMinSize = 12;    // 12
  kMaxSize = 500;

  // ================================================
  // Parameters for BBOX
  kBBoxMinHeight = 0.2;   //  0.75
  kBBoxMaxHeight = 3;   //  0.75
  h = kBBoxMaxHeight - kBBoxMinHeight;

  kBBoxMinWidth = 0.1;  
  kBBoxMaxWidth = 2;
  w = 4;

  kBBoxMinLength = 0.1;
  kBBoxMaxLength = 5;
  l = 6;
}
inline Box ObjectDetector::BoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster)
{
  // Find bounding box for one of the clusters
  pcl::PointXYZI minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box = {};
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;
  // box.yaw = 0.0;

  return box;
}
inline pcl::PointCloud<pcl::PointXYZI>::Ptr ObjectDetector::FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint)
{
  // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();

    /*** Voxel grid point reduction and region based filtering ***/

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // Create the filtering object: down-sample the dataset using a given leaf size
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filtered_cloud);

    // Filter point cloud that is out of region of interest
    pcl::CropBox<pcl::PointXYZI> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);        // filtered_cloud
    region.filter(*filtered_cloud);              // filtered_cloud

    /*// Pass Through filter
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(filtered_cloud);
    
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-50, 80);
    //pass.setFilterLimitsNegative(true);
    pass.filter(*filtered_cloud);*/
    
    std::vector<int> indices;
    // NIRO
    // 전장 : 4,375
    // 전폭 : 1,805
    // 전고 : 1,570
    // 축거 : 2,700
    
    // Filter point cloud on the roof of host vehicle
    region.setMin(Eigen::Vector4f( -2.2, -1.0, -0.5, 1));  // -1.5 1.7 -1 1
    region.setMax(Eigen::Vector4f(  2.2,  1.0,  2.5, 1));  // 2.6 1.7 -0.4 1
    region.setInputCloud(filtered_cloud);   // filtered_cloud
    region.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int index : indices)
    {
      inliers->indices.push_back(index);
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the point cloud on roof
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // true
    extract.filter(*filtered_cloud);

    /*auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;*/

    return filtered_cloud;
}

inline std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> ObjectDetector::SeparateClouds(const pcl::PointIndices::Ptr& inliers, const  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>());


  // Copy inliers point cloud as plane
  for (int index : inliers->indices)
  {
    plane_cloud->points.push_back(cloud->points[index]);
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  // Extract the inliers so that we can get obstacles point cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstacle_cloud, plane_cloud);
  return segResult;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> ObjectDetector::SegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  //auto startTime = std::chrono::steady_clock::now();

  /*** PCL IMPLEMENTATION START ***/
	// Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
      std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
  }

  /*auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;*/

  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ObjectDetector::Clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)
{
  // Time clustering process
  //auto startTime = std::chrono::steady_clock::now();
  std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  /*** Perform euclidean clustering to group detected obstacles ***/

  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto& get_indices : cluster_indices)
  {
      typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());

      for (const auto index : get_indices.indices)
      {
          cloud_cluster->points.push_back(cloud->points[index]);
      }

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      if (cloud_cluster->width >= minSize && cloud_cluster->width <= maxSize)
      {
          clusters.push_back(cloud_cluster);
      }
  }

  /*auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;*/

  return clusters;
}

inline void ObjectDetector::BBOX(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud)
{
  // For UDP topic
  int info_id = 0;

  // UDP msg
  SocketBox_msg.box_info.resize(BOX_SIZE);
  
  // Do bounding box
  if (cloud.size())
  {
      for (const auto& cluster : cloud)
      {
          // std::cout << "cluster: " << cluster->size() << std::endl;
          // // Do boxing!!!
          Box box = BoundingBox(cluster);

          // // Filter out some cluster with little points and shorter in height
          if (cluster->points.size() >= kMinSize*2 && info_id < 30 && box.x_max-box.x_min < w*2 && box.y_max-box.y_min < l*1.5 && box.z_max-box.z_min > h*0.05)
          {
            // Point point(box, info_id+1);
            Point point(info_id+1);
            point.getTF(cluster);

            // // Store UDP socket topic
            // std::cout << "socket size: " << SocketBox_msg.box_info.size() << std::endl;

            SocketBox_msg.box_info[info_id].box.x_min = box.x_min;
            SocketBox_msg.box_info[info_id].box.y_min = box.y_min;
            SocketBox_msg.box_info[info_id].box.z_min = box.z_min;
            SocketBox_msg.box_info[info_id].box.x_max = box.x_max;
            SocketBox_msg.box_info[info_id].box.y_max = box.y_max;
            SocketBox_msg.box_info[info_id].box.z_max = box.z_max;
            // SocketBox_msg.box_info[info_id].box.angle = 0.0;
            

            info_id++;

            // Do pushing!!!!
            marker_array.markers.push_back(point.marker_);
          }
      }

      if (info_id)
      {   
          // std::cout <<"info_id: " << info_id << std::endl;

          SocketBox_msg.num_of_box = info_id;

          pub_marker.publish(marker_array);
          // pub_udpBox.publish(SocketBox_msg);
      }
  }

  SocketBox_msg.box_info.clear();
}

inline void ObjectDetector::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  
  // Convert point cloud to PCL native point cloud
  pcl::fromROSMsg(*cloud_msgs, *cloud_ptr);

  // Apply sensor hegiht
  for (size_t i = 0; i < cloud_ptr->points.size(); i++)
  {
    cloud_ptr->points[i].z += VEHICLE_HEIGHT;
  }

  // 1. Filtering
  auto filtered_cloud = FilterCloud(cloud_ptr, kFilterResolution, kMinPoint, kMaxPoint);
  
  // 2. Segmentation
  auto segmented_cloud = SegmentPlane(filtered_cloud, kMaxIterations, kDistanceThreshold);
  
  // 3. Clustering
  auto clustered_cloud = Clustering(segmented_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

  std::cout << "cluster size: " << clustered_cloud.size() << std::endl;

  // Clear memory UDPBOX
  // memset(&udp_box_info, 0, sizeof(UDPBOX));
  memset(&SocketBox_msg, 0, sizeof(lidar_msgs::SocketBox));

  // 4. Bounding boxing
  BBOX(clustered_cloud);

  pcl::toROSMsg(*cloud_ptr, ros_origin);// second -> ground
  pcl::toROSMsg(*segmented_cloud.first, ros_object);// second -> ground

  ros_origin.header.frame_id = "/os_sensor";
  pub_origin.publish(ros_origin);
  pub_object.publish(ros_object);

  marker_array.markers.clear();
}