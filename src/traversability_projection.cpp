#include "traversability_projection.h"

void TraversabilityProjection::colorDepthCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                                  const sensor_msgs::ImageConstPtr& depth_msg) {


  float cx_ = 601.8873;
  float cy_ = 183.1104;
  float fx_ = 707.0912;
  float fy_ = 707.0912;
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_img = cv_ptr->image;

  int width = depth_img.cols;
  int height = depth_img.rows;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int32_t i = 0; i < width * height; ++i) {
    int ux = i % width;
    int uy = i / width;

    float pix_depth = (float) depth_img.at<uint16_t>(uy, ux) / 1000;
    pcl::PointXYZ pt;
    pt.x = (ux - cx_) * (1.0 / fx_) * pix_depth;
    pt.y = (uy - cy_) * (1.0 / fy_) * pix_depth;
    pt.z = pix_depth;
    cloud.points.push_back(pt);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "robot";
 
  sensor_msgs::PointCloud cloud1;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, cloud1);

  // Fetch tf transform
  try {
    tfListener_.transformPointCloud("/map",
                              cloud_msg.header.stamp,
                              cloud1,
                              cloud_msg.header.frame_id,
                              cloud1);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  sensor_msgs::convertPointCloudToPointCloud2(cloud1, cloud_msg);
  cloud_msg.header.frame_id = "/map";

  pt_queue.push(cloud_msg);
  color_queue.push(*color_msg);

  project(width, height);
}

void TraversabilityProjection::project(int width, int height) {
  std::cout << pt_queue.size() << std::endl;
  std::cout << color_queue.size() << std::endl;
  if (pt_queue.size() >= 40 && gridmap_set_) {

    sensor_msgs::PointCloud2 cloud2 = pt_queue.front();
    pt_queue.pop();
    cloud_pub_.publish(cloud2);

    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
    sensor_msgs::Image color_msg = color_queue.front();
    color_queue.pop();

   // cv::Mat proj_img(cv::Size(camera_info_msg->width, camera_info_msg->height), CV_32FC1);
    
    cv::Mat proj_img(cv::Size(width, height), CV_8UC3, cv::Scalar(255, 255, 255));
    for (size_t i = 0; i < height; ++i) {
      for (size_t j = 0; j < width; ++j) {
        grid_map::Index index;
        grid_map::Position position(cloud.points[i*width + j].x,
                                    cloud.points[i*width + j].y);
        if (!gridmap_.getIndex(position, index))
          continue;
        float traversability = gridmap_.at("traversability", index);
        //proj_img.at<float>(i, j) = traversability;
        proj_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255*traversability, 0, 0);
      }
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr->image;
    cv::addWeighted(image, 0.5, proj_img,0.5,0, proj_img);
    cv::imwrite("test.png", proj_img);
  }
}
void TraversabilityProjection::colorDepthInfoCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                                      const sensor_msgs::ImageConstPtr& depth_msg,
                                                      const sensor_msgs::CameraInfoConstPtr & camera_info_msg) {
  
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info_msg);
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::PointField::FLOAT32,
                                       "intensity", 1, sensor_msgs::PointField::FLOAT32);
  depth_image_proc::convert<uint16_t>(depth_msg, cloud_msg, camera_model);

  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud);
  // Fetch tf transform
  try {
    tfListener_.transformPointCloud("/odom",
                              cloud_msg->header.stamp,
                              cloud,
                              cloud_msg->header.frame_id,
                              cloud);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  sensor_msgs::convertPointCloudToPointCloud2(cloud, *cloud_msg);
  cloud_msg->header.frame_id = "/odom";
  cloud_pub_.publish(*cloud_msg);
  //
  if (gridmap_set_) {
   // cv::Mat proj_img(cv::Size(camera_info_msg->width, camera_info_msg->height), CV_32FC1);
    cv::Mat proj_img(cv::Size(camera_info_msg->width, camera_info_msg->height), CV_8UC3, cv::Scalar(255, 255, 255));
    for (size_t i = 0; i < camera_info_msg->height; ++i) {
      for (size_t j = 0; j < camera_info_msg->width; ++j) {
        grid_map::Index index;
        grid_map::Position position(cloud.points[i*camera_info_msg->width + j].x,
                                    cloud.points[i*camera_info_msg->width + j].y);
        if (!gridmap_.getIndex(position, index))
          continue;
        float traversability = gridmap_.at("traversability", index);
        //proj_img.at<float>(i, j) = traversability;
        if (traversability > 0.5)
          proj_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
        else
          proj_img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      }
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr->image;
    cv::addWeighted(image, 0.5, proj_img,0.5,0, proj_img);
    cv::imwrite("test.png", proj_img);
  }
}

void TraversabilityProjection::gridmapCallback(const grid_map_msgs::GridMap& gridmap_msg) {
  grid_map::GridMapRosConverter::fromMessage(gridmap_msg, gridmap_);
  gridmap_set_ = true;
  std::cout << "Received grid map." << std::endl;
}
