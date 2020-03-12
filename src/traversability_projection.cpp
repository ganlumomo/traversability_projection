#include "traversability_projection.h"

void TraversabilityProjection::colorDepthCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                                  const sensor_msgs::ImageConstPtr& depth_msg) {

  // Camera parameters for KITTI
  float cx_ = 601.8873;
  float cy_ = 183.1104;
  float fx_ = 707.0912;
  float fy_ = 707.0912;
  float depth_scaling_ = 1000;

  // Depth image to point cloud
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_img = cv_ptr->image;
  
  int width = depth_img.cols;
  int height = depth_img.rows;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int32_t i = 0; i < width * height; ++i) {
    int ux = i % width;
    int uy = i / width;

    float pix_depth = (float) depth_img.at<uint16_t>(uy, ux) / depth_scaling_;
    pcl::PointXYZ pt;
    pt.x = (ux - cx_) * (1.0 / fx_) * pix_depth;
    pt.y = (uy - cy_) * (1.0 / fy_) * pix_depth;
    pt.z = pix_depth;
    cloud.points.push_back(pt);
  }

  // Transform point cloud from left color camera to map frame
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "/left_color_camera";
  sensor_msgs::PointCloud cloud1;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, cloud1);

  // Fetch tf transform
  try {
    tf_listener_.transformPointCloud("/map",
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

  // Add msgs to queues
  depth_queue_.push(*depth_msg);
  cloud_queue_.push(cloud_msg);
  std::cout << depth_queue_.size() << std::endl;

  project(width, height);
}

void TraversabilityProjection::project(int width, int height) {

  std::string proj_img_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/07/traversability_new/";
  
  if (depth_queue_.size() >= 40 && gridMap_set_) {
    sensor_msgs::PointCloud2 cloud2 = cloud_queue_.front();
    cloud_queue_.pop();
    cloud_pub_.publish(cloud2);
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);

    sensor_msgs::Image depth_msg = depth_queue_.front();
    depth_queue_.pop();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_img = cv_ptr->image;

    //cv::Mat proj_img(cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat proj_img(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < height; ++i) {
      for (size_t j = 0; j < width; ++j) {
        grid_map::Index index;
        grid_map::Position position(cloud.points[i*width + j].x,
                                    cloud.points[i*width + j].y);
        if (!gridMap_.getIndex(position, index))
          continue;
        float traversability = gridMap_.at("traversability", index);
        if (traversability > 0.5 && depth_img.at<uint16_t>(i, j) != 0 && depth_img.at<uint16_t>(i, j) != 65535) {
          //proj_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
          proj_img.at<uint8_t>(i, j) = (uint8_t) 1;
        }
      }
    }
    
    //cv::addWeighted(color_img, 0.5, proj_img, 0.5, 0, proj_img);

    char scan_id_c[256];
    sprintf(scan_id_c, "%06d", scan_id_);
    std::string proj_img_name = proj_img_dir + std::string(scan_id_c) + ".png";
    cv::imwrite(proj_img_name, proj_img);

    scan_id_++;
  }
}

void TraversabilityProjection::gridMapCallback(const grid_map_msgs::GridMap& gridMap_msg) {
  grid_map::GridMapRosConverter::fromMessage(gridMap_msg, gridMap_);
  gridMap_set_ = true;
  std::cout << "Received grid map." << std::endl;
}
