#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <queue>

class TraversabilityProjection {
  public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    
    TraversabilityProjection(ros::NodeHandle& nh)
      : nh_(nh)
      , gridmap_set_(false) {
        color_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "/kitti_player_mini_node/left_color_image", 10);
        depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "/kitti_player_mini_node/depth_image", 10);
        //camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo> (nh_, "/camera/color/camera_info", 10);
        //sync_ = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(100), *color_sub_, *depth_sub_, *camera_info_sub_);
        //sync_->registerCallback(boost::bind(&TraversabilityProjection::colorDepthInfoCallback, this, _1, _2, _3));
        sync_ = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(100), *color_sub_, *depth_sub_);
        sync_->registerCallback(boost::bind(&TraversabilityProjection::colorDepthCallback, this, _1, _2));
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("test_points", 1);
        gridmap_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &TraversabilityProjection::gridmapCallback, this);
        
      }

    void colorDepthInfoCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

    void colorDepthCallback(const sensor_msgs::ImageConstPtr& color_msg,
                            const sensor_msgs::ImageConstPtr& depth_msg);

    void gridmapCallback(const grid_map_msgs::GridMap& gridmap_msg);

    void project(int height, int width);
  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image>* color_sub_;
    message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>* camera_info_sub_;
    message_filters::Synchronizer<MySyncPolicy>* sync_;
    tf::TransformListener tfListener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber gridmap_sub_;
    grid_map::GridMap gridmap_;
    bool gridmap_set_;
    std::queue<sensor_msgs::Image> color_queue;
    std::queue<sensor_msgs::PointCloud2> pt_queue;
};
