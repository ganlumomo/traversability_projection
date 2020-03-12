#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <queue>

class TraversabilityProjection {
  public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    TraversabilityProjection(ros::NodeHandle& nh)
      : nh_(nh)
      , gridMap_set_(false)
      , scan_id_(0) {
        color_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "/kitti_player_mini_node/left_color_image", 10);
        depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "/kitti_player_mini_node/depth_image", 10);
        sync_ = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(100), *color_sub_, *depth_sub_);
        sync_->registerCallback(boost::bind(&TraversabilityProjection::colorDepthCallback, this, _1, _2));
        gridMap_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &TraversabilityProjection::gridMapCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("visualized_cloud", 1);
      }

    void colorDepthCallback(const sensor_msgs::ImageConstPtr& color_msg,
                            const sensor_msgs::ImageConstPtr& depth_msg);

    void gridMapCallback(const grid_map_msgs::GridMap& gridMap_msg);

    void project(int width, int height);

  private:
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* color_sub_;
    message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;
    ros::Subscriber gridMap_sub_;
    ros::Publisher cloud_pub_;
    tf::TransformListener tf_listener_;
    
    bool gridMap_set_;
    int scan_id_;
    grid_map::GridMap gridMap_;
    std::queue<sensor_msgs::Image> depth_queue_;
    std::queue<sensor_msgs::PointCloud2> cloud_queue_;
};
