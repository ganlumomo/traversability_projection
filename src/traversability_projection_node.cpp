#include "traversability_projection.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "traversability_projection_node");
  ros::NodeHandle nh("~");
  
  TraversabilityProjection traversablity_projection(nh);
  ros::spin();

  return 0;
}
