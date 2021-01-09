#include "traversability_projection.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "traversability_projection_node");
  ros::NodeHandle nh("~");
  
  TraversabilityProjection traversability_projection(nh);
  //ros::spin();
  traversability_projection.project();

  return 0;
}
