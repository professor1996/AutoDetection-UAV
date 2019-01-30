#include "image_matching.hpp"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_down");
  ImageMatching ic;
  ros::spin();
  return 0;
}