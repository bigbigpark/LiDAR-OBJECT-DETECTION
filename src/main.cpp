/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-15 10:30
 */
#include <lidar_object_detection/object_detection.h>

using namespace std;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::Rate r(10);

    ObjectDetector OD;
    OD.init();

    while(ros::ok())
    {
      ros::spinOnce();

      r.sleep();
    }
}