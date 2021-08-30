#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_publisher");
  ros::NodeHandle node;
  
  tf::TransformBroadcaster br; 
  tf::TransformListener listener;
  tf::Transform odom;

  ros::Rate rate(10.0);

  bool recieved = false;

  while (ros::ok()){
    tf::StampedTransform transform;
    if(recieved == false) {
      try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        std::cout << "transform exist\n";
        odom.setOrigin(transform.getOrigin());
        odom.setRotation(transform.getRotation());
        recieved = true;
      }
      catch (tf::TransformException ex) {
        std::cout << "No Transformation Exists" << std::endl;
      }
    }
    br.sendTransform(tf::StampedTransform(odom, ros::Time::now(), "map", "odom"));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
        
