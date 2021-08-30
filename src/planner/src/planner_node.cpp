#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>


geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped start;

void goal_callback (const geometry_msgs::PoseStamped& g_ori) {
  goal = g_ori;
  return;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  ros::Subscriber sub1 = nh.subscribe("/move_base_simple/goal", 1, goal_callback);
  ros::Publisher pub1 = nh.advertise<nav_msgs::Path>("/global_path", 1); 
  nav_msgs::GetPlan srv;
  nav_msgs::Path p;
  tf::TransformListener listener;
  ros::Rate loop_rate(2);
  while(ros::ok()) {
    if ((goal.pose.position.x != start.pose.position.x) && (goal.pose.position.y != start.pose.position.y)) {
      tf::StampedTransform transform;
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      start.header.frame_id = "map";
      start.header.stamp = transform.stamp_;
      start.pose.position.x = transform.getOrigin().x();
      start.pose.position.y = transform.getOrigin().y();
      start.pose.orientation.x = transform.getRotation().x();
      start.pose.orientation.y = transform.getRotation().y();
      start.pose.orientation.z = transform.getRotation().z();
      start.pose.orientation.w = transform.getRotation().w();
      srv.request.start = start;
      srv.request.goal = goal;
      if (client.call(srv)) {
        p = srv.response.plan;
        pub1.publish(p);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

