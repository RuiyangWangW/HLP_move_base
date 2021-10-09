#include <ros/ros.h>
#include <algorithm>
#include <tuple>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

double euc_dist (const geometry_msgs::Point p1, geometry_msgs::Point p2) {
    return sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0));
}

std::vector<double> cum_dist_along_path (const std::vector<geometry_msgs::Point> path) {
    std::vector<double> cum_dist;
    geometry_msgs::Point first_point, second_point;
    double prev_dist = 0;
    double d_i = 0;
    for (int i = 0; i<path.size(); i++) {
        if (i>0) {
            first_point = path.at(i-1);
            second_point = path.at(i);
            d_i = prev_dist + euc_dist(first_point,second_point);
            cum_dist.push_back(d_i);
            prev_dist = d_i;
        }else {
            cum_dist.push_back(0);
        }
    }
    return cum_dist;
}

std::tuple<int, double, double> minimum_dist_on_poly_line (const geometry_msgs::Point state, const std::vector<geometry_msgs::Point> path) {
    std::tuple<double, double, int> min_d;
    std::vector<double> cum_dist = cum_dist_along_path(path);
    std::vector<double> agent_dist;
    for (int i = 0; i<path.size(); i++) {
        agent_dist.push_back(euc_dist(state,path.at(i)));
    }
    int p_idx = std::min_element(agent_dist.begin(),agent_dist.end()) - agent_dist.begin();
    double d0 = cum_dist.at(p_idx);
    double d1 = euc_dist(path.at(p_idx),state);
    double d = d0 + d1;
    double d_best = cum_dist.back();
    return std::make_tuple(p_idx,d,d_best);
}

double cal_tri_area (geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3) {
    return 0.5*abs((p1.x*(p2.y-p3.y))+(p2.x*(p3.y-p1.y))+(p3.x*(p1.y-p2.y)));
}

double cal_curvature (geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3) {
    return 4*cal_tri_area(p1,p2,p3)/(euc_dist(p1,p2)*euc_dist(p1,p3)*euc_dist(p2,p3));
}

geometry_msgs::Point match_trajectory_xy(double dist, std::vector<double> cum_dist, std::vector<geometry_msgs::Point> path) {
    geometry_msgs::Point result;
    if (dist <= cum_dist.back()) {
        for(int i = 0; i < cum_dist.size(); i++) {
            if ((dist >= cum_dist.at(i)) && (dist <= cum_dist.at(i+1))) {
                double p = (dist-cum_dist.at(i))/(cum_dist.at(i+1)-cum_dist.at(i));
                result.x = path.at(i).x + (path.at(i+1).x-path.at(i).x)*p;
                result.y = path.at(i).y + (path.at(i+1).y-path.at(i).y)*p;
                result.z = 0;
                break;
            }
        }
    }else {
        result = path.back();
    }
    return result;
}

double match_trajectory_heading(double dist, std::vector<double> cum_dist, std::vector<geometry_msgs::Point> path) {
    double result;
    if (dist <= cum_dist.back()) {
        for(int i = 0; i < cum_dist.size(); i++) {
            if ((dist >= cum_dist.at(i)) && (dist <= cum_dist.at(i+1))) {
                double p = (dist-cum_dist.at(i))/(cum_dist.at(i+1)-cum_dist.at(i));
                result = path.at(i).z + (path.at(i+1).z-path.at(i).z);
                break;
            }
        }
    }else {
        result = path.back().z;
    }
    return result;
}

std::vector<geometry_msgs::Point> get_waypoints (const geometry_msgs::Point state, const std::vector<geometry_msgs::Point> path) {
    //Constants Definition
    double reduced_curve_lookahead = 7.0;
    double lookahead_distance, heading_lookahead;
    std::vector<double> curvatures;
    geometry_msgs::Point wp, wp1, wp2;
    //Determine the index of the closest point from state to path, and the distance from state to
    //the first waypoint of the path through the closest waypoint
    //and distance to the last point of the path
    auto min_d = minimum_dist_on_poly_line(state,path);
    int p_idx = std::get<0>(min_d);
    double d = std::get<1>(min_d);
    double d_best = std::get<2>(min_d);
    if (p_idx + int(reduced_curve_lookahead*20) < path.size()) {
        for (int i = p_idx; i <= p_idx + int(reduced_curve_lookahead*20); i++) {
            if (i == 0) {
                curvatures.push_back(abs(cal_curvature(path.at(i),path.at(i+1),path.at(i+2))));
            }else if(i == path.size()-1) {
                curvatures.push_back(abs(cal_curvature(path.at(i-2),path.at(i-1),path.at(i))));
            }else {
                curvatures.push_back(abs(cal_curvature(path.at(i-1),path.at(i),path.at(i+1)))); 
            }
        }
        double k = *std::min_element(curvatures.begin(),curvatures.end());
        if (k > 0.2) {
            lookahead_distance = 1.5*1;
        }else {
            lookahead_distance = 1.5*2;
        }
    }else {
        lookahead_distance = 1.0;
    }

    heading_lookahead = 1.0 + lookahead_distance;
    std::vector<double> cum_dist = cum_dist_along_path(path); 
    double d_lkhd = std::min(d_best, d + lookahead_distance);
    wp = match_trajectory_xy(d_lkhd, cum_dist, path);
    double d_lkhd1 = std::min(d_best, d + lookahead_distance/3);
    wp1 = match_trajectory_xy(d_lkhd1, cum_dist, path);
    double d_lkhd2 = std::min(d_best, d + 2*lookahead_distance/3);
    wp2 = match_trajectory_xy(d_lkhd2, cum_dist, path);

    d_lkhd = std::min(d_best, d + heading_lookahead);
    wp.z = match_trajectory_heading(d_lkhd, cum_dist, path);
    d_lkhd1 = std::min(d_best, d + heading_lookahead/3);
    wp1.z = match_trajectory_heading(d_lkhd1, cum_dist, path);
    d_lkhd2 = std::min(d_best, d + 2*heading_lookahead/3);
    wp2.z = match_trajectory_heading(d_lkhd2, cum_dist, path);

    std::vector<geometry_msgs::Point> result{wp,wp1,wp2};
    return result;
}

double cal_yaw(const tf::Quaternion q) {
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return yaw; 
}

std::vector<geometry_msgs::Point> curr_path;
geometry_msgs::Point agent_state;
double path_updated = false;

void path_callback(const nav_msgs::Path path) {
    int s = path.poses.size();
    geometry_msgs::Point way_point;
    double roll, pitch, yaw;
    if (s>3) {
        curr_path.clear();
    	for(int i = 0; i < s; i++) {
          way_point.x = path.poses[i].pose.position.x;
          way_point.y = path.poses[i].pose.position.y;
          tf::Quaternion q(
          path.poses[i].pose.orientation.x,
          path.poses[i].pose.orientation.y,
          path.poses[i].pose.orientation.z,
          path.poses[i].pose.orientation.w);
          way_point.z = cal_yaw(q);
          curr_path.push_back(way_point);
        }
        path_updated = true;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "waypoint_interpolator");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("/move_base/GlobalPlanner/plan", 1, path_callback);
    ros::Publisher pub1 = nh.advertise<nav_msgs::Path>("/interpolator/way_points",100);
    ros::Rate loop_rate(1);
    tf::TransformListener listener;
    std::vector<geometry_msgs::Point> way_points;
    nav_msgs::Path wpts;
    while(ros::ok()) {
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped w;
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        if (path_updated == true) {
          agent_state.x = transform.getOrigin().x();
          agent_state.y = transform.getOrigin().y();
          agent_state.z = cal_yaw(transform.getRotation());
          way_points = get_waypoints(agent_state,curr_path);
          wpts.header.stamp = ros::Time::now();
          wpts.header.frame_id = "map";
          for(int i = 0; i < 3; i++) {
            w.pose.position = way_points.at(i);
            wpts.poses.push_back(w);
          }
          pub1.publish(wpts);
          wpts.poses.clear();
          path_updated = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
