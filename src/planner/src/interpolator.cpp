#include <ros/ros.h>
#include <algorithm.h>
#include <tuple>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

double euc_dist (const geometry_msgs::Point p1, geometry_msgs::Point p2) {
    double d = sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0));
    return d;
}

std::tuple<double, double, int> minimum_dist_on_poly_line (const geometry_msgs::Point state, const std::vector<geometry_msgs::Point> path) {
    std::tuple<double, double, int> min_d;
    std::vector<double> cum_dist, agent_dist;
    double d_i;
    geometry_msgs::Point first_point, second_point;
    double prev_dist = 0;
    for (int i = 0; i<path.size(); i++) {
        if (i>=1) {
            first_point = path.at(i-1);
            second_point = path.at(i);
            d_i = prev_dist + euc_dist(first_point,second_point);
            cum_dist.push_back(d_i);
            prev_dist = d_i;
       }else {
            cum_dist.push_back(0);
       }
       agent_dist.push_back(euc_dist(state,path.at(i)));
    }
    int p_idx = std::min_element(agent_dist.begin(),agent_dist.end()) - agent_dist.begin();
    double d0 = cum_dist.at(p_idx);
    double d1 = agent_dist.at(p_idx);
    double d = d0 + d1;
    double d_best = cum_dist.back();
    min_d = std::make_tuple(d,d_best,p_idx);
    return min_d;
}

double tri_area (geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3) {
    return 0.5*abs((p1.x*(p2.y-p3.y))+(p2.x*()))
}