// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// define parameters here
#define STEER_LENGTH 0.30
#define TERMINATE_LENGTH 0.10
#define LOOKAHEAD_DISTANCE 0.60
#define KP 1.00
#define PI 3.1415927
#define ETA 0.60
#define MAX_ITERATION 100

// enum type for the switch between RRT and RRT*
enum RRT_type {
    RRT_base,
    RRT_star
};


// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh, RRT_type rrt_Type);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;

    ros::Publisher drive_pub_;
    ros::Publisher mapvisual_pub_;
    ros::Publisher points_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher edges_pub_;

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params
    std::vector<std::vector<bool>> occupancy_grids; //binary occupancy grid
    std::vector<std::vector<bool>> occupancy_grids_prior;  // prior of vector occupancy grid

    // Current goal point
    double x_goal;
    double y_goal;

    // parameters for the sample space
    double x_limit_top;
    double x_limit_bot;
    double y_limit_left;
    double y_limit_right;

    // parameters for the current car's way point
    double x_target;
    double y_target;

    // Current vehicle location
    double x_current;
    double y_current;

    // markers for visualization
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_2;
    visualization_msgs::Marker marker_3;
    visualization_msgs::Marker marker_4;
    // defines RRT type
    RRT_type rrt_type;

    //function for control
    double angle, heading_current;
    void reactive_control();

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const nav_msgs::Odometry &odom_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

