#ifndef YAML_READ_H
#define YAML_READ_H
#include "yaml-cpp/yaml.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include "carrot_planner/OccMapTransform.h"
using namespace cv;
namespace readyamlfile{
    class readfile{
    public:
        readfile(geometry_msgs::PoseStamped robot_pose , std::vector<geometry_msgs::PoseStamped> pose_vector){
            robot_pose_ = robot_pose;
            pose_vector_ = pose_vector;
        }
        readfile(){}

        std::vector<geometry_msgs::PoseStamped>read_yaml(std::string filename);
        void publish_marker(const std::vector<geometry_msgs::PoseStamped> &points);



    private:
        geometry_msgs::PoseStamped robot_pose_;
        std::vector<geometry_msgs::PoseStamped> pose_vector_;
        
        ros::NodeHandle nh;
        ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        OccupancyGridParam OccGridParam;
        std::vector<geometry_msgs::PoseStamped> plan;
        Point2d world_point;
        std::vector<Point2d> world_point_vec;
        std::vector<Point> image_point;

};

};
#endif

