/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <angles/angles.h>
#include <carrot_planner/carrot_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "carrot_planner/Astar.h"
#include "carrot_planner/OccMapTransform.h"
std::string dir_package = "/home/wasp2/robot_ws/src/homeservicerobot/navigation/carrot_planner/config";
std::string dir_package_file = dir_package + "newfile.yaml";
#include <chrono>
#include <thread>
using namespace cv ;
using namespace std;
using namespace std::chrono;
std::chrono::milliseconds interval1(1000);

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(carrot_planner::CarrotPlanner, nav_core::BaseGlobalPlanner)

namespace carrot_planner {

  CarrotPlanner::CarrotPlanner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    
  }

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  CarrotPlanner::~CarrotPlanner() {
    // deleting a nullptr is a noop
    delete world_model_;
  }
  
  void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 




      initialized_ = true;

    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double CarrotPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


  bool CarrotPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    // bool map_received = false;

    // while(!map_received){



    //   if (){map_received = true } else {map_received = false;}


    // }


    // ros::spinOnce();
    // std::this_thread::sleep_for(interval1 );

      ros::spinOnce();
      std::this_thread::sleep_for(interval1 );
 

    
    
    map_sub = nh.subscribe("map", 10, &CarrotPlanner::MapCallback , this);
    bool done = false;
    try {
        YAML::Node config = YAML::LoadFile("/home/wasp2/robot_ws/src/homeservicerobot/navigation/carrot_planner/config/newfile.yaml");

        // The outer element is an array
        for(auto dict : config) {
            // The array element is a map containing the Pos and Rectangle keys:
            auto name = dict["Pos"];
            std::cout << "Name: " << name << '\n';

            auto rect = dict["Rectangle"];
          
            // loop over the positions Rectangle and print them:
            for(auto pos : rect) {
                // std::cout << pos["x"].as<double>() << ",\t"
                //           << pos["y"].as<double>() << ",\t"
                //           << pos["z"].as<double>() << '\n';
                yaml.x = pos["x"].as<double>();
                yaml.y = pos["y"].as<double>();
                OccGridParam.Map2ImageTransform(yaml, img_yml);
                list_yaml.push_back(img_yml);
                cout << "dbg0" << endl;
            }
        }

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    cout << "dbg1" << endl;
    if(is_received){
      start_new.x = start.pose.position.x;
      start_new.y = start.pose.position.y;
      OccGridParam.Map2ImageTransform(start_new,start_img);

      cout << "dbg2" << endl;
      for(size_t i = 0 ; i < list_yaml.size(); ++i){
        if(i == 0){
          cout << "dbg3" << endl;
          astar.PathPlanning(start_img,list_yaml[0] , path_new);
          cout << "dbg4" << endl;
        }
        else{
          cout << "dbg5" << endl;
          astar.PathPlanning(list_yaml[i-1],list_yaml[i] , path_new);
          cout << "dbg6" << endl;


        }
    }
  }
    

    if(!path_new.empty())
      {

          cout << "dbg7" << endl;
          for(int i=0;i<path_new.size();i++)
          {
              Point2d dst_point;
              OccGridParam.Image2MapTransform(path_new[i], dst_point);
              cout << "dbg8" << endl;
              geometry_msgs::PoseStamped pose_stamped;
              pose_stamped.header.stamp = ros::Time::now();
              pose_stamped.header.frame_id = "map";
              pose_stamped.pose.position.x = dst_point.x;
              pose_stamped.pose.position.y = dst_point.y;
              pose_stamped.pose.position.z = 0.0;
              pose_stamped.pose.orientation.x = 0.0;
              pose_stamped.pose.orientation.y = 0.0;
              pose_stamped.pose.orientation.z = 0.0;
              pose_stamped.pose.orientation.w = 0.0;
              cout << "dbg9" << endl;
              plan.push_back(pose_stamped);
              for(auto p : plan)
                std::cout << p.pose.position.x <<"," << p.pose.position.y <<  "," << p.pose.orientation.x <<std::endl;
              cout << "dbg10" << endl;
              done = true;
              cout << "dbg11" << endl;
              
          }
         
      }
      else
      {
          ROS_ERROR("Can not find a valid path");
      }

    
    return (done);
    

	}


  void CarrotPlanner::MapCallback(const nav_msgs::OccupancyGrid& msg)
  {
      // Get parameter
      std::cout << "hi i came once here" << std::endl;
      if(!is_map_received){
      OccGridParam.GetOccupancyGridParam(msg);
      int height = OccGridParam.height;
      cout << height << endl;
      int width = OccGridParam.width;
      cout << width << endl;
      int OccProb;
      Mat Map(height, width, CV_8UC1);
      // Get map

      for(int i=0;i<height;i++)
      {
          for(int j=0;j<width;j++)
          {
              OccProb = msg.data[i * width + j];
              OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
              // The origin of the OccGrid is on the bottom left corner of the map
              Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
          }
      }
      std::cout << "really0 ? " << std::endl;
      // Initial Astar
      Mat Mask;
      std::cout << "really1 ? " << std::endl;
      config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
      std::cout << "really2 ? " << std::endl;
      astar.InitAstar(Map, Mask, config);
      std::cout << "really ? " << std::endl;
      is_received  = true;
      is_map_received = true;
      
      
      }


  }

};
