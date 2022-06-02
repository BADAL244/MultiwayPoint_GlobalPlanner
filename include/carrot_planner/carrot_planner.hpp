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
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include "carrot_planner/Astar.h"
#include "carrot_planner/OccMapTransform.h"
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include<vector>
#include "carrot_planner/csvwriter.h"
#include "carrot_planner/2d_space.hpp"
#include "carrot_planner/math_funcs.hpp"
#include "carrot_planner/visualization.hpp"
#include "carrot_planner/yaml_read.h"

#ifndef CARROT_PLANNER_CPP
#define CARROT_PLANNER_CPP


using namespace cv;

namespace carrot_planner{
  /**
   * @class CarrotPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class CarrotPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the CarrotPlanner
       */
      CarrotPlanner();
      /**
       * @brief  Constructor for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Destructor
       */
      ~CarrotPlanner();

      /**
       * @brief  Initialization function for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      void MapCallback(const nav_msgs::OccupancyGrid& msg);
      std::vector<geometry_msgs::PoseStamped>convert_point(std::vector<Point>& path);

      bool inline IsValid(double x , double y);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
      std::string name_;



      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);


      bool initialized_;
      ros::Subscriber map_sub;
      bool is_map_received{false};
      ros::NodeHandle nh;
      pathplanning::AstarConfig config;

      pathplanning::Astar astar;
      OccupancyGridParam OccGridParam;


      Mat map_new;
      
      Point start_img , goal_img;
  
      Point img_yml;
      vector<Point> list_yaml;
      vector<Point> path_new1;
      vector<Point> path_new2;
      vector<Point> path_new0;
      Point2d robot_point;
      bool is_received{false};
      bool map_flag;
      nav_msgs::OccupancyGrid OccGridMask;
      ros::Publisher mask_pub;
      Mat Mask;
      int H, W;
      double R;
      std::vector<geometry_msgs::Point> new_point;
      geometry_msgs::Point pt;
      
      CSVWriter csv;
      bool done = false;
      double wx , wy;
      unsigned int mx , my;
      std::vector<geometry_msgs::Point> footprint;
      double goal_tol, d, robot_radius;
      int K_in;
      bool viz_tree;
      ros::Publisher plan_pub_, tree_pub_;
      geometry_msgs::PoseStamped pose_new;
      std::vector<geometry_msgs::PoseStamped> new_poses;
      readyamlfile::readfile read = readyamlfile::readfile(pose_new,new_poses);
      std::string file= "/home/wasp2/robot_ws/src/homeservicerobot/navigation/carrot_planner/config/newfile.yaml";
      std::vector<geometry_msgs::PoseStamped> return_pose;
      std::vector<geometry_msgs::PoseStamped> plan1;
      std::vector<geometry_msgs::PoseStamped> plan2 , plan0;
      
      

  };
};  
#endif



struct tree_node
{
  int parent_id{};
  geometry_msgs::Point vertex{};
};

/**
 * RRT Data Structure.
 *
 */
class rrt
{
public:
  geometry_msgs::Point x_initial{};
  // Vertices and Edges in RRT
  std::vector<tree_node> tree_nodes{};
  std::vector<std::vector<geometry_msgs::Point>> edges{};
  // Free state space
  costmap_2d::Costmap2DROS* X_space;
  // Whether tree reached goal
  bool success{ 0 };

  // Constructor for RRT
  rrt(geometry_msgs::Point x_init, costmap_2d::Costmap2DROS* costmap_ros)
  {
    x_initial = x_init;

    // Populate X_space
    this->X_space = costmap_ros;

    // Add initial point to vertices
    tree_node initial_node;
    initial_node.parent_id = 0;
    initial_node.vertex = x_init;
    add_vertex(initial_node);
  }

  // Adds node to node vector
  void add_vertex(const tree_node new_node)
  {
    this->tree_nodes.push_back(new_node);
  }

  // Adds edge to list of edges
  void add_edge(geometry_msgs::Point point1, geometry_msgs::Point point2)
  {
    std::vector<geometry_msgs::Point> edge{};
    edge.push_back(point1);
    edge.push_back(point2);
    this->edges.push_back(edge);
  }

  ~rrt(){};
};

/**
 *  @brief Finds closest tree node to an arbitary point.
 *
 *  @details
 *   Finds the closest existing tree node in an RRT object
 *   to an arbitary point.
 *
 *  @param point1 An arbitary point.
 *  @param rrt Existing RRT object.
 *  @return The closest (L2) tree node to point1.
 *
 **/
tree_node getNearestNeighbor(const geometry_msgs::Point point1, const rrt* T)
{
  geometry_msgs::Point nearest_neighbor{};
  tree_node nearest_neighbor_node{};
  int parent_id{};
  double nearest_distance{ HUGE_VAL };
  double current_distance{ HUGE_VAL };

  // For each vertex (a tree_node)
  for (int ii = 0; ii < T->tree_nodes.size(); ii++)
  {
    // Make sure it's not the same point
    if (point1.x != T->tree_nodes.at(ii).vertex.x && point1.y != T->tree_nodes.at(ii).vertex.y)
    {
      // Get the closest existing vertex
      current_distance = getDistance(point1, T->tree_nodes.at(ii).vertex);
      if (current_distance < nearest_distance)
      {
        nearest_distance = current_distance;
        nearest_neighbor = T->tree_nodes.at(ii).vertex;
        parent_id = ii;
      }
    }
  }

  nearest_neighbor_node.vertex = nearest_neighbor;
  nearest_neighbor_node.vertex.z = 0.;  // Assume planar for now
  nearest_neighbor_node.parent_id = parent_id;

  return nearest_neighbor_node;
}

/**
 *  @brief Extends pose_near towards point_rand.
 *
 *  @details
 *   Creates a new tree node extending from point_near towards point_rand.
 *
 *  @param point_near Closest existing tree node to random point.
 *  @param point_rand Random point in unoccupied space.
 *  @param d Distance to extend tree.
 *  @return A new tree node extending from point_near towards point_rand.
 *
 **/
tree_node extendTree(const tree_node point_near, const geometry_msgs::Point point_rand, const double d)
{
  tree_node point_new{};
  point_new.vertex.z = 0.;  // Assume z=0 for now

  double theta = atan2(point_rand.y - point_near.vertex.y, point_rand.x - point_near.vertex.x);
  point_new.vertex.x = point_near.vertex.x + d * cos(theta);
  point_new.vertex.y = point_near.vertex.y + d * sin(theta);

  point_new.parent_id = point_near.parent_id;

  return point_new;
}

/**
 *  @brief Generates a RRT between start and goal robot poses.
 *
 *  @details
 *   Generates a RRT between start and goal robot poses checking global
 *   costmap for occupied space.
 *
 *  @see http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 *
 *  @param x_init Starting robot pose (map frame).
 *  @param x_final Goal robot pose (map frame).
 *  @param costmap_ros Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius Padded circumscribed robot footprint.
 *  @param goal_tol Cartesian goal tolerance.
 *  @param K Max. number of iterations.
 *  @param d Distance to extend tree per step.
 *  @return Full RRT linking x_init and x_final.
 *
 **/
rrt generateRRT(geometry_msgs::PoseStamped x_init, geometry_msgs::PoseStamped x_final,
                costmap_2d::Costmap2DROS* costmap_ros, double robot_radius, double goal_tol, int K, double d)
{
  // Initialize RRT with x_init
  rrt T(x_init.pose.position, costmap_ros);
  // Initialize local variables
  geometry_msgs::Point x_rand;
  tree_node x_near, x_new;

  // Build Tree
  for (int k = 1; k <= K; k++)
  {
    bool edgeIsFree{ 0 };
    std::vector<geometry_msgs::Point> edge{};

    // Get random configuration
    x_rand = getRandomState(T.X_space, robot_radius);
    // Get nearest existing neighbor to random pose
    x_near = getNearestNeighbor(x_rand, &T);
    // Extend x_near toward x_rand
    x_new = extendTree(x_near, x_rand, d);

    // Check if x_new and x_near can connect
    edge.push_back(x_new.vertex);
    edge.push_back(x_near.vertex);

    // Check if edge is in free space
    edgeIsFree = edgeInFreeSpace(edge, T.X_space, robot_radius);
    if (edgeIsFree)
    {
      T.add_vertex(x_new);
      T.add_edge(x_near.vertex, x_new.vertex);
    }
    else
    {
      continue;
    };

    // ROS_INFO("Processed %i/%i RRT vertices.", k, K);

    if (getDistance(x_new.vertex, x_final.pose.position) <= goal_tol)
    {
      ROS_INFO("Found solution with %i/%i RRT vertices.", k, K);
      T.success = 1;
      break;
    }
  }

  return T;
}

/**
 *  @brief Calculates the robot radius from footprint.
 *
 *  @details
 *   Calculates the circumscribed, inflated robot radius from 2D footprint.
 *
 *  @param footprint A polygon representing the 2D robot footprint.
 *  @return Maximum distance from center.
 *
 **/
double getRobotRadius(std::vector<geometry_msgs::Point> footprint)
{
  double max_dist{ 0. }, dist{};
  geometry_msgs::Point origin{};
  origin.x = 0.;
  origin.y = 0.;
  origin.z = 0.;

  for (auto pt : footprint)
  {
    dist = getDistance(origin, pt);
    if (dist > max_dist)
    {
      max_dist = dist;
    }
  }
  return max_dist;
}

/**
 *  @brief Calculates the global path from start to goal using
 *  an existing RRT.
 *
 *  @details
 *   Assumes RRT construction was successful. Walks from final
 *   tree node at goal back to start, calculating orientations.
 *
 *  @param tree Pointer to a tree linking the start and goal poses.
 *  @param plan Pointer to a plan object to populate.
 *  @param start Robot start pose.
 *  @param goal Robot goal pose.
 *  @return true
 *
 **/
bool getGlobalPath(const rrt* tree, std::vector<geometry_msgs::PoseStamped>* plan,
                   const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  int prev_id;
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;

  plan->clear();

  // Add last vertex (closest to goal)
  int current_id = tree->tree_nodes.size() - 1;
  // Set goal orientation for last vertex
  pose_stamped.pose.orientation = goal.pose.orientation;

  // Work our way back to x_initial, building plan
  while (current_id != 0)
  {
    // Retrieve pose of current ID
    pose_stamped.pose.position = tree->tree_nodes.at(current_id).vertex;
    // Add pose to plan
    plan->push_back(pose_stamped);
    // Identify next vertex in path (parent node), store previous ID
    prev_id = current_id;
    current_id = tree->tree_nodes.at(current_id).parent_id;

    // Set orientation for next iteration
    double dy, dx, yaw;
    dy = tree->tree_nodes.at(prev_id).vertex.y - tree->tree_nodes.at(current_id).vertex.y;
    dx = tree->tree_nodes.at(prev_id).vertex.x - tree->tree_nodes.at(current_id).vertex.x;
    // Get yaw from atan2 using current point and prev. point.
    yaw = atan2(dy, dx);
    // Convert RPY to quat
    quat_tf.setRPY(0, 0, yaw);
    // Convert Quat TF to msg
    quat_msg = tf2::toMsg(quat_tf);
    // set orientation.
    pose_stamped.pose.orientation = quat_msg;
  }

  // Add x_initial
  pose_stamped.pose.position = tree->tree_nodes.at(0).vertex;
  pose_stamped.pose.orientation = start.pose.orientation;
  plan->push_back(pose_stamped);

  // Reverse so that x_initial is first and goal is last.
  std::reverse(plan->begin(), plan->end());

  return true;
}