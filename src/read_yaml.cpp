#include<carrot_planner/yaml_read.h>


namespace readyamlfile{
    std::vector<geometry_msgs::PoseStamped> readfile::read_yaml(std::string filename){
        try {

        YAML::Node config = YAML::LoadFile(filename);

        // The outer element is an array
        for(auto dict : config) {

            auto rect = dict["Rectangle"];
          

            for(auto pos : rect) {
                std::cout << pos["x"].as<double>() << ",\t"
                          << pos["y"].as<double>() << ",\t"
                          << pos["z"].as<double>() << '\n';
                robot_pose_.header.stamp = ros::Time::now();
                robot_pose_.header.frame_id = "map";
                robot_pose_.pose.position.x = pos["x"].as<double>();
                robot_pose_.pose.position.y = pos["y"].as<double>();
                world_point.x = pos["x"].as<double>();
                world_point.y = pos["y"].as<double>();
                robot_pose_.pose.position.z = 0.0;
                robot_pose_.pose.orientation.x = 0.0;
                robot_pose_.pose.orientation.y = 0.0;
                robot_pose_.pose.orientation.z = 0.0;
                robot_pose_.pose.orientation.w = 1.0;
                pose_vector_.push_back(robot_pose_);
                world_point_vec.push_back(world_point);
            }

        }

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return pose_vector_;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return pose_vector_;
    }
    
    return pose_vector_;
 }

   void readfile::publish_marker(const std::vector<geometry_msgs::PoseStamped> &points){
    visualization_msgs::Marker mar_points;
    mar_points.header.frame_id = "map";
    mar_points.header.stamp = ros::Time::now();
    mar_points.ns =  "points_and_lines";
    mar_points.action = visualization_msgs::Marker::ADD;
    mar_points.pose.orientation.w = 1.0;
    mar_points.id = 0;
    mar_points.type = visualization_msgs::Marker::POINTS;
    mar_points.scale.x = 0.2;
    mar_points.scale.y = 0.2;
    mar_points.color.g = 1.0;
    mar_points.color.a = 1.0;
    for(size_t i = 0 ; i <= points.size()-1; ++i){
      geometry_msgs::Point p;
      p.x = points[i].pose.position.x;
      p.y = points[i].pose.position.y;
      p.z = 0.0;
      mar_points.points.push_back(p);
    }
  
    
    marker_pub.publish(mar_points);
  }



};