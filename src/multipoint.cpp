#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <atomic>
#include <queue>

using namespace std;

struct VelocityCommand {
    double time;
    double linear_velocity;
    double angular_velocity;
};

ros::Publisher cmd_vel_pub;
ros::Subscriber odom_subscriber;
ros::Publisher pathPublisher;
std::atomic_bool path_clear(false);
std::atomic_bool yaml_processing_done(false);
std::queue<string> yaml_queue;
std::mutex yaml_mutex;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    cout << "Data is coming..." << endl;
    if (path_clear) {
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = odom_msg->pose.pose;
        pose_stamped.header = odom_msg->header;

        path.poses.push_back(pose_stamped);
        path.header.frame_id = odom_msg->header.frame_id;
        path.header.stamp = ros::Time::now();
        pathPublisher.publish(path);
    }
}

std::vector<VelocityCommand> readVelocityCommandsFromYAML(const std::string& file_path) {
    std::vector<VelocityCommand> velocity_commands;

    YAML::Node config = YAML::LoadFile(file_path);

    for (const auto& node : config) {
        VelocityCommand command;
        command.time = node["time"].as<double>();
        command.linear_velocity = node["linear_velocity"].as<double>();
        command.angular_velocity = node["angular_velocity"].as<double>();
        velocity_commands.push_back(command);
    }

    return velocity_commands;
}


void yamlReaderCallback() {
    while (ros::ok()) {
        string yamlFile;

        {
            std::lock_guard<std::mutex> lock(yaml_mutex);
            if (!yaml_queue.empty()) {
                yamlFile = yaml_queue.front();
                yaml_queue.pop();
            }
        }

        if (!yamlFile.empty()) {
            try {
                std::vector<VelocityCommand> velocity_commands = readVelocityCommandsFromYAML(yamlFile);
                int sizeof_vec = velocity_commands.size();
                cout << sizeof_vec << " this is the size of the vector, I am going to process" << endl;

                for (size_t i = 0; i < velocity_commands.size() - 1; i++) {
                    // Create a Twist message with the velocities
                    cout << i << "th cmd of angular and linear is getting processed. Please wait for the next YAML file." << endl;
                    const VelocityCommand& currentWaypoint = velocity_commands[i];
                    const VelocityCommand& nextWaypoint = velocity_commands[i + 1];

                    geometry_msgs::Twist twist_msg;
                    twist_msg.linear.x = velocity_commands[i].linear_velocity;
                    twist_msg.angular.z = velocity_commands[i].angular_velocity;
                    double time_to_go = velocity_commands[i].time;
                    cout << twist_msg.linear.x << "---" << twist_msg.angular.z << "++++" << time_to_go << endl;

                    // Publish the Twist message
                    cmd_vel_pub.publish(twist_msg);
                    cout << "Published a message." << endl;

                    // Sleep for the specified time
                    ros::Rate loop_rate(10); // Adjust the rate as per your requirement
                    double time_elapsed = 0.0;
                    ros::Time start_time = ros::Time::now();
                    while ((ros::Time::now() - start_time).toSec() < time_to_go) {
                        cmd_vel_pub.publish(twist_msg);
                        loop_rate.sleep();
                        ros::spinOnce();
                    }
                }

                path_clear = true;
                yaml_processing_done = true;
                cout << "I have finished processing the YAML file. Press Enter to process the next YAML file." << endl;
            }
            catch (const YAML::Exception& e) {
                ROS_ERROR("Error while parsing YAML file: %s", e.what());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_subscriber = nh.subscribe("/odom", 10, odomCallback);
    pathPublisher = nh.advertise<nav_msgs::Path>("/path_topic", 10);
    std::this_thread::sleep_for(std::chrono::duration<double>(10.0));

    std::vector<std::string> yamlFiles;
    std::string paramServer;

    if (!nh.getParam("/yaml_files", yamlFiles)) {
        ROS_ERROR("Failed to retrieve YAML file list.");
        return 1;
    }

    cout << "I have received the list of YAML files." << endl;




for (const auto& yamlFile : yamlFiles) {
    cout << "Processing YAML file: " << yamlFile << endl;
    path_clear = false;
    yaml_processing_done = false;

    {
        std::lock_guard<std::mutex> lock(yaml_mutex);
        yaml_queue.push(yamlFile);
    }

    std::thread yamlReaderThread(yamlReaderCallback);
    yamlReaderThread.detach();

    // Wait for user input or YAML processing to finish
    cout << "Press Enter to continue to the next YAML file..." << endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (!yaml_processing_done) {
        // If YAML processing is not done, signal it to stop and join the thread
        {
            std::lock_guard<std::mutex> lock(yaml_mutex);
            yaml_queue = std::queue<string>(); // Clear the queue
        }
    }
}




    return 0;
}
