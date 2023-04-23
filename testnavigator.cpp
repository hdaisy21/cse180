/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
//CURRENTLY ON MY END THIS CODE SPINS THEN ABORTS SO I CANT TEST THE TERMINAL 


#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>

//added for laser scan
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
//for algorithm
#include <algorithm>
//maybe 
#include <vector> 

//added functions for laser scan from textbook
class FindClosest : public rclcpp::Node 
{
  public:
    FindClosest():Node("pubsubstl")
    {
      pubf = this->create_publisher<std_msgs::msg::Float32>("closest", 1000);
      sub = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("scan", 10, std::bind(&FindClosest::processScan, this, std::placeholders::_1));
      
      //added this
      //changed node to this
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", 10, std::bind(&FindClosest::processScan, this, std::placeholders::_1));
    }
  private:
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
      std::vector<float>::const_iterator minval = min(msg->ranges.begin(), msg->ranges.end());
      std_msgs::msg::Float32 msg_to_send;
      msg_to_send.data = *minval;
      pubf->publish(msg_to_send); // publish result

      //for loop to output laser scan
      //added
      for (size_t i = 0; i < msg->ranges.size(); i++) {
            RCLCPP_INFO(rclcpp::get_logger("laser_scan"), "Values %d: %f", i, msg->ranges[i]);
        }
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubf;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};




int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  //laser scan subscriber object
 


  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();

  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
    //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  }
//  rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  goal_pose = []

  goal_pos = PoseStamped()
  goal_pos.header.frame_id = 'map'
  goal_pos.header.stamp = navigator.get_clock().now().to_msg()
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  goal_pose.append(goal_pos)

  goal_pos = PoseStamped()
  goal_pos.header.frame_id = 'map'
  goal_pos.header.stamp = navigator.get_clock().now().to_msg()
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  goal_pose.append(goal_pos)

  goal_pos = PoseStamped()
  goal_pos.header.frame_id = 'map'
  goal_pos.header.stamp = navigator.get_clock().now().to_msg()
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  goal_pose.append(goal_pos)

  goal_pos = PoseStamped()
  goal_pos.header.frame_id = 'map'
  goal_pos.header.stamp = navigator.get_clock().now().to_msg()
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  goal_pose.append(goal_pos)

  
  navigator.GoToPose(goal_pose);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  }
  //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin
  }
  //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin

  //added find closest 
  //rclcpp::spin(std::make_shared<FindClosest>()) ; // create and spin

  // complete here....
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}



