//#include<simple_layers/simple_layer.h>
#include "simple_layer.h"
#include "ros/ros.h"		//for subscribing to topic
#include "std_msgs/String.h"	//""

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  
}

void SimpleLayer::onInitialize()
{

  ros::NodeHandle nh("~/" + name_);
  std::string chatter;
  current_ = true;

  sub = nh.subscribe(chatter, 1, &SimpleLayer::chatterCallback, this);
//  sub_ = nh.subscribe("chatter", 1000, &SimpleLayer::chatterCallback);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

//  ROS_INFO("uBOUNDS");
  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
//  ros::init();

//  ros::NodeHandle n;

//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//  ros::Subscriber subscriber("chatter", chatterCallback);
//  ros::spinOnce();

//  ROS_INFO("uCost");
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace

