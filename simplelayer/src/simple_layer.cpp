#include "simple_layer.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

//Our callback function for the topic subscriber
//everything that goes to the topic falls here
void SimpleLayer::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  sprintf(byte, "%s", msg->data.c_str());	//pass the message to local char byte.
  ROS_INFO("%s",byte);				//debug: show incoming message
}

//This makes the plugin initialize on roscore boot
void SimpleLayer::onInitialize()
{

  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  //Subscribe to a topic our custom topic
  //this should be the oe to feed us the fake / robot pose
  sub = nh.subscribe("rmc", 1000, &SimpleLayer::chatterCallback, this);

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

  //do something
}

//This is called periodically by the updateMap func of ROS
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  unsigned int i = 0;
  char * ph;

  ph = strtok(byte, " ,");
  while (ph != NULL)
  {
    if(i == 0) bot_x_ = atof(ph);
    else bot_y_ = atof(ph);

    //ROS_INFO("%s", ph);
    ph = strtok (NULL, " ,");
    i++;
  }

  i = 0;

  if(master_grid.worldToMap(bot_x_, bot_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}


} // end namespace
