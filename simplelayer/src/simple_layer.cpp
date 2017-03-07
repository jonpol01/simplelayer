#include "simple_layer.h"

#include <pluginlib/class_list_macros.h>

//#define _DEBUG_
#define _POSEARRAY_MSG_

#ifdef _DEBUG_
#undef	_POSEARRAY_MSG_
//	#define _STRING_MSG_
#endif

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

//Our callback function for the topic subscriber
//everything that goes to the topic falls here
#ifdef _POSEARRAY_MSG_
void SimpleLayer::chatterCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  pose = msg->poses[0];
  position = pose.position;
}
#elif defined _DEBUG_
void SimpleLayer::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   sprintf(byte, "%s", msg->data.c_str());    //pass the message to local char byte.
  //  ROS_INFO("%s",byte);                               //debug: show incoming message
}
#endif

void SimpleLayer::poseCallback(const geometry_msgs::Pose::ConstPtr& r_pose)
{
  r_position = r_pose->position;
  ROS_INFO("robot_pose [%f][%f]", r_position.x, r_position.y);
}

//This makes the plugin initialize on roscore boot
void SimpleLayer::onInitialize()
{

  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  //basic parameters
  nh.param("controller_frequency", controller_frequency_, 10.0);

  //Subscribe to a topic our custom topic
  //this should be the oe to feed us the fake / robot pose
  sub = nh.subscribe("rmc_update", 1000, &SimpleLayer::chatterCallback, this);
  pos = nh.subscribe("/robot_pose", 1000, &SimpleLayer::poseCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

//  map->updateOrigin(5.000,5.000);

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
  if (position.x > 0 && position.y > 0) {
    double o_x = position.x - r_position.x;
    double o_y = position.y - r_position.y;
    bot_x_ = -(robot_x + o_x);
    bot_y_ = -(robot_y + o_y);
    // ROS_INFO("robot_costrmap_pose [%f][%f]", robot_x, robot_y);
    ROS_INFO("obstacle [%f][%f]", bot_x_, bot_y_);
    // ROS_INFO("robot_pose [%f][%f]", r_pose.position.x, r_pose.position.y);
  }

//  bot_x_ = position.x;
//  bot_y_ = position.y;
  // bot_x_ = floor(position.x * 1000) / 1000;
  // bot_y_ = floor(position.y * 1000) / 1000;
  //JLK004 dock
//  bot_x_ = floor(7.498 * 1000) / 1000;
//  bot_y_ = floor(16.433 * 1000) / 1000;
  //GOAL B coordinates
//  bot_x_ = floor(rx * 1000) / 1000;
//  bot_y_ = floor(ry * 1000) / 1000;

  //x: 5.886000, y: 4.966000
//  ROS_INFO("robot_pose [%f][%f]", robot_x, robot_y);
//  bot_x_ = floor(5.866000 * 1000) / 1000;
//  bot_y_ = floor(4.966000 * 1000) / 1000;

  *min_x = std::min(*min_x, bot_x_);
  *min_y = std::min(*min_y, bot_y_);
  *max_x = std::min(*max_x, bot_x_);
  *max_y = std::min(*max_y, bot_y_);

}

//This is called periodically by the updateMap func of ROS
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned int mx, my, i;
  char * ph;

#ifdef _POSEARRAY_MSG_

  //unimplemented
//  ros::Rate r(controller_frequency_);
  bot_x_ = bot_x_ * 0.25;
  bot_y_ = bot_y_ * 0.25;

  if(master_grid.worldToMap(bot_x_, bot_y_, mx, my)){
    if(bot_x_ != 0.000 && bot_y_ != 0.000)
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }

  ROS_INFO("robot_pose [%f][%f]", bot_x_, bot_y_);
  ROS_INFO("origin:[%f][%f][%f], mx:[%ld], my:[%ld]", master_grid.getOriginX(), master_grid.getOriginY(), master_grid.getResolution(), (long int)mx, (long int)my);

//  master_grid.worldToMapEnforceBounds(bot_x_, bot_y_, mx, my);
//  ROS_INFO("mx: [%ld], my: [%ld]", (long int)mx, (long int)my);
//  master_grid.setCost(mx, my, LETHAL_OBSTACLE);

#elif defined _DEBUG_

  ph = strtok(byte, " ,");
  while (ph != NULL)
  {
    if(i == 0) bot_x_ = atof(ph);
    else bot_y_ = atof(ph);

    // ROS_INFO("%s", ph);
    ph = strtok (NULL, " ,");
    i++;
  }

  i = 0;

  ROS_INFO("x: [%f], y: [%f]", bot_x_, bot_y_);

  if(master_grid.worldToMap(bot_x_, bot_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }

#endif

}


} // end namespace
