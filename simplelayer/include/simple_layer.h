#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/voxel_layer.h>
#include <dynamic_reconfigure/server.h>
#include "std_msgs/String.h"    //""
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
  public:
    SimpleLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                              double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  geometry_msgs::Point position;
  double mark_x_, mark_y_;
  double bot_x_, bot_y_;
  double controller_frequency_;
  geometry_msgs::Pose pose;
  costmap_2d::Costmap2D *map;
  costmap_2d::VoxelLayer *map2;
    geometry_msgs::Point r_position;

    char byte[1000];
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    virtual void chatterCallback(const geometry_msgs::PoseArray::ConstPtr&);
    // virtual void chatterCallback(const std_msgs::String::ConstPtr&);
  //  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub);
    void poseCallback(const geometry_msgs::Pose::ConstPtr&);
    ros::Subscriber sub;
    ros::Subscriber pos;

  };
}
#endif

