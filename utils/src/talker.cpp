#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>

#include <sstream>

#define Leg_y 10.00
#define Leg_x 13.00

//#define _DEBUG_
#define _POSEARRAY_MSG_

#ifdef _DEBUG_
#undef _POSEARRAY_MSG_
//      #define _STRING_MSG_
#endif

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  double _x = Leg_x;
  double _y = Leg_y;
  double controller_frequency_;
  bool x_flg, y_flg;
  char byte[1000];


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //basic parameters
  n.param("controller_frequency", controller_frequency_, 10.0);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
#ifdef _POSEARRAY_MSG_
  // ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("/move_base/global_costmap/SimpleLayer/rmc_update", 1000);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("/move_base/local_costmap/SimpleLayer/rmc_update", 1000);
#elif defined _DEBUG_
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/move_base/global_costmap/SimpleLayer/rmc_update", 1000);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/move_base/local_costmap/SimpleLayer/rmc_update", 1000);
#endif

  ros::Rate loop_rate(controller_frequency_);


  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

#ifdef _POSEARRAY_MSG_
    geometry_msgs::PoseArray Fake_pose;
    geometry_msgs::PoseStamped cpose;

    Fake_pose.header.stamp = ros::Time::now();

//   cpose.pose.position.x =  8.10;
//   cpose.pose.position.y =  3.77;
   cpose.pose.position.x = 8.02;
   cpose.pose.position.y = 3.46;

    // cpose.pose.position.x =  _x;
    // cpose.pose.position.y =  _y;

    Fake_pose.poses.push_back(cpose.pose);

    ROS_INFO("x: %f, y: %f", cpose.pose.position.x, cpose.pose.position.y);
#elif defined _DEBUG_
    std_msgs::String Fake_pose;

    std::stringstream ss;
    ss << "hello world " << count;
    sprintf(byte, "%.3f,%.3f", _x, _y);

    ss << byte;
    Fake_pose.data = ss.str();

    ROS_INFO("%s", Fake_pose.data.c_str());
#endif

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(Fake_pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    if(_x >= 9) x_flg = true;
    else if(_x <= 8.02) x_flg = false;

    if(x_flg == true) _x -= 0.008;
    else _x += 0.008;

    if(_y >= 3.46) y_flg = true;
    else if(_y <= 2.74) y_flg = false;

    if(y_flg == true) _y -= 0.005;
    else _y += 0.005;
  }


  return 0;
}
