# Custom Obstacle Layer

#Install:

cd < catkin workspace>

git clone https://github.com/jonpol01/simplelayer.git

cd < dir >/catkin_ws

source ./deve/setup.bash

catkin_make

#Usage:
note. This requires you to run roscore or your own environment up running.

1. first on your costmap file

   add this on your both global and local costmap params under plugins

   - {name: SimpleLayer,          type: "simple_layer_namespace::SimpleLayer"}

2. Launch your robot / stage

3. Then run the fake / robot pose publisher

rosrun beginner_tutorials talker
