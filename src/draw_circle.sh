#! /usr/bin/zsh
ros2 service call /reset std_srvs/srv/Empty
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist 'linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0
' --times 10
