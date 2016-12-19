export ROS_MASTER_URI=http://odroid:11311
export ROS_HOSTNAME=odroid
. /opt/ros/kinetic/setup.sh
. ~/catkin_ws/devel/setup.sh
exec "$@"
