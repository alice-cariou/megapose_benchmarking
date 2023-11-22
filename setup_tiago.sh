export TIAGO_IP=10.68.0.1
export ROS_MASTER_URI=http://$TIAGO_IP:11311
export ROS_IP="$(ip route get $TIAGO_IP | awk '{print $5}')"

source ~/openrobots_ws/setup-env.sh
