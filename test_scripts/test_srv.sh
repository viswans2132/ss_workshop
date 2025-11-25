while true; do
  rosservice call /go1_gazebo/control_status
  sleep 1
done
