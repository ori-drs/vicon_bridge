#!/bin/bash
# Get transformations between the base and vicon markers.

echo "Marker 1"
rosrun tf tf_echo base vicon/marker1 | head -n 5
echo "Marker 2"
rosrun tf tf_echo base vicon/marker2 | head -n 5
echo "Marker 3"
rosrun tf tf_echo base vicon/marker3 | head -n 5
echo "Marker 4"
rosrun tf tf_echo base vicon/marker4 | head -n 5
echo "Marker 5"
rosrun tf tf_echo base vicon/marker5 | head -n 5
