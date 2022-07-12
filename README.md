# ATCsimROS_2
TFG Alberto Hermida Vazquez ETSIT, URJC. Madrid 2022

REQUIREMENTS: 

C++ compiler, ROS and its 'Noetic Ninjemys' distribution are required.
  
To clone the repository
  git clone https://github.com/ahermidavaz/ATCSimRos_2
  
This is the second version of this project. To clone the repository of the previous version
  git clone https://github.com/dherraiz/ATCsimROS
  
To execute the simulator:
  mkdir -p catkin_ws/src
  cd catkin_ws/src
  git clone https://github.com/ahermidavaz/ATCSimRos_2
  source /opt/ros/noetic/setup.bash
  cd..
  catkin_make -j1
  source devel/setup.bash
  roslaunch atcsim demo.launch
  

