# tidyup_interactions
Tidy Up My Room, Robot! An Investigation into Human-Robot Teamwork with a Living Room Setting - Final Year Project 2nd Edition.

Previous works by Jake Santini can be found : https://github.com/JakeSantini/Tidy-Up-My-Room-Fetch-

This repoistory stores the source file for the catkin workspace. The code runs in two stages. 
  1. Fetch tidys autonomously picking up all colours 
  2. Fetch and partipants allocate tasks and Fetch picks up only assigned colours

To run on a Fetch (1080) robot :

0. Make sure you have all the packages required which can be found in requirements.txt 
      
      pip install -r reqiurements.txt
 
1. On Fetch, launch navigation stack

      roslaunch fetch_navigation fetch_nav.launch map_file:=/home/hrigroup/jake/jsan_ws/src/destination/Maps/map5yaml

2. On your laptop terminal, open RVIZ

      roscd fetch_navigation/config
      export ROS_MASTER_URI=http://fetch1080:11311
      rviz -d navigation.rviz

3. On Fetch, run MoveIt

      roslaunch fetch_moveit_config move_group.launch

4. On your laptop terminal, run the GUI (replacing for your directory and IP address)

      source /home/jenny/base/devel/setup.bash
      
      export ROS_MASTER_URI=http://fetch1080:11311
      
      export ROS_IP=160.69.69.123
      
      rosrun destination button_GUI.py
      
      
5. For Stage 2, in your laptop terminal, run speech script (replacing for your directory and IP address)

      source /home/jenny/base/devel/setup.bash
      
      export ROS_MASTER_URI=http://fetch1080:11311
      
      export ROS_IP=160.69.69.123
      
      rosrun destination user_input.py
      

6. On Fetch, run Tidy code 

      source /home/hrigroup/jake/jsan_ws/devel/setup.bash
      
      export ROS_MASTER_URI=http://fetch1080:11311
      
      
      roslaunch destination destination_Stage1.launch
      
      (or) 
      
      roslaunch destination destination_Stage2.launch




     

