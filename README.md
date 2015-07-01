agent_system homeworks
===

Slide archives: http://1drv.ms/1FdAUDy 

# 2015/4/15

This is homework to understand git/github

**NOTE:** This homework is located on another repository.
https://github.com/agent-system/homework20150415/

# 2015/4/22

This homework is for understanding a concept of Robot Architecture, and ROS(Robot Operating System).

Before doing this homework, you have to setup catkin ROS workspace for executing homework.

```bash
# install catkin tools
$ sudo apt-get install python-catkin-tools python-rosdep ros-hydro-desktop-full
# create directory for workspace
$ mkdir -p $HOME/ros/hydro/src
$ cd $HOME/ros/hydro
# initialize workspace
$ source /opt/ros/hydro/setup.bash
$ catkin init
$ cd src
# put homework on your workspace
$ git clone https://github.com/agent-system/homework
$ cd ..
# install dependencies
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r
$ source /opt/ros/hydro/setup.bash
# build homework20150422 package
$ catkin build homework20150422
$ source $HOME/ros/hydro/devel/setup.bash
$ echo 'source $HOME/ros/hydro/devel/setup.bash' >> $HOME/.bashrc
```

# 2015/4/29-

To be Continued...

# 2015/6/17

See the gazebo slides in the slide archives: http://1drv.ms/1FdAUDy 

Getting Started with Gazebo

```bash
# launch gazebo with samplerobot

wget -q -O /tmp/jsk.rosbuild https://raw.github.com/jsk-ros-pkg/jsk_common/master/jsk.rosbuild
bash /tmp/jsk.rosbuild -s --rtm hydro
roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers.launch
rtmlaunch hrpsys_gazebo_tutorials samplerobot_hrpsys_bringup.launch

# with kinect

roscd hrpsys_gazebo_tutorials
git remote add furushchev https://github.com/furushchev/rtmros_tutorials.git
git fetch furushchev
git checkout -b samplerobot-with-kinect furushchev/samplerobot-with-kinect
catkin bt

source ~/ros/hydro/devel/setup.bash
# terminal 1
roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers_drc_testbed.launch

# terminal 2
rtmlaunch hrpsys_gazebo_tutorials samplerobot_hrpsys_bringup.launch

# terminal 3
rosrun rviz rviz
# add RobotModel, PointCloud2 and Camera on rviz menu
# choose /xtion/depth/points/ at Topic of PointCloud2
# choose /xtion/depth/merge_raw at Image Topic of Camera

# terminal 4
roscd hrpsys_ros_bridge_tutorials/euslisp
roseus samplerobot-interface.l
# you may need rosrun roseus generate-all-msg-srv.sh 

;; 以下roseus
(samplerobot-init)
(send *ri* :angle-vector (send *sr* :reset-pose) 4000)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
(send *ri* :go-pos -0.5 0 0) ;; x y theta [m]

```

## Tips

- when executing `catkin bt`, error `collada_urdf_jsk_patch not found` occurs in indigo

    please add `jsk_3rdparty` repository to your local and build again

    ```bash
roscd
cd ../src
wstool set jsk-ros-pkg/jsk_3rdparty --git https://github.com/jsk-ros-pkg/jsk_3rdparty.git
wstool up jsk-ros-pkg/jsk_3rdparty
cd jsk-ros-pkg/jsk_3rdparty/3rdparty/assimp_devel
touch CATKIN_IGNORE # this is for speedup compiling
cd ..
catkin build collada_urdf_jsk_patch
catkin build hrpsys_gazebo_tutorials
source devel/setup.bash
```
