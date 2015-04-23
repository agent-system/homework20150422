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
# install catkin tools (except Ubuntu 14.04)
$ sudo apt-get install python-catkin-tools python-rosdep ros-hydro-desktop-full
# install catkin tools (Ubuntu 14.04)
$ sudo apt-get install -y python-pip
$ sudo pip install -U catkin
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
