# homework 20150708

update source code
```bash
git checkout -b otsubo/sample-basket-update
git pull otsubo samplerobot-basket-update
```

launch gazebo
```bash
# terminal 1
roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers_basket.launch
```

launch hrpsys
```bash
# terminal 2
rtmlaunch hrpsys_gazebo_tutorials samplerobot_hrpsys_bringup.launch
```

control robot
```bash
# terminal 3
roscd hrpsys_gazebo_tutorials/euslisp
emacs -nw samplerobot-reaching2.l

# in Shell
(main)
#in (main)
(roseus samplerobot-reaching2.l)
(send *ri* :calibrate-inertia-sensor)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
(send *ri* :start-impedance :arms)
(reaching-pose)
(send *ri* :go-pos 0.2 0 0)
(grasp)
(reaching-pose2)
(send *ri* :go-pos 0 0.5 0)
(send *ri* :go-pos 0 0.5 0)
(reaching-pose)
```

