# homework20150422

1. execute `talker/listener` sample
  
  **Goal:** Comprehend node, topic, publish/subscriber and command line tools for ROS

  - Open new terminal and execute command below:
  ```bash
# Terminal 1
roscore
```
  - Open new terminal and execute command below
  ```bash
  # Terminal 2
rosrun homework20150422 talker.l
```
  - Open new terminal and execute command below:
  ```bash
  # Terminal 3
rosrun homework20150422 listener.l
```

  - Open new terminal and execute commands below:
  ```bash
  # Terminal 4
rostopic list # you can find all topics advertised
rostopic info /chatter # you can see detail information of /chatter topic
rostopic echo /chatter # you can also see the message published as /chatter topic
```

2. execute `turtlesim` sample

  **Goal:** Comprehend ros distributed system with multiple launguages, and its launching system called `roslaunch`

  - Open new terminal and execute commands below:
  ```bash
  # Terminal 1
roslaunch homework20150422 turtlesim.launch
```

  - Focus the terminal and push `up` `down` `right` `left` keys.

  - See what nodes are launching by `rosnode list`:
  ```bash
  # Terminal 2
rosnode list # you can see what nodes are on ROS
```

  - Now let's take a look at `turtlesim.launch`
  ```xml
<launch>
  <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node" />
  <node name="turtlesim_teleop" pkg="roseus" type="roseus"
        args="$(find homework20150422)/euslisp/turtlesim-teleop.l"
        output="screen">
     <remap from="cmd_vel" to="/turtle1/cmd_vel" />
  </node>
</launch>
```

  - You can see 2 `<node></node>` tags in launch file. This means these 2 nodes are launched.:
  ```xml
  <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node" />
```
    - `name` attribute: name of the node

  **NOTE** (each node name is already named in each nodes, but once you set name different from that written at original node, it will be overwriten (we call it `remapped`))
    - `pkg` attribute: package of the node
    - `type` attribute: program name of the node (we can use each independent node by `rosrun <pkg> <type>`)

  - Now let's look at next node example:
  ```xml
  <node name="turtlesim_teleop" pkg="roseus" type="roseus"
        args="$(find homework20150422)/euslisp/turtlesim-teleop.l"
        output="screen">
     <remap from="cmd_vel" to="/turtle1/cmd_vel" />
  </node>
```
    - `args` attribute (optional): arguments passed when launching node
    - `output` attribute (optional): when this has value `screen`, we can see output log of this node. (This is important when launching nodes on another machine, and need to transmit log output to your terminal)
    - `<remap from="from_topic" to="to_topic" />`: remapping topic name
      Detail: `turtlesim-teleop.l` has publisher of `cmd_vel`, so if you launch this node with `rosrun`, this node publishes `/cmd_vel` topic. But, if remapped `cmd_vel` to `/turtle1/cmd_vel` as described above, this node changes the topic name from `/cmd_vel` to `/turtle1/cmd_vel`.
      You can change subscribe/publish topic of the node without changing any code.


3. execute `face_detector` sample:

  **GOAL** Comprehend nested `launch`, topic remapping

  - Open new terminal and execute command below:
  ```bash
roslaunch homework20150422 camera.launch
```

  - You can see the image captured from the camera of your PC

  - Press `Ctrl-C` to stop this launch

  - Open new terminal and execute command below:
  ```bash
roslaunch homework20150422 face_detect.launch
```

  - You can see 2 image_view (one shows raw image view, and another shows the image drawn rectangle over the detected face using OpenCV)

  ```xml
  <arg name="show_debug_image" default="true" />
  ...
  <node name="debug_image_viewer" pkg="image_view" type="image_view"
        if="$(arg show_debug_image)">
  ...
```

  - You can use arguments in launch file
  - You can also set arguments when launches launch file
    e.g.: `roslaunch homework20150422 face_detect.launch show_debug_image:=false`

  ```xml
  <include file="$(find homework20150422)/launch/camera.launch" />
```
  - You can include another `launch` files.

4. Make your own image processing node

  - previous `face_detector` example has node doing 3 things:
    - Subscribe `sensor_msgs/Image` message
    - Processs image subscribed (detecting face, and draw rectangle on each image)
    - Publish processed image
  - Let's make your own node to process image, and visualize processed image
  - Please send pull request below:
    - your source codes of image processing that:
      - Subscribe `sensor_msgs/Image` message
      - Processs image subscribed (any processing is ok)
      - Publish processed image
    - make `launch` file to show following:
      - original camera image
      - processed image
    - add `README` which describes your node
