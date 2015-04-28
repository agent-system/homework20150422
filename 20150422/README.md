# homework20150422 48156504 Akimichi Ichinose
Using indigo (Ubuntu 14.04)

Run $roslaunch homework20150422 48156504.launch

My own node (edge_detecter) is doing 3 things:
   1.Subscribe sensor_msgs/Image message
   2.Process image subscribed (detecting edge with Canny filter)
   3.Publish processed image
