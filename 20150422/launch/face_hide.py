<launch>
  <arg name="show_debug_image" default="true" />

  <include file="$(find homework20150422)/launch/camera.launch" />

  <node name="face_detect_mono" pkg="homework20150422" type="face_hide.py" output="screen">
    <remap from="image" to="camera/image_raw"/>
    <remap from="debug_image" to="camera/debug_image" />
  </node>

  <node name="debug_image_viewer" pkg="image_view" type="image_view"
        if="$(arg show_debug_image)">
    <remap from="image" to="camera/debug_image" />
  </node>
</launch>
