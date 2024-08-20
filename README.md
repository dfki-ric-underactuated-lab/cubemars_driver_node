# CubeMars (TMotors) Hardware Interface for Ros2 Control 

### Enable CAN interface


``` bash
  ip link set can0 up type can bitrate 1000000
  ip link set can0 txqueuelen 1000
```

### Example Xacro
``` xml

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find cubemars_hardware_interface)/ros2_control/cubemars.ros2_control.xacro" />

  <ros2_control name="Example" type="system">
    <hardware>
      <plugin>cubemars_hardware_interface/CubemarsHardwareInterface</plugin>
      <param name="can_interface">can0</param>
    </hardware>

    <xacro:AK80-9_V2 name="right_shoulder_joint0" can_id="12" kp="3." kd="1." prefix=""/>
    <xacro:AK10-9 name="right_shoulder_joint1" can_id="11" kp="3." kd="1." prefix=""/>
    <xacro:AK10-9 name="right_shoulder_joint2" can_id="23" kp="5." kd="5." prefix=""/>
    <xacro:AK80-9_V2 name="right_elbow_joint"  can_id="14" kp="5." kd="5." prefix=""/>
  </ros2_control>


</robot>
```