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


Get serial number for CAN device (e.g. can0): `udevadm info -a -p $(udevadm info -q path -p /sys/class/net/can0)| grep serial| head -n 1`
### /etc/udev/rules.d/80-can.rules
```
SUBSYSTEM=="net", ACTION=="add|change", ATTRS{serial}=="JE013488", NAME="can0" 
SUBSYSTEM=="net", ACTION=="add|change", ATTRS{serial}=="JE013486", NAME="can1"

SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add|change", ATTR{tx_queue_len}="1000"
```

### /etc/systemd/network/80-can.network
```
[Match]
Name=can*

[CAN]
BitRate=1000K
RestartSec=1000ms
```