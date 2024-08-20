# CubeMars (TMotors) Hardware Interface for Ros2 Control 

### Enable CAN interface


``` bash
  ip link set can0 up type can bitrate 1000000
  ip link set can0 txqueuelen 1000
```