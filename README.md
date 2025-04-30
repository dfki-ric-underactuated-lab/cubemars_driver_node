# Enable CAN interface


``` bash
  ip link set can0 up type can bitrate 1000000
  ip link set can0 txqueuelen 1000
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

# Use CubeMars (TMotors) Ros 2 Lifecycle Node
An example config can be found in [config/test_params.yaml]().

Start the node with:

```bash
ros2 run cubemars_hardware_interface cubemars_hardware_node --ros-args --params-file src/cubemars_driver_node/config/test_params.yaml
```
It will start in **uncofigured**. The motors are not activated. It is not even communicating via CAN.
Configure it with 
```bash
ros2 lifecycle set /cubemars_hardware_node configure
```
Now it is **configured**. The motors are enabled but only zeros are trasnmitted such that the motors are still not moving (not activate) 
The `joint_states` are being published.
Before the motors can be activated a valid `joint_commands` message has to be send, with a frequency higher than specified in the config. 
Activate the motors with:
```bash
ros2 lifecycle set /cubemars_hardware_node activate
```
Now the driver is **active**, sending the `joint_commands` to the motors. Whenever an error occurs, no `joint_commands` are being received with the expected frequency or the user triggers with:
``bash
```bash
ros2 lifecycle set /cubemars_hardware_node deactivate
```
The driver is again only **configured**. This time, as the motors have been activated before, the driver sends a damping command to the motors. You can either activate again with the command above, or clean up:
```bash
ros2 lifecycle set /cubemars_hardware_node cleanup
```
Now the driver is again **unconfigured**, means the motors are disabled and not CAN communication happends.

