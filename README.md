# UR5 Controller

ROS 2 (humble) node that streams continuous Cartesian velocity (`speedl`) commands
to a UR5 using the [`ur_rtde`](https://pypi.org/project/ur-rtde/) library.

The controller subscribes to a desired TCP velocity (`geometry_msgs/Twist`) and, in
a timed loop at `control_rate`, repeatedly calls `speedL` on the robot so motion is
continuous. It also publishes the measured TCP pose.

## Prerequisites (robot side)
Unlike a socket-only driver, `ur_rtde` requires the robot to run the official
**External Control** program:

1. Install the `ExternalControl-x.x.x.urcap` URCap on the robot (from the
   [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
   resources or UR support).
2. Add the **External Control** program node to an empty program on the teach
   pendant and start it. It listens on the two RTDE ports (30003 data / 30004 script)
   against which this controller connects.

The controller connects to the robot as a *client*, so `host` must be the robot's IP
(e.g. `192.168.137.3`), not the PC's.

## Setup (PC side)
If eth0 exists, configure it
```
sudo ip addr add 192.168.137.1/24 dev eth0
sudo ip link set eth0 up
```
Verify it's configured
```
ip addr show eth0
```

Now try pinging the UR5
````
ping 192.168.137.3
```

## Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `host` | `192.168.137.1` | UR5 robot IP (RTDE client target) |
| `tcp` | `[0,0,0,0,0,0]` | Tool centre point pose |
| `payload` | `0.0` | Payload in kg |
| `acc` | `0.5` | Acceleration passed to `speedL` |
| `control_rate` | `50.0` | Speed command rate (Hz) |
| `velocity_timeout` | `0.2` | Stop the robot if no new command in this many seconds |
| `desired_velocity_topic` | `/desired_velocity` | Input `Twist` (m/s, rad/s) |
| `measured_pose_topic` | `/measured_pose` | Output `PoseStamped` |

## Topics
- **Subscribes**: `desired_velocity_topic` (`geometry_msgs/Twist`) — desired TCP
  velocity `[vx, vy, vz, wx, wy, wz]` in the robot base frame.
- **Publishes**: `measured_pose_topic` (`geometry_msgs/PoseStamped`).

> Safety: if no velocity command arrives within `velocity_timeout`, the controller
> stops the robot with `speedStop()`. On shutdown it also calls `speedStop()` and
> `stopScript()`.