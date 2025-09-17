# tum_drone
ROS 2 Gazebo quadcopter simulator. 

## Drone Topics

### Sensors
The folowing sensors are currently implemented:
- ~/front/image_raw [__sensor_msgs/msg/Image__]
- ~/bottom/image_raw [__sensor_msgs/msg/Image__]
- ~/sonar/out [__sensor_msgs/msg/Range__]
- ~/imu/out [__sensor_msgs/msg/Imu__]
- ~/gps/nav [__sensor_msgs/msg/NavSatFix__]
- ~/gps/vel [__geometry_msgs/msg/TwistStamped__]
- ~/joint_states [__sensor_msgs/msg/JointState__]

### Control 
The following control topics are currently subscribed to:
- ~/cmd_vel [__geometry_msgs/msg/Twist__]: Steers the drone
- ~/land [__std_msgs/msg/Empty__]: Lands the drone
- ~/takeoff [__std_msgs/msg/Empty__]: Starts the drone
- ~/posctrl [__std_msgs/msg/Bool__]: Toggling between position control (give drone a pose via cmd_vel) and normal control (only use cmd_vel)
- ~/dronevel_mode [__std_msgs/msg/Bool__]: Toggeling between velocity and tilt control in normal control mode.
- ~/cmd_mode [__std_msgs/msg/Bool__]: Publishes the current control mode (position or normal control)
- ~/state [__std_msgs/msg/Int8__]: Publishes the current state of the drone (0: landed, 1: flying, 2: hovering)
- ~/reset [__std_msgs/msg/Empty__]: Resets the drone

### Ground Truth
The following ground truth topics are currently published:
- ~/gt_acc [__geometry_msgs/msg/Twist__]: ground truth acceleration
- ~/gt_pose [__geometry_msgs/msg/Pose__]: ground truth pose
- ~/gt_vel [__geometry_msgs/msg/Twist__]: ground truth velocity




## Configure Plugin

The `plugin_drone` plugin is used to control the drone. It can be configured using the following parameters:

```yaml
# ROS namespace for the drone. All topics and tf frames will be prefixed with this namespace.
namespace: /simple_drone

# Proportional gain for roll and pitch PID controllers. Controls the drone's response to roll and pitch errors.
rollpitchProportionalGain: 10.0
# Differential gain for roll and pitch PID controllers. Helps to reduce overshoot and improve stability.
rollpitchDifferentialGain: 5.0
# Maximum absolute value for roll and pitch control outputs, limiting maximum tilt angle.
rollpitchLimit: 0.5

# Proportional gain for yaw PID controller. Determines how strongly the drone responds to yaw position errors.
yawProportionalGain: 2.0
# Differential gain for yaw PID controller. Dampens the rate of change of yaw error for smoother rotation.
yawDifferentialGain: 1.0
# Maximum absolute value for yaw control output, limiting rotational rate.
yawLimit: 1.5

# Proportional gain for horizontal velocity PID controllers. Controls response to changes in horizontal velocity.
velocityXYProportionalGain: 5.0
# Differential gain for horizontal velocity PID controllers. Controls acceleration/deceleration in horizontal plane.
velocityXYDifferentialGain: 2.3
# Maximum limit for horizontal velocity control output, restricting maximum horizontal speed.
velocityXYLimit: 2

# Proportional gain for vertical velocity PID controller. Influences response to altitude changes.
velocityZProportionalGain: 5.0
# Integral gain for vertical velocity PID controller. Set to zero, indicating no error integration over time.
velocityZIntegralGain: 0.0
# Differential gain for vertical velocity PID controller. Helps control vertical acceleration and deceleration.
velocityZDifferentialGain: 1.0
# Maximum limit for vertical velocity control output. Negative value may indicate special control scenario or error.
velocityZLimit: -1

# Proportional gain for horizontal position PID controllers. Controls response to horizontal displacement errors.
positionXYProportionalGain: 1.1
# Differential gain for horizontal position PID controllers. Set to zero, meaning no rate of change consideration.
positionXYDifferentialGain: 0.0
# Integral gain for horizontal position PID controllers. Set to zero, indicating no cumulative error correction.
positionXYIntegralGain: 0.0
# Maximum limit for horizontal position control output, restricting maximum correctional force for horizontal errors.
positionXYLimit: 5

# Proportional gain for vertical position PID controller. Influences altitude adjustment in response to height errors.
positionZProportionalGain: 1.0
# Differential gain for vertical position PID controller. Smooths adjustment of altitude changes.
positionZDifferentialGain: 0.2
# Integral gain for vertical position PID controller. Set to zero, indicating no error integration over time.
positionZIntegralGain: 0.0
# Maximum limit for vertical position control output. Negative value could indicate special requirement or error.
positionZLimit: -1

# Maximum force that the drone can exert, limiting maximum thrust to prevent aggressive maneuvers.
maxForce: 30
# Parameter for adding small random noise to drone's motion. Set to zero, indicating no small noise addition.
motionSmallNoise: 0.00
# Parameter for drift noise. Set to zero, meaning no drift noise is being applied.
motionDriftNoise: 0.00
# Time interval for updating motion drift noise. Relevant only if `motionDriftNoise` is non-zero.
motionDriftNoiseTime: 50
```
