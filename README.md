# AV1Tenth Simulator
This simulator uses the bicycle model to simulate the motion of the av1tenth vehicle.

### Notes:
Steering motion delay might be neglible for the vehicles. The servo is pretty aggressive on response 
to new commands. Steering signal delay might be a better parameter to use to simulate the latency between sending the
ROS 2 driver a AckermannDriveStamped command to when the servo starts actuating.

## Use with Teleop
To use the class teleop project with the simulator, include the **remappings** options in the launch file for the teleop node.

```python
teleop = Node(
    package='class_teleop',
    executable='teleop',
    output='screen',
    remappings=[
        ('/vehicle_command_ackermann', '/simulator/vehicle_command_ackermann')
    ]
)
```
## I/O
All ***MessageTypes*** used are standard ROS messages and their specifics can be found by a google search.

### Inputs
| Format       | Message Type                | Name                                 |
|--------------|-----------------------------|--------------------------------------|
| Subscription | ***AckermannDriveStamped*** | /simulator/vehicle_command_ackermann |

### Outputs
| Format               | Message Type           | Name                |
|----------------------|------------------------|---------------------|
| Publisher            | ***PoseStamped***      | /simulator/utm_pose |
| TransformBroadcaster | ***TransformStamped*** | /tf2                |


## Launch File Example
If you want to start this simulator from a ROS 2 launch file, then use the sample code below.

**Note**: If you specify 'utm_local' for the *output_coordinates*, you must provide a static transform publisher
between 'utm' and 'utm_local'.

```Python
simulator = Node(
    package = 'av1tenth_sim',
    executable = 'simulator',
    output = 'screen',
    parameters = [
        {
            'initial_position': [29.967755, -85.472341], # Lattitude, Longitude
            'initial_heading': 10.0, # Degrees from east
            'veh_sim.sim_time_step': 0.02, # seconds
            'veh_sim.output_coordinates': 'utm', # either 'utm' or 'utm_local'
            'veh_sim.track': 0.1937, # meters
            'veh_sim.kingpin_width': 0.1524, # meters
            'veh_sim.wheelbase': 0.3143, # meters
            'veh_sim.steering_motion_delay': 2.0, # steering rate (rad/s)
            'veh_sim.steering_signal_delay': 0.0, # seconds
            'veh_sim.speed_delay': 2.5, # acceleration (m/s^2)
            'veh_sim.position_feedback_noise': 0.002,  # standard deviation (meters)
            'veh_sim.velocity_feedback_noise': 0.04,  # standard deviation (m/s)
        }
    ]
)
```

Here is an example of specifying the 'utm' to 'utm_local' transform in a launch file.
```Python
import utm # Place this at the top
reference_point = [29.967765, -85.472428]
easting, northing, _, _ = utm.from_latlon(latitude=reference_point[0], longitude=reference_point[1])
tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='utm_to_utm_local',
    arguments = ['--x', str(easting), '--y', str(northing), '--frame-id', 'utm', '--child-frame-id', 'utm_local']
)
ld.add_action(tf)
```

## Params File Example
If you want to load the parameters using a **params.yaml** file you can use the sample below.

**Note**: If you specify 'utm_local' for the *output_coordinates*, you must provide a static transform publisher
between 'utm' and 'utm_local'.

```Yaml
/**: # Matches what ever calls it. Only for single node param files
  ros__parameters:
    initial_position: [29.967765, -85.472428]  # Lattitude, Longitude
    initial_heading: 10.0  # Degrees from east
    veh_sim:
      sim_time_step: 0.02 # seconds
      output_coordinates: 'utm' # either 'utm' or 'utm_local'
      track: 0.1937 # meters
      kingpin_width: 0.1524 # meters
      wheelbase: 0.3143 # meters
      steering_motion_delay: 0.698132 # steering rate (rad/s)
      steering_signal_delay: 0.0 # seconds
      speed_delay: 0.5 # acceleration (m/s^2)
      position_feedback_noise: 0.002  # standard deviation (meters)
      velocity_feedback_noise: 0.0  # standard deviation (m/s)
```