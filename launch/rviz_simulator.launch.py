from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    simulator = Node(
        package = 'av1tenth_sim',
        executable = 'simulator',
        output = 'screen',
        parameters = [
            {
                'initial_position': [29.967755, -85.472341], # Lattitude, Longitude
                'initial_heading': 10.0, # Degrees from east
                'veh_sim.sim_time_step': 0.02, # seconds
                'veh_sim.shift_coordinates_relative_to_initial_position': False, 
                'veh_sim.track': 0.1937, # meters
                'veh_sim.kingpin_width': 0.1524, # meters
                'veh_sim.wheelbase': 0.3143, # meters
                'veh_sim.steering_motion_delay': 0.698132, # rad/s
                'veh_sim.steering_signal_delay': 0.0, # seconds
                'veh_sim.speed_delay': 0.5, # acceleration, m/s^2 
                'veh_sim.position_feedback_noise': 0.002,  # standard deviation (meters)
                'veh_sim.velocity_feedback_noise': 0.04,  # standard deviation (m/s)
            }
        ]
    )
    ld.add_action(simulator)

    visualizer = Node(
        package = 'av1tenth_sim',
        executable = 'visualizer',
        output = 'screen'
    )
    ld.add_action(visualizer)

    config_dir = get_package_share_directory("av1tenth_sim")
    rviz = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", config_dir + "/config/av1tenth.rviz"]
    )
    ld.add_action(rviz)

    return ld