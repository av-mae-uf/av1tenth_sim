from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    simulator = Node(
        package = 'av1tenth_sim',
        executable = 'simulator',
        output = 'screen',
        parameters = [
            {
                'initial_position': [29.967755, -85.472441], # Lattitude, Longitude
                'initial_heading': 10.0, # Degrees from east
                'veh_sim.sim_time_step': 0.02, # seconds
                'veh_sim.output_coordinates': 'utm', # either 'utm' or 'utm_local'
                'veh_sim.track': 0.1937, # meters
                'veh_sim.kingpin_width': 0.1524, # meters
                'veh_sim.wheelbase': 0.3143, # meters
                'veh_sim.steering_motion_delay': 2.0, # rad/s
                'veh_sim.steering_signal_delay': 0.0, # seconds
                'veh_sim.speed_delay': 2.5, # acceleration, m/s^2 
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

    return ld