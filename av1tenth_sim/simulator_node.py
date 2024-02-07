import rclpy
from rclpy.node import Node
import tf2_ros
# from tf2_ros import TransformBroadcaster, TransformListener, Buffer, Time, Duration

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

import av1tenth_sim.sim_utils.vehicle_sim as vs


class VehicleSimulator(Node):
    def __init__(self):
        super().__init__("vehicle_simulator")

        init_state_params = self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_1.initial_position', [0.0, 0.0]),
                ('vehicle_1.initial_heading', 0.0)
            ]
        )
        vehicle1_initial_state = vs.VehicleState(*[param.value for param in init_state_params])

        init_state_params = self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_2.initial_position', [0.0, 0.0]),
                ('vehicle_2.initial_heading', 0.0)
            ]
        )
        vehicle2_initial_state = vs.VehicleState(*[param.value for param in init_state_params])

        init_state_params = self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_3.initial_position', [0.0, 0.0]),
                ('vehicle_3.initial_heading', 0.0)
            ]
        )
        # Unpack the list of parameter values as arguments.
        vehicle3_initial_state = vs.VehicleState(*[param.value for param in init_state_params])
        
        veh_sim_params = self.declare_parameters(
            namespace='',
            parameters=[
                ('veh_sim.sim_time_step', 0.05),
                ('veh_sim.output_coordinates', 'utm_local'),
                ('veh_sim.track', 0.1937),
                ('veh_sim.kingpin_width', 0.1524),
                ('veh_sim.wheelbase', 0.3143),
                ('veh_sim.steering_motion_delay', 0.087),
                ('veh_sim.steering_signal_delay', 0.0),
                ('veh_sim.speed_delay', 1.0),
                ('veh_sim.position_feedback_noise', 0.0),
                ('veh_sim.velocity_feedback_noise', 0.0)
            ]
        )
        # Unpack the list of parameter values as arguments.
        vs_params = vs.VehicleSimParameters(*[param.value for param in veh_sim_params])
        self.get_logger().info(str(vs_params))

        # ======= Timers =======
        self.timer = self.create_timer(timer_period_sec=vs_params.sim_time_step, callback=self.timer_callback)

        # ======= Subscribers =======
        self.vehicle_1_ackermann_sub = self.create_subscription(
            AckermannDriveStamped, '/vehicle_1/vehicle_command_ackermann', self.vehicle_1_ackermann_callback, 1
        )
        self.vehicle_2_ackermann_sub = self.create_subscription(
            AckermannDriveStamped, '/vehicle_2/vehicle_command_ackermann', self.vehicle_2_ackermann_callback, 1
        )
        self.vehicle_3_ackermann_sub = self.create_subscription(
            AckermannDriveStamped, '/vehicle_3/vehicle_command_ackermann', self.vehicle_3_ackermann_callback, 1
        )

        self.tf2_buff = tf2_ros.Buffer() # TF2 requires an object to cache transforms.
        self.tf_lis = tf2_ros.TransformListener(buffer=self.tf2_buff, node=self)

        # ======= Publishers =======
        self.vehicle_1_pose_pub = self.create_publisher(PoseStamped, '/vehicle_1/utm_pose', 1)
        self.vehicle_2_pose_pub = self.create_publisher(PoseStamped, '/vehicle_2/utm_pose', 1)
        self.vehicle_3_pose_pub = self.create_publisher(PoseStamped, '/vehicle_3/utm_pose', 1)

        self.br = tf2_ros.TransformBroadcaster(self)

        # ======= Variables =======
        self.vehicle_1 = vs.VehicleSim(initial_state=vehicle1_initial_state, params=vs_params)
        self.vehicle_2 = vs.VehicleSim(initial_state=vehicle2_initial_state, params=vs_params)
        self.vehicle_3 = vs.VehicleSim(initial_state=vehicle3_initial_state, params=vs_params)
        self.transform_received = False

    def vehicle_1_ackermann_callback(self, msg: AckermannDriveStamped)-> None:
        """ Update commands relating to the motion of the vehicle. """
        self.vehicle_1.update_commands(steering_angle=msg.drive.steering_angle, speed=msg.drive.speed)

    def vehicle_2_ackermann_callback(self, msg: AckermannDriveStamped)-> None:
        """ Update commands relating to the motion of the vehicle. """
        self.vehicle_2.update_commands(steering_angle=msg.drive.steering_angle, speed=msg.drive.speed)

    def vehicle_3_ackermann_callback(self, msg: AckermannDriveStamped)-> None:
        """ Update commands relating to the motion of the vehicle. """
        self.vehicle_3.update_commands(steering_angle=msg.drive.steering_angle, speed=msg.drive.speed)

    def timer_callback(self)-> None:
        """ Simulator state update loop. """

        # Look up static transform before simulator starts? Don't know if this will work.
        if self.vehicle_1.params.output_coordinates == 'utm' and self.transform_received is False:
            while True:
                try:
                    utm_local_to_utm: TransformStamped = self.tf2_buff.lookup_transform(target_frame="utm_local", source_frame="utm",
                                                                    time=tf2_ros.Time(seconds=0),
                                                                    timeout=tf2_ros.Duration(seconds=0.04))
                    
                    self.vehicle_1.utm_output(transform=utm_local_to_utm.transform)
                    self.vehicle_2.utm_output(transform=utm_local_to_utm.transform)
                    self.vehicle_3.utm_output(transform=utm_local_to_utm.transform)
                    self.transform_received = True
                except tf2_ros.TransformException as ex:
                    self.get_logger().warn(f'{type(ex).__name__}: {ex}', throttle_duration_sec=2)
                    return

        self.get_logger().info(f'Simulator starting with update rate of {1/self.vehicle_1.params.sim_time_step} Hz.', once=True)
        
        # Perform vehicle and trailer sim updates
        self.vehicle_1.update_state()
        self.vehicle_2.update_state()
        self.vehicle_3.update_state()

        # Generate the tf2 data for the vehicle_1
        t_v = TransformStamped()
        t_v.header.stamp = self.get_clock().now().to_msg()
        t_v.header.frame_id = self.vehicle_1.params.output_coordinates
        t_v.child_frame_id = "vehicle_1/chassis"
        t_v.transform = self.vehicle_1.output_transform()

        t_lw = TransformStamped()
        t_lw.header.stamp = t_v.header.stamp
        t_lw.header.frame_id = 'vehicle_1/chassis'
        t_lw.child_frame_id = 'vehicle_1/left_wheel'
        t_lw.transform = self.vehicle_1.output_left_wheel_transform()

        t_rw = TransformStamped()
        t_rw.header.stamp = t_v.header.stamp
        t_rw.header.frame_id = 'vehicle_1/chassis'
        t_rw.child_frame_id = 'vehicle_1/right_wheel'
        t_rw.transform = self.vehicle_1.output_right_wheel_transform()

        # Send the transformations
        self.br.sendTransform([t_v, t_lw, t_rw])

        # Generate the tf2 data for the vehicle_2
        t_v = TransformStamped()
        t_v.header.stamp = self.get_clock().now().to_msg()
        t_v.header.frame_id = self.vehicle_2.params.output_coordinates
        t_v.child_frame_id = "vehicle_2/chassis"
        t_v.transform = self.vehicle_2.output_transform()

        t_lw = TransformStamped()
        t_lw.header.stamp = t_v.header.stamp
        t_lw.header.frame_id = 'vehicle_2/chassis'
        t_lw.child_frame_id = 'vehicle_2/left_wheel'
        t_lw.transform = self.vehicle_2.output_left_wheel_transform()

        t_rw = TransformStamped()
        t_rw.header.stamp = t_v.header.stamp
        t_rw.header.frame_id = 'vehicle_2/chassis'
        t_rw.child_frame_id = 'vehicle_2/right_wheel'
        t_rw.transform = self.vehicle_2.output_right_wheel_transform()
        
        # Send the transformations
        self.br.sendTransform([t_v, t_lw, t_rw])

        # Generate the tf2 data for the vehicle_3
        t_v = TransformStamped()
        t_v.header.stamp = self.get_clock().now().to_msg()
        t_v.header.frame_id = self.vehicle_3.params.output_coordinates
        t_v.child_frame_id = "vehicle_3/chassis"
        t_v.transform = self.vehicle_3.output_transform()

        t_lw = TransformStamped()
        t_lw.header.stamp = t_v.header.stamp
        t_lw.header.frame_id = 'vehicle_3/chassis'
        t_lw.child_frame_id = 'vehicle_3/left_wheel'
        t_lw.transform = self.vehicle_3.output_left_wheel_transform()

        t_rw = TransformStamped()
        t_rw.header.stamp = t_v.header.stamp
        t_rw.header.frame_id = 'vehicle_3/chassis'
        t_rw.child_frame_id = 'vehicle_3/right_wheel'
        t_rw.transform = self.vehicle_3.output_right_wheel_transform()
    
        # Send the transformations
        self.br.sendTransform([t_v, t_lw, t_rw])

        pose = PoseStamped()
        pose.header.stamp = t_v.header.stamp
        pose.header.frame_id = self.vehicle_1.params.output_coordinates
        pose.pose = self.vehicle_1.output_pose()

        self.vehicle_1_pose_pub.publish(pose)

        pose = PoseStamped()
        pose.header.stamp = t_v.header.stamp
        pose.header.frame_id = self.vehicle_2.params.output_coordinates
        pose.pose = self.vehicle_2.output_pose()

        self.vehicle_2_pose_pub.publish(pose)

        pose = PoseStamped()
        pose.header.stamp = t_v.header.stamp
        pose.header.frame_id = self.vehicle_3.params.output_coordinates
        pose.pose = self.vehicle_3.output_pose()

        self.vehicle_3_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    node = VehicleSimulator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()