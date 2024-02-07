import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class SimVisualizer(Node):

    def __init__(self):
        super().__init__('sim_visualizer')

        # Use these to publish the .STL files for Rviz
        # Create a latching publisher for task_paths_viz
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.vehicle_1_chassis_pub = self.create_publisher(Marker, 'vehicle_1/chassis', latching_qos)
        self.vehicle_1_left_wheel_pub = self.create_publisher(Marker, 'vehicle_1/left_wheel', latching_qos)
        self.vehicle_1_right_wheel_pub = self.create_publisher(Marker, 'vehicle_1/right_wheel', latching_qos)

        self.vehicle_2_chassis_pub = self.create_publisher(Marker, 'vehicle_2/chassis', latching_qos)
        self.vehicle_2_left_wheel_pub = self.create_publisher(Marker, 'vehicle_2/left_wheel', latching_qos)
        self.vehicle_2_right_wheel_pub = self.create_publisher(Marker, 'vehicle_2/right_wheel', latching_qos)

        self.vehicle_3_chassis_pub = self.create_publisher(Marker, 'vehicle_3/chassis', latching_qos)
        self.vehicle_3_left_wheel_pub = self.create_publisher(Marker, 'vehicle_3/left_wheel', latching_qos)
        self.vehicle_3_right_wheel_pub = self.create_publisher(Marker, 'vehicle_3/right_wheel', latching_qos)

        self.timer = self.create_timer(timer_period_sec=0.2, callback=self.timer_callback)

    def timer_callback(self):
        """
            Look into using the frame_locked functionality
        """

        # ===== Vehicle 1 =====
        stamp = self.get_clock().now().to_msg()
        chassis_marker = Marker()
        chassis_marker.header.stamp = stamp
        chassis_marker.header.frame_id = 'vehicle_1/chassis'
        chassis_marker.id = 0
        chassis_marker.type = Marker.MESH_RESOURCE
        chassis_marker.action = Marker.ADD
        chassis_marker.color = ColorRGBA(r=0.97,g=0.57,b=0.22, a=1.0)
        chassis_marker.pose = Pose(position=Point(x=0.0, y=0.0, z=-0.0), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        # Pose(position=Point(x=0.385, y=-0.120, z=-0.085), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        chassis_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        chassis_marker.mesh_resource = 'package://av1tenth_sim/models/simple_model.STL'
        chassis_marker.mesh_use_embedded_materials = False
        chassis_marker.frame_locked = True

        left_wheel_marker = Marker()
        left_wheel_marker.header.stamp = stamp
        left_wheel_marker.header.frame_id = 'vehicle_1/left_wheel'
        left_wheel_marker.id = 1
        left_wheel_marker.type = Marker.MESH_RESOURCE
        left_wheel_marker.action = Marker.ADD
        left_wheel_marker.color = ColorRGBA(r=0.97,g=0.57,b=0.22, a=1.0)
        left_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        left_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        left_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        left_wheel_marker.mesh_use_embedded_materials = False
        left_wheel_marker.frame_locked = True
        
        right_wheel_marker = Marker()
        right_wheel_marker.header.stamp = stamp
        right_wheel_marker.header.frame_id = 'vehicle_1/right_wheel'
        right_wheel_marker.id = 2
        right_wheel_marker.type = Marker.MESH_RESOURCE
        right_wheel_marker.action = Marker.ADD
        right_wheel_marker.color = ColorRGBA(r=0.97,g=0.57,b=0.22, a=1.0)
        right_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        right_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        right_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        right_wheel_marker.mesh_use_embedded_materials = False
        right_wheel_marker.frame_locked = True

        self.vehicle_1_chassis_pub.publish(chassis_marker)
        self.vehicle_1_left_wheel_pub.publish(left_wheel_marker)
        self.vehicle_1_right_wheel_pub.publish(right_wheel_marker)

        # ===== Vehicle 2 =====
        stamp = self.get_clock().now().to_msg()
        chassis_marker = Marker()
        chassis_marker.header.stamp = stamp
        chassis_marker.header.frame_id = 'vehicle_2/chassis'
        chassis_marker.id = 3
        chassis_marker.type = Marker.MESH_RESOURCE
        chassis_marker.action = Marker.ADD
        chassis_marker.color = ColorRGBA(r=0.0,g=0.148,b=0.340, a=1.0)
        chassis_marker.pose = Pose(position=Point(x=0.0, y=0.0, z=-0.0), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        # Pose(position=Point(x=0.385, y=-0.120, z=-0.085), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        chassis_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        chassis_marker.mesh_resource = 'package://av1tenth_sim/models/simple_model.STL'
        chassis_marker.mesh_use_embedded_materials = False
        chassis_marker.frame_locked = True

        left_wheel_marker = Marker()
        left_wheel_marker.header.stamp = stamp
        left_wheel_marker.header.frame_id = 'vehicle_2/left_wheel'
        left_wheel_marker.id = 4
        left_wheel_marker.type = Marker.MESH_RESOURCE
        left_wheel_marker.action = Marker.ADD
        left_wheel_marker.color = ColorRGBA(r=0.0,g=0.148,b=0.340, a=1.0)
        left_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        left_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        left_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        left_wheel_marker.mesh_use_embedded_materials = False
        left_wheel_marker.frame_locked = True
        
        right_wheel_marker = Marker()
        right_wheel_marker.header.stamp = stamp
        right_wheel_marker.header.frame_id = 'vehicle_2/right_wheel'
        right_wheel_marker.id = 5
        right_wheel_marker.type = Marker.MESH_RESOURCE
        right_wheel_marker.action = Marker.ADD
        right_wheel_marker.color = ColorRGBA(r=0.0,g=0.148,b=0.340, a=1.0)
        right_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        right_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        right_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        right_wheel_marker.mesh_use_embedded_materials = False
        right_wheel_marker.frame_locked = True

        self.vehicle_2_chassis_pub.publish(chassis_marker)
        self.vehicle_2_left_wheel_pub.publish(left_wheel_marker)
        self.vehicle_2_right_wheel_pub.publish(right_wheel_marker)

        # ===== Vehicle 3 =====
        stamp = self.get_clock().now().to_msg()
        chassis_marker = Marker()
        chassis_marker.header.stamp = stamp
        chassis_marker.header.frame_id = 'vehicle_3/chassis'
        chassis_marker.id = 6
        chassis_marker.type = Marker.MESH_RESOURCE
        chassis_marker.action = Marker.ADD
        chassis_marker.color = ColorRGBA(r=0.133,g=0.531,b=0.297, a=1.0)
        chassis_marker.pose = Pose(position=Point(x=0.0, y=0.0, z=-0.0), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        # Pose(position=Point(x=0.385, y=-0.120, z=-0.085), orientation=Quaternion(w=0.0, x=0.0, y=0.7072068, z=0.7071068))
        chassis_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        chassis_marker.mesh_resource = 'package://av1tenth_sim/models/simple_model.STL'
        chassis_marker.mesh_use_embedded_materials = False
        chassis_marker.frame_locked = True

        left_wheel_marker = Marker()
        left_wheel_marker.header.stamp = stamp
        left_wheel_marker.header.frame_id = 'vehicle_3/left_wheel'
        left_wheel_marker.id = 7
        left_wheel_marker.type = Marker.MESH_RESOURCE
        left_wheel_marker.action = Marker.ADD
        left_wheel_marker.color = ColorRGBA(r=0.133,g=0.531,b=0.297, a=1.0)
        left_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        left_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        left_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        left_wheel_marker.mesh_use_embedded_materials = False
        left_wheel_marker.frame_locked = True
        
        right_wheel_marker = Marker()
        right_wheel_marker.header.stamp = stamp
        right_wheel_marker.header.frame_id = 'vehicle_3/right_wheel'
        right_wheel_marker.id = 8
        right_wheel_marker.type = Marker.MESH_RESOURCE
        right_wheel_marker.action = Marker.ADD
        right_wheel_marker.color = ColorRGBA(r=0.133,g=0.531,b=0.297, a=1.0)
        right_wheel_marker.pose = Pose(position=Point(x=-0.060, y=-0.020, z=0.060), orientation=Quaternion(w=0.7072068, x=-0.7072068, y=0.0, z=0.0))
        right_wheel_marker.scale = Vector3(x=0.001, y=0.001, z=0.001)
        right_wheel_marker.mesh_resource = 'package://av1tenth_sim/models/simple_wheel.STL'
        right_wheel_marker.mesh_use_embedded_materials = False
        right_wheel_marker.frame_locked = True

        self.vehicle_3_chassis_pub.publish(chassis_marker)
        self.vehicle_3_left_wheel_pub.publish(left_wheel_marker)
        self.vehicle_3_right_wheel_pub.publish(right_wheel_marker)

        # self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = SimVisualizer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()