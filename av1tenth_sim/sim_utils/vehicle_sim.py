from dataclasses import dataclass, field
from typing import List
import time # Will need time to implement signal delays
from math import sin, cos, tan, atan, isclose
from random import normalvariate

import numpy as np
import utm

from geometry_msgs.msg import Pose, Transform, Point, Quaternion, Vector3, Twist


@dataclass
class VehicleState:
    """ Assumes position is provided in Lat/Long. It is then converted to UTM coordinates. """
    position: List[float]
    heading: float # in radians
    velocity: List[float] = field(default_factory=list)
    steering_angle: float = 0.0 # in radians

    def __post_init__(self):
        # easting, northing, _, _ = utm.from_latlon(self.position[0], self.position[1])
        self.position.append(0.0)
        self.position = np.array(self.position)

        self.heading = np.radians(self.heading)
        if len(self.velocity) == 0:
            self.velocity = [0.0, 0.0, 0.0]

@dataclass
class VehicleSimParameters:
    """
        - steering_motion_delay: radians/s value that represents the turning speed of the wheels
        - steering_signal_delay: seconds value that represents the time delay between receiving
                                     a command and the start of motion
    """
    sim_time_step: float # in decimal seconds
    output_coordinates: str # either 'utm' or 'utm_local'

    track: float            # meters
    kingpin_width: float    # meters
    wheelbase: float        # meters

    steering_motion_delay: float    # radian/s
    steering_signal_delay: float    # seconds

    speed_delay: float # "accleration", m/s^2
    
    position_feedback_noise: float # Gaussian noise, need to define standard deviation
    velocity_feedback_noise: float # Gaussian noise, need to define standard deviation


class VehicleSim:
    """ 
        This class will contain all the math used to simulate motion for an Ackermann steered vehicle.

        Simulated Components:
            * Steering with motion delay
            * Speed with motion delay

    """
    def __init__(self, initial_state: VehicleState, params: VehicleSimParameters)-> None:
        self.state = initial_state
        self.params = params
        self.steering_angle_cmd = 0.0
        self.speed_cmd = 0.0
        self.speed = 0.0
    
    def utm_local_output(self, transform: Transform)-> None:
        """ Shift the working coordinates by the provided 'utm_local' coordinates. """
        self.state.position = self.state.position - np.array([transform.translation.x, transform.translation.y, 0.0])
    
    def utm_output(self, transform: Transform)-> None:
        """ Shift the working coordinates by the provided 'utm' coordinates. """
        self.state.position = self.state.position - np.array([transform.translation.x, transform.translation.y, 0.0])
    
    def update_state(self)-> None:
        """ 
            Call this function to update the state of the system using the commands given 
            with the "update_commands" function. This function should be called at a rate that matches the
            sim_time_step provided in the parameters.
            
        """

        # Delay changes in speed so they are not instantaneous
        error = self.speed_cmd - self.speed
        direction = np.sign(error)
        if not isclose(error, 0.0, abs_tol=0.005):
            self.speed = self.speed + direction * self.params.speed_delay * self.params.sim_time_step
        elif not isclose(error, 0.0, abs_tol=0.002):
            self.speed = self.speed + direction * 0.05 * self.params.speed_delay * self.params.sim_time_step
        
        # Update velocity direction based on heading
        self.state.velocity[0] = self.speed * cos(self.state.heading)
        self.state.velocity[1] = self.speed * sin(self.state.heading)

        # Simulate steering angle motion delay
        # There is an inherihant error in this method. I choose to keep it to give the simulator some noise
        error = self.steering_angle_cmd - self.state.steering_angle
        direction = np.sign(error)
        if not isclose(error, 0.0, abs_tol=0.005):
            self.state.steering_angle = self.state.steering_angle + direction * self.params.steering_motion_delay * self.params.sim_time_step
        elif not isclose(error, 0.0, abs_tol=0.002):
            self.state.steering_angle = self.state.steering_angle + direction * 0.05 * self.params.steering_motion_delay * self.params.sim_time_step

        # Update X, Y coordinates of the rear axle
        self.state.position[0] = self.state.position[0] + self.state.velocity[0] * self.params.sim_time_step
        self.state.position[1] = self.state.position[1] + self.state.velocity[1] * self.params.sim_time_step

        # Update heading after motion
        self.state.heading = self.state.heading + self.speed * tan(self.state.steering_angle) / self.params.wheelbase * self.params.sim_time_step

    def update_commands(self, steering_angle: float, speed: float):
        # Keep speed below 1.5 m/s
        self.speed_cmd = speed
        if (abs(self.speed_cmd) > 1.5):
            self.speed_cmd = np.sign(self.speed_cmd) * 1.5
        
        self.steering_angle_cmd = steering_angle
        if (abs(self.steering_angle_cmd) > np.pi/6.0):
            self.steering_angle_cmd = np.sign(self.steering_angle_cmd) * np.pi / 6.0
    
    # ---- Class Outputs -----
    def output_pose(self)-> Pose:
        """ Pack most recent state information into a Pose message. """
        pose = Pose()
        pose.position = Point(x=normalvariate(self.state.position[0], self.params.position_feedback_noise)
                              , y=normalvariate(self.state.position[1], self.params.position_feedback_noise)
                              , z=self.state.position[2])
        pose.orientation = Quaternion(x=0.0, y=0.0, z=sin(self.state.heading / 2.0), w=cos(self.state.heading / 2.0))
        return pose

    def output_transform(self)-> Transform:
        """ Pack most recent state information into a Transform message. """
        transform = Transform()
        transform.translation = Vector3(x=self.state.position[0], y=self.state.position[1], z=self.state.position[2])
        transform.rotation = Quaternion(x=0.0, y=0.0, z=sin(self.state.heading / 2.0), w=cos(self.state.heading / 2.0))
        return transform
    
    def output_left_wheel_transform(self)-> Transform:
        """ Returns the transform between the rear axle and front left wheel. """
        transform = Transform()
        transform.translation = Vector3(x=self.params.wheelbase, y=self.params.track/2.0)
        if abs(self.state.steering_angle) > 0.005:
            turningRadius = self.params.wheelbase / tan(self.state.steering_angle)
            delta_left_wheel = atan(self.params.wheelbase / (turningRadius - self.params.kingpin_width / 2.0))
        else: 
            delta_left_wheel = 0.0
        transform.rotation = Quaternion(x=0.0, y=0.0, z=sin(delta_left_wheel / 2.0), w=cos(delta_left_wheel / 2.0))
        return transform
    
    def output_right_wheel_transform(self)-> Transform:
        """ Returns the transform between the rear axle and front right wheel. """
        transform = Transform()
        transform.translation = Vector3(x=self.params.wheelbase, y=-self.params.track/2.0)
        if abs(self.state.steering_angle) > 0.005:
            turningRadius = self.params.wheelbase / tan(self.state.steering_angle)
            delta_right_wheel = atan(self.params.wheelbase / (turningRadius + self.params.kingpin_width / 2.0))
        else:
            delta_right_wheel = 0.0
        transform.rotation = Quaternion(x=0.0, y=0.0, z=sin(delta_right_wheel / 2.0), w=cos(delta_right_wheel / 2.0))
        return transform
    
    def output_twist(self)-> Twist:
        """ Pack most recent state information into a Twist message. """
        twist = Twist()
        twist.linear.x = normalvariate(self.speed, self.params.velocity_feedback_noise)
        twist.angular.z = self.speed * tan(self.state.steering_angle) / self.params.wheelbase
        return twist
