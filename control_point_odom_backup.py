#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv.ams import msg_to_pose, wrap_to_pi
import math as math


def main():

    gamma_ref = math.pi / 2 
    K_gamma = 2
    K_omega = 2
    K_vel = 0.5
    x_ref = -0.2
    y_ref = 0.0
    d = 0.127 
    v_max = 0.2

    # Handle odometry
    def handle_odometry(msg):
        x, y, phi = msg_to_pose(msg.pose.pose)
        gamma = msg.pose.pose.position.z
        nonlocal K_gamma, gamma_ref, d, v_max, K_omega, K_vel, x_ref, y_ref

        vs, ws = 0.0, 0.0

        ex = x_ref - x
        ey = y_ref - y
        phi_ref = math.atan2(ey, ex)
        e_phi = wrap_to_pi(phi_ref - phi)
        e_dist = math.sqrt(ex * ex + ey * ey)

        print(e_dist, x, y, phi, gamma)

        if e_dist < 0.05:
            vs, ws = 0.0, 0.0
            msg_cmd_vel.linear.x = vs
            msg_cmd_vel.angular.z = ws
            pub_cmd_vel.publish(msg_cmd_vel)
            return

        # Control to point
        v =  K_vel * e_dist
        w = K_omega * e_phi

        # Kinematic trnasformation
        gamma_ref = math.atan2(w * d, v)
        e_gamma = wrap_to_pi(gamma_ref - gamma)
        vs = math.sqrt((w*d)**2 + v**2)

        ws = K_gamma * (e_gamma)

        vs = min(vs, v_max)

        # Velocity commands message
        msg_cmd_vel.linear.x = vs
        msg_cmd_vel.angular.z = ws
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)



    rclpy.init()
    node = Node('control_point_odom')
    
    try:
        msg_cmd_vel = Twist()

        # Velocity commands publisher.
        pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos_cmd)
        # Odometry subscriber
        sub_odom = node.create_subscription(Odometry, 'odom', handle_odometry, qos_sensor)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
