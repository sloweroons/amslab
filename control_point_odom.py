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
    K_gamma = 4
    K_omega = 4
    K_vel = 0.5

    phi_ref = -math.pi
    x_ref = 0.5
    y_ref = 0.5

    d = 0.127
    r = 0.20

    d_switch = 0.05
    g_tolerance = 0.05
    v_max = 0.2
    stage = 1

    x_t = x_ref - r * math.cos(phi_ref)
    y_t = y_ref - r * math.sin(phi_ref)

    # Handle odometry
    def handle_odometry(msg):
        x, y, phi = msg_to_pose(msg.pose.pose)
        gamma = msg.pose.pose.position.z
        nonlocal K_gamma, phi_ref, d, v_max, K_omega, K_vel, x_ref, y_ref, stage, d_switch, g_tolerance, x_t, y_t

        # Intermediate point
        d_ip = math.sqrt((x_t - x)**2 + (y_t - y)**2)

        if  stage == 1 and d_ip < d_switch:
            stage = 2

        if stage == 1:
            x_target = x_t
            y_target = y_t
        else:
            x_target = x_ref
            y_target = y_ref

        # Position errors
        ex = x_target - x
        ey = y_target - y
        e_dist = math.sqrt(ex * ex + ey * ey)

        # Desired orientation
        phi_target = math.atan2(ey, ex)

        # Heading error
        e_phi = wrap_to_pi(phi_target - phi)

        # Control to point - desired robot body velocities
        v =  K_vel * e_dist
        w = K_omega * e_phi

        # Kinematic trnasformation
        gamma_ref = math.atan2(w * d, v)
        e_gamma = wrap_to_pi(gamma_ref - gamma)
        vs = math.sqrt((w*d)**2 + v**2)

        phi_dot = (vs / d) * math.sin(e_gamma)

        ws = phi_dot + K_gamma * e_gamma

        vs = min(vs, v_max)

        # Stop
        if stage == 2 and e_dist < g_tolerance:
            vs = 0.0
            ws = 0.0

        # Final orientation error
        ephi_final = wrap_to_pi(phi_ref - phi)

        print("stage: ", stage, "edist: ", e_dist, "ephi_final: ", ephi_final)

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
