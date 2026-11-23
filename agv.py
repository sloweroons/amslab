#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
from amsagv.ams import pose_to_msg, get_frame_prefix
from amsagv.agvapi import Agv, find_line_edges

def main():
    prev_enc_left = prev_enc_right = prev_enc_heading = 0.0
    d = 0.127  # Distance between wheels
    sample_time = 1 / 50.0
    normalize = 110600 * sample_time  # Normalization factor for encoders
    normalize_angle = (2.0 * math.pi) / pow(2.0, 13)
    angle_offset = 2.35  # Offset for the heading angle
    phi = 0.0
    
    with Agv() as robot:
        # Handle velocity commands
        def handle_cmd_vel(msg):
            robot.set_vel(msg.linear.x, msg.angular.z)

        # Main loop, executed at 50 Hz
        def handle_timer():
            nonlocal x, y, phi, gamma, fd
            nonlocal prev_enc_left, prev_enc_right, prev_enc_heading

            t = node.get_clock().now().to_msg()

            # Read sensors
            robot.read_sensors()

            #
            # Odometry
            #

            # Encoders

            enc_left, enc_right, enc_heading = robot.get_encoders()

            # TODO Implement odometry here ...
            # print('Encoders: left={}, right={}, heading={}'.format(enc_left, enc_right, enc_heading))

            # TODO: normalize everything;
            vl = (prev_enc_left - enc_left) / normalize
            vr = (enc_right - prev_enc_right) / normalize
            gamma = -1 * (enc_heading * normalize_angle - angle_offset)
            vs = (vr + vl) / 2.0 
            dphi = vs / d * math.sin(gamma) * sample_time 
            x += vs * math.cos(gamma) * math.cos(phi) * sample_time
            y += vs * math.cos(gamma) * math.sin(phi) * sample_time
            phi += dphi

            prev_enc_left, prev_enc_right, prev_enc_heading = enc_left, enc_right, enc_heading
            print('vs={:.2f}[cm/s] ({:.2f} | {:.2f})\tphi={:.2f}[rad] ({:.2f})\tgamma={:.2f}[rad]'.format(vs, vl, vr, phi, dphi, gamma))
            print(f'{x=}, {y=}, {phi=}, {gamma=}')

            # Odometry message
            msg_odom.header.stamp = t
            msg_odom.pose.pose = pose_to_msg(x, y, phi)
            msg_odom.pose.pose.position.z = gamma
            # Publish odometry message
            pub_odom.publish(msg_odom)

            #
            # Line sensor
            #

            # Line-sensor values
            line_values = robot.get_line_values()
            # Left and right line edge
            edge_left, edge_right = find_line_edges(line_values)

            # Line-sensor message
            msg_line.header.stamp = t
            msg_line.line.values = line_values
            msg_line.line.left = float(edge_left) if edge_left is not None else float('nan')
            msg_line.line.right = float(edge_right) if edge_right is not None else float('nan')
            msg_line.line.heading = gamma
            msg_line.line.distance = fd
            # Publish line-sensor message
            pub_line.publish(msg_line)



        rclpy.init()
        node = Node('agv')
        
        try:
            prefix = get_frame_prefix(node)
            # Name of the odometry frame
            param_odom_frame_id = node.declare_parameter('odom_frame_id', f'{prefix}odom').value
            # Name of the AGV frame
            param_agv_frame_id = node.declare_parameter('agv_frame_id', f'{prefix}agv').value

            # Odometry publisher
            pub_odom = node.create_publisher(Odometry, 'odom', qos_sensor)
            # Line sensor publisher
            pub_line = node.create_publisher(LineStamped, 'line', qos_sensor)
            # Velocity commands subscriber.
            sub_cmd_vel = node.create_subscription(Twist, 'cmd_vel', handle_cmd_vel, qos_cmd)

            # Line-sensor message
            msg_line = LineStamped()

            # Odometry message
            msg_odom = Odometry()
            msg_odom.header.frame_id = param_odom_frame_id
            msg_odom.child_frame_id = param_agv_frame_id

            # Odometry initial state
            x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
            fd = 0.0 # Travelled distance of the front cart
            
            robot.read_sensors()

            prev_enc_left, prev_enc_right, prev_enc_heading = robot.get_encoders()
            

            timer = node.create_timer(1.0/50.0, handle_timer)

            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()



if __name__ == '__main__':
    main()
