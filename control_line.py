#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor, qos_event
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from amsagv.agvapi import Agv, find_line_edges
from amsagv.world import MTAG



def main():
    tag = None

    vmax = 0.3
    vcmd = 0.1
    err = 0.0
    Kp = 2.0
    offset = 0.43135

    # Handle line sensor
    def handle_line(msg):
        # print(msg.line.left, msg.line.right)

        nonlocal vmax, err, Kp, vcmd
        v, w = 0.0, 0.0            

        if isnan(msg.line.right):
            return

        v = vcmd
        err = msg.line.right + offset
        w = Kp * err

        print('(v = {:.4f}, w = {:.4f} | err = {:.4f} - {:.4f} {:.4f})'.format(v, w, err, msg.line.left, msg.line.right))
        
        # Velocity commands message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = v
        msg_cmd_vel.angular.z = w
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)


    # Handle tag message
    def handle_tag(msg):
        nonlocal tag
        tag = MTAG.get(msg.tag.id, None)
        print(f'New tag: {msg.tag.id} -> {tag}')



    rclpy.init()
    node = Node('control_line')
    
    try:
        # Velocity commands publisher.
        pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos_cmd)
        # Line sensor subscriber
        sub_line = node.create_subscription(LineStamped, 'line', handle_line, qos_sensor)
        # Tag subscriber
        sub_tag = node.create_subscription(TagStamped, 'tag', handle_tag, qos_event)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
