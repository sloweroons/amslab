#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_sensor, get_frame_prefix, phi_to_quaternion_msg
import tf2_ros
from geometry_msgs.msg import TransformStamped
from amsagv_msgs.msg import LineStamped



def main():
    # Handle line sensor
    def handle_line(msg):
        trans.header.stamp = node.get_clock().now().to_msg()
        trans.transform.rotation = phi_to_quaternion_msg(msg.line.heading)
        tf_broadcaster.sendTransform(trans)



    rclpy.init()
    node = Node('front')
    
    try:
        prefix = get_frame_prefix(node)

        trans = TransformStamped()
        trans.header.frame_id = f'{prefix}front'
        trans.child_frame_id = f'{prefix}cart'

        tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        # Line sensor subscriber
        sub_line = node.create_subscription(LineStamped, 'line', handle_line, qos_sensor)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
