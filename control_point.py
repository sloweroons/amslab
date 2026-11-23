#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from amsagv.ams import msg_to_pose



def main():
    # Handle odometry
    def handle_odometry(msg):
        try:
            trans = tf_buffer.lookup_transform('world', 'agv', Time())
            x, y, phi = msg_to_pose(trans)
        except tf2_ros.TransformException:
            x, y, phi = None, None, None
        gamma = msg.pose.pose.position.z
        print(x, y, phi, gamma)

        vs, ws = 0.0, 0.0
    
        # Velocity commands message
        msg_cmd_vel.linear.x = vs
        msg_cmd_vel.angular.z = ws
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)



    rclpy.init()
    node = Node('control_point')
    
    try:
        # TF buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        # Velocity commands message
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
