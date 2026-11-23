#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_event, qos_actions
from amsagv_msgs.msg import TagsStamped, ActionsStamped, TagStamped, ActionStamped
from amsagv.planning import Planning
from itertools import tee



def main():
    def handle_start_goal(msg):
        try:
            path = planning.find_path(msg.tags[0].tag.id, msg.tags[-1].tag.id)

            msg_path_tags = TagsStamped()
            for tag in path:
                path_tag = TagStamped()
                path_tag.tag.id = int(tag)
                msg_path_tags.tags.append(path_tag)
            pub_path_tags.publish(msg_path_tags)

            actions = planning.generate_actions(path)
            msg_path_actions = ActionsStamped()
            for action in actions:
                path_action = ActionStamped()
                path_action.action.name = str(action[0])
                path_action.action.id = int(action[1])
                path_action.action.distance = float(action[2])
                msg_path_actions.actions.append(path_action)
            pub_path_actions.publish(msg_path_actions)
        except AttributeError:
            print('Error! Something went wrong :(')



    rclpy.init()
    node = Node('path_planning')
    
    try:
        planning = Planning()

        # Publisher of the path, which is given as a list of tag IDs
        pub_path_tags = node.create_publisher(TagsStamped, 'path_tags', qos_actions)
        # Publisher of the actions
        pub_path_actions = node.create_publisher(ActionsStamped, 'path_actions', qos_actions)
        # Subscriber to the start and goal tag ID
        sub_start_goal = node.create_subscription(TagsStamped, 'start_goal_tag', handle_start_goal, qos_event)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
