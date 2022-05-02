#!/usr/bin/env python3

import os
import yaml
import argparse
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import rclpy.node
from rclpy.time import Time
from dummy_perception_publisher.msg import Object
from dummy_perception_publisher.msg import InitialState
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import Shape

class DummyPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("DummyPublisher")

        # load yaml
        parser = argparse.ArgumentParser()
        parser.add_argument('path')
        args = parser.parse_args()

        # pkg_share_path = get_package_share_directory('psim_avoidance_tester')
        yaml_path = args.path
        with open(yaml_path) as yaml_path_fp:
            yaml_info = yaml.safe_load(yaml_path_fp)
            """
            pedestrians = yaml_info["pedestrians"]
            self.pedestrians = dict() # map ["name"] => [x, y, z]
            for i in range(len(pedestrians)):
                pedestrian_name = list(pedestrians[i].keys())[0]
                position = pedestrians[i][pedestrian_name]["position"]
                self.pedestrians[pedestrian_name] = []
                self.pedestrians[pedestrian_name].append(position['x'])
                self.pedestrians[pedestrian_name].append(position['y'])
                self.pedestrians[pedestrian_name].append(position['z'])
            """
            cars = yaml_info["cars"]
            self.cars = dict() # map ["name"] => [[x, y, z], [x, y, z, w]]
            for i in range(len(cars)):
                car_name = list(cars[i].keys())[0]
                position = cars[i][car_name]["position"]
                orientation = cars[i][car_name]["orientation"]
                self.cars[car_name] = [[], []]
                self.cars[car_name][0].append(position['x'])
                self.cars[car_name][0].append(position['y'])
                self.cars[car_name][0].append(position['z'])
                self.cars[car_name][1].append(orientation['x'])
                self.cars[car_name][1].append(orientation['y'])
                self.cars[car_name][1].append(orientation['z'])
                self.cars[car_name][1].append(orientation['w'])

        self.get_logger().info(f"Loaded {yaml_path}")

        self.object_msgs = []
        """
        for (k, v) in self.pedestrians.items():
            self.object_msgs.append(self.gen_dummy_msg(v[0], v[1], v[2], 7, 1, [0.6, 0.6, 2.0]))
        """
        for (k, v) in self.cars.items():
            self.object_msgs.append(self.gen_dummy_msg(v[0], v[1], 1, 0, [4.0, 1.8, 2.0]))

        # create pub
        self.pub = self.create_publisher(Object, '/simulation/dummy_perception_publisher/object_info', 1)
        # pub at 0.1Hz
        self.timer = self.create_timer(10, self.timer_cb)

    def timer_cb(self):
        for msg in self.object_msgs:
            self.pub.publish(msg)
        self.get_logger().info("Published dummy msg")

    def gen_dummy_msg(self, position, orientation, label, box_type, shape):
        msg = Object()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # pose
        pose = msg.initial_state.pose_covariance.pose
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        # classification
        msg.classification.label = label
        msg.classification.probability = 1.0
        # shape
        msg.shape.type = box_type
        msg.shape.footprint.points = []
        msg.shape.dimensions.x = shape[0]
        msg.shape.dimensions.y = shape[1]
        msg.shape.dimensions.z = shape[2]

        return msg

def main(args=None):
    rclpy.init(args=args)
    p = DummyPublisher()
    rclpy.spin(p)
    p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
