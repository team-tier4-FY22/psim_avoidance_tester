#!/usr/bin/env python3

import os
import yaml
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
        pkg_share_path = get_package_share_directory('psim_avoidance_tester')
        yaml_path = os.path.join(pkg_share_path, "param/dummy_objects.yaml")
        with open(yaml_path) as yaml_path_fp:
            yaml_info = yaml.safe_load(yaml_path_fp)
            pedestrians = yaml_info["pedestrians"]
            self.pedestrians = dict() # map ["name"] => [x, y, z]
            for i in range(len(pedestrians)):
                pedestrian_name = list(pedestrians[i].keys())[0]
                position = pedestrians[i][pedestrian_name]["position"]
                self.pedestrians[pedestrian_name] = []
                self.pedestrians[pedestrian_name].append(position['x'])
                self.pedestrians[pedestrian_name].append(position['y'])
                self.pedestrians[pedestrian_name].append(position['z'])

        self.get_logger().info(f"Loaded {yaml_path}")

        self.msgs = []
        for (k, v) in self.pedestrians.items():
            self.msgs.append(self.gen_dummy_msg(v[0], v[1], v[2]))

        # create pub
        self.pub = self.create_publisher(Object, '/simulation/dummy_perception_publisher/object_info', 1)
        # pub at 0.1Hz
        self.timer = self.create_timer(10, self.timer_cb)

    def timer_cb(self):
        for msg in self.msgs:
            self.pub.publish(msg)
        self.get_logger().info("Published dummy msg")

    def gen_dummy_msg(self, x, y, z):
        msg = Object()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # pose
        pose = msg.initial_state.pose_covariance.pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        # classification
        msg.classification.label = 7
        msg.classification.probability = 1.0
        # shape
        msg.shape.type = 1
        msg.shape.footprint.points = []
        msg.shape.dimensions.x = 0.6
        msg.shape.dimensions.y = 0.6
        msg.shape.dimensions.z = 2.0

        return msg

def main(args=None):
    rclpy.init(args=args)
    p = DummyPublisher()
    rclpy.spin(p)
    p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
