#!/usr/bin/env python3

import os
import yaml
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import rclpy.node

class DummyPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("DummyPublisher")
        pkg_share_path = get_package_share_directory('psim_avoidance_tester')
        yaml_path = os.path.join(pkg_share_path, "param/dummy_objects.yaml")
        yaml_path_fp = open(yaml_path)
        yaml_info = yaml.safe_load(yaml_path_fp)
        pedestrians = yaml_info["pedestrians"]
        self.pedestrians = dict() # map ["name"] => [x, y, z]
        n_pedestrians = len(pedestrians)
        for i in range(n_pedestrians):
            pedestrian_name = list(pedestrians[i].keys())[0]
            position = pedestrians[i][pedestrian_name]["position"]
            self.pedestrians[pedestrian_name] = []
            self.pedestrians[pedestrian_name].append(position['x'])
            self.pedestrians[pedestrian_name].append(position['y'])
            self.pedestrians[pedestrian_name].append(position['z'])
            
def main(args=None):
    rclpy.init(args=args)
    p = DummyPublisher()
    print(p.pedestrians)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
