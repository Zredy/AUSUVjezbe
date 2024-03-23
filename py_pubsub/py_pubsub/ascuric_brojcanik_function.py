# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16


class BrojcanikNode(Node):

    def __init__(self):
        super().__init__('brojcanik_node')
        self.publisher_ = self.create_publisher(Int16, 'broj', 10)
        timer_period = 1 # 1Hz potrebno, 1 s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int16()
        msg.data = self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    brojcanik_node = BrojcanikNode()

    rclpy.spin(brojcanik_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    brojcanik_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
