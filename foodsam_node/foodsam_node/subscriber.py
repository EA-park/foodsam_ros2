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

#!/usr/bin/python

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from foodsam_msgs.msg import BBoxList as BBoxMsgList

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            BBoxMsgList,
            'detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Print the received msgs in the cmd
        """
        for m in msg.bboxes:
            if m.class_label.data == 'background':
                continue

            string_padding_ = '..' if len(m.class_label.data) > 7 else ''
            print(f"bbox id: {m.box_id.data}\tname: {m.class_label.data[:7]}{string_padding_}\tx0: {m.x0}\ty0: {m.y0}\tw: {m.w}\th: {m.h}")
        print()

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
