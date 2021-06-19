# Create Keybase
mkdir ~/aws_model_keystore
cd ~/aws_model_keystore
ros2 security create_keystore demo_keys


ros2 security create_key demo_keys /pub_sub/talker
ros2 security create_key demo_keys /pub_sub/robot_1
ros2 security create_key demo_keys /pub_sub/robot_2
ros2 security create_key demo_keys /pub_sub/robot_3

# export section
export ROS_SECURITY_KEYSTORE=~/aws_model_keystore/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# create policy
ros2 security create_permission demo_keys /pub_sub/talker policies/sample.policy.xml
ros2 security create_permission demo_keys /pub_sub/robot_1 policies/sample.policy.xml
ros2 security create_permission demo_keys /pub_sub/robot_2 policies/sample.policy.xml
ros2 security create_permission demo_keys /pub_sub/robot_3 policies/sample.policy.xml

# Policy Files
Sample.policy.xml
```
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0"
  xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <xi:include href="talker.policy.xml"
      xpointer="xpointer(/policy/enclaves/*)"/>
    <xi:include href="add_two_ints.policy.xml"
      xpointer="xpointer(/policy/enclaves/*)"/>
    <xi:include href="minimal_action.policy.xml"
      xpointer="xpointer(/policy/enclaves/*)"/>
    <enclave path="/sample_policy/admin">
      <profiles>
        <profile ns="/" node="admin">
          <xi:include href="common/node.xml"
            xpointer="xpointer(/profile/*)"/>
          <actions call="ALLOW" execute="ALLOW">
            <action>fibonacci</action>
          </actions>
          <services reply="ALLOW" request="ALLOW">
            <service>add_two_ints</service>
          </services>
          <topics publish="ALLOW" subscribe="ALLOW">
            <topic>chatter</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```

talker.policy.xml
```
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0"
  xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <enclave path="/pub_sub/talker">
      <profiles>
        <profile ns="/" node="talker">
          <xi:include href="common/node.xml"
            xpointer="xpointer(/profile/*)"/>
          <topics publish="ALLOW" >
            <topic>map</topic>
            <topic>task1</topic>
            <topic>task2</topic>
            <topic>task3</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
    <enclave path="/pub_sub/robot_1">
      <profiles>
        <profile ns="/" node="robot1">
          <xi:include href="common/node.xml"
            xpointer="xpointer(/profile/*)"/>
          <topics subscribe="ALLOW" >
            <topic>map</topic>
            <topic>task1</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
    <enclave path="/pub_sub/robot_2">
      <profiles>
        <profile ns="/" node="robot2">
          <xi:include href="common/node.xml"
            xpointer="xpointer(/profile/*)"/>
          <topics subscribe="ALLOW" >
            <topic>map</topic>
            <topic>task2</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
    <enclave path="/pub_sub/robot_3">
      <profiles>
        <profile ns="/" node="robot3">
          <xi:include href="common/node.xml"
            xpointer="xpointer(/profile/*)"/>
          <topics subscribe="ALLOW" >
            <topic>map</topic>
            <topic>task3</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```
Subscriber:
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('robot1')
        self.subscription_map = self.create_subscription(
            String,
            'map',
            self.listener_callback,
            10)
        self.subscription_map  # prevent unused variable warning
        self.subscription_task = self.create_subscription(
            String,
            'task1',
            self.listener_callback,
            10)
        self.subscription_task  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Publisher:
```
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


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_map = self.create_publisher(String, 'map', 10)
        self.publisher_t1 = self.create_publisher(String, 'task1', 10)
        self.publisher_t2 = self.create_publisher(String, 'task2', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Map data: %d' % self.i
        msg1 = String()
        msg1.data = 'Task1 data: %d' %self.i
        msg2 = String()
        msg2.data = 'Task2 data: %d' %self.i
        self.publisher_map.publish(msg)
        self.publisher_t1.publish(msg1)
        self.publisher_t2.publish(msg2)
        self.get_logger().info('Publishing messages to all topics')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
