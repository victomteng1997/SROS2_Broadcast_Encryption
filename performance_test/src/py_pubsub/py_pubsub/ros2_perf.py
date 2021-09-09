
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import  math, time, random, string


class testNode(Node):

    def __init__(self):
        super().__init__('testNode')
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.S = [('keyfiles/0_public.pem', 5),('keyfiles/1_public.pem', 6)]
        
        # test_related
        self.total_time = 0.0
        self.sent = 0.0
        self.received = 0.0
        self.false = 0.0
        self.msg = String()
        self.msg.data = 'Hello World'
        

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          String, 
          'topic', 
          self.listener_callback, 
          100)
        self.subscription # prevent unused variable warning

        # self.array = Int16MultiArray()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.received += 1
        self.total_time += time.time() - self.send_time
        if data.data != self.msg.data:
            self.false += 1
        if self.sent %100 == 0:
            print('time', self.total_time / float(self.sent), 'rate', self.received/self.sent, 'missed', self.false)
            
    def get_random_string(self, length):
        # choose from all lowercase letter
        letters = string.ascii_lowercase
        result_str = ''.join(random.choice(letters) for i in range(length))
        return result_str

        
        
    def timer_callback(self):
        msg = String()
        msg.data = self.get_random_string(1024)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.sent += 1
        self.send_time = time.time()


def main(args=None):
    rclpy.init(args=args)

    test = testNode()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    testNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
