
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import  math, time, random, string

import os
import psutil


class testNode(Node):

    def __init__(self, nodename):
        super().__init__(nodename)
        
        number_of_topics = 1
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.frequency = 10
        self.timer_period = 1.0/self.frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.S = [('keyfiles/0_public.pem', 5),('keyfiles/1_public.pem', 6)]
        
        # test_related
        self.total_time = 0.0
        self.sent = 0.0
        self.received = 0.0
        self.false = 0.0
        self.msg = String()
        self.avg_mem = 0.0
        self.avg_cpu = 0.0
        self.data_point_num = 0
        
        # performance_related
        name = 'ROS2_perf_test'
        for proc in psutil.process_iter():
            if name in proc.name():
                self.python_process = proc

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          String, 
          'topic', 
          self.listener_callback, 
          100)
        self.subscription # prevent unused variable warning

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.received += 1
        self.total_time += time.time() - self.send_time
        if data.data != self.msg.data:
            print(data.data, 'a', self.msg.data)
            self.false += 1
        if self.sent %100 == 0:
            self.data_point_num += 1.0
            #print('time', self.total_time / float(self.sent), 'rate', self.received/self.sent, 'missed', self.false)
            cpu_usage = self.python_process.cpu_percent()
            #print('cpu usage', cpu_usage)
            self.avg_cpu += cpu_usage
            memoryUse = self.python_process.memory_info()[0]/2.**20  
            #print('memory use:', memoryUse)
            self.avg_mem += memoryUse
       
        if self.sent > self.frequency * 60:
            print(self.sent, self.frequency)
            print("Final Result for frequency %s Hz " %(str(self.frequency)))
            print("-------------------------------------------------------------")
            print('Time', self.total_time / float(self.sent), 'rate', self.received/self.sent, 'missed', self.false)
            print('Avg cpu', self.avg_cpu / self.data_point_num, 'avg mem', self.avg_mem / self.data_point_num)
            self.frequency += 10
            if self.frequency >100:
                self.destroy_node()
            self.timer_period = 1.0/self.frequency
            
            # re-initialize the variables
            self.total_time = 0.0
            self.sent = 0.0
            self.received = 0.0
            self.false = 0.0
            self.avg_mem = 0.0
            self.avg_cpu = 0.0
            self.data_point_num = 0
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            print('reset')
             
        
        
            
    def get_random_string(self, length):
        # choose from all lowercase letter
        letters = string.ascii_lowercase
        result_str = ''.join(random.choice(letters) for i in range(length))
        return result_str

        
        
    def timer_callback(self):
        self.msg.data = self.get_random_string(1024)
        self.publisher_.publish(self.msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.sent += 1
        self.send_time = time.time()


def main(args=None):
    rclpy.init(args=args)

    test = testNode('node1')

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    testNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
