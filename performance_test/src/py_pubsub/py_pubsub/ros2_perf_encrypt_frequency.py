import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import  math, time


from ecdsa import SigningKey, VerifyingKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import binascii
import os
#from M2Crypto import BIO, Rand, SMIME, X509
#from x509_modules import x509_encrypt, x509_decrypt
from .efficient_encryption import EfficientBroadcastEncryption
import random
import string
import hashlib
import pickle
import codecs

import os
import psutil



class testNode(Node):

    def __init__(self):
        super().__init__('testNode')
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.frequency = 10
        self.timer_period = 1.0/self.frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.EncryptionModel = EfficientBroadcastEncryption()
        
        # test_related
        self.total_time = 0.0
        self.sent = 0.0
        self.received = 0.0
        self.false = 0.0
        self.encryption_time = 0.0
        self.decryption_time = 0.0
        self.msg = String()
        self.avg_mem = 0.0
        self.avg_cpu = 0.0
        self.data_point_num = 0
        
        # performance_related
        name = 'ROS2_perf_test'
        for proc in psutil.process_iter():
            if name in proc.name():
                self.python_process = proc
        # hardcoded
        self.S = [('/home/gelei/SROS2_Broadcast_Encryption/performance_test/src/py_pubsub/py_pubsub/keyfiles/0_public.pem', 5),('/home/gelei/SROS2_Broadcast_Encryption/performance_test/src/py_pubsub/py_pubsub/keyfiles/1_public.pem', 6)] # test public key
        

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
        
        # decrypt
        before_decryption = time.time()
        result = self.EncryptionModel.decrypt(data.data)
        self.decryption_time += time.time() - before_decryption
        if result.decode("utf-8") != self.raw:
            self.false += 1
        if self.sent %100 == 0:
            self.data_point_num += 1.0
            #print('time', self.total_time / float(self.sent), 'rate', self.received/self.sent, 'missed', self.false/self.sent)
            #print('encryption time', self.encryption_time / float(self.sent), 'decryption time', self.decryption_time / float(self.sent))
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
            print('encryption time', self.encryption_time / float(self.sent), 'decryption time', self.decryption_time / float(self.sent))
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
            self.encryption_time = 0.0
            self.decryption_time = 0.0
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            print('reset')

    def get_random_string(self, length):
        # choose from all lowercase letter
        letters = string.ascii_lowercase
        result_str = ''.join(random.choice(letters) for i in range(length))
        return result_str

               
        
    def timer_callback(self):
        rawdata = self.get_random_string(1024)
        self.raw = rawdata
        before_encryption = time.time()
        ciphertext = self.EncryptionModel.encrypt(str.encode(rawdata), self.S)
        self.encryption_time += time.time() - before_encryption
        self.msg.data = ciphertext

        self.publisher_.publish(self.msg)
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
