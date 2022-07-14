#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# importing necessary libraries
import rclpy
from rclpy.node import Node
# now this package is depend on example_interfaces also, it has to be written to 'package.xml'
from example_interfaces.msg import String  # importing message type of interface
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

class servo_subscriber_node(Node):
    def __init__(self):
        super().__init__("subscriber_servo")

        # inputs are message_type, topic name, callback function, qos profile {buffer size}
        self.subscriber_ = self.create_subscription(
            String, 'servo_angle', self.callback,100)  # creating subscriber on robot_news topic

        # assigning the logger to inform that subscriber is started
        self.get_logger().info('Subscriber node for steering has been started.')
        
    def callback(self, msg):
        """

        This function subscribe to servo_angle topic to listen every published message 

        Parameters
        ----------
        msg : type of message
            message sent to servo_angle topic.

        """
        self.get_logger().info(msg.data)  # creating logger when message is captured
        ser.write(str.encode(msg.data))

def main(args=None):
    rclpy.init(args=args) # initiliazing ros python library
    node = servo_subscriber_node() # initiliazing node object
    rclpy.spin(node) # spinning on node object until program is closed
    rclpy.shutdown() # shutting down ros / killing all nodes when program is closed

if __name__ == "__main__":
    main()
