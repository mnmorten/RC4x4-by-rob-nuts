#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# importing necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from scipy.interpolate import interp1d
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
from PIL import Image as im


class Camera_Publisher_Node(Node):

    def __init__(self):
        super().__init__("publisher")
        
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()


        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                self.found_rgb = True
                break
        if not self.found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        self.steering = 320
        self.vel_esc = 1600
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # inputs are message_type, topic name, qos profile {buffer size}
        self.publisher_steering = self.create_publisher(
            String, 'servo_angle', 100) 
        
        self.publisher_esc = self.create_publisher(
            String, 'ESC_pwm', 100) 

        # publishing data at 1 Hz
        self.timer_ =self.create_timer(1, self.publish_servo_angle) 
        
        # assigning the logger to inform that publisher is started
        self.get_logger().info('Publisher has been started')

    def publish_servo_angle(self):

        # Get frameset of color and depth
        self.frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        self.aligned_frames = self.align.process(self.frames)

        # Get aligned frames
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        self.color_frame = self.aligned_frames.get_color_frame()

        # Validate that both frames are valid
        
        if not self.aligned_depth_frame or not self.color_frame:
            rclpy.spin(Camera_Publisher_Node())
        
        self.depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
       
        # Remove background - Set pixels further than clipping_distance to grey

        self.black_color = 0
        self.orange_color_low = np.array([50,90,180])
        self.orange_color_high = np.array([70,115,235])

        self.depth_image_3d = np.dstack((self.depth_image,self.depth_image,self.depth_image)) #depth image is 1 channel, color is 3 channels
        self.bg_removed = np.where((self.depth_image_3d > self.clipping_distance) | (self.depth_image_3d <= 0), self.black_color, self.color_image)
        
        self.bg_removed_red = np.where(self.bg_removed[:,:,2] >= self.orange_color_low[2], self.bg_removed[:,:,2], self.black_color)
        self.bg_removed_green = np.where((self.bg_removed[:,:,1] >= self.orange_color_low[1]) & (self.bg_removed[:,:,1] <= self.orange_color_high[1]), self.bg_removed_red, self.black_color)
        self.bg_removed_blue = np.where((self.bg_removed[:,:,0] >= self.orange_color_low[0]) & (self.bg_removed[:,:,0] <= self.orange_color_high[0]), self.bg_removed_green, self.black_color)
               
        self.count_nonblack = np.count_nonzero(self.bg_removed_blue)
        if self.count_nonblack <= 10:
            self.vel_esc = 1600
            pass
        else:
            self.non_black_mean = np.sum(np.nonzero(self.bg_removed_blue)[1])
            self.steering_ = self.non_black_mean / self.count_nonblack
            
            #self.normed_arr = interp1d([0,640],[130,55])
            self.steering = int(self.steering_)
            self.vel_esc = 1680

        print(self.vel_esc)
        print(self.steering)
        print(self.count_nonblack)
        # Render images:
        #   depth align to color on left
        #   depth on right
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', self.bg_removed_blue)
        self.key = cv2.waitKey(1)

        if self.key & 0xFF == ord('q') or self.key == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()
        
        self.steering = 's' + str(self.normed_result) + 'f'
        msg = String()  # creating the message variable
        msg.data = str(self.steering)
        self.publisher_steering.publish(msg)  # publishing message

        esc = 'e' + str(self.vel_esc) + 'f'
        msg = String()  # creating the message variable
        msg.data = str(esc)
        self.publisher_esc.publish(msg)  # publishing message





def main(args = None):
    rclpy.init(args=args)
    node=Camera_Publisher_Node()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
