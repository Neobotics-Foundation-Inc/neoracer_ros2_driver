#!/usr/local/bin/python3

"""
camera.py

Lightweight ROS2 node that subscribes to a USB camera object and
streams data to the /camera topic.

This script has the following dependencies:
- opencv-python==4.8.1.78
- numpy==1.26.2
"""

import rclpy
from sensor_msgs.msg import Image 
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np 
import time

def main(args=None):

    # ===== INITIALIZE ROS =====
    rclpy.init(args=args)
    node = rclpy.create_node('camera_node') # create node
    pub_cam = node.create_publisher(Image, '/camera', qos_profile_sensor_data) # create publisher
    print("[DEBUG] Camera topic set up!")

    # ===== SETUP CAMERA, BREAK IF NOT FOUND =====
    final_cam_id = None 
    final_cap = None
    for curr_cam_id in range(10): # loops through the /dev/video* dir
        start_time = time.time() # start a timer

        while time.time() - start_time < 3: # for every 3 seconds
            curr_cap = cv2.VideoCapture(curr_cam_id) # try to access camera
            ret, frame = curr_cap.read() # try to read a frame

            # If unable to find camera, break and try next ID
            if not ret:
                node.get_logger().warn(f"[WARN] Webcam at /dev/video{curr_cam_id} not found, trying next ID...")
                curr_cap.release()
                break

            # If camera found, set final ID and capture object
            node.get_logger().info(f"[INFO] Webcam found at /dev/video{curr_cam_id}!")
            final_cam_id = curr_cam_id
            final_cap = curr_cap
            break

        # Break the outer loop if camera is found
        if final_cam_id is not None:
            break
    
    # If no camera found, exit the program
    if final_cam_id is None:
        node.get_logger().error("[ERROR] No webcam found! Exiting...")
        rclpy.shutdown()
        return

    # ===== STREAM CAMERA FRAMES =====
    node.get_logger().info(f"[INFO] Streaming frames from /dev/video{final_cam_id}...")
    try:
        while rclpy.ok():
            ret, frame = final_cap.read() # read a frame from the camera

            if not ret:
                node.get_logger().warn("[WARN] Unable to read frame from webcam! Is the camera unplugged?", throttle_duration_sec=1.0)
                continue

            # Compress image into jpeg format
            _, frame_jpeg = cv2.imencode('.jpg', frame)

            # Convert the frame to a ROS2 Image message
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'jpeg'
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = np.array(frame_jpeg).tobytes()

            # Publish the Image message to the /camera topic, loop as fast as possible
            pub_cam.publish(msg)

    except KeyboardInterrupt as e:
        final_cap.release()
        node.get_logger().info("[INFO] Camera node stopped by user.")
        rclpy.shutdown()
        return
    
if __name__ == '__main__':
    main()