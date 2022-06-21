#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
from threading import Thread

from sensor_msgs.msg import CompressedImage
from emotion_detection_ros.emotion_predictor import EmotionPredictor
from emotion_detection_ros_msgs.msg import BoundingBox
from emotion_detection_ros_msgs.msg import BoundingBoxes

def draw_fps(frame, fps):
    fps_text = 'FPS = {:.1f}'.format(fps)
    cv2.putText(frame, fps_text, (20, 24), cv2.FONT_HERSHEY_PLAIN,
                1, (0, 0, 255), 1)

def draw_bounding_box(frame, bounding_box, label):
    # Drawing bounding box
    cv2.rectangle(frame, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (255, 255, 0), 2)

    # Drawing label
    (w, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(frame, (bounding_box[0], bounding_box[1] - 20), (bounding_box[0] + w, bounding_box[1]), (255, 255, 0), -1)
    cv2.putText(frame, label, (bounding_box[0], bounding_box[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

class EmotionDetection:
    def __init__(self):
        # Read ROS parameters
        self.read_parameters()

        # Detector of emotions
        self.emotion_predictor = EmotionPredictor(
            model_algorithm=self.model_algorithm, 
            max_num_faces=self.model_max_num_faces
        )

        # Last frame received in raspicam_callback
        self.frame = np.zeros((410, 308, 3), np.uint8)

        # Variables to calculate fps
        self.counter = 0
        self.fps_avg_frame_count = 10
        self.fps = 0
        self.start_time = time.time()

        # Camera subscriber
        self.sub_camera = rospy.Subscriber(self.camera_topic,
            CompressedImage, self.raspicam_callback, queue_size = self.camera_queue_size)

        # Bounding boxes publisher
        self.pub_bounding_boxes = rospy.Publisher(self.bounding_boxes_topic,
            BoundingBoxes, queue_size = self.bounding_boxes_queue_size, latch = self.bounding_boxes_latch)

        # Thread to process frames
        self.t = Thread(target=self.process_frame).start()

    def read_parameters(self):
        # Model config
        self.model_algorithm = rospy.get_param('/model/algorithm')
        self.model_max_num_faces = rospy.get_param('/model/max_num_faces')

        # ROS config
        # Subscribers
        self.camera_topic = rospy.get_param('/subscribers/camera_reading/topic')
        self.camera_queue_size = rospy.get_param('/subscribers/camera_reading/queue_size')
        # Publishers
        self.bounding_boxes_topic = rospy.get_param('/publishers/bounding_boxes/topic')
        self.bounding_boxes_queue_size = rospy.get_param('/publishers/bounding_boxes/queue_size')
        self.bounding_boxes_latch = rospy.get_param('/publishers/bounding_boxes/latch')

        # Image view config
        self.image_view_enabled = rospy.get_param('/image_view/enable')
        self.image_view_wait_key_delay = rospy.get_param('/image_view/wait_key_delay')

    def raspicam_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        np_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.frame = cv2.flip(np_frame, 1)

    def update_fps_value(self):
        self.counter += 1
        if self.counter % self.fps_avg_frame_count == 0:
            end_time = time.time()
            self.fps = self.fps_avg_frame_count / (end_time - self.start_time)
            self.start_time = time.time()

    def process_frame(self):
        while not rospy.is_shutdown():
            # List of bounding box message
            bounding_boxes_msg = BoundingBoxes()

            # Making prediction in frame
            emotions = self.emotion_predictor.predict(self.frame)

            # For each emotion detected, bounding boxes are drawn 
            # in frame and published in ros topic
            for emotion in emotions:
                draw_bounding_box(self.frame, emotion.bounding_box, emotion.label)
                bounding_box_msg = BoundingBox()
                bounding_box_msg.Class = emotion.class_name
                bounding_box_msg.probability = emotion.probability
                bounding_box_msg.xmin = emotion.bounding_box[0]
                bounding_box_msg.ymin = emotion.bounding_box[1]
                bounding_box_msg.xmax = emotion.bounding_box[2]
                bounding_box_msg.ymax = emotion.bounding_box[3]
                bounding_boxes_msg.bounding_boxes.append(bounding_box_msg)
            self.pub_bounding_boxes.publish(bounding_boxes_msg)

            # Updating FPS
            self.update_fps_value()

            # Draw the FPS
            draw_fps(self.frame, self.fps)

            if self.image_view_enabled:
                cv2.imshow('emotion_detection_ros', self.frame)
                cv2.waitKey(self.image_view_wait_key_delay)

if __name__ == '__main__':
    rospy.init_node('emotion_detection')
    EmotionDetection()
    rospy.spin()