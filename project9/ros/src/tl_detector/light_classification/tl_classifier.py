import rospy
import time
# from absl import app, flags, logging
# from absl.flags import FLAGS
import cv2
import numpy as np
import requests

from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self, remote=True, flip_channels=False, url = 'http://localhost:5000/detections',
                 classes=7, weights='./weights/yolov3.tf', cls_names='./data/labels/obj.names', size=416):
        """YOLO  Classes:
        0 - stop
        1 - go
        2 - warning
        3 - stopLeft
        4 - goLeft
        5 - warningLeft
        6 - goForward
        """
        self.yelo_stop = 0
        self.yelo_go = 1
        self.yelo_warning = 2
        self.yelo_others = 7
        self.remote = remote
        self.flip_channels = flip_channels
        self.out_state = TrafficLight.UNKNOWN
        self.detect_time = 0

        if self.remote:
            self.url = url
            img = np.random.random((320, 320, 3)).astype(np.float32)
            t1 = time.time()
            r = self.get_remote_response(img)
            t2 = time.time()
            rospy.logwarn('status: {}'.format(r.status_code))
            rospy.logwarn('sanity check passed \ntime: {}'.format(t2 - t1))
        else:
            import tensorflow as tf
            from yolov3_tf2.models import (YoloV3)
            from yolov3_tf2.dataset import transform_images
            # from yolov3_tf2.utils import draw_outputs


            rospy.logwarn('tensorflow version: ({0})'.format(tf.__version__))
            physical_devices = tf.config.experimental.list_physical_devices('GPU')
            if len(physical_devices) > 0:
                tf.config.experimental.set_memory_growth(physical_devices[0], True)
            self.yolo = YoloV3(classes=classes)
            self.yolo.load_weights(weights).expect_partial()
            rospy.logwarn('weights loaded')
            self.class_names = [c.strip() for c in open(cls_names).readlines()]
            rospy.logwarn('classes loaded')

            img = np.random.random((1, 320, 320, 3)).astype(np.float32)
            self.img_size = size
            img = transform_images(img, size)
            t1 = time.time()
            prediction = self.yolo(img)
            t2 = time.time()
            rospy.logwarn('sanity check passed \ntime: {}'.format(t2 - t1))

    def get_classification(self, image):
        if self.remote:
            return self.get_remote_classification(image)
        else:
            return self.get_local_classification(image)

    def get_local_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            TrafficLight.UNKNOWN
            TrafficLight.GREEN
            TrafficLight.YELLOW
            TrafficLight.RED
        """
        if self.flip_channels:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img = tf.expand_dims(image, 0)
        img = transform_images(img, self.img_size)

        t1 = time.time()
        boxes, scores, classes, nums = self.yolo(img)
        t2 = time.time()
        rospy.logwarn('pred time: {}'.format(t2 - t1))

        stats = {self.yelo_stop: 0, self.yelo_go: 0, self.yelo_warning: 0, self.yelo_others: 0}
        rospy.logwarn('detections:')
        if nums[0] == 0:
            rospy.logwarn('no detections')
            return TrafficLight.UNKNOWN
        for i in range(nums[0]):
            rospy.logwarn('\t{}, {}, {}'.format(self.class_names[int(classes[0][i])],
                                        np.array(scores[0][i]),
                                        np.array(boxes[0][i])))
            if int(classes[0][i]) < 3:
                stats[int(classes[0][i])] += 1
            else:
                stats[self.yelo_others] += 1
        # returns the smallest key among equal values and we have RED < YELLOW < GREEN
        out_state = max(stats, key=stats.get)
        if out_state == self.yelo_stop:
            return TrafficLight.RED
        if out_state == self.yelo_go:
            return TrafficLight.GREEN
        if out_state == self.yelo_warning:
            return TrafficLight.YELLOW
        return TrafficLight.UNKNOWN
        # img = cv2.cvtColor(raw_img.numpy(), cv2.COLOR_RGB2BGR)
        # img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
        # cv2.imwrite('./detections/' + 'detection' + str(num) + '.jpg', img)

    def get_remote_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            TrafficLight.UNKNOWN
            TrafficLight.GREEN
            TrafficLight.YELLOW
            TrafficLight.RED
        """
        t2 = time.time()
        if t2 - self.detect_time < 0.75:
            return self.out_state
        r = self.get_remote_response(image)
        self.detect_time = time.time()
        stats = {self.yelo_stop: 0, self.yelo_go: 0, self.yelo_warning: 0, self.yelo_others: 0}
        #rospy.logwarn('detections:')
        if r.status_code != 200:
            rospy.logwarn('bad status: ', r.status_code)
            self.out_state = TrafficLight.UNKNOWN
            return self.out_state
        if len(r.json().get('response')[0].get('detections')) == 0:
            rospy.logwarn('no detections')
            self.out_state = TrafficLight.UNKNOWN
            return self.out_state
        for detection in r.json().get('response')[0].get('detections'):
            if int(detection.get('class_id')) < 3:
                stats[int(detection.get('class_id'))] += 1
            else:
                stats[self.yelo_others] += 1
        # returns the smallest key among equal values and we have RED < YELLOW < GREEN
        detect_state = max(stats, key=stats.get)
        if detect_state == self.yelo_stop:
            self.out_state = TrafficLight.RED
        elif detect_state == self.yelo_go:
            self.out_state = TrafficLight.GREEN
        elif detect_state == self.yelo_warning:
            self.out_state = TrafficLight.YELLOW
        else:
            self.out_state = TrafficLight.UNKNOWN
        return self.out_state

    def get_remote_response(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            requests.Response: with all classification data
        """
        t1 = time.time()
        if self.flip_channels:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        is_success, im_buf_arr = cv2.imencode(".jpg", image)
        byte_im = im_buf_arr.tobytes()

        files = [('images', ('filename.jpg', byte_im))]
        r = requests.post(self.url, files=files, timeout=5.0)
        t2 = time.time()
        rospy.logwarn('pred time: {}'.format(t2 - t1))
        return r