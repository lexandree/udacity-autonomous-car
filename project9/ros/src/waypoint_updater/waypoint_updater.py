#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0


def point_distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # last part
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb) or PoseStamped

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.publish_rate = 10

        #  rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if None not in [self.pose, self.base_waypoints]:
                # get closest waypoint and publish lane
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        # check if closest is ahead or behind of the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        # equation for hyperplane through closest_coords
        cl_vector = np.array(closest_coord)
        prev_vector = np.array(prev_coord)
        pos_vector = np.array([x, y])
        val = np.dot(cl_vector - prev_vector, pos_vector - cl_vector)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        if False:  # for part one
            closest_idx = self.get_closest_waypoint_idx()
            lane = Lane()
            lane.header = self.base_waypoints.header
            lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        else:  # for last part
            lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        new_points = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # 2-4 waypoints back from line so front of car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 4, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            new_points.append(p)
        return new_points

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if self.waypoints_2d is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        rospy.logwarn('Waypoints loaded successfully ({0} points)'.format(len(self.base_waypoints.waypoints)))

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        self.stopline_wp_idx = msg.data

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def distance(waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2):
            dist += point_distance(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
