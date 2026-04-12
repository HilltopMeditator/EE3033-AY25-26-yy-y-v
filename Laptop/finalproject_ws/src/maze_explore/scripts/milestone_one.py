#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import actionlib
import math
from enum import Enum

from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from visualization_msgs.msg import Marker

# ---------------------- STATE MACHINE ----------------------

class RobotState(Enum):
    EXPLORING = 0
    SCANNING = 1
    APPROACHING = 2
    LOCK_TARGET = 3
    GO_HOME = 4


# ---------------------- MAIN CLASS ----------------------

class MultiWaypointNav:

    def __init__(self):
        rospy.init_node('multi_waypoint_nav')

        # ---------------- Core Systems ----------------
        self.listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	#self.marker_pub=rospy.Publisher('/target_marker',Marker,queue_size=10)

        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.detection_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("Waiting for move_base...")
        self.move_base.wait_for_server()

        # ---------------- State ----------------
        self.state = RobotState.EXPLORING
        self.manual_control = False

        # ---------------- Detection ----------------
        self.target_class = "bottle"
        self.target_detected = False
        self.target_close = False
        self.last_detection_time = rospy.Time(0)
	self.lost=0
	self.overtime=0
	#self.marker_id=0

        # ---------------- Scan ----------------
        self.scan_angle_step = math.radians(10)
        self.angular_speed = 0.3

        # ---------------- Map / Waypoints ----------------
        self.start_pose = Pose(Point(-0.002, -0.044, 0.000), Quaternion(0.000, 0.000, 0.112, 0.994))
        self.waypoints = self.load_waypoints()
        self.current_wp_index = 0


        self.latest_scan = None

        rospy.on_shutdown(self.shutdown)

        self.run()

    # ---------------------- MAIN LOOP ----------------------

    def run(self):
        rospy.loginfo("Starting state machine...")
	#marker=Marker()
	#marker.action = Marker.DELETEALL
	#self.marker_pub.publish(marker)

        while not rospy.is_shutdown():

            if self.state == RobotState.EXPLORING:
                self.handle_exploring()

            elif self.state == RobotState.SCANNING:
                self.handle_scanning()

            elif self.state == RobotState.APPROACHING:
                self.handle_approaching()

            elif self.state == RobotState.LOCK_TARGET:
                self.handle_lock_target()

            elif self.state == RobotState.GO_HOME:
                self.handle_go_home()
                break

            rospy.sleep(0.1)

    # ---------------------- STATE HANDLERS ----------------------

    def handle_exploring(self):
        rospy.loginfo("State: EXPLORING")

        goal_pose = self.waypoints[self.current_wp_index]
        success = self.send_goal(goal_pose)

        if success:
            rospy.loginfo("Reached waypoint")
            self.state = RobotState.SCANNING
        else:
            rospy.logwarn("Failed to reach waypoint")

        self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)

    def handle_scanning(self):
        rospy.loginfo("State: SCANNING")

        found = self.scan_360()

        if found:
            self.state = RobotState.APPROACHING
        else:
            self.state = RobotState.EXPLORING

    def handle_approaching(self):
        rospy.loginfo("State: APPROACHING")

        self.take_manual_control()

        while not rospy.is_shutdown():

            if self.target_close:
                rospy.loginfo("Target reached")
                self.release_manual_control()
                self.state = RobotState.LOCK_TARGET
                return
		
	    if self.microscan == 0:
	    	self.micro_scan()
            samples = self.stabilize_and_sample()

	    if len(samples) == 0:
    		rospy.logwarn("No stable detection → scanning")
    	        self.state = RobotState.SCANNING
    		return

	    error_n = sum(samples) / len(samples)
	    self.apply_approach_control(error_n)
            rospy.sleep(0.05)

    def handle_lock_target(self):
        rospy.loginfo("State: LOCK_TARGET")

        self.lock_target_coordinates(self.latest_scan)
	
        self.state = RobotState.GO_HOME

    def handle_go_home(self):
        rospy.loginfo("State: GO_HOME")

        self.send_goal(self.start_pose)
        rospy.loginfo("Mission complete")

    # ---------------------- NAVIGATION ----------------------

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(120))

        if not finished:
            self.move_base.cancel_goal()
            return False

        return True

    def load_waypoints(self):
	#waypoint_poses = rospy.wait_for_message('/control/waypoints', PoseArray).poses
        #rospy.loginfo("[ROBOT] Received {} waypoints! Executing...".format(len(waypoint_poses)))
	#return waypoint_poses
        return [

	    Pose(Point(-0.002, -0.044, 0.000), Quaternion(0.000, 0.000, 0.112, 0.994)), 
	    Pose(Point(0.460, 0.600, 0.000), Quaternion(0.000, 0.000, -0.626, 0.780)), 
	    Pose(Point(1.559, 0.296, 0.000), Quaternion(0.000, 0.000, 0.996, 0.091))

        ]



    # ---------------------- SCANNING ----------------------

    def scan_360(self):
        self.take_manual_control()
	rospy.loginfo("Starting 360 Scan")
	self.microscan = 0
	if self.lost == 1:
	    self.go_back()

        twist = Twist()
        twist.angular.z = self.angular_speed

	steps = int((2 * math.pi / self.scan_angle_step) + 11)

        for _ in range(steps):

            start = rospy.Time.now()
            duration = self.scan_angle_step / self.angular_speed

            while (rospy.Time.now() - start).to_sec() < duration:
                self.cmd_vel_pub.publish(twist)

                if self.target_detected:
                    self.release_manual_control()
		    rospy.loginfo("Target Detected")
                    return True

                rospy.sleep(0.02)

            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.2)

	if self.scan_end_stabilization():
		return True
	
    	

        self.release_manual_control()
	rospy.loginfo("Target Not Found")
        return False

    # ---------------------- APPROACH ----------------------

    def apply_approach_control(self, error_n):
	twist = Twist()
	rospy.loginfo("Approaching")
	rospy.loginfo("Error_N: " + str(error_n))

	# If it is centered enough stop rotating
	if abs(error_n) > 0.2:
    	    twist.angular.z = -0.3 * error_n   # proportional turning


    	    twist.linear.x = 0.0  # don't move forward while turning
	    rospy.loginfo("Rotating")

	else:
    	    twist.angular.z = 0.0
    	    twist.linear.x = 0.06  # move forward slowly when aligned
	    rospy.loginfo("Moving Forward")
	
	starttime = rospy.Time.now()
	Duration = rospy.Duration(3.0)
	rospy.loginfo("angular_z: " + str(twist.angular.z))
	while (rospy.Time.now() - starttime) < Duration:
    		self.cmd_vel_pub.publish(twist)
    		rospy.sleep(0.05)   

    # ---------------------- DETECTION ----------------------

    def detection_callback(self, msg):
	
        boxes = [b for b in msg.bounding_boxes if b.Class == self.target_class]

	rospy.loginfo("Detetction CMD")
        if boxes:
	    self.last_detection_time=rospy.Time.now()
	    self.updated_time = self.last_detection_time.to_sec()
	    rospy.loginfo("Updated detection time: " + str(self.updated_time))
            best = max(boxes, key=lambda b: b.probability)

            self.target_detected = True
            self.last_detection_time = rospy.Time.now()
	    self.last_time1 = self.last_detection_time

            self.box_center_x = (best.xmin + best.xmax) / 2
            self.image_width = 640
	    self.box_width = best.xmax - best.xmin
	    rospy.loginfo("box width: " + str(self.box_width))

            #self.area = (best.xmax - best.xmin) * (best.ymax - best.ymin)
            # Utilizing width instead of area, more stable
	    self.target_close = self.box_width >= 150

        else:
            self.target_detected = (rospy.Time.now() - self.last_detection_time).to_sec() < 5.0
            self.target_close = False

    # ---------------------- TARGET LOCK ----------------------

    def lock_target_coordinates(self, scan):
        self.listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))

        robot_x, robot_y = trans[0], trans[1]
        _, _, yaw = euler_from_quaternion(rot)
	rospy.loginfo ("Robot Locking Target")
	rospy.loginfo("Robot_x : "+str(robot_x))
	rospy.loginfo("Robot_y: "+str(robot_y))
	# Robot Marker
	#self.publish_target_marker(robot_x, robot_y)

        theta = (self.box_center_x - self.image_width/2) / (self.image_width/2) * (math.radians(60)/2)

        if scan:
            idx = int((theta - scan.angle_min) / scan.angle_increment)
            idx = max(0, min(idx, len(scan.ranges)-1))
            # Only possible if object is within laser scan height
	    vals = []

	    for i in range(idx - 3, idx + 4):  # small window
    		if 0 <= i < len(scan.ranges):
        	    r = scan.ranges[i]
        	    if not math.isnan(r) and not math.isinf(r):
            		vals.append(r)

	
	    if vals:
    		d = min(vals)   # min vals = least likely to be wall
		rospy.loginfo("d min vals: " + str(d))
	    # if value seems like wall set default distance from robot
	    else:
    		d = 0.15
	    if d > 0.5:
		d=0.15
	
	rospy.loginfo("d Val:" +str(d))
        target_x = robot_x + d * math.cos(yaw + theta)
        target_y = robot_y + d * math.sin(yaw + theta)

        rospy.loginfo("Target locked at:")
	rospy.loginfo("X :" +str(target_x))
	rospy.loginfo("Y :" + str(target_y))
	self.coordinates=target_x


	# Target Marker
	#self.publish_target_marker(target_x, target_y)
	


    # ---------------------- HELPERS ----------------------

    def take_manual_control(self):
        if not self.manual_control:
	    state = self.move_base.get_state()
	    if state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            	self.move_base.cancel_goal()
            self.manual_control = True

    def release_manual_control(self):
        if self.manual_control:
            self.cmd_vel_pub.publish(Twist())
            self.manual_control = False

    def yaw_to_quaternion(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def shutdown(self):
        rospy.loginfo("Shutting down")
        self.move_base.cancel_goal()
        self.cmd_vel_pub.publish(Twist())

    # ---------------------- Stabilizers ----------------------   
    
    def scan_end_stabilization(self, duration=10.0):
	rospy.loginfo("Stabilizing after 360 Scan")
    	self.cmd_vel_pub.publish(Twist())
    	rospy.sleep(0.2)  # let motion fully stop  

	start = rospy.Time.now()  
        while (rospy.Time.now() - start).to_sec() < duration:
	    if self.target_detected:
                    self.release_manual_control()
		    rospy.loginfo("Target Detected")
                    return True
	return False
		

    def stabilize_and_sample(self, samples_needed=3, duration=10.0):
	rospy.loginfo("Stabilizing and Sampling")
    	self.cmd_vel_pub.publish(Twist())
    	rospy.sleep(0.2)  # let motion fully stop


    	samples = []
	last_time = self.last_time1

	while len(samples) < samples_needed:
	    age = (rospy.Time.now()-self.last_detection_time).to_sec()
	    if self.last_detection_time != last_time:
		rospy.loginfo("Detection age in sec: " + str(age))
		last_time = self.last_time1
		image_center = self.image_width / 2
        	error_x = self.box_center_x - image_center
		self.error_n = float(error_x) / image_center
            	samples.append(self.error_n)
	    timeout = rospy.Duration(20.0)
	    if rospy.Time.now() - self.last_detection_time < timeout and self.overtime ==1:
		self.overtime=0
            if rospy.Time.now() - self.last_detection_time > timeout and self.overtime==0:
		rospy.loginfo("Detection age in sec: " + str(age))
                rospy.logwarn("Lost target → scanning")
                self.release_manual_control()
                self.state = RobotState.SCANNING
		self.lost=1
		self.overtime=1
		self.target_detected = False
		samples = []
                return samples
            rospy.sleep(0.05)
    	return samples

    def micro_scan(self):
	rospy.loginfo("Micro Scan")
	self.microscan=1
    	twist = Twist()
    	twist.angular.z = -0.8  
	starttime = rospy.Time.now()
	duration = rospy.Duration(1.0)
	while (rospy.Time.now() - starttime) < duration:
    		self.cmd_vel_pub.publish(twist)
    		rospy.sleep(0.05)

    	self.cmd_vel_pub.publish(Twist())
    	rospy.sleep(0.5)

    def go_back(self):
	rospy.loginfo("Going back a bit")
	self.lost=0
	twist = Twist()
    	twist.linear.x = -0.1  
	starttime = rospy.Time.now()
	duration = rospy.Duration(1.0)
	while (rospy.Time.now() - starttime) < duration:
    		self.cmd_vel_pub.publish(twist)
    		rospy.sleep(0.05)

 # ---------------------- Markers ----------------------   

    def publish_target_marker(self, x, y):
    	marker = Marker()

    	marker.header.frame_id = "map"
    	marker.header.stamp = rospy.Time.now()

    	marker.ns = "target"
    	self.marker_id += 1
	marker.id = self.marker_id
    	marker.type = Marker.SPHERE
    	marker.action = Marker.ADD

    	# Position
    	marker.pose.position.x = x
    	marker.pose.position.y = y
    	marker.pose.position.z = 0.1 

    	marker.pose.orientation.w = 1.0

    	# Size
    	marker.scale.x = 0.2
    	marker.scale.y = 0.2
    	marker.scale.z = 0.2
	

    	# Color (RED)
	if self.marker_id == 1:
	    marker.color.r = 0.0
    	    marker.color.g = 0.0
    	    marker.color.b = 1.0
    	    marker.color.a = 1.0
	else:
    	    marker.color.r = 1.0
    	    marker.color.g = 0.0
    	    marker.color.b = 0.0
    	    marker.color.a = 1.0

    	# Keep it visible
    	marker.lifetime = rospy.Duration(0)

    	self.marker_pub.publish(marker)


# ---------------------- MAIN ----------------------

if __name__ == "__main__":
    try:
        MultiWaypointNav()
    except rospy.ROSInterruptException:
        pass
