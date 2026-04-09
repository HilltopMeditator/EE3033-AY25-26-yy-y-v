#!/usr/bin/env python
import rospy
import tf
import math
import os
import roslaunch
from std_msgs.msg import Bool

class WaypointOrchestrator:
    def __init__(self):
        rospy.init_node('laptop_orchestrator')
        
        # Parameters
        self.log_distance = 0.05        # Meters to move before logging a raw breadcrumb
        self.corner_angle_deg = 20.0    # Degrees of heading change to count as a corner
        self.max_los_distance = 5.0     # Max distance between points before injecting midpoints
        self.idle_timeout = 30.0        # Seconds of no movement to trigger completion
        
        self.tf_listener = tf.TransformListener()
        self.complete_pub = rospy.Publisher('/exploration_complete', Bool, queue_size=1, latch=True)
        
        self.raw_path = []
        self.last_pose = None
        self.last_move_time = rospy.Time.now()
        self.is_exploring = True

        # Setup roslaunch 
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        # Resolve the absolute path to your home directory workspace 
        # HARDCODED for simplicity. Adjust as needed.
        home_dir = os.path.expanduser("~")
        launch_file_path = os.path.join(home_dir, "explore_lite_ws/src/launch/laptop_explore.launch")
        
        if not os.path.exists(launch_file_path):
            rospy.logerr("Launch file not found at: {}".format(launch_file_path))
            return
            
        rospy.loginfo("Starting explore_lite programmatically...")
        self.explore_launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        self.explore_launch.start()

        rospy.loginfo("Laptop Orchestrator Started. Logging breadcrumbs...")

    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return (trans[0], trans[1], yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def distance(self, p1, p2):
        return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

    def process_breadcrumbs(self):
        if len(self.raw_path) < 2:
            rospy.logwarn("Not enough data to generate path.")
            return

        rospy.loginfo("Processing path: Extracting corners and ensuring coverage...")
        optimized_path = [self.raw_path[0]]
        
        # 1. Corner Detection (Simplified Pruning)
        for i in range(1, len(self.raw_path) - 1):
            prev_pt = optimized_path[-1]
            curr_pt = self.raw_path[i]
            
            # Calculate heading from prev to current
            heading = math.atan2(curr_pt[1] - prev_pt[1], curr_pt[0] - prev_pt[0])
            heading_deg = math.degrees(heading)
            
            # Very basic check: if yaw changed significantly, keep it
            yaw_diff = abs(math.degrees(curr_pt[2]) - math.degrees(prev_pt[2]))
            if yaw_diff > self.corner_angle_deg or self.distance(prev_pt, curr_pt) > self.max_los_distance:
                optimized_path.append(curr_pt)
                
        optimized_path.append(self.raw_path[-1]) # Always keep the last point
        
        # 2. Mirror for Front->Back->Front
        reversed_path = list(reversed(optimized_path))[1:] # Exclude the turnaround duplicate
        full_patrol = optimized_path + reversed_path
        
        self.print_for_teammate(full_patrol)

    def print_for_teammate(self, path):
        print("\n" + "="*50)
        print(" EXPLORATION COMPLETE. COPY THIS TO multi_waypoint_nav.py")
        print("="*50)
        for i in range(len(path) - 1):
            pt1 = path[i]
            pt2 = path[i+1]
            # Calculate the yaw pointing to the NEXT waypoint for smooth MoveBase flow
            target_yaw = math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0])
            print("locations.append(Pose(Point({:.3f}, {:.3f}, 0.0), self.yaw_to_quaternion({:.3f})))".format(
                pt1[0], pt1[1], target_yaw
            ))
        
        # Final point faces 0 or whatever home orientation you prefer
        final_pt = path[-1]
        print("locations.append(Pose(Point({:.3f}, {:.3f}, 0.0), self.yaw_to_quaternion(0.0)))".format(final_pt[0], final_pt[1]))
        print("="*50 + "\n")

    def run(self):
        rate = rospy.Rate(5.0) # 5 Hz is plenty
        while not rospy.is_shutdown() and self.is_exploring:
            pose = self.get_robot_pose()
            if pose:
                if not self.last_pose:
                    self.raw_path.append(pose)
                    self.last_pose = pose
                    self.last_move_time = rospy.Time.now()
                else:
                    dist = self.distance(self.last_pose, pose)
                    
                    if dist > self.log_distance:
                        self.raw_path.append(pose)
                        self.last_pose = pose
                        self.last_move_time = rospy.Time.now()
                        rospy.loginfo("Breadcrumb dropped. Total: {}".format(len(self.raw_path)))
                    
                    # Timeout Trigger: If stationary for X seconds, assume explore_lite is done
                    if (rospy.Time.now() - self.last_move_time).to_sec() > self.idle_timeout:
                        rospy.loginfo("Robot idle timeout reached. Ending exploration.")
                        self.is_exploring = False
                        
            rate.sleep()
            
        if not self.is_exploring:
            self.process_breadcrumbs()

            # Kill explore_lite before sending the completion signal
            rospy.loginfo("Shutting down explore_lite node...")
            self.explore_launch.shutdown()
            rospy.sleep(2) # Give it a moment to clear the ROS network

            self.complete_pub.publish(Bool(True))

            

if __name__ == '__main__':
    try:
        orchestrator = WaypointOrchestrator()
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass