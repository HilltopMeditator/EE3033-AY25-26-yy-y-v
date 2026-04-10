#!/usr/bin/env python

import sys
import os
import rospy
import smach
import roslaunch
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PoseArray

if sys.version_info.major < 3:
    input = raw_input

current_active_launch = None

def cleanup():
    if current_active_launch:
        current_active_launch.shutdown()
        rospy.sleep(2.0)
    os.system('pkill -9 -f "explore_lite" > /dev/null 2>&1')
    os.system('pkill -9 -f "rviz" > /dev/null 2>&1')

def shutdown_hook():
    rospy.logwarn("\n[LAPTOP] Shutting down. Cleaning up processes...")
    cleanup()
    os._exit(0)

class RunExplore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.pub_done = rospy.Publisher('/control/explore_lite_done', Empty, queue_size=1)

    def execute(self, userdata):
        global current_active_launch
        rospy.loginfo("=== STATE: EXPLORATION ===")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch Explore Lite (Modify package/launch name as needed)
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
       
        try:
            input("\n[LAPTOP] Explore Lite is running. Press [Enter] when mapping is complete...\n")
        except KeyboardInterrupt:
            return 'done'

        rospy.loginfo("Killing Explore Lite...")
        cleanup()
        
        # Tell the robot mapping is done
        self.pub_done.publish(Empty())
        return 'done'

class CollectWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        
        # Latched publisher so the robot can grab it whenever it's ready
        self.pub_waypoints = rospy.Publisher('/control/waypoints', PoseArray, queue_size=1, latch=True)
        self.pub_done = rospy.Publisher('/control/waypoints_done', Empty, queue_size=1)

    def execute(self, userdata):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"

        def callback(msg):
            pose_array.poses.append(msg.pose)
            rospy.loginfo("Waypoint {} saved.".format(len(pose_array.poses)))

        sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
        
        # Launch laptop-side visualization/navigation tools (e.g., RViz)
        try:
            print("\n[LAPTOP] Please launch RViz.")
            input("[LAPTOP] Please click waypoints in RViz using '2D Nav Goal'.\nPress [Enter] when finished...\n")
        except KeyboardInterrupt:
            pass

        sub.unregister()
        pose_array.header.stamp = rospy.Time.now()
        
        # Publish the array and the trigger signal
        self.pub_waypoints.publish(pose_array)
        self.pub_done.publish(Empty())
        
        rospy.loginfo("Published {} waypoints to the robot.".format(len(pose_array.poses)))
        return 'done'

class LaptopNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        global current_active_launch
        rospy.loginfo("=== STATE: LAPTOP NAVIGATION ===")

        rospy.loginfo("[LAPTOP] Standing by. Press Ctrl+C to kill everything gracefully.")
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
        return 'exit'

def main():
    rospy.init_node('laptop_commander')
    rospy.on_shutdown(shutdown_hook)
    roslaunch.rlutil.get_or_generate_uuid(None, True)

    sm = smach.StateMachine(outcomes=['complete'])
    with sm:
        smach.StateMachine.add('EXPLORE', RunExplore(), transitions={'done':'WAYPOINTS'})
        smach.StateMachine.add('WAYPOINTS', CollectWaypoints(), transitions={'done':'NAV_STAY_ALIVE'})
        smach.StateMachine.add('NAV_STAY_ALIVE', LaptopNavigation(), transitions={'exit':'complete'})
    sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
