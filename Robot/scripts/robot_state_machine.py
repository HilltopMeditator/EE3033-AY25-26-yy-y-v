#!/usr/bin/env python

import os
import rospy
import smach
import roslaunch
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray

current_active_launch = None

def cleanup(targets):
    if current_active_launch:
        current_active_launch.shutdown()
        rospy.sleep(2.0)
    for keyword in targets:
        os.system('pkill -9 -f "{}" > /dev/null 2>&1'.format(keyword))

def shutdown_hook():
    rospy.logwarn("\n[ROBOT] Emergency shutdown. Wiping active nodes...")
    cleanup(['mapping.launch', 'navigation.launch', 'slam_gmapping', 'amcl', 'move_base'])
    os._exit(0)

class HeadlessMapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        global current_active_launch
        rospy.loginfo("=== STATE: MAPPING ===")
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launchfile = ['turn_on_wheeltec_robot', 'mapping.launch']
        launchargs = ["navigation:=true"]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], launchargs)]
        current_active_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        current_active_launch.start()
        
        rospy.loginfo("[ROBOT] Mapping active. Waiting for explore_lite_done signal from laptop...")
        
        # Block until the laptop sends the "done" signal
        rospy.wait_for_message('/control/explore_lite_done', Empty)
        
        return 'done'

class HeadlessSaveAndKill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global current_active_launch
        rospy.loginfo("=== STATE: SAVE_MAP ===")
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'map_saver.launch'])[0]
        saver = roslaunch.parent.ROSLaunchParent(uuid, [path])
        saver.start()
        rospy.sleep(5) 
        
        rospy.loginfo("[ROBOT] Map saved. Tearing down mapping nodes...")
        cleanup(['mapping.launch', 'slam_gmapping'])
        current_active_launch = None 
        rospy.sleep(3) 
        return 'success'

class HeadlessNavigationBringup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready'])

    def execute(self, userdata):
        global current_active_launch
        rospy.loginfo("=== STATE: NAVIGATION BRINGUP ===")
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'navigation.launch'])[0]
        current_active_launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        current_active_launch.start()
        
        # Wait a moment for AMCL and move_base to fully initialize
        rospy.sleep(5)
        return 'ready'

class WaitAndExecuteWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo("[ROBOT] Waiting for waypoints from laptop...")
        

        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            
        return 'finished'

def main():
    rospy.init_node('robot_worker')
    rospy.on_shutdown(shutdown_hook)
    roslaunch.rlutil.get_or_generate_uuid(None, True)

    sm = smach.StateMachine(outcomes=['complete'])
    with sm:
        smach.StateMachine.add('MAPPING', HeadlessMapping(), transitions={'done':'SAVE_AND_KILL'})
        smach.StateMachine.add('SAVE_AND_KILL', HeadlessSaveAndKill(), transitions={'success':'NAV_BRINGUP'})
        smach.StateMachine.add('NAV_BRINGUP', HeadlessNavigationBringup(), transitions={'ready':'EXEC_WAYPOINTS'})
        smach.StateMachine.add('EXEC_WAYPOINTS', WaitAndExecuteWaypoints(), transitions={'finished':'complete'})
    sm.execute()

if __name__ == '__main__':
    main()