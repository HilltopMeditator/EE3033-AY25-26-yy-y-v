#!/usr/bin/env python

import sys
import os
import rospy
import smach
import roslaunch

# --- Python Version Detection Block ---
if sys.version_info.major < 3:
    input = raw_input
# --------------------------------------

# Global variable to track the currently active roslaunch process
current_active_launch = None

def brute_force_cleanup(launch_obj, target_keywords):
    """
    Attempts a polite shutdown, then escalates to pkill for stubborn nodes.
    """
    if launch_obj is not None:
        rospy.loginfo("Attempting polite ROS shutdown (SIGINT)...")
        launch_obj.shutdown()
        rospy.sleep(2.0) 

    rospy.logwarn("Escalating to OS-level SIGKILL for stubborn processes...")
    for keyword in target_keywords:
        os.system('pkill -9 -f "{}" > /dev/null 2>&1'.format(keyword))
        
    rospy.loginfo("Force cleanup complete.")


def shutdown_hook():
    """Triggered automatically when the script is killed (e.g., via Ctrl+C)"""
    global current_active_launch
    rospy.logwarn("\n[SHUTDOWN] Emergency kill signal received. Wiping all active nodes...")
    
    # Target anything related to mapping or navigation
    targets = ['mapping.launch', 'navigation.launch', 'slam_gmapping', 'amcl', 'move_base', 'rviz']
    brute_force_cleanup(current_active_launch, targets)
    
    rospy.loginfo("[SHUTDOWN] Exiting Python interpreter forcefully.")
    
    # THE NUCLEAR OPTION: Instantly kill the Python script and all its threads, 
    # breaking out of rospy.spin() safely.
    os._exit(0)


class LaunchMapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    def execute(self, userdata):
        global current_active_launch
        
        rospy.loginfo("=== STATE: MAPPING ===")
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'mapping.launch'])[0]
        
        current_active_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path])
        current_active_launch.start()
        
        # Wrap input in a try-except to catch Ctrl+C during the prompt
        try:
            input("\n[ACTION REQUIRED] Mapping active. Drive the robot around.\nPress [Enter] when ready to save the map...\n")
        except KeyboardInterrupt:
            rospy.signal_shutdown("User pressed Ctrl+C during mapping.")
            return 'done'
        
        return 'done'


class SaveAndKill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global current_active_launch
        
        if rospy.is_shutdown():
            return 'success'

        rospy.loginfo("=== STATE: SAVE_MAP ===")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'map_saver.launch'])[0]
        saver = roslaunch.parent.ROSLaunchParent(uuid, [path])
        
        rospy.loginfo("Launching map_saver.launch...")
        saver.start()
        
        rospy.sleep(5) 
        
        rospy.loginfo("Map saved. Initiating aggressive teardown of mapping nodes...")
        brute_force_cleanup(current_active_launch, ['mapping.launch', 'slam_gmapping'])
        current_active_launch = None 
            
        rospy.sleep(3) 
        
        return 'success'


class LaunchNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready'])

    def execute(self, userdata):
        global current_active_launch
        
        if rospy.is_shutdown():
            return 'ready'

        rospy.loginfo("=== STATE: NAVIGATION ===")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'navigation.launch'])[0]
        
        current_active_launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        
        rospy.loginfo("Starting AMCL and MoveBase...")
        current_active_launch.start()
        
        rospy.loginfo("\n>>> Navigation active. AMCL has taken over the map->odom transform. <<<")
        rospy.loginfo("Keep this terminal open to keep navigation running. Press Ctrl+C to exit.")
        
        # Restored standard rospy.spin() behavior
        rospy.spin()
        
        return 'ready'


def main():
    rospy.init_node('smach_handoff_test', log_level=rospy.INFO)
    roslaunch.rlutil.get_or_generate_uuid(None, True)

    rospy.on_shutdown(shutdown_hook)

    sm = smach.StateMachine(outcomes=['test_complete'])

    with sm:
        smach.StateMachine.add('MAPPING', LaunchMapping(), 
                               transitions={'done':'SAVE_AND_KILL'})
        
        smach.StateMachine.add('SAVE_AND_KILL', SaveAndKill(),
                               transitions={'success':'NAVIGATION'})
        
        smach.StateMachine.add('NAVIGATION', LaunchNavigation(),
                               transitions={'ready':'test_complete'})

    rospy.loginfo("Starting SMACH Orchestrator...")
    
    # Catch SMACH execution exceptions
    try:
        sm.execute()
    except Exception as e:
        rospy.logerr("SMACH execution interrupted: " + str(e))

if __name__ == '__main__':
    main()