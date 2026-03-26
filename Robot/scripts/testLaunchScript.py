#!/usr/bin/env python

import sys
import rospy
import smach
import roslaunch

# --- Python Version Detection Block ---
if sys.version_info.major < 3:
    input = raw_input
# --------------------------------------

# Global variable to hold the active launch process
active_mapping_launch = None

class LaunchMapping(smach.State):
    def __init__(self):
        # Removed output_keys since we are using a global variable
        smach.State.__init__(self, outcomes=['done'])
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    def execute(self, userdata):
        global active_mapping_launch
        
        rospy.loginfo("=== STATE: MAPPING ===")
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'mapping.launch'])[0]
        active_mapping_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path])
        active_mapping_launch.start()
        
        input("\n[ACTION REQUIRED] Mapping active. Drive the robot around to map the area.\nPress [Enter] when ready to save the map...\n")
        
        return 'done'

class SaveAndKill(smach.State):
    def __init__(self):
        # Removed input_keys
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global active_mapping_launch
        
        rospy.loginfo("=== STATE: SAVE_MAP ===")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'map_saver.launch'])[0]
        saver = roslaunch.parent.ROSLaunchParent(uuid, [path])
        
        rospy.loginfo("Launching map_saver.launch...")
        saver.start()
        
        rospy.sleep(5) 
        
        rospy.loginfo("Map saved. Shutting down mapping nodes to free up the TF tree...")
        if active_mapping_launch is not None:
            active_mapping_launch.shutdown()
        else:
            rospy.logwarn("No active mapping launch found to shut down!")
            
        rospy.sleep(3) 
        
        return 'success'

class LaunchNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE: NAVIGATION ===")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = roslaunch.rlutil.resolve_launch_arguments(['turn_on_wheeltec_robot', 'navigation.launch'])[0]
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        
        rospy.loginfo("Starting AMCL and MoveBase...")
        launch.start()
        
        rospy.loginfo("\n>>> Navigation active. AMCL has taken over the map->odom transform. <<<")
        rospy.loginfo("Keep this terminal open to keep navigation running. Press Ctrl+C to exit.")
        
        rospy.spin() 
        
        return 'ready'

def main():
    rospy.init_node('smach_handoff_test', log_level=rospy.INFO)
    roslaunch.rlutil.get_or_generate_uuid(None, True)

    sm = smach.StateMachine(outcomes=['test_complete'])

    with sm:
        smach.StateMachine.add('MAPPING', LaunchMapping(), 
                               transitions={'done':'SAVE_AND_KILL'})
        
        smach.StateMachine.add('SAVE_AND_KILL', SaveAndKill(),
                               transitions={'success':'NAVIGATION'})
        
        smach.StateMachine.add('NAVIGATION', LaunchNavigation(),
                               transitions={'ready':'test_complete'})

    rospy.loginfo("Starting SMACH Orchestrator...")
    sm.execute()

if __name__ == '__main__':
    main()
