#!/usr/bin/env python

import time
import rospy
import rosgraph
import smach
import roslaunch
from std_msgs.msg import Empty

class RobotWorkerNode:
    """
    Main application class managing the ROS node, SMACH, and dynamic launch files.
    """

    # ==========================================
    # NESTED SMACH STATES
    # ==========================================
    class _HeadlessMapping(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: MAPPING ===")
            self.app.start_launch('mapping', 'turn_on_wheeltec_robot', 'mapping.launch', ["navigation:=true"])
            
            rospy.loginfo("[ROBOT] Mapping active. Waiting for explore_lite_done...")
            rospy.wait_for_message('/control/explore_lite_done', Empty)
            return 'done'

    class _HeadlessSaveAndKill(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['success'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: SAVE_MAP ===")
            self.app.start_launch('map_saver', 'turn_on_wheeltec_robot', 'map_saver.launch')
            rospy.sleep(5.0) 
            
            rospy.loginfo("[ROBOT] Map saved. Tearing down mapping nodes...")
            self.app.stop_launch('map_saver')
            self.app.stop_launch('mapping')
            rospy.sleep(3.0) 
            return 'success'

    class _HeadlessNavigationBringup(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['ready'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: NAVIGATION BRINGUP ===")
            self.app.start_launch('navigation', 'turn_on_wheeltec_robot', 'navigation.launch')
            rospy.sleep(5.0)
            return 'ready'

    class _WaitAndExecuteWaypoints(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['finished'])

        def execute(self, userdata):
            rospy.loginfo("[ROBOT] Waiting for waypoints from laptop...")
            while not rospy.is_shutdown():
                rospy.sleep(1.0)
            return 'finished'

    # ==========================================
    # MAIN APPLICATION LOGIC
    # ==========================================
    def __init__(self):
        # 1. Setup UUID FIRST (Required to start roscore)
        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        
        self._active_launches = {}
        self._roscore = None

        # 2. Check for an existing roscore; start one programmatically if missing
        if not rosgraph.is_master_online():
            print("[APP] No roscore found. Starting local roscore...")
            # is_core=True tells ROSLaunchParent to act as the ROS Master
            self._roscore = roslaunch.parent.ROSLaunchParent(self._uuid, [], is_core=True)
            self._roscore.start()
            
            # Give the master a brief moment to open its XMLRPC server sockets
            time.sleep(1.0) 
        else:
            print("[APP] Existing roscore detected. Binding to it...")

        # 3. Now initialize the node (This is safe because the master is guaranteed to be up)
        rospy.init_node('robot_worker', disable_signals=False)
        
        self._sm = self._build_state_machine()
        rospy.on_shutdown(self._shutdown_hook)

    def _build_state_machine(self):
        sm = smach.StateMachine(outcomes=['complete'])
        with sm:
            smach.StateMachine.add('MAPPING', self._HeadlessMapping(self), 
                                   transitions={'done':'SAVE_AND_KILL'})
            smach.StateMachine.add('SAVE_AND_KILL', self._HeadlessSaveAndKill(self), 
                                   transitions={'success':'NAV_BRINGUP'})
            smach.StateMachine.add('NAV_BRINGUP', self._HeadlessNavigationBringup(self), 
                                   transitions={'ready':'EXEC_WAYPOINTS'})
            smach.StateMachine.add('EXEC_WAYPOINTS', self._WaitAndExecuteWaypoints(), 
                                   transitions={'finished':'complete'})
        return sm

    def start_launch(self, name, pkg, launch_file, args=None):
        if args is None: args = []
        rospy.loginfo("[APP] Starting {} ({}/{})...".format(name, pkg, launch_file))
        resolved_path = roslaunch.rlutil.resolve_launch_arguments([pkg, launch_file])
        roslaunch_file = [(resolved_path[0], args)]
        
        parent = roslaunch.parent.ROSLaunchParent(self._uuid, roslaunch_file)
        parent.start()
        self._active_launches[name] = parent

    def stop_launch(self, name):
        if name in self._active_launches:
            rospy.loginfo("[APP] Tearing down {}...".format(name))
            self._active_launches[name].shutdown()
            del self._active_launches[name]

    def _shutdown_hook(self):
        rospy.logwarn("\n[ROBOT] Emergency shutdown. Tearing down all active nodes...")
        
        # 1. Kill the child nodes first
        for name in list(self._active_launches.keys()):
            self.stop_launch(name)
            
        # 2. Kill the ROS Master last
        if self._roscore:
            rospy.logwarn("[APP] Shutting down local roscore...")
            self._roscore.shutdown()
            rospy.loginfo("[APP] Roscore successfully terminated.")

    def run(self):
        rospy.loginfo("[APP] Starting Robot Worker State Machine...")
        self._sm.execute()


if __name__ == '__main__':
    try:
        app = RobotWorkerNode()
        app.run()
    except rospy.ROSInterruptException:
        pass