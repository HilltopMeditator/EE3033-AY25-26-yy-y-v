#!/usr/bin/env python2

import time
import rospy
import rosgraph
import smach
import roslaunch
import tf
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Log

class RobotWorkerNode:
    """
    Main application class managing the Robot-side Worker node.
    """

    def __init__(self):
        # 1. Setup UUID and check Master
        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        self._active_launches = {}
        self._roscore = None

        if not rosgraph.is_master_online():
            rospy.loginfo("[APP] No roscore found. Starting local roscore...")
            self._roscore = roslaunch.parent.ROSLaunchParent(self._uuid, [], is_core=True)
            self._roscore.start()
            time.sleep(1.0) 

        # 2. Initialize Node
        rospy.init_node('robot_worker', disable_signals=False)
        self._sm = self._build_state_machine()
        
        # GLOBAL CLEANUP: Triggered by Ctrl+C
        rospy.on_shutdown(self._shutdown_hook)

    # ==========================================
    # NESTED SMACH STATES
    # ==========================================

    class _HeadlessMapping(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done', 'aborted'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: MAPPING ===")
            try:
                self.app.start_launch('mapping', 'turn_on_wheeltec_robot', 'mapping.launch', ["navigation:=true"])
                rospy.loginfo("[ROBOT] Mapping active. Waiting for laptop signal...")
                
                # Wait for the laptop to signal exploration is done
                rospy.wait_for_message('/control/explore_lite_done', Empty)
                return 'done'
            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C during Mapping")
                return 'aborted'

    class _HeadlessSaveAndKill(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['success', 'aborted'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: SAVE_MAP & POSE ===")
            try:
                # 1. POSE HANDOVER: Save position before killing SLAM
                listener = tf.TransformListener()
                rospy.sleep(1.0) # Buffer tf
                try:
                    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    yaw = tf.transformations.euler_from_quaternion(rot)[2]
                    rospy.set_param('/initial_pose_x', trans[0])
                    rospy.set_param('/initial_pose_y', trans[1])
                    rospy.set_param('/initial_pose_a', yaw)
                    rospy.loginfo("[ROBOT] Pose Handover Saved: x={:.2f}, y={:.2f}".format(trans[0], trans[1]))
                except Exception as e:
                    rospy.logwarn("[ROBOT] Failed to save pose: {}".format(e))

                # 2. SAVE MAP
                self.app.start_launch('map_saver', 'turn_on_wheeltec_robot', 'map_saver.launch')
                rospy.sleep(5.0) 
                return 'success'

            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C during Map Save")
                return 'aborted'
            finally:
                # LOCAL CLEANUP: Ensure mapping is dead before Nav starts
                self.app.stop_launch('map_saver')
                self.app.stop_launch('mapping')

    class _HeadlessNavigationBringup(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['ready', 'aborted'])
            self.app = app_context

        def execute(self, userdata):
            rospy.loginfo("=== STATE: NAVIGATION BRINGUP ===")
            try:
                # Starts AMCL/Navigation using params from SaveAndKill
                self.app.start_launch('navigation', 'turn_on_wheeltec_robot', 'navigation.launch')
                rospy.sleep(5.0)
                return 'ready'
            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C during Nav Bringup")
                return 'aborted'

    class _WaitAndExecuteWaypoints(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['finished', 'aborted'])

        def execute(self, userdata):
            rospy.loginfo("=== STATE: EXECUTE WAYPOINTS ===")
            try:
                while not rospy.is_shutdown():
                    rospy.sleep(1.0)
                return 'finished'
            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C during execution")
                return 'aborted'

    # ==========================================
    # LOGIC & TEARDOWN
    # ==========================================

    def _build_state_machine(self):
        sm = smach.StateMachine(outcomes=['complete', 'failed'])
        with sm:
            smach.StateMachine.add('MAPPING', self._HeadlessMapping(self), 
                                   transitions={'done':'SAVE_AND_KILL', 'aborted':'failed'})
            smach.StateMachine.add('SAVE_AND_KILL', self._HeadlessSaveAndKill(self), 
                                   transitions={'success':'NAV_BRINGUP', 'aborted':'failed'})
            smach.StateMachine.add('NAV_BRINGUP', self._HeadlessNavigationBringup(self), 
                                   transitions={'ready':'EXEC_WAYPOINTS', 'aborted':'failed'})
            smach.StateMachine.add('EXEC_WAYPOINTS', self._WaitAndExecuteWaypoints(), 
                                   transitions={'finished':'complete', 'aborted':'failed'})
        return sm

    def _shutdown_hook(self):
        """GLOBAL CLEANUP: Shuts down nodes and the Master"""
        rospy.logwarn("\n[ROBOT] Global cleanup: Tearing down launches...")
        for name in list(self._active_launches.keys()):
            self.stop_launch(name)
            
        if self._roscore:
            rospy.logwarn("[APP] Shutting down local roscore...")
            self._roscore.shutdown()

    def start_launch(self, name, pkg, launch_file, args=None):
        resolved = roslaunch.rlutil.resolve_launch_arguments([pkg, launch_file])
        parent = roslaunch.parent.ROSLaunchParent(self._uuid, [(resolved[0], args or [])])
        parent.start()
        self._active_launches[name] = parent

    def stop_launch(self, name):
        if name in self._active_launches:
            self._active_launches[name].shutdown()
            del self._active_launches[name]

    def run(self):
        self._sm.execute()

if __name__ == '__main__':
    try:
        app = RobotWorkerNode()
        app.run()
    except rospy.ROSInterruptException:
        pass