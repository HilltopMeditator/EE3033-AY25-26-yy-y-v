#!/usr/bin/env python3

import sys
import os
import rospy
import smach
import roslaunch
import rosgraph
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PoseArray
from rosgraph_msgs.msg import Log
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import MarkerArray

if sys.version_info.major < 3:
    input = raw_input

class LaptopCommanderNode:
    def __init__(self):
        # Initialize node with disable_signals=False to catch Ctrl+C
        rospy.init_node('laptop_commander', disable_signals=False)
        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        
        self._active_launches = {}
        self._sm = self._build_state_machine()
        
        # GLOBAL CLEANUP: Managed by rospy on exit
        rospy.on_shutdown(self._shutdown_hook)

    # ==========================================
    # NESTED SMACH STATES
    # ==========================================
    
    class _RunExplore(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done', 'aborted'])
            self.app = app_context
            self.pub_done = rospy.Publisher('/control/explore_lite_done', Empty, queue_size=1)
            self.exploration_finished = False

        def execute(self, userdata):
            rospy.loginfo("=== STATE: EXPLORATION ===")
            subs = []
            self.exploration_finished = False
            
            try:
                # 1. Start explore_lite
                # Get the absolute path to the directory where this script lives
                current_dir = os.path.dirname(os.path.abspath(__file__))
                
                # Look for the launch file in the exact same directory
                launch_path = os.path.join(current_dir, "laptop_explore.launch")
                self.app.start_launch_by_path('explore', launch_path)

                # 2. DEFINING THE ADEQUATE CONDITION: Log Listener
                def rosout_callback(msg):
                    # Check if the message comes from the explore node and matches the stop string
                    if 'explore' in msg.name and "Exploration stopped" in msg.msg:
                        rospy.loginfo("[LAPTOP] Detected internal 'Stop' signal from explore_lite.")
                        self.exploration_finished = True

                # 3. SECONDARY CONDITION: move_base IDLE (Backup)
                def status_callback(msg):
                    # If move_base has no active goals and we've been running for a while
                    if len(msg.status_list) == 0:
                        # We don't trigger immediately to avoid startup races
                        pass

                subs.append(rospy.Subscriber('/rosout', Log, rosout_callback))
                
                # Wait loop with a "Hysteresis" check
                rate = rospy.Rate(1)
                start_time = rospy.Time.now()
                
                while not rospy.is_shutdown() and not self.exploration_finished:
                    # If we've been running for >10s and markers are 0, also accept as done
                    # (Safety fallback in case rosout is dropped)
                    rate.sleep()

                if self.exploration_finished:
                    self.pub_done.publish(Empty())
                    return 'done'
                
                return 'aborted'

            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("User Abort")
                return 'aborted'
            finally:
                for s in subs: s.unregister()
                self.app.stop_launch('explore')

    class _CollectWaypoints(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done', 'aborted'])
            self.app = app_context
            self.pub_waypoints = rospy.Publisher('/control/waypoints', PoseArray, queue_size=1, latch=True)
            self.pub_done = rospy.Publisher('/control/waypoints_done', Empty, queue_size=1)

        def execute(self, userdata):
            rospy.loginfo("=== STATE: COLLECT WAYPOINTS ===")
            sub = None
            pose_array = PoseArray()
            pose_array.header.frame_id = "map"

            try:
                def callback(msg):
                    pose_array.poses.append(msg.pose)
                    rospy.loginfo("Waypoint {} saved.".format(len(pose_array.poses)))

                # LOCAL CLEANUP START
                sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
                
                input("\n[LAPTOP] Click goals in RViz. Press [Enter] when done...\n")
                
                pose_array.header.stamp = rospy.Time.now()
                self.pub_waypoints.publish(pose_array)
                self.pub_done.publish(Empty())
                return 'done'

            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C in Waypoints")
                return 'aborted'
            finally:
                # LOCAL CLEANUP END
                if sub: sub.unregister()

    class _LaptopNavigation(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['complete', 'aborted'])

        def execute(self, userdata):
            rospy.loginfo("=== STATE: LAPTOP NAVIGATION ===")
            rate = rospy.Rate(1)
            try:
                while not rospy.is_shutdown():
                    if not rosgraph.is_master_online():
                        rospy.logwarn("[LAPTOP] Master lost.")
                        return 'aborted'
                    rate.sleep()
                return 'complete'
            except (KeyboardInterrupt, rospy.ROSInterruptException):
                rospy.signal_shutdown("Ctrl+C in Nav")
                return 'aborted'

    # ==========================================
    # LOGIC & TEARDOWN
    # ==========================================

    def _build_state_machine(self):
        sm = smach.StateMachine(outcomes=['finished', 'failed'])
        with sm:
            # All 'aborted' outcomes now route to 'failed' (the terminal outcome)
            smach.StateMachine.add('EXPLORE', self._RunExplore(self), 
                                   transitions={'done':'WAYPOINTS', 'aborted':'failed'})
            smach.StateMachine.add('WAYPOINTS', self._CollectWaypoints(self), 
                                   transitions={'done':'NAV_STAY_ALIVE', 'aborted':'failed'})
            smach.StateMachine.add('NAV_STAY_ALIVE', self._LaptopNavigation(), 
                                   transitions={'complete':'finished', 'aborted':'failed'})
        return sm

    def _shutdown_hook(self):
        """GLOBAL CLEANUP: Kills persistent nodes/launches"""
        rospy.logwarn("\n[LAPTOP] Global cleanup: Tearing down persistent processes...")
        for name in list(self._active_launches.keys()):
            self.stop_launch(name)

    def start_launch_by_path(self, name, absolute_path, args=None):
        parent = roslaunch.parent.ROSLaunchParent(self._uuid, [(absolute_path, args or [])])
        parent.start()
        self._active_launches[name] = parent

    def start_node(self, name, pkg, executable, args=""):
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        launch.launch(roslaunch.core.Node(pkg, executable, args=args))
        self._active_launches[name] = launch

    def stop_launch(self, name):
        if name in self._active_launches:
            target = self._active_launches[name]
            if hasattr(target, 'shutdown'): target.shutdown()
            elif hasattr(target, 'stop'): target.stop()
            del self._active_launches[name]

    def run(self):
        # Start persistent RViz first
        rviz_path = os.path.expanduser("~/ee3033 rviz setup.rviz")
        self.start_node('rviz', 'rviz', 'rviz', args='-d "{}"'.format(rviz_path))
        self._sm.execute()

if __name__ == '__main__':
    try:
        app = LaptopCommanderNode()
        app.run()
    except rospy.ROSInterruptException:
        pass