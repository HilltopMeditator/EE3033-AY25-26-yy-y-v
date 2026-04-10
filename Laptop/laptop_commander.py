#!/usr/bin/env python

import sys
import os
import rospy
import smach
import roslaunch
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PoseArray

# Python 2/3 compatibility for legacy ROS 1
if sys.version_info.major < 3:
    input = raw_input

class LaptopCommanderNode:
    """
    Main application class managing the laptop-side Commander node.
    """

    # ==========================================
    # NESTED SMACH STATES
    # ==========================================
    class _RunExplore(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done'])
            self.app = app_context
            self.pub_done = rospy.Publisher('/control/explore_lite_done', Empty, queue_size=1)

        def execute(self, userdata):
            rospy.loginfo("=== STATE: EXPLORATION ===")

            # Resolve the absolute path to your home directory workspace 
            home_dir = os.path.expanduser("~")
            launch_file_path = os.path.join(home_dir, "explore_lite_ws/src/launch/laptop_explore.launch")
            
            if not os.path.exists(launch_file_path):
                rospy.logerr("[LAPTOP] Launch file not found at: {}".format(launch_file_path))
                return 'done'
                
            rospy.loginfo("[LAPTOP] Starting explore_lite programmatically...")
            self.app.start_launch_by_path('explore', launch_file_path)
           
            try:
                input("\n[LAPTOP] Explore Lite is running. Press [Enter] when mapping is complete...\n")
            except KeyboardInterrupt:
                # FIX: Actually trigger the shutdown!
                rospy.signal_shutdown("User pressed Ctrl+C")
                return 'done'

            rospy.loginfo("[LAPTOP] Tearing down Explore Lite...")
            self.app.stop_launch('explore')
            
            # Tell the robot mapping is done
            self.pub_done.publish(Empty())
            return 'done'

    class _CollectWaypoints(smach.State):
        def __init__(self, app_context):
            smach.State.__init__(self, outcomes=['done'])
            self.app = app_context
            # Latched publisher so the robot can grab it whenever it's ready
            self.pub_waypoints = rospy.Publisher('/control/waypoints', PoseArray, queue_size=1, latch=True)
            self.pub_done = rospy.Publisher('/control/waypoints_done', Empty, queue_size=1)

        def execute(self, userdata):
            rospy.loginfo("=== STATE: COLLECT WAYPOINTS ===")
            
            pose_array = PoseArray()
            pose_array.header.frame_id = "map"

            def callback(msg):
                pose_array.poses.append(msg.pose)
                rospy.loginfo("Waypoint {} saved.".format(len(pose_array.poses)))

            sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
            
            # Launch RViz with the custom configuration file
            raw_path = "~/ee3033 rviz setup.rviz"
            expanded_path = os.path.expanduser(raw_path)
            rviz_args = '-d "{}"'.format(expanded_path)
            
            rospy.loginfo("[LAPTOP] Launching RViz with custom config: {}".format(expanded_path))
            self.app.start_node('rviz', 'rviz', 'rviz', args=rviz_args)

            try:
                input("\n[LAPTOP] Please click waypoints in RViz using '2D Nav Goal'.\nPress [Enter] when finished...\n")
            except KeyboardInterrupt:
                # Trigger the shutdown
                rospy.signal_shutdown("User pressed Ctrl+C")
                pass

            sub.unregister()
            pose_array.header.stamp = rospy.Time.now()
            
            # Publish the array and the trigger signal
            self.pub_waypoints.publish(pose_array)
            self.pub_done.publish(Empty())
            
            rospy.loginfo("Published {} waypoints to the robot.".format(len(pose_array.poses)))
            
            # Leaving RViz running for the final navigation phase
            return 'done'

    class _LaptopNavigation(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['exit'])

        def execute(self, userdata):
            rospy.loginfo("=== STATE: LAPTOP NAVIGATION ===")
            rospy.loginfo("[LAPTOP] Standing by. Press Ctrl+C to kill everything gracefully.")
            
            while not rospy.is_shutdown():
                rospy.sleep(1.0)
            return 'exit'

    # ==========================================
    # MAIN APPLICATION LOGIC
    # ==========================================
    def __init__(self):
        rospy.init_node('laptop_commander', disable_signals=False)
        
        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        self._active_launches = {}
        
        self._sm = self._build_state_machine()
        rospy.on_shutdown(self._shutdown_hook)

    def _build_state_machine(self):
        sm = smach.StateMachine(outcomes=['complete'])
        with sm:
            smach.StateMachine.add('EXPLORE', self._RunExplore(self), 
                                   transitions={'done':'WAYPOINTS'})
            smach.StateMachine.add('WAYPOINTS', self._CollectWaypoints(self), 
                                   transitions={'done':'NAV_STAY_ALIVE'})
            smach.StateMachine.add('NAV_STAY_ALIVE', self._LaptopNavigation(), 
                                   transitions={'exit':'complete'})
        return sm

    def start_launch_by_path(self, name, absolute_path, args=None):
        """Starts a launch file using an absolute file path."""
        if args is None: args = []
        roslaunch_file = [(absolute_path, args)]
        
        parent = roslaunch.parent.ROSLaunchParent(self._uuid, roslaunch_file)
        parent.start()
        self._active_launches[name] = parent

    def start_node(self, name, pkg, executable, args=""):
        node = roslaunch.core.Node(pkg, executable, args=args)
        launch = roslaunch.scriptapi.ROSLaunch() # <--- This is the manager
        launch.start()
        process = launch.launch(node)
        
        # FIX: Save the manager so we can fire him later
        self._active_launches[name] = launch

    def stop_launch(self, name):
        """Gracefully shuts down a tracked process or launch file."""
        if name in self._active_launches:
            rospy.loginfo("[LAPTOP] Tearing down {}...".format(name))
            target = self._active_launches[name]
            
            if hasattr(target, 'shutdown'):
                target.shutdown()
            elif hasattr(target, 'stop'):
                target.stop()
                
            del self._active_launches[name]

    def _shutdown_hook(self):
        """Executed automatically by rospy on Ctrl+C."""
        rospy.logwarn("\n[LAPTOP] Shutting down. Cleaning up processes...")
        for name in list(self._active_launches.keys()):
            self.stop_launch(name)

    def run(self):
        """Begins execution of the node's main logic."""
        rospy.loginfo("[LAPTOP] Starting Commander State Machine...")
        self._sm.execute()


if __name__ == '__main__':
    try:
        app = LaptopCommanderNode()
        app.run()
    except rospy.ROSInterruptException:
        pass