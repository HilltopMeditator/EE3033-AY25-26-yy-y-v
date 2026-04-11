#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class IMUCorrector:
    def __init__(self):
        rospy.init_node('imu_corrector')
        
        self.param_name = '/wheeltec/z_gyro_offset'
        self.pub = rospy.Publisher('/imu_corrected', Imu, queue_size=10)
        
        # Integration state variables
        self.last_time = None
        self.clean_yaw = 0.0 
        
        # --- CHECK FOR EXISTING OFFSET ---
        if rospy.has_param(self.param_name):
            self.z_gyro_offset = rospy.get_param(self.param_name)
            self.is_calibrating = False
            rospy.loginfo("[IMU Corrector] Respawned. Applying saved Z-offset: {:.6f}".format(self.z_gyro_offset))
            rospy.loginfo("[IMU Corrector] Active correction running.")
        else:
            # Entering active value tuning
            self.is_calibrating = True
            self.calibration_duration = rospy.Duration(5.0)
            self.start_time = None
            self.z_samples = []
            self.z_gyro_offset = 0.0
            rospy.logwarn("[IMU Corrector] First boot detected. Starting 5-second calibration.")
            rospy.logwarn("[IMU Corrector] ---> PLEASE KEEP ROBOT PERFECTLY STILL <---")

        rospy.Subscriber('/imu_32', Imu, self.callback)

    def callback(self, msg):
        current_time = rospy.Time.now()

        # ==========================================
        # PHASE 1: ACTIVE CALIBRATION (VALUE TUNING)
        # ==========================================
        if self.is_calibrating:
            if self.start_time is None:
                self.start_time = current_time

            if (current_time - self.start_time) < self.calibration_duration:
                # We are actively tuning. Record the sample.
                self.z_samples.append(msg.angular_velocity.z)
                
                # Prevent EKF drift while tuning
                msg.angular_velocity.z = 0.0 
                self.pub.publish(msg)
                return 
                
            else:
                # Tuning is complete. Calculate and save.
                if len(self.z_samples) > 0:
                    self.z_gyro_offset = sum(self.z_samples) / len(self.z_samples)
                
                rospy.set_param(self.param_name, float(self.z_gyro_offset))
                self.is_calibrating = False
                
                rospy.loginfo("[IMU Corrector] Calibration complete and saved to Parameter Server!")
                rospy.loginfo("[IMU Corrector] Active correction running (Offset: {:.6f})".format(self.z_gyro_offset))

        # ==========================================
        # PHASE 2: NORMAL OPERATION (CORRECTION)
        # ==========================================
        
        # Initialize the integration timer on the very first corrected message
        if self.last_time is None:
            self.last_time = current_time
            # Capture the starting orientation from the STM32
            _, _, self.clean_yaw = euler_from_quaternion([
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w
            ])
            self.pub.publish(msg)
            return

        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Step 1: Clean the raw velocity
        clean_z_vel = msg.angular_velocity.z - self.z_gyro_offset
        msg.angular_velocity.z = clean_z_vel

        # Step 2: Integrate the clean velocity into a drift-free Yaw
        self.clean_yaw += (clean_z_vel * dt)

        # Step 3: Convert our clean Yaw back into a Quaternion
        q = quaternion_from_euler(0, 0, self.clean_yaw)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        # Step 4: Detune the Covariances so the EKF listens to the wheels
        orient_cov = list(msg.orientation_covariance)
        orient_cov[8] = 1e-2  
        msg.orientation_covariance = orient_cov

        ang_vel_cov = list(msg.angular_velocity_covariance)
        ang_vel_cov[8] = 1e-2
        msg.angular_velocity_covariance = ang_vel_cov

        # Publish the corrected message
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        corrector = IMUCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass