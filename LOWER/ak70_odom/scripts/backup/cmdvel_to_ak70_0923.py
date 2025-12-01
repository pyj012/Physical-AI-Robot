#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import threading
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion
import can  # python-can (socketcan backend)
from cubemars_lib import *

class WheelState(object):
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0
        self.tau = 0.0
        self.stamp = rospy.Time(0)

class DiffDriveNode(object):
    def __init__(self):
        rospy.init_node("cmdvel_to_ak70")
        self.prev_time = 0
        self.ifname     = rospy.get_param("~can_interface", "can0")
        self.left_id    = int(rospy.get_param("~left_id",  "0x01"), 0)
        self.right_id   = int(rospy.get_param("~right_id", "0x02"), 0)
        self.wheel_R    = float(rospy.get_param("~wheel_radius", 0.1))
        self.base_L     = float(rospy.get_param("~wheel_base",   0.339))
        self.dir_left   = int(rospy.get_param("~left_dir",  +1))
        self.dir_right  = int(rospy.get_param("~right_dir", -1)) 
        self.rpm_limit = float(rospy.get_param("~rpm_limit", 5000.0))
        self.acc_limit  = float(rospy.get_param("~acc_limit", 5000.0))
        self.tff        = float(rospy.get_param("~torque_ff", 0.0))
        self.zero_on_start = bool(rospy.get_param("~zero_on_start", False))
        self.use_right  = bool(rospy.get_param("~use_right", True))
        self.frame_odom  = rospy.get_param("~odom_frame", "odom")
        self.frame_base  = rospy.get_param("~base_frame", "base_link")
        self.pub_joint   = bool(rospy.get_param("~publish_joint_states", True))
        self.rate_hz     = int(rospy.get_param("~rate_hz", 100))
        
        self.motor_controller = SmartMotorController(can_interface="can0", motor_amount=2)
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.wl    = 0.0
        self.wr    = 0.0
        self.dt    = 1.0 / float(self.rate_hz)
        self.running = True
        self.x=0
        self.y=0
        self.yaw=0
        self.left_motor_state = WheelState()
        self.right_motor_state = WheelState()
        self.last = rospy.Time.now()
        # publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10) if self.pub_joint else None
        rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=10)

        self.th = threading.Thread(target=self.loop)

        print("left_motor_id : ", self.left_id)
        print("right_motor_id : ", self.right_id)
        print("rpm_limit : ", self.rpm_limit)
        print("acc_limit : ", self.acc_limit)
        print("dir_left : ", self.dir_left)
        print("dir_right : ", self.dir_right)
        print("wheel_R : ", self.wheel_R)
        print("base_L : ", self.base_L)

        self.th.daemon = True
        self.th.start()

    def on_cmdvel(self, msg):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z
        

    def setZeroPosition(self,motor):
        pos, _, _ = motor.set_zero_position()
        while abs(np.rad2deg(pos)) > 0.5:
            pos, vel, curr = motor.set_zero_position()
            print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        step = self.acc_limit * self.dt
        while not rospy.is_shutdown() and self.running:
            wl_ref = (2.0*self.v_ref - self.w_ref*self.base_L) / (2.0*self.wheel_R) 
            wr_ref = (2.0*self.v_ref + self.w_ref*self.base_L) / (2.0*self.wheel_R)

            def ramp(ref, cur):
                d = ref - cur
                if d >  step: return cur + step
                if d < -step: return cur - step
                return ref

            self.wl = ramp(wl_ref, self.wl)
            self.wr = ramp(wr_ref, self.wr)
            #while True:
             #pos, vel, curr = self.left_motor.send_rad_command(0, 90, 0, 2, 0)
             #print("left_pos :",pos)
             #time.sleep(1)
            RAD_PER_SEC_TO_RPM = 60 / (2 * math.pi)
            GEAR_RATIO = 10.0 
            rpm_l = self.wl * RAD_PER_SEC_TO_RPM * GEAR_RATIO
            rpm_r = self.wr * RAD_PER_SEC_TO_RPM * GEAR_RATIO
            rpm_l = max(min(rpm_l, self.rpm_limit), -self.rpm_limit)
            rpm_r = max(min(rpm_r, self.rpm_limit), -self.rpm_limit)
            try:
                try:
                    self.motor_controller.motor_set_rpm(self.left_id, self.dir_left * rpm_l)
                    self.motor_controller.motor_set_rpm(self.right_id, self.dir_right * rpm_r)
                    
                    motor_state = self.motor_controller.decode_motor_status()
                    if motor_state['motor_id'] == self.left_id:
                        self.left_motor_state.vel = motor_state['speed_rpm'] * self.dir_left

                    if motor_state['motor_id'] == self.right_id:
                        self.right_motor_state.vel = motor_state['speed_rpm'] * self.dir_right
                    # print("l_rpm :,",rpm_l, "l_stuts :,",self.left_motor_state.vel, "r_rpm :,",rpm_r , ", r_stat : ", self.right_motor_state.vel)

                    self.right_motor_state.stamp = rospy.Time.now()
                    self.left_motor_state.stamp = rospy.Time.now()
                except Exception as e:
                    print("SEND ERR:", e)

            except can.CanError as e:
                rospy.logwarn("CAN tx error: %s", e)
                time.sleep(0.01)
            self.integrate_odom()
            rate.sleep()

    def integrate_odom(self):
        now = rospy.Time.now()
        dt = (now - self.last).to_sec()
        if dt <= 0.0:
            return
        self.last = now
        deadzone =  20
        RPM_TO_RAD_PER_SEC = (2 * math.pi) / 60.0
        GEAR_RATIO = 10.0  
        left_rpm_feedback = self.left_motor_state.vel
        right_rpm_feedback = self.right_motor_state.vel
        
        if abs(left_rpm_feedback) < deadzone:
            left_rpm_feedback = 0.0
        if abs(right_rpm_feedback) < deadzone:
            right_rpm_feedback = 0.0
            
        left_wheel_rpm = left_rpm_feedback / GEAR_RATIO
        right_wheel_rpm = right_rpm_feedback / GEAR_RATIO
        w_l_rads = left_wheel_rpm * RPM_TO_RAD_PER_SEC
        w_r_rads = right_wheel_rpm * RPM_TO_RAD_PER_SEC
        v_l = w_l_rads * self.wheel_R
        v_r = w_r_rads * self.wheel_R
        # 로봇의 선속도와 각속도 계산
        v_raw  = 0.5 * (v_r + v_l)
        wz_raw = (v_r - v_l) / self.base_L

        # 보정 계수 적용
        linear_correction=0.05
        angular_correction=0.05
        v  = v_raw  * linear_correction
        wz = wz_raw *angular_correction

        self.yaw += wz * dt
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        self.x += v * cy * dt
        self.y += v * sy * dt

        # odom msg
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0,self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # TF
        self.tf_br.sendTransform(
            (self.x, self.y, 0.0), (q[0], q[1], q[2], q[3]), now, self.frame_base, self.frame_odom
        )

        # joint states
        if self.joint_pub:
            js = JointState()
            js.header.stamp = now
            js.name = ['left_joint', 'right_joint']
            js.position = [self.left_motor_state.pos, self.right_motor_state.pos]
            js.velocity = [self.left_motor_state.vel, self.right_motor_state.vel]
            self.joint_pub.publish(js)
       
    def shutdown(self):
        self.running = False
        try:
            self.th.join(1.0)
        except:
            pass
        try:
            self.left_motor.send_rad_command(0.0, 0, 0.0, 0, 0)
            if self.use_right:
                self.right_motor.send_rad_command(0.0, 0, 0.0, 0, 0)
        except:
            pass
        try:
            self.left_motor.disable_motor()
            if self.use_right:
                self.right_motor.disable_motor()
        except:
            pass

if __name__ == "__main__":
    node = DiffDriveNode()
    rospy.spin()


