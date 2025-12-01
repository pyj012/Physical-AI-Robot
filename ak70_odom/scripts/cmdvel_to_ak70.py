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
        self.vel = 0.0 # 이 변수는 이제 ERPM 피드백을 저장합니다.
        self.tau = 0.0
        self.stamp = rospy.Time(0)

class DiffDriveNode(object):
    def __init__(self):
        rospy.init_node("cmdvel_to_ak70")
        rospy.on_shutdown(self.shutdown)

        # ----- 상수 정의 -----
        self.POLE_PAIRS = 21.0
        self.GEAR_RATIO = 10.0
        
        # TF 프레임 파라미터
        self.ifname     = rospy.get_param("~can_interface", "can0")
        self.left_id    = int(rospy.get_param("~left_id",  "0x02"), 0)
        self.right_id   = int(rospy.get_param("~right_id", "0x01"), 0)
        self.wheel_R    = float(rospy.get_param("~wheel_radius", 0.1))
        self.base_L     = float(rospy.get_param("~wheel_base",   0.339))
        self.dir_left   = int(rospy.get_param("~left_dir",  +1))
        self.dir_right  = int(rospy.get_param("~right_dir", -1)) 
        
        # ----- 파라미터 이름 및 기본값 변경 -----
        self.erpm_limit = float(rospy.get_param("~erpm_limit", 5000.0))
        
        self.acc_limit  = float(rospy.get_param("~acc_limit", 5000.0)) # ERPM/s 단위의 가속도
        self.frame_odom  = rospy.get_param("~odom_frame", "odom")
        self.frame_base  = rospy.get_param("~base_frame", "base_link")
        self.pub_joint   = bool(rospy.get_param("~publish_joint_states", True))
        self.rate_hz     = int(rospy.get_param("~rate_hz", 100)) # 피드백 주기에 맞게 조절
        
        # ----- 보정 계수 파라미터화 -----
        self.linear_correction = float(rospy.get_param("~linear_correction", 1.0))
        self.angular_correction = float(rospy.get_param("~angular_correction", 1.0))

        # 변수 초기화
        self.motor_controller = SmartMotorController(can_interface=self.ifname, motor_amount=2)
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.erpm_l = 0.0 # RPM이 아닌 ERPM을 저장
        self.erpm_r = 0.0
        self.dt = 1.0 / float(self.rate_hz)
        self.running = True
        self.x=0; self.y=0; self.yaw=0
        self.left_motor_state = WheelState()
        self.right_motor_state = WheelState()
        self.last = rospy.Time.now()

        # Publisher & Subscriber
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10) if self.pub_joint else None
        rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=10)

        self.th = threading.Thread(target=self.loop)
        self.th.daemon = True
        self.th.start()
        rospy.loginfo("AK70 Driver (ERPM Control) is ready.")

    def on_cmdvel(self, msg):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z
        
    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        step = self.acc_limit * self.dt
        
        # ----- rad/s를 ERPM으로 변환하기 위한 상수 -----
        # (60 / 2pi)는 rad/s -> RPM, 여기에 GEAR_RATIO와 POLE_PAIRS를 곱함
        RAD_PER_SEC_TO_ERPM = (60 / (2 * math.pi)) * self.GEAR_RATIO * self.POLE_PAIRS

        while not rospy.is_shutdown() and self.running:
            # 1. cmd_vel (m/s, rad/s) -> 바퀴 각속도 (rad/s)
            wl_ref = (2.0*self.v_ref - self.w_ref*self.base_L) / (2.0*self.wheel_R) 
            wr_ref = (2.0*self.v_ref + self.w_ref*self.base_L) / (2.0*self.wheel_R)

            # 2. 바퀴 각속도 (rad/s) -> 목표 ERPM
            erpm_l_ref = wl_ref * RAD_PER_SEC_TO_ERPM
            erpm_r_ref = wr_ref * RAD_PER_SEC_TO_ERPM

            # 가속도 제한 적용 (Ramping)
            def ramp(ref, cur):
                d = ref - cur
                if d > step: return cur + step
                if d < -step: return cur - step
                return ref
            self.erpm_l = ramp(erpm_l_ref, self.erpm_l)
            self.erpm_r = ramp(erpm_r_ref, self.erpm_r)

            # ERPM 제한 적용
            erpm_l_cmd = max(min(self.erpm_l, self.erpm_limit), -self.erpm_limit)
            erpm_r_cmd = max(min(self.erpm_r, self.erpm_limit), -self.erpm_limit)

            # 3. 모터에 ERPM 명령 전송
            try:
                self.motor_controller.motor_set_rpm(self.left_id, self.dir_left * erpm_l_cmd)
                self.motor_controller.motor_set_rpm(self.right_id, self.dir_right * erpm_r_cmd)
                
                # 4. 모터로부터 ERPM 피드백 수신
                motor_state = self.motor_controller.decode_motor_status()
                if motor_state['motor_id'] == self.left_id:
                    self.left_motor_state.vel = motor_state['speed_rpm']
                if motor_state['motor_id'] == self.right_id:
                    self.right_motor_state.vel = motor_state['speed_rpm']
                
                self.right_motor_state.stamp = rospy.Time.now()
                self.left_motor_state.stamp = rospy.Time.now()
            except Exception as e:
                rospy.logwarn("CAN communication error: %s", e)

            self.integrate_odom()
            rate.sleep()

    def integrate_odom(self):
        now = rospy.Time.now()
        dt = (now - self.last).to_sec()
        if dt <= 0.0: return
        self.last = now

        # ----- ERPM 피드백을 rad/s로 변환하기 위한 상수 -----
        RPM_TO_RAD_PER_SEC = (2 * math.pi) / 60.0

        # 1. ERPM 피드백 값 가져오기
        erpm_l_feedback = self.left_motor_state.vel * self.dir_left
        erpm_r_feedback = self.right_motor_state.vel * self.dir_right
        
        # deadzone_erpm = 20 * self.POLE_PAIRS * self.GEAR_RATIO # 데드존도 ERPM 기준으로 변환
        # if abs(erpm_l_feedback) < deadzone_erpm: erpm_l_feedback = 0.0
        # if abs(erpm_r_feedback) < deadzone_erpm: erpm_r_feedback = 0.0
            
        # 2. ERPM -> 최종 바퀴 RPM으로 변환
        left_wheel_rpm = erpm_l_feedback / self.POLE_PAIRS / self.GEAR_RATIO
        right_wheel_rpm = erpm_r_feedback / self.POLE_PAIRS / self.GEAR_RATIO
        
        # 3. 최종 바퀴 RPM -> 바퀴 각속도(rad/s)로 변환
        w_l_rads = left_wheel_rpm * RPM_TO_RAD_PER_SEC
        w_r_rads = right_wheel_rpm * RPM_TO_RAD_PER_SEC
        
        # 4. 오도메트리 계산 (이하는 표준 공식)
        v_l = w_l_rads * self.wheel_R
        v_r = w_r_rads * self.wheel_R

        v_raw  = (v_r + v_l) / 2.0
        wz_raw = (v_r - v_l) / self.base_L

        v  = v_raw  * self.linear_correction
        wz = wz_raw * self.angular_correction

        self.yaw += wz * dt
        cy = math.cos(self.yaw); sy = math.sin(self.yaw)
        self.x += v * cy * dt
        self.y += v * sy * dt

        # 5. odom 메시지 및 TF 발행 (rad/s 기준)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0,self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        self.tf_br.sendTransform((self.x, self.y, 0.0), q, now, self.frame_base, self.frame_odom)

        # ----- joint_states 발행값 수정 -----
        if self.joint_pub:
            js = JointState()
            js.header.stamp = now
            js.name = ['left_joint', 'right_joint']
            js.position = [self.left_motor_state.pos, self.right_motor_state.pos]
            js.velocity = [w_l_rads, w_r_rads] # ERPM이 아닌 rad/s 값 발행
            self.joint_pub.publish(js)
       
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.running = False
        self.th.join(1.0)
        try:
            self.motor_controller.motor_set_rpm(self.left_id, 0)
            self.motor_controller.motor_set_rpm(self.right_id, 0)
        except Exception as e:
            rospy.logerr("Failed to send stop command: %s", e)
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        DiffDriveNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
