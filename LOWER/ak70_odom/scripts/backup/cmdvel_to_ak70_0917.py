#!/usr/bin/env python
# -*- coding: ascii -*-

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
from pythoncan_canmotorlib import *

# ===== AK70 limits =====
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.0, 45.0
T_MIN, T_MAX = -25.0, 25.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

class WheelState(object):
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0
        self.tau = 0.0
        self.stamp = rospy.Time(0)

def float_to_uint(x, x_min, x_max, bits):
    if x > x_max:
        x = x_max
    if x < x_min:
        x = x_min
    span = (x_max - x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)

def encode_cmd(pos_deg, vel, kp, kd, torq):
    pos_rad = math.radians(pos_deg)
    p_int  = float_to_uint(pos_rad, P_MIN, P_MAX, 16)
    v_int  = float_to_uint(vel,     V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp,      KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd,      KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(torq,    T_MIN, T_MAX, 12)

    d = bytearray(8)
    d[0] = (p_int >> 8) & 0xFF
    d[1] = p_int & 0xFF
    d[2] = (v_int >> 4) & 0xFF
    d[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    d[4] = kp_int & 0xFF
    d[5] = (kd_int >> 4) & 0xFF
    d[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    d[7] = t_int & 0xFF
    return d

class DiffDriveNode(object):
    def __init__(self):
        rospy.init_node("cmdvel_to_ak70")
        self.prev_time = 0
        self.ifname     = rospy.get_param("~can_interface", "can0")
        self.left_id    = int(rospy.get_param("~left_id",  "0x01"), 0)
        self.right_id   = int(rospy.get_param("~right_id", "0x02"), 0)
        self.wheel_R    = float(rospy.get_param("~wheel_radius", 0.1))
        self.base_L     = float(rospy.get_param("~wheel_base",   0.339))
        self.dir_left   = int(rospy.get_param("~left_dir",  -1))
        self.dir_right  = int(rospy.get_param("~right_dir", +1)) 
        self.acc_limit  = float(rospy.get_param("~acc_limit", 5.0))
        self.kd_vel     = float(rospy.get_param("~kd_vel", 0.5))
        self.tff        = float(rospy.get_param("~torque_ff", 0.0))
        self.zero_on_start = bool(rospy.get_param("~zero_on_start", False))
        self.use_right  = bool(rospy.get_param("~use_right", True))
        self.frame_odom  = rospy.get_param("~odom_frame", "odom")
        self.frame_base  = rospy.get_param("~base_frame", "base_link")
        self.pub_joint   = bool(rospy.get_param("~publish_joint_states", True))
        self.rate_hz     = int(rospy.get_param("~rate_hz", 100))

        self.left_motor= CanMotorController(
            self.ifname , motor_id= self.left_id, motor_type="AK70_10_V1p1"
            )
        self.right_motor= CanMotorController(
            self.ifname , motor_id= self.right_id, motor_type="AK70_10_V1p1"
            )
        
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

        rospy.loginfo("Entering MIT mode...")
        self.left_motor.disable_motor()
        self.left_motor.enable_motor()
        self.left_motor.send_rad_command(0.0, 0, 0.0, 0, 0)
        if self.use_right:
            self.right_motor.disable_motor()
            self.right_motor.enable_motor()
            self.right_motor.send_rad_command(0.0, 0, 0.0, 0, 0)

        time.sleep(0.05)
        self.zero_on_start = True
        if self.zero_on_start:
            print("ZERO POS")
            self.setZeroPosition(self.left_motor)
            if self.use_right:
                self.setZeroPosition(self.right_motor)
            time.sleep(0.05)

        rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=10)

        self.th = threading.Thread(target=self.loop)
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
            wl_ref = (2.0*self.v_ref - self.w_ref*self.base_L) / (2.0*self.wheel_R) * 1.07
            wr_ref = (2.0*self.v_ref + self.w_ref*self.base_L) / (2.0*self.wheel_R)

            def ramp(ref, cur):
                d = ref - cur
                if d >  step: return cur + step
                if d < -step: return cur - step
                return ref

            self.wl = ramp(wl_ref, self.wl)
            self.wr = ramp(wr_ref, self.wr)
            if time.time()-self.prev_time >= 0.5:
            
            #  print('left', self.wl, 'right', self.wr)
             self.prev_time = time.time()

            #while True:
             #pos, vel, curr = self.left_motor.send_rad_command(0, 90, 0, 2, 0)
             #print("left_pos :",pos)
             #time.sleep(1)

            try:
                try:
                 self.left_motor_state.pos, self.left_motor_state.vel, self.left_motor_state.tau  =self.left_motor.send_rad_command(0.0, self.dir_left * self.wl, 0.0, self.kd_vel, self.tff)
                 self.left_motor_state.vel = self.dir_left * self.left_motor_state.vel
                 self.left_motor_state.stamp = rospy.Time.now()
                except Exception as e:
                 print("SEND ERR:", e)

                if self.use_right:
                    self.right_motor_state.pos, self.right_motor_state.vel, self.right_motor_state.tau =self.right_motor.send_rad_command(0.0, self.dir_right * self.wr, 0.0,self.kd_vel, self.tff)
                    self.right_motor_state.vel = self.dir_right * self.right_motor_state.vel
                    self.right_motor_state.stamp = rospy.Time.now()
                # rospy.loginfo("left_pos :")
                # rospy.loginfo(self.left_motor_state.pos)
                # rospy.loginfo("right_pos :")
                # rospy.loginfo(self.right_motor_state.pos)


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
        ODOM_SCALE_CORRECTION =2
        deadzone = 0.08
        if abs(self.left_motor_state.vel)<deadzone:
            self.left_motor_state.vel=0.0
        if abs(self.right_motor_state.vel)<deadzone:
            self.right_motor_state.vel=0.0
        v_l = self.left_motor_state.vel  * self.wheel_R* ODOM_SCALE_CORRECTION
        v_r = self.right_motor_state.vel * self.wheel_R* ODOM_SCALE_CORRECTION

        v  = 0.5 * (v_r + v_l)
        wz = (v_r - v_l) / self.base_L

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
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


