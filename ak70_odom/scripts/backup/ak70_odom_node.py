#!/usr/bin/env python
# -*- coding: ascii -*-

import time
import math
import can
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from pythoncan_canmotorlib import *

# ===== MIT motor protocol scale (AK70/AK80) =====
P_MIN, P_MAX = -12.5, 12.5      # rad
V_MIN, V_MAX = -45.0, 45.0      # rad/s
T_MIN, T_MAX = -25.0, 25.0      # Nm

def decode_feedback(data):
    """
    data: 8-byte bytearray from motor (MIT feedback)
    returns (pos_rad, vel_rad_s, torque_Nm)
    layout:
      pos 16-bit, vel 12-bit, cur 12-bit (big-endian bit packing)
    """
    if len(data) != 8:
        raise ValueError("feedback payload must be 8 bytes")
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | (data[3] >> 4)
    t_int = ((data[3] & 0x0F) << 8) | data[4]

    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    t = uint_to_float(t_int, T_MIN, T_MAX, 12)
    return (p, v, t)

class WheelState(object):
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0
        self.tau = 0.0
        self.stamp = rospy.Time(0)

class AK70OdomNode(object):
    def __init__(self):
        rospy.init_node("ak70_odom")

        # parameters
        self.interface   = rospy.get_param("~can_interface", "can0")
        self.left_id     = int(rospy.get_param("~left_id",  "0x01"), 0)
        self.right_id    = int(rospy.get_param("~right_id", "0x02"), 0)
        self.wheel_R     = float(rospy.get_param("~wheel_radius", 0.06))
        self.base_L      = float(rospy.get_param("~wheel_base",   0.399))
        self.frame_odom  = rospy.get_param("~odom_frame", "odom")
        self.frame_base  = rospy.get_param("~base_frame", "base_link")
        self.pub_joint   = bool(rospy.get_param("~publish_joint_states", True))
        self.rate_hz     = int(rospy.get_param("~rate_hz", 100))
        self.dir_left    = int(rospy.get_param("~left_dir",  +1))
        self.dir_right   = int(rospy.get_param("~right_dir", -1))

        # CAN
        #self.bus = can.interface.Bus(bustype='socketcan', channel=self.interface)
        self.left_motor= CanMotorController(
            self.interface , motor_id= self.left_id, motor_type="AK70_10_V1p1"
            )
        self.right_motor= CanMotorController(
            self.interface , motor_id= self.right_id, motor_type="AK70_10_V1p1"
            )

        # states
        self.left = WheelState()
        self.right = WheelState()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last = rospy.Time.now()

        # publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10) if self.pub_joint else None

        rospy.loginfo("AK70 odom node started (left_id=0x%X, right_id=0x%X)", self.left_id, self.right_id)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                #"Motor {} Status: Pos: {}, Vel: {}, Torque: {}"
                self.left.pos, self.left.vel, self.left.tau = self.left_motor.enable_motor() # 이게 모터 상태가져오는 함수
                self.left.vel = self.dir_left * self.left.vel
                self.left.stamp = rospy.Time.now()

                self.right.pos, self.right.vel, self.right.tau = self.right_motor.enable_motor()
                self.right.vel = self.dir_right * self.right.vel
                self.right.stamp = rospy.Time.now()

            except can.CanError as e:
                rospy.logwarn("CAN rx error: %s", e)
            self.integrate_odom()
            rate.sleep()


    def integrate_odom(self):
        now = rospy.Time.now()
        dt = (now - self.last).to_sec()
        if dt <= 0.0:
            return
        self.last = now

        v_l = self.left.vel  * self.wheel_R
        v_r = self.right.vel * self.wheel_R

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
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
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
            js.position = [self.left.pos, self.right.pos]
            js.velocity = [self.left.vel, self.right.vel]
            self.joint_pub.publish(js)

    def shutdown(self):
        try:
            self.bus.shutdown()
        except:
            pass

if __name__ == "__main__":
    node = AK70OdomNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


