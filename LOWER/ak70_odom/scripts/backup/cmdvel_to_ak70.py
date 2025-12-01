#!/usr/bin/env python
# -*- coding: ascii -*-

import time
import math
import threading
import rospy
from geometry_msgs.msg import Twist
import can  # python-can (socketcan backend)

# ===== AK70 limits =====
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.0, 45.0
T_MIN, T_MAX = -25.0, 25.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

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

class AK70Motor(object):
    def __init__(self, bus, can_id):
        self.bus = bus
        self.id = can_id

    def _send(self, data8):
        if not isinstance(data8, bytearray):
            data8 = bytearray(data8)
        if len(data8) != 8:
            raise ValueError("CAN payload must be 8 bytes")
        msg = can.Message(arbitration_id=self.id, data=data8, extended_id=False)
        self.bus.send(msg)

    def enter_mit(self):
        self._send(bytearray([0xFF]*7 + [0xFC]))
        time.sleep(0.01)

    def set_zero(self):
        self._send(bytearray([0xFF]*7 + [0xFE]))
        time.sleep(0.01)

    def disable(self):
        self._send(bytearray([0xFF]*7 + [0xFD]))
        time.sleep(0.01)

    def set_velocity(self, omega_rad_s, kd=0.5, torq_ff=0.0):
        if omega_rad_s > V_MAX:
            omega_rad_s = V_MAX
        if omega_rad_s < V_MIN:
            omega_rad_s = V_MIN
        self._send(encode_cmd(0.0, omega_rad_s, 0.0, kd, torq_ff))

class DiffDriveNode(object):
    def __init__(self):
        rospy.init_node("cmdvel_to_ak70")
        self.prev_time = 0
        self.ifname     = rospy.get_param("~can_interface", "can0")
        self.left_id    = int(rospy.get_param("~left_id",  "0x01"), 0)
        self.right_id   = int(rospy.get_param("~right_id", "0x02"), 0)
        self.R          = float(rospy.get_param("~wheel_radius", 0.06))
        self.L          = float(rospy.get_param("~wheel_base",   0.399))
        self.dir_left   = int(rospy.get_param("~left_dir",  +1))
        self.dir_right  = int(rospy.get_param("~right_dir", -1))
        self.rate_hz    = int(rospy.get_param("~rate_hz", 40))
        self.acc_limit  = float(rospy.get_param("~acc_limit", 5.0))
        self.kd_vel     = float(rospy.get_param("~kd_vel", 0.5))
        self.tff        = float(rospy.get_param("~torque_ff", 0.0))
        self.zero_on_start = bool(rospy.get_param("~zero_on_start", False))
        self.use_right  = bool(rospy.get_param("~use_right", True))

        self.bus = can.interface.Bus(bustype='socketcan', channel=self.ifname)

        self.left  = AK70Motor(self.bus, self.left_id)
        self.right = AK70Motor(self.bus, self.right_id)

        self.v_ref = 0.0
        self.w_ref = 0.0
        self.wl    = 0.0
        self.wr    = 0.0
        self.dt    = 1.0 / float(self.rate_hz)
        self.running = True

        rospy.loginfo("Entering MIT mode...")
        self.left.enter_mit()
        if self.use_right:
            self.right.enter_mit()
        time.sleep(0.05)
        if self.zero_on_start:
            self.left.set_zero()
            if self.use_right:
                self.right.set_zero()
            time.sleep(0.05)

        rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=10)

        self.th = threading.Thread(target=self.loop)
        self.th.daemon = True
        self.th.start()

    def on_cmdvel(self, msg):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z

    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        step = self.acc_limit * self.dt
        while not rospy.is_shutdown() and self.running:
            wl_ref = (2.0*self.v_ref - self.w_ref*self.L) / (2.0*self.R)*1.17
            wr_ref = (2.0*self.v_ref + self.w_ref*self.L) / (2.0*self.R)

            def ramp(ref, cur):
                d = ref - cur
                if d >  step: return cur + step
                if d < -step: return cur - step
                return ref

            self.wl = ramp(wl_ref, self.wl)
            self.wr = ramp(wr_ref, self.wr)
            if time.time()-self.prev_time >= 0.5:
            
             print('left', self.wl, 'right', self.wr)
             self.prev_time = time.time()
            try:
                self.left.set_velocity(self.dir_left * self.wl, kd=self.kd_vel, torq_ff=self.tff)
                if self.use_right:
                    self.right.set_velocity(self.dir_right * self.wr, kd=self.kd_vel, torq_ff=self.tff)
            except can.CanError as e:
                rospy.logwarn("CAN tx error: %s", e)
                time.sleep(0.01)

            rate.sleep()

    def shutdown(self):
        self.running = False
        try:
            self.th.join(1.0)
        except:
            pass
        try:
            self.left.set_velocity(0.0, kd=self.kd_vel, torq_ff=self.tff)
            if self.use_right:
                self.right.set_velocity(0.0, kd=self.kd_vel, torq_ff=self.tff)
        except:
            pass
        try:
            self.left.disable()
            if self.use_right:
                self.right.disable()
        except:
            pass
        try:
            self.bus.shutdown()
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


