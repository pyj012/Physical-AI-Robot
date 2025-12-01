#!/usr//bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import sys

# --- '양쪽 바퀴 1000 ERPM 직진' 테스트에 대한 이론값 ---
EXPECTED_VX = 0.05  # 수정된 값
EXPECTED_WZ = 0.0   # 수정된 값 (직진이므로 0)
# ---

def odom_callback(msg):
    """/odom 토픽을 받을 때마다 호출되는 콜백 함수"""
    
    current_vx = msg.twist.twist.linear.x
    current_wz = msg.twist.twist.angular.z
    
    vx_error = abs(current_vx - EXPECTED_VX)
    wz_error = abs(current_wz - EXPECTED_WZ)

    # 1초에 한 번씩만 로그를 출력합니다.
    log_message = (
        "Expected [vx: {:.4f}, wz: {:.4f}] | "
        "Measured [vx: {:.4f}, wz: {:.4f}] | "
        "Error [vx: {:.4f}, wz: {:.4f}]"
    ).format(EXPECTED_VX, EXPECTED_WZ, current_vx, current_wz, vx_error, wz_error)
    
    rospy.loginfo_throttle(1.0, log_message) # 1.0초 간격으로 로그 출력

def odom_checker():
    """메인 함수"""
    rospy.init_node('odom_checker', anonymous=True)
    
    rospy.loginfo("Odometry checker node started.")
    rospy.loginfo("Waiting for /odom topic...")
    
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    print("--- Odometry Calculation Test (Straight Motion) ---")
    print("Checks if /odom values match theoretical values for a 1000 ERPM straight command.")
    print("-" * 70)
    
    try:
        odom_checker()
    except rospy.ROSInterruptException:
        print("\nTest stopped.")