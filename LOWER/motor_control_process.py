import socket
import struct
import threading
import time
import os
import subprocess
from lib.cubemars_lib import *
from lib.servo_control_lib import *
LEFT_GRIP =0x30
LEFT_WRIST = 0x32
RIGHT_GRIP =0x3F
RIGHT_WRIST = 0x3D

class SmartMotorController():
    def __init__(self, commandQ, grapQ):
        print("Loading Motor Control Process")
        load_start= time.time()
        self.commandQ = commandQ
        self.grapQ = grapQ
        self.motor_angle_dict={
            1: 0.0, 2: 0.0, 3: 0.0, 
            4: 0.0, 5: 0.0, 6: 0.0, 
            7: 0.0, 8: 0.0, 9: 0.0, 
            10: 0.0,11:0.0,12:0.0
            }
        self.cubemars_motor=cubemars(can_interface="can1", motor_amount=10)
        self.grip_state = 0
        self.motor_activate = True
        self.servo_activate = True
        self.servo_motor = None
        if self.servo_activate:
            self.servo_motor = serial_servo()
            self.servo_motor.init_motor()
            pass
        print("Load  Motor Control Complete : ",time.time()-load_start )
        self.work() 

    def work(self):
            prev_servo_time=0

            while True:
                #max = 40000
                self.spd = 40000
                self.acc = 60000
                try:
                    if self.commandQ.qsize()>0:
                        motor_angle_dict = self.commandQ.get()
                        self.motor_angle_dict.update(motor_angle_dict)

                        if self.servo_activate:
                            left_wrist_deg = self.servo_motor.mapping_deg_to_servo(motor_angle_dict[11])    
                            right_wrist_deg = self.servo_motor.mapping_deg_to_servo(motor_angle_dict[12])
                            self.servo_motor.left_hand_deg[LEFT_WRIST]=round(left_wrist_deg)
                            self.servo_motor.right_hand_deg[RIGHT_WRIST]=round(right_wrist_deg)
                            # print(self.servo_motor.right_hand_deg)
                            # self.servo_motor.left_hand_deg[0]=left_wrist
                            # self.servo_motor.right_hand_deg[0]=right_wrist

                    if self.grapQ.qsize()>0:
                        self.grip_state = self.grapQ.get()
                        if self.servo_activate:
                            self.servo_motor.grip(self.grip_state[0], self.grip_state[1])

                    if self.motor_activate:
                        for motor_id in range(1, 11):
                            self.cubemars_motor.motor_control(motor_id, self.motor_angle_dict[motor_id], self.spd, self.acc)
                            time.sleep(0.0001)

                    if time.time() - prev_servo_time>=0.025:
                        prev_servo_time = time.time()
                        if self.servo_activate:
                            self.servo_motor.angle()

                    time.sleep(0.02)
                except Exception as e:
                    print(traceback.format_exc(e))
                    print("MOTOR ERR:", e)
                    pass