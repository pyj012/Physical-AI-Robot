import time
import smbus
import board
import busio
import serial
from threading import Thread
import json
import traceback
from lib.protocol_lib import *
# from protocol_lib import *
LEFT_INDEX =0x30
LEFT_MIDDLE =0x31
LEFT_RING =0x32
LEFT_LITTLE =0x33
LEFT_THUMB =0x34
LEFT_THUMB_J =0x35
LEFT_WRIST = 0x37

RIGHT_INDEX =0x38
RIGHT_MIDDLE =0x39
RIGHT_RING =0x3A
RIGHT_LITTLE =0x3B
RIGHT_THUMB =0x3C
RIGHT_THUMB_J =0x3D
RIGHT_WRIST = 0x3F

RIGHT_GRIP = 180
RIGHT_REALESE =80
RIGHT_THUMB_J_GRIP = 40
RIGHT_THUMB_J_REALESE = 120

LEFT_GRIP = 30
LEFT_REALESE = 140
LEFT_THUMB_J_GRIP = 90
LEFT_THUMB_J_REALESE = 15
WRIST_MIN = 40
WRIST_MAX = 140
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class serial_servo():
    def __init__(self):
        self.smartprotocol = smartprotocol()
        self.ser=None
        while True:
            try:
                print("WAIT FOR CONNECT SERVO")
                self.ser = serial.Serial(
                    port = '/dev/ttyUSB0', 
                    baudrate=115200, 
                    parity='N',
                    stopbits=1,
                    bytesize=8,
                    timeout=0
                    )
                if(self.ser.isOpen()):
                    print("OPEN!")
                    break
            except Exception as e:
                print("usb not ready")
                time.sleep(3)
                pass

        time.sleep(3)
        self.rl = ReadLine(self.ser)
        self.left_hand_deg ={LEFT_INDEX:LEFT_REALESE,LEFT_MIDDLE:LEFT_REALESE,LEFT_RING:LEFT_REALESE,LEFT_LITTLE:LEFT_REALESE,LEFT_THUMB:LEFT_REALESE,LEFT_THUMB_J:LEFT_THUMB_J_REALESE,0x36:0,LEFT_WRIST:90}
        self.right_hand_deg ={RIGHT_INDEX:RIGHT_REALESE,RIGHT_MIDDLE:RIGHT_REALESE,RIGHT_RING:RIGHT_REALESE,RIGHT_LITTLE:RIGHT_REALESE,RIGHT_THUMB:RIGHT_REALESE,RIGHT_THUMB_J:RIGHT_THUMB_J_REALESE,0x3E:90,RIGHT_WRIST:90}
        self.motion_planning_min=-90
        self.motion_planning_max=90
        self.cnt=0
        self.prev_time=0
        self.delay_time=0.1
        self.right_wave_seq=0
        self.prev_send_time = 0
        self.send_feq = 0.05

        self.left_grip_seq=0
        self.right_grip_seq=0

        self.left_hand_seq=0
        self.right_hand_seq = 0 
        self.left_realese_prev_time = 0
        self.right_realese_prev_time = 0
        self.left_grip_state=False
        self.prev_left_grip_state = False
        self.right_grip_state=False
        self.prev_right_grip_state=False
        self.user_input=0

        # self.sendThred = Thread(target=self.send, args = ())
        # self.sendThred.daemon = True
        # self.sendThred.start()
        # self.inputThred = Thread(target=self.userinput, args = ())
        # self.inputThred.daemon = True
        # self.inputThred.start()
        # self.readThred = Thread(target=self.read, args = ())
        # self.readThred.daemon = True
        # self.readThred.start()
        # self.work()

    # def work(self):
    #     self.init_motor()
        # time.sleep(3)
        # while True:
        #     try:
                # self.left_hand_deg[LEFT_INDEX]=self.user_input
                # self.left_hand_deg[LEFT_MIDDLE]=self.user_input
                # self.left_hand_deg[LEFT_RING]=self.user_input
                # self.left_hand_deg[LEFT_LITTLE]=self.user_input
                # self.left_hand_deg[LEFT_THUMB]=self.user_input
                # self.left_hand_deg[LEFT_THUMB_J]=self.user_input

                # self.right_hand_deg[RIGHT_INDEX]=self.user_input
                # self.right_hand_deg[RIGHT_MIDDLE]=self.user_input
                # self.right_hand_deg[RIGHT_RING]=self.user_input
                # self.right_hand_deg[RIGHT_LITTLE]=self.user_input
                # self.right_hand_deg[RIGHT_THUMB]=self.user_input
                # self.right_hand_deg[RIGHT_THUMB_J]=self.user_input
                # self.right_hand_deg[RIGHT_WRIST]=self.user_input
                # self.left_hand_deg[LEFT_WRIST]=self.user_input
                # self.grip_test()
            # except Exception as e:
            #     print("SERVO SEND ERR", e)
            #     pass
    def grip_test(self):
        if self.user_input== 0:
            # self.right_hand_deg[RIGHT_WRIST]=5
            # self.left_hand_deg[LEFT_WRIST]=5
            self.GRIP("L",False)
            self.GRIP("R",False)

        elif self.user_input ==1:
            # self.right_hand_deg[RIGHT_WRIST]=175
            # self.left_hand_deg[LEFT_WRIST]=175
            self.GRIP("L",True)
            self.GRIP("R",True)
            
    def start_servo(self):
        pass
        # self.inputThred.start()
        # self.sendThred.start()

    def init_motor(self):
        for servo_id in self.left_hand_deg:
            if servo_id ==LEFT_THUMB_J:
                self.left_hand_deg[servo_id]=LEFT_THUMB_J_REALESE
            elif servo_id ==LEFT_WRIST:
                self.left_hand_deg[servo_id]=90
            else:
                self.left_hand_deg[servo_id]=LEFT_REALESE

        for servo_id in self.right_hand_deg:
            if servo_id ==RIGHT_THUMB_J:
                self.right_hand_deg[servo_id]=RIGHT_THUMB_J_REALESE
            elif servo_id ==RIGHT_WRIST:
                self.right_hand_deg[servo_id]=90
            else:
                self.right_hand_deg[servo_id]=RIGHT_REALESE
        
        self.angle()

    def GRIP(self, hand, state):
        if hand == "L":
            if state != self.prev_left_grip_state:
                self.prev_left_grip_state = state
                self.left_hand_seq=0

            if state:
                if self.left_hand_seq==0:
                    self.left_hand_deg[LEFT_THUMB_J]=LEFT_THUMB_J_GRIP
                    self.left_hand_deg[LEFT_THUMB]=LEFT_GRIP- 80
                    self.left_hand_deg[LEFT_INDEX]=LEFT_GRIP- 80
                    self.left_hand_deg[LEFT_MIDDLE]=LEFT_GRIP- 80
                    self.left_hand_deg[LEFT_RING]=LEFT_GRIP- 80
                    self.left_hand_deg[LEFT_LITTLE]=LEFT_GRIP - 80
                    self.left_hand_seq+=1
                    self.left_realese_prev_time= time.time()

                elif self.left_hand_seq==1:    
                    if time.time()-self.left_realese_prev_time >= 0.5:
                        self.left_hand_deg[LEFT_THUMB]=LEFT_GRIP
                        self.left_hand_deg[LEFT_INDEX]=LEFT_GRIP
                        self.left_hand_deg[LEFT_MIDDLE]=LEFT_GRIP
                        self.left_hand_deg[LEFT_RING]=LEFT_GRIP
                        self.left_hand_deg[LEFT_LITTLE]=LEFT_GRIP 
   
            else:
                if self.left_hand_seq==0:
                    self.left_hand_deg[LEFT_THUMB]=LEFT_REALESE
                    self.left_hand_deg[LEFT_RING]=LEFT_REALESE
                    self.left_hand_deg[LEFT_LITTLE]=LEFT_REALESE 
                    self.left_hand_deg[LEFT_INDEX]=LEFT_REALESE
                    self.left_hand_deg[LEFT_MIDDLE]=LEFT_REALESE

                    self.left_realese_prev_time= time.time()
                    self.left_hand_seq+=1

                elif self.left_hand_seq==1:                
                    if time.time()-self.left_realese_prev_time>=0.8:
                        self.left_hand_deg[LEFT_THUMB_J]=LEFT_THUMB_J_REALESE
                        self.left_realese_prev_time= time.time()
                        self.left_hand_seq = 0

        elif hand == "R":
            if state != self.prev_right_grip_state:
                self.prev_right_grip_state = state
                self.right_hand_seq=0

            if state:
                if self.right_hand_seq==0:
                    self.right_hand_deg[RIGHT_THUMB]=RIGHT_GRIP-100

                    self.right_realese_prev_time= time.time()
                    self.right_hand_seq+=1

                elif self.right_hand_seq==1:
                    if time.time()-self.right_realese_prev_time>=0.5:
                        self.right_hand_deg[RIGHT_THUMB_J]=RIGHT_THUMB_J_GRIP
                        self.right_hand_deg[RIGHT_INDEX]=RIGHT_GRIP-100
                        self.right_hand_deg[RIGHT_MIDDLE]=RIGHT_GRIP-100
                        self.right_hand_deg[RIGHT_RING]=RIGHT_GRIP-100
                        self.right_hand_deg[RIGHT_LITTLE]=RIGHT_GRIP-100
                        self.right_realese_prev_time= time.time()
                        self.right_hand_seq+=1

                elif self.right_hand_seq==2:
                    if time.time()-self.right_realese_prev_time>=0.2:
                        self.right_hand_deg[RIGHT_THUMB]=RIGHT_GRIP
                        self.right_hand_deg[RIGHT_INDEX]=RIGHT_GRIP
                        self.right_hand_deg[RIGHT_MIDDLE]=RIGHT_GRIP
                        self.right_hand_deg[RIGHT_RING]=RIGHT_GRIP
                        self.right_hand_deg[RIGHT_LITTLE]=RIGHT_GRIP
            else:
                if self.right_hand_seq==0:
                    self.right_hand_deg[RIGHT_THUMB]=RIGHT_REALESE
                    self.right_hand_deg[RIGHT_RING]=RIGHT_REALESE
                    self.right_hand_deg[RIGHT_LITTLE]=RIGHT_REALESE
                    self.right_hand_deg[RIGHT_INDEX]=RIGHT_REALESE
                    self.right_hand_deg[RIGHT_MIDDLE]=RIGHT_REALESE
                    self.right_realese_prev_time= time.time()
                    self.right_hand_seq+=1

                elif self.right_hand_seq==1:
                    if time.time()-self.right_realese_prev_time>=0.8:
                        self.right_realese_prev_time= time.time()
                        self.right_hand_deg[RIGHT_THUMB_J]=RIGHT_THUMB_J_REALESE

    def wave(self):
        if time.time()-self.prev_time>=self.delay_time:
            self.prev_time= time.time()
            self.right_hand_deg[RIGHT_WRIST]=90
            self.left_hand_deg[LEFT_WRIST]=90
            self.right_hand_deg[RIGHT_THUMB_J]=THUMB_J_REALESE
            self.left_hand_deg[LEFT_THUMB_J]=THUMB_J_GRIP
            if self.right_wave_seq==0:
                self.right_hand_deg[RIGHT_LITTLE]=GRIP
                self.left_hand_deg[LEFT_LITTLE]=GRIP

            elif self.right_wave_seq==1:
                self.right_hand_deg[RIGHT_RING]=GRIP
                self.left_hand_deg[LEFT_RING]=GRIP
            elif self.right_wave_seq==2:
                self.right_hand_deg[RIGHT_MIDDLE]=GRIP
                self.left_hand_deg[LEFT_MIDDLE]=GRIP
            elif self.right_wave_seq==3:
                self.right_hand_deg[RIGHT_INDEX]=GRIP
                self.left_hand_deg[LEFT_INDEX]=GRIP
            elif self.right_wave_seq==4:
                self.right_hand_deg[RIGHT_THUMB]=30
                self.left_hand_deg[LEFT_THUMB]=30
            elif self.right_wave_seq==5:
                self.right_hand_deg[RIGHT_LITTLE]=REALESE
                self.left_hand_deg[LEFT_LITTLE]=REALESE
            elif self.right_wave_seq==6:
                self.right_hand_deg[RIGHT_RING]=REALESE
                self.left_hand_deg[LEFT_RING]=REALESE
            elif self.right_wave_seq==7:
                self.right_hand_deg[RIGHT_MIDDLE]=REALESE
                self.left_hand_deg[LEFT_MIDDLE]=REALESE
            elif self.right_wave_seq==8:
                self.right_hand_deg[RIGHT_INDEX]=REALESE
                self.left_hand_deg[LEFT_INDEX]=REALESE
            elif self.right_wave_seq==9:
                self.right_hand_deg[RIGHT_THUMB]=REALESE
                self.left_hand_deg[LEFT_THUMB]=REALESE
                self.right_wave_seq=-1
            print(self.right_wave_seq)
            self.right_wave_seq+=1


    def map_range(self, x, in_min=0, in_max=180, out_min=0, out_max=124):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    
    def mapping_deg_to_servo(self, deg):
        servo_map_angle = 0
        servo_map_angle= self.map_range(deg,self.motion_planning_min, self.motion_planning_max,WRIST_MIN,WRIST_MAX)
        return servo_map_angle
    

    def angle(self):
        try:
            id_array=[]
            value_array=[]
            left_id_array = list(self.left_hand_deg.keys())
            left_value_array = list(self.left_hand_deg.values())
            right_id_array = list(self.right_hand_deg.keys())
            right_value_array = list(self.right_hand_deg.values())
            id_array.extend(left_id_array)
            id_array.extend(right_id_array)
            value_array.extend(left_value_array)
            value_array.extend(right_value_array)
            send_data = self.smartprotocol.contorol_motor(id_array, value_array)
            self.ser.write(send_data)
        except Exception as e:
            print(traceback.format_exc(e))
            print("servo angle err", e)

    def userinput(self):
        while True:
            self.user_input = int(input("angle : "))

    def send(self):
        while True:
            try:
                if time.time() - self.prev_send_time >= self.send_feq:
                    self.prev_send_time=time.time()
                    self.angle()
            except Exception as e:
                print("SERVO SEND ERR", e)
                pass

    def read(self):
        while True:
            try:
                print(self.rl.readline().decode(), end="")
            except Exception as e:
                print("SERVO READ ERR", e)
                pass

if __name__ == "__main__":
    servo = serial_servo()
    servo.init_motor()
    time.sleep(3)
    while True:
        servo.grip_test()
