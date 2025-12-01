import socket
import struct
import threading
import time
import os
import subprocess

class cubemars():
    def __init__(self, can_interface="can1", motor_amount=10):
        self.can_interface = can_interface
        self.motor_amount = motor_amount
        self.motor_spd = 50000  # ERPM
        self.motor_acc = 25000   # ERPM/s²

    def comm_can_set_pos_spd(self, controller_id: int, pos_deg: float, spd_erpm: int, acc_erpm_s2: int):
        multiple=15
        if controller_id == 2 or controller_id ==7:
            multiple=13
        elif controller_id == 3 or  controller_id ==8:
            multiple=11
        elif controller_id == 5 or  controller_id ==10:
            multiple=1

        motor_deg_delta = pos_deg * multiple
        pos_val = int(motor_deg_delta * 10000.0)
        spd_val = int(spd_erpm / 10.0)
        acc_val = int(acc_erpm_s2 / 10.0)

        buffer = bytearray(8)
        struct.pack_into(">i", buffer, 0, pos_val)
        struct.pack_into(">h", buffer, 4, spd_val)
        struct.pack_into(">h", buffer, 6, acc_val)

        CAN_PACKET_SET_POS_SPD = 6
        ext_id = (CAN_PACKET_SET_POS_SPD << 8) | controller_id
        can_id = ext_id | socket.CAN_EFF_FLAG
        frame = struct.pack("=IB3x8s", can_id, 8, buffer)
        return frame

    def motor_set_origin(self, motor_id):
        CAN_PACKET_SET_ORIGIN_HERE = 5
        buffer_temp = bytes(0x00)
        zero_ext_id = (CAN_PACKET_SET_ORIGIN_HERE << 8) | motor_id
        extended_frame = zero_ext_id | socket.CAN_EFF_FLAG
        frame = struct.pack("=IB3x8s", extended_frame, 0, buffer_temp)
        self.send_frame(frame)
#        print(f"📤 모터 {motor_id} 원점 설정 명령 전송")  
        
    def motor_control(self, motor_id, angle, spd=10000, acc=5000):
        frame = self.comm_can_set_pos_spd(
            motor_id,
            angle,
            spd,
            acc
        )
        self.send_frame(frame)

    def send_frame(self, frame):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.can_interface, ))
            self.sock.send(frame)
            self.sock.close()
        except Exception as e:
            print("Motor Control ERR: ", e)
            self.stop()

    def input_thread(self):
        while True:
            try:
                user_input = input("모터ID, 각도 입력 : ").strip()
                if not user_input:
                    continue
                parts = user_input.split()
                if len(parts) != 2:
                    continue
                motor_id = int(parts[0])
                delta = float(parts[1])
                self.angle_array[motor_id - 1] = delta
            except Exception as e:
                print(e)
                #pass

    def stop(self):
        self.sock.close()

    def init_motor(self):
        for motor_id in range(1, self.motor_amount + 1):
            self.motor_set_origin(motor_id)
            time.sleep(0.005)
            self.motor_set_origin(motor_id)
            time.sleep(0.005)
   
if __name__ == "__main__":
    try:
     prev_time=0
     controller = cubemars()
     #max = 40000
     controller.spd = 10000
     controller.acc = 5000

     for motor_id in range(1, controller.motor_amount + 1):
      controller.motor_set_origin(motor_id)
      time.sleep(0.1)
     time.sleep(2)

     input_thread_obj = threading.Thread(target=controller.input_thread, args = ())
     input_thread_obj.daemon=True
     input_thread_obj.start()

     
     while True:
       for motor_id in range(1, controller.motor_amount +1):
         controller.motor_control(motor_id, controller.angle_array[motor_id-1], controller.spd, controller.acc)
         time.sleep(0.001)
       if time.time() - prev_time>=0.1:
         prev_time = time.time()
         print(controller.angle_array)
       #pass
       
    except Exception as e:  
      controller.stop()
      print(e)

