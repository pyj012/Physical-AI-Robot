# -*- coding: utf-8 -*-

import can
import struct
import threading
import time
# can0  00000601   [8]  00 01 86 A0 03 E8 01 F4



class SmartMotorController():
    def __init__(self, can_interface="can0", motor_amount=10):
        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
            print("✅ CAN bus on '{}' initialized.".format(can_interface))
        except can.CanError as e:
            print("❌ Error initializing CAN bus on '{}': {}".format(can_interface, e))
            raise

        self.motor_amount = motor_amount
        self.angle_array = [0.0] * motor_amount
        # spd, acc 속성을 motor_spd, motor_acc로 명확하게 변경
        self.motor_spd = 10000
        self.motor_acc = 5000

    def comm_can_set_rpm(self, controller_id, rpm):
        """
        RPM 제어를 위한 can.Message 객체를 생성합니다. (아두이노 코드 기반)
        """
        # 1. RPM 모드 ID는 3입니다.
        CAN_PACKET_SET_RPM = 3
        ext_id = (CAN_PACKET_SET_RPM << 8) | controller_id

        # 2. 4바이트 버퍼를 생성하고, RPM 값을 빅 엔디안 4바이트 정수(">i")로 패킹합니다.
        buffer = bytearray(4)
        struct.pack_into(">i", buffer, 0, int(rpm))

        # 3. can.Message 객체를 생성하여 반환합니다.
        msg = can.Message(
            arbitration_id=ext_id,
            data=buffer,
            is_extended_id=True
        )
        return msg
    
    def comm_can_set_pos_spd(self, controller_id, pos_deg, spd_erpm, acc_erpm_s2):
        multiple = 1
        motor_deg_delta = pos_deg * multiple
        pos_val = int(motor_deg_delta * 10000.0)
        spd_val = int(spd_erpm / 10.0)
        acc_val = int(acc_erpm_s2 / 10.0)

        buffer = bytearray(8)
        # 빅 엔디안('>')에서 리틀 엔디안('<')으로 변경
        struct.pack_into(">i", buffer, 0, pos_val)
        struct.pack_into(">h", buffer, 4, spd_val)
        struct.pack_into(">h", buffer, 6, acc_val)
        CAN_PACKET_SET_POS_SPD = 6
        ext_id = (CAN_PACKET_SET_POS_SPD << 8) | controller_id
        msg = can.Message(
            arbitration_id=ext_id,
            data=buffer,
            is_extended_id=True
        )
        return msg
    
    def motor_set_rpm(self, controller_id, rpm):
        """생성된 RPM 제어 메시지를 CAN 버스로 전송합니다."""
        rpm_msg = self.comm_can_set_rpm(controller_id, rpm)
        try:
            self.bus.send(rpm_msg)
        except can.CanError as e:
            print("Error sending RPM command: {}".format(e))

    def motor_set_origin(self, motor_id):
        CAN_PACKET_SET_ORIGIN_HERE = 5
        ext_id = (CAN_PACKET_SET_ORIGIN_HERE << 8) | motor_id
        
        msg = can.Message(arbitration_id=ext_id, dlc=0, is_extended_id=True)
        
        print("🚀 Sending SET ORIGIN -> ID: {}".format(hex(ext_id)))
        self.bus.send(msg)
        
    def motor_set_pos_spd(self, motor_id, angle, spd, acc):
        msg = self.comm_can_set_pos_spd(motor_id, angle, spd, acc)
        data_hex = ' '.join('{:02X}'.format(b) for b in msg.data)
        self.bus.send(msg)

    def input_thread(self):
        while True:
            try:
                user_input = raw_input("모터ID, 각도 입력 (예: 1 90) : ").strip() # Python 2는 raw_input
                if not user_input or len(user_input.split()) != 2: continue
                
                parts = user_input.split()
                motor_id = int(parts[0])
                delta = float(parts[1])

                if 1 <= motor_id <= self.motor_amount:
                    self.angle_array[motor_id - 1] = delta
                    print("✅ Motor {} target angle set to {} degrees.".format(motor_id, delta))
                else:
                    print("⚠️  ID {} is out of range (1-{}).".format(motor_id, self.motor_amount))
            except Exception as e:
                print("Error in input: {}".format(e))

    def stop(self):
        if self.bus:
            self.bus.shutdown()
            print("🔌 CAN bus shut down.")
        
if __name__ == "__main__":
    controller = None
    try:
        controller = SmartMotorController(can_interface="can0", motor_amount=2)
        
        # spd, acc 값은 controller 객체의 속성을 직접 사용
        speed = 10000
        acceleration = 5000
        
        # 1. 원점 설정
        print("\n--- 1. Setting Origin for Motor 1 ---")
        controller.motor_set_origin(1)
        time.sleep(2)

        # 2. 단일 위치 제어 명령 테스트
        print("\n--- 2. Testing Position Control for Motor 1 ---")
        motor_id_to_test = 1
        target_angle = 10.0
        
        print("Moving motor {} to {} degrees...".format(motor_id_to_test, target_angle))
        controller.angle_array[1]=30
        
        print("\n✅ Test command sent. Check if the motor moved.")
        print("Now starting interactive mode...")
        
        # 3. 사용자 입력 루프 시작
        input_thread_obj = threading.Thread(target=controller.input_thread, args=())
        input_thread_obj.daemon = True
        input_thread_obj.start()

        while True:
            for i in range(1, 5000, 50):
                controller.motor_set_rpm(1, i)
                controller.motor_set_rpm(2, i)
                time.sleep(0.1)


    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        if controller:
            print("Returning all motors to 0 degrees before shutdown...")
            for i in range(1, controller.motor_amount + 1):
                controller.motor_control(i, 0, 0, 0)
                time.sleep(0.01)
            time.sleep(2)
            controller.stop()