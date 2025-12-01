# -*- coding: utf-8 -*-
import can
import time
import math
from bitstring import BitArray

# --- 모터 파라미터 및 상수 (기존과 동일) ---
legitimate_motors = [
    "AK80_6_V1", "AK80_6_V1p1", "AK80_6_V2", "AK80_9_V1p1",
    "AK80_9_V2", "AK70_10_V1p1", "AK10_9_V1p1"
]
# Constants for conversion
# Working parameters for AK80-6 V1.0 firmware
AK80_6_V1_PARAMS = {
                "P_MIN" : -95.5,
                "P_MAX" : 95.5,
                "V_MIN" : -45.0,
                "V_MAX" : 45.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : -1
                }

# Working parameters for AK80-6 V1.1 firmware
AK80_6_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -22.5,
                "V_MAX" : 22.5,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -12.0,
                "T_MAX" : 12.0,
                "AXIS_DIRECTION" : -1
                }

# Working parameters for AK80-6 V2.0 firmware
AK80_6_V2_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -76.0,
                "V_MAX" : 76.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500.0,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -12.0,
                "T_MAX" : 12.0,
                "AXIS_DIRECTION" : 1
                }

# Working parameters for AK80-9 V1.1 firmware
AK80_9_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -22.5,
                "V_MAX" : 22.5,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : 1
                }

# Working parameters for AK80-9 V2.0 firmware
AK80_9_V2_PARAMS = {
                    "P_MIN" : -12.5,
                    "P_MAX" : 12.5,
                    "V_MIN" : -25.64,
                    "V_MAX" : 25.64,
                    "KP_MIN" : 0.0,
                    "KP_MAX" : 500.0,
                    "KD_MIN" : 0.0,
                    "KD_MAX" : 5.0,
                    "T_MIN" : -18.0,
                    "T_MAX" : 18.0,
                    "AXIS_DIRECTION" : 1
                    }

#  Working parameters for AK70-10 V1.1 firmware
AK70_10_V1p1_params = {
                      "P_MIN" :  -12.5,
                      "P_MAX" :  12.5,
                      "V_MIN" :  -50,
                      "V_MAX" :  50,
                      "KP_MIN" :  0,
                      "KP_MAX" :  500,
                      "KD_MIN" :  0,
                      "KD_MAX" :  5,
                      "T_MIN" :  -24.0,
                      "T_MAX" :  24.0,
                      "AXIS_DIRECTION" :  1
                    }

# Working parameters for AK10-9 V1.1 firmware
AK10_9_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -50.0,
                "V_MAX" : 50.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -65.0,
                "T_MAX" : 65.0,
                "AXIS_DIRECTION" : -1
                }

maxRawPosition = 2**16 - 1
maxRawVelocity = 2**12 - 1
maxRawTorque = 2**12 - 1
maxRawKp = 2**12 - 1
maxRawKd = 2**12 - 1
maxRawCurrent = 2**12 - 1

dt_sleep = 0.0001
set_zero_sleep = 1.5

def float_to_uint(x, x_min, x_max, num_bits):
    span = x_max - x_min
    offset = x_min
    bit_range = (1 << num_bits) - 1
    return int(((x - offset) * bit_range) / span)

def uint_to_float(x_int, x_min, x_max, num_bits):
    span = x_max - x_min
    offset = x_min
    bit_range = (1 << num_bits) - 1
    return ((x_int * span) / bit_range) + offset


class CanMotorController:
    """
    python-can 라이브러리를 사용하여 모터를 제어하는 클래스.
    """
    _bus = None # 클래스 변수로 CAN 버스를 관리

    def __init__(self, can_channel="can0", motor_id=0x01, motor_type="AK80_6_V1p1", bustype='socketcan'):
        self.motor_id = motor_id
        self.motorParams = self._select_motor_params(motor_type)
        
        # CAN 버스가 아직 초기화되지 않았다면, 클래스 변수로 초기화
        if CanMotorController._bus is None:
            try:
                CanMotorController._bus = can.interface.Bus(channel=can_channel, bustype=bustype)
            except can.CanError as e:
                raise
        
        # BitArray 초기화
        self._p_des_BitArray = BitArray(uint=0, length=16)
        self._v_des_BitArray = BitArray(uint=0, length=12)
        self._kp_BitArray = BitArray(uint=0, length=12)
        self._kd_BitArray = BitArray(uint=0, length=12)
        self._tau_BitArray = BitArray(uint=0, length=12)
        self._cmd_bytes = BitArray(uint=0, length=64)

    def _select_motor_params(self, motor_type):
        assert motor_type in legitimate_motors, "Motor Type not in list of accepted motors."
        # 실제 사용 시 필요한 모든 모터 파라미터를 여기에 추가해야 합니다.
        if motor_type == "AK80_6_V1p1":
            return AK80_6_V1p1_PARAMS
        # ... 다른 모터 타입에 대한 elif 문 추가 ...
        else: # 기본값
            return AK80_6_V1p1_PARAMS

    def _send_and_receive(self, data_bytes, timeout=0.1):
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data_bytes,
            is_extended_id=False
        )
        try:
            self._bus.send(msg)
            time.sleep(dt_sleep)
            
            # 응답 메시지 수신
            response = self._bus.recv(timeout)
            
            # 응답이 없거나 ID가 다르면 에러 처리
            if response is None:
                print("Timeout: No response from motor.")
                return None
            if response.arbitration_id != self.motor_id:
                return None
                
            return response.data
        except can.CanError as e:
            return None

    def enable_motor(self):
        motor_status_data = self._send_and_receive(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC")
        if motor_status_data:
            pos, vel, curr = self.decode_and_convert(motor_status_data)
            return pos, vel, curr
        return 0.0, 0.0, 0.0

    def disable_motor(self):
        motor_status_data = self._send_and_receive(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD")
        if motor_status_data:
            pos, vel, curr = self.decode_and_convert(motor_status_data)
            print("Motor Disabled.")
            return pos, vel, curr
        print("Error Disabling Motor!")
        return 0.0, 0.0, 0.0
    
    def set_zero_position(self):
        print("Setting zero position...")
        motor_status_data = self._send_and_receive(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE")
        time.sleep(set_zero_sleep)
        if motor_status_data:
            pos, vel, curr = self.decode_and_convert(motor_status_data)
            print("Zero position set.")
            return pos, vel, curr
        print("Error setting zero position!")
        return 0.0, 0.0, 0.0
        
    def send_rad_command(self, p_des_rad, v_des_rad, kp, kd, tau_ff):
        p_des_rad = min(max(self.motorParams["P_MIN"], p_des_rad), self.motorParams["P_MAX"])
        v_des_rad = min(max(self.motorParams["V_MIN"], v_des_rad), self.motorParams["V_MAX"])
        kp = min(max(self.motorParams["KP_MIN"], kp), self.motorParams["KP_MAX"])
        kd = min(max(self.motorParams["KD_MIN"], kd), self.motorParams["KD_MAX"])
        tau_ff = min(max(self.motorParams["T_MIN"], tau_ff), self.motorParams["T_MAX"])

        raw_pos, raw_vel, raw_kp, raw_kd, raw_tau = self.convert_physical_rad_to_raw(p_des_rad, v_des_rad, kp, kd, tau_ff)

        self._p_des_BitArray.uint = raw_pos
        self._v_des_BitArray.uint = raw_vel
        self._kp_BitArray.uint = raw_kp
        self._kd_BitArray.uint = raw_kd
        self._tau_BitArray.uint = raw_tau
        
        cmd_bit_array = self._p_des_BitArray.bin + self._v_des_BitArray.bin + self._kp_BitArray.bin + self._kd_BitArray.bin + self._tau_BitArray.bin
        self._cmd_bytes.bin = cmd_bit_array
        
        motor_status_data = self._send_and_receive(self._cmd_bytes.tobytes())

        if motor_status_data:
            pos, vel, curr = self.decode_and_convert(motor_status_data)
            return pos, vel, curr
        
        print("Error sending rad command!")
        return 0.0, 0.0, 0.0

    def send_deg_command(self, p_des_deg, v_des_deg, kp, kd, tau_ff):
        p_des_rad = math.radians(p_des_deg)
        v_des_rad = math.radians(v_des_deg)
        pos_rad, vel_rad, curr = self.send_rad_command(p_des_rad, v_des_rad, kp, kd, tau_ff)
        return math.degrees(pos_rad), math.degrees(vel_rad), curr

    def decode_and_convert(self, data_frame):
        """디코딩과 물리량 변환을 한 번에 처리하는 헬퍼 함수"""
        raw_p, raw_v, raw_t = self.decode_motor_status(data_frame)
        pos, vel, curr = self.convert_raw_to_physical_rad(raw_p, raw_v, raw_t)
        return pos, vel, curr

    def decode_motor_status(self, data_frame):
        """
        [수정된 부분]
        모터의 응답 메시지를 올바른 비트 위치에 따라 파싱합니다.
        프로토콜: [모터ID 8비트] [위치 16비트] [속도 12비트] [토크 12비트]
        """
        data_bits = BitArray(bytes=data_frame)
        
        # 첫 8비트는 Motor ID이므로 건너뛰고 파싱합니다.
        # 이전 코드: data_bits[0:16] -> 잘못된 위치
        position_raw = data_bits[8:24].uint

        # 이전 코드: data_bits[16:28] -> 잘못된 위치
        velocity_raw = data_bits[24:36].uint

        # 이전 코드: data_bits[28:40] -> 잘못된 위치
        current_raw = data_bits[36:48].uint
        
        return position_raw, velocity_raw, current_raw

    def convert_raw_to_physical_rad(self, position_raw, velocity_raw, current_raw):
        pos = uint_to_float(position_raw, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16)
        vel = uint_to_float(velocity_raw, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12)
        curr = uint_to_float(current_raw, self.motorParams["T_MIN"], self.motorParams["T_MAX"], 12)
        
        pos *= self.motorParams["AXIS_DIRECTION"]
        vel *= self.motorParams["AXIS_DIRECTION"]
        curr *= self.motorParams["AXIS_DIRECTION"]
        
        return pos, vel, curr

    def convert_physical_rad_to_raw(self, p_des_rad, v_des_rad, kp, kd, tau_ff):
        p_des_rad *= self.motorParams["AXIS_DIRECTION"]
        v_des_rad *= self.motorParams["AXIS_DIRECTION"]
        tau_ff *= self.motorParams["AXIS_DIRECTION"]
        
        raw_p = float_to_uint(p_des_rad, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16)
        raw_v = float_to_uint(v_des_rad, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12)
        raw_kp = float_to_uint(kp, self.motorParams["KP_MIN"], self.motorParams["KP_MAX"], 12)
        raw_kd = float_to_uint(kd, self.motorParams["KD_MIN"], self.motorParams["KD_MAX"], 12)
        raw_tau = float_to_uint(tau_ff, self.motorParams["T_MIN"], self.motorParams["T_MAX"], 12)
        
        return raw_p, raw_v, raw_kp, raw_kd, raw_tau