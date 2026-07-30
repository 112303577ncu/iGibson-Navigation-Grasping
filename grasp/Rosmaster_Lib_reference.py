#!/usr/bin/env python3
# coding: utf-8
# SOURCE: Rosmaster_Lib v3.3.9 (from Jetson: /home/jetson/software/py_install/Rosmaster_Lib)
# Saved here for reference only — do not deploy this file to Jetson.

import struct
import time
import serial
import threading


# V3.3.9
class Rosmaster(object):
    __uart_state = 0

    def __init__(self, car_type=1, com="/dev/myserial", delay=.002, debug=False):
        # com = "COM30"
        # com="/dev/ttyTHS1"
        # com="/dev/ttyUSB0"
        # com="/dev/ttyAMA0"

        self.ser = serial.Serial(com, 115200)

        self.__delay_time = delay
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_RGB = 0x05
        self.FUNC_RGB_EFFECT = 0x06

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_REPORT_ICM_RAW = 0x0E

        self.FUNC_RESET_STATE = 0x0F

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_YAW_PID = 0x14
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23
        self.FUNC_ARM_OFFSET = 0x24

        self.FUNC_AKM_DEF_ANGLE = 0x30
        self.FUNC_AKM_STEER_ANGLE = 0x31

        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.CARTYPE_X3 = 0x01
        self.CARTYPE_X3_PLUS = 0x02
        self.CARTYPE_X1 = 0x04
        self.CARTYPE_R2 = 0x05

        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0

        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

        self.__encoder_m1 = 0
        self.__encoder_m2 = 0
        self.__encoder_m3 = 0
        self.__encoder_m4 = 0

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__battery_voltage = 0

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        self.__read_car_type = 0

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("Rosmaster Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")
        # 打开机械臂的扭矩力，避免6号舵机首次插上去读不到角度。
        self.set_uart_servo_torque(1)
        time.sleep(.002)

    def __del__(self):
        self.ser.close()
        self.__uart_state = 0
        print("serial Close!")

    def __parse_data(self, ext_type, ext_data):
        if ext_type == self.FUNC_REPORT_SPEED:
            self.__vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            gyro_ratio = 1 / 3754.9
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*-gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*-gyro_ratio
            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio
            mag_ratio = 1.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            gyro_ratio = 1 / 1000.0
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*gyro_ratio
            accel_ratio = 1 / 1000.0
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio
            mag_ratio = 1 / 1000.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0
        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.__encoder_m1 = struct.unpack('i', bytearray(ext_data[0:4]))[0]
            self.__encoder_m2 = struct.unpack('i', bytearray(ext_data[4:8]))[0]
            self.__encoder_m3 = struct.unpack('i', bytearray(ext_data[8:12]))[0]
            self.__encoder_m4 = struct.unpack('i', bytearray(ext_data[12:16]))[0]
        else:
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_val = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                if self.__debug:
                    print("FUNC_UART_SERVO:", self.__read_id, self.__read_val)
            elif ext_type == self.FUNC_ARM_CTRL:
                self.__read_arm[0] = struct.unpack('h', bytearray(ext_data[0:2]))[0]
                self.__read_arm[1] = struct.unpack('h', bytearray(ext_data[2:4]))[0]
                self.__read_arm[2] = struct.unpack('h', bytearray(ext_data[4:6]))[0]
                self.__read_arm[3] = struct.unpack('h', bytearray(ext_data[6:8]))[0]
                self.__read_arm[4] = struct.unpack('h', bytearray(ext_data[8:10]))[0]
                self.__read_arm[5] = struct.unpack('h', bytearray(ext_data[10:12]))[0]
                self.__read_arm_ok = 1
                if self.__debug:
                    print("FUNC_ARM_CTRL:", self.__read_arm)
            elif ext_type == self.FUNC_VERSION:
                self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_VERSION:", self.__version_H, self.__version_L)
            elif ext_type == self.FUNC_SET_MOTOR_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_MOTOR_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])
            elif ext_type == self.FUNC_SET_YAW_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_YAW_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])
            elif ext_type == self.FUNC_ARM_OFFSET:
                self.__arm_offset_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__arm_offset_state = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_ARM_OFFSET:", self.__arm_offset_id, self.__arm_offset_state)
            elif ext_type == self.FUNC_AKM_DEF_ANGLE:
                id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__akm_def_angle = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                self.__akm_readed_angle = True
                if self.__debug:
                    print("FUNC_AKM_DEF_ANGLE:", id, self.__akm_def_angle)
            elif ext_type == self.FUNC_SET_CAR_TYPE:
                car_type = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_car_type = car_type

    def __receive_data(self):
        self.ser.flushInput()
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            print("check sum error:", ext_len, ext_type, ext_data)

    def __request_data(self, function, param=0):
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        time.sleep(0.002)

    def __arm_convert_value(self, s_id, s_angle):
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value

    def __arm_convert_angle(self, s_id, s_value):
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle

    def __limit_motor_value(self, value):
        if value == 127:
            return 127
        elif value > 100:
           return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    # -------------------------------------------------------------------------
    # Threading
    # -------------------------------------------------------------------------

    # create_receive_threading() is REQUIRED before any get_* call.
    # NOT required for set_* / write calls — those write directly to serial.
    def create_receive_threading(self):
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                print("----------------create receive threading--------------")
                self.__uart_state = 1
                time.sleep(.05)
        except:
            print('---create_receive_threading error!---')
            pass

    # -------------------------------------------------------------------------
    # Set methods
    # -------------------------------------------------------------------------

    def set_auto_report_state(self, enable, forever=False):
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_auto_report_state error!---')
            pass

    def set_beep(self, on_time):
        try:
            if on_time < 0:
                print("beep input error!")
                return
            value = bytearray(struct.pack('h', int(on_time)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("beep:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_beep error!---')
            pass

    def set_pwm_servo(self, servo_id, angle):
        try:
            if servo_id < 1 or servo_id > 4:
                if self.__debug:
                    print("set_pwm_servo input invalid")
                return
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pwmServo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo error!---')
            pass

    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL,
                   int(angle_s1), int(angle_s2), int(angle_s3), int(angle_s4)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("all Servo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo_all error!---')
            pass

    def set_colorful_lamps(self, led_id, red, green, blue):
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB, id, r, g, b]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_lamps error!---')
            pass

    def set_colorful_effect(self, effect, speed=255, parm=255):
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB_EFFECT, eff, spe, par]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb_effect:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_effect error!---')
            pass

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_motor error!---')
            pass

    def set_car_run(self, state, speed, adjust=False):
        try:
            car_type = self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack('h', int(speed)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_CAR_RUN,
                   car_type, int(state & 0xff), t_speed[0], t_speed[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_run:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_run error!---')
            pass

    def set_car_motion(self, v_x, v_y, v_z):
        '''
        input range:
        X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
        X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
        R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]
        '''
        try:
            vx_parms = bytearray(struct.pack('h', int(v_x * 1000)))
            vy_parms = bytearray(struct.pack('h', int(v_y * 1000)))
            vz_parms = bytearray(struct.pack('h', int(v_z * 1000)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__CAR_TYPE,
                   vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motion:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_motion error!---')
            pass

    def set_pid_param(self, kp, ki, kd, forever=False):
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                print("PID value must be:[0, 10.00]")
                return
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.append(kp_params[0])
            cmd.append(kp_params[1])
            cmd.append(ki_params[0])
            cmd.append(ki_params[1])
            cmd.append(kd_params[0])
            cmd.append(kd_params[1])
            cmd.append(state)
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pid:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_pid_param error!---')
            pass

    def set_car_type(self, car_type):
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_type:", cmd)
            time.sleep(.1)
        else:
            print("set_car_type input invalid")

    # -------------------------------------------------------------------------
    # CRITICAL: set_uart_servo_angle_array
    # angle_s ranges: S1-S4=[0,180], S5=[0,270], S6=[0,180]
    # If ANY angle is out of range → silent fail: prints "angle_s input error!" and returns.
    # run_time unit: milliseconds, clamped to [0, 2000].
    # Does NOT require create_receive_threading().
    # -------------------------------------------------------------------------
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        try:
            if not self.__arm_ctrl_enable:
                return
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                print("set uart servo input error")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xff
            value = bytearray(struct.pack('h', int(pulse_value)))
            r_time = bytearray(struct.pack('h', int(run_time)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_UART_SERVO,
                   s_id, value[0], value[1], r_time[0], r_time[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo:", servo_id, int(pulse_value), cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo error!---')
            pass

    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        try:
            if s_id == 1:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_1 set error!")
            elif s_id == 2:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_2 set error!")
            elif s_id == 3:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_3 set error!")
            elif s_id == 4:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_4 set error!")
            elif s_id == 5:
                if 0 <= s_angle <= 270:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_5 set error!")
            elif s_id == 6:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_6 set error!")
        except:
            print('---set_uart_servo_angle error! ID=%d---' % s_id)
            pass

    def set_uart_servo_id(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("servo id input error!")
                return
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_ID, int(servo_id)]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_id:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_id error!---')
            pass

    def set_uart_servo_torque(self, enable):
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_torque:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_torque error!---')
            pass

    def set_uart_servo_ctrl_enable(self, enable):
        if enable:
            self.__arm_ctrl_enable = True
        else:
            self.__arm_ctrl_enable = False

    # CRITICAL NOTE: angle validation for set_uart_servo_angle_array:
    #   S1-S4: must be in [0, 180] — if not, entire call silently fails (no move, no exception)
    #   S5:    must be in [0, 270]
    #   S6:    must be in [0, 180]
    # Default home: [90, 90, 90, 90, 90, 180]
    def set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500):
        try:
            if not self.__arm_ctrl_enable:
                return
            if 0 <= angle_s[0] <= 180 and 0 <= angle_s[1] <= 180 and 0 <= angle_s[2] <= 180 and \
                    0 <= angle_s[3] <= 180 and 0 <= angle_s[4] <= 270 and 0 <= angle_s[5] <= 180:
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                temp_val = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    temp_val[i] = self.__arm_convert_value(i + 1, angle_s[i])

                value_s1 = bytearray(struct.pack('h', int(temp_val[0])))
                value_s2 = bytearray(struct.pack('h', int(temp_val[1])))
                value_s3 = bytearray(struct.pack('h', int(temp_val[2])))
                value_s4 = bytearray(struct.pack('h', int(temp_val[3])))
                value_s5 = bytearray(struct.pack('h', int(temp_val[4])))
                value_s6 = bytearray(struct.pack('h', int(temp_val[5])))
                r_time = bytearray(struct.pack('h', int(run_time)))
                cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_CTRL,
                       value_s1[0], value_s1[1], value_s2[0], value_s2[1],
                       value_s3[0], value_s3[1], value_s4[0], value_s4[1],
                       value_s5[0], value_s5[1], value_s6[0], value_s6[1],
                       r_time[0], r_time[1]]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, self.__COMPLEMENT) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    print("arm:", cmd)
                    print("value:", temp_val)
                time.sleep(self.__delay_time)
            else:
                print("angle_s input error!")  # silent fail — no exception, no movement
        except:
            print('---set_uart_servo_angle_array error!---')
            pass

    def set_uart_servo_offset(self, servo_id):
        try:
            self.__arm_offset_id = 0xff
            self.__arm_offset_state = 0
            s_id = int(servo_id) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_OFFSET, s_id]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_offset:", cmd)
            time.sleep(self.__delay_time)
            for i in range(200):
                if self.__arm_offset_id == servo_id:
                    if self.__debug:
                        if self.__arm_offset_id == 0:
                            print("Arm Reset Offset Value")
                        else:
                            print("Arm Offset State:", self.__arm_offset_id, self.__arm_offset_state, i)
                    return self.__arm_offset_state
                time.sleep(.001)
            return self.__arm_offset_state
        except:
            print('---set_uart_servo_offset error!---')
            pass

    def set_akm_default_angle(self, angle, forever=False):
        try:
            if int(angle) > 120 or int(angle) < 60:
                return
            id = self.__AKM_SERVO_ID
            state = 0
            if forever:
                state = 0x5F
                self.__akm_def_angle = angle
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_DEF_ANGLE, id, int(angle), state]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm set def angle:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_akm_default_angle error!---')
            pass

    def set_akm_steering_angle(self, angle, ctrl_car=False):
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            id = self.__AKM_SERVO_ID
            if ctrl_car:
                id = self.__AKM_SERVO_ID + 0x80
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_STEER_ANGLE, id, int(angle) & 0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm_steering_angle:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_akm_steering_angle error!---')
            pass

    def reset_flash_value(self):
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            time.sleep(self.__delay_time)
            time.sleep(.1)
        except:
            print('---reset_flash_value error!---')
            pass

    def reset_car_state(self):
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("reset_car_state:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---reset_car_state error!---')
            pass

    def clear_auto_report_data(self):
        self.__battery_voltage = 0
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

    # -------------------------------------------------------------------------
    # Get methods (all require create_receive_threading() to be called first)
    # -------------------------------------------------------------------------

    def get_akm_default_angle(self):
        if not self.__akm_readed_angle:
            self.__request_data(self.FUNC_AKM_DEF_ANGLE, self.__AKM_SERVO_ID)
            akm_count = 0
            while True:
                if self.__akm_readed_angle:
                    break
                akm_count = akm_count + 1
                if akm_count > 100:
                    return -1
                time.sleep(.01)
        return self.__akm_def_angle

    def get_uart_servo_value(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("get servo id input error!")
                return
            self.__read_id = 0
            self.__read_val = 0
            self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
            timeout = 30
            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val
                timeout = timeout - 1
                time.sleep(.001)
            return -1, -1
        except:
            print('---get_uart_servo_value error!---')
            return -2, -2

    def get_uart_servo_angle(self, s_id):
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 1 and read_id == 1:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    angle = -1
            elif s_id == 2 and read_id == 2:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    angle = -1
            elif s_id == 3 and read_id == 3:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    angle = -1
            elif s_id == 4 and read_id == 4:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    angle = -1
            elif s_id == 5 and read_id == 5:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 270 or angle < 0:
                    angle = -1
            elif s_id == 6 and read_id == 6:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    angle = -1
            else:
                if self.__debug:
                    print("read servo:%d error!" % s_id)
            if self.__debug:
                print("request angle %d: %d, %d" % (s_id, read_id, value))
            return angle
        except:
            print('---get_uart_servo_angle error!---')
            return -2

    # get_uart_servo_angle_array: reads all 6 at once via FUNC_ARM_CTRL request.
    # Requires create_receive_threading(). Returns [-1,...] on timeout (30ms).
    def get_uart_servo_angle_array(self):
        try:
            angle = [-1, -1, -1, -1, -1, -1]
            self.__read_arm = [-1, -1, -1, -1, -1, -1]
            self.__read_arm_ok = 0
            self.__request_data(self.FUNC_ARM_CTRL, 1)
            timeout = 30
            while timeout > 0:
                if self.__read_arm_ok == 1:
                    for i in range(6):
                        if self.__read_arm[i] > 0:
                            angle[i] = self.__arm_convert_angle(i + 1, self.__read_arm[i])
                    if self.__debug:
                        print("angle_array:", 30 - timeout, angle)
                    break
                timeout = timeout - 1
                time.sleep(.001)
            return angle
        except:
            print('---get_uart_servo_angle_array error!---')
            return [-2, -2, -2, -2, -2, -2]

    def get_accelerometer_data(self):
        return self.__ax, self.__ay, self.__az

    def get_gyroscope_data(self):
        return self.__gx, self.__gy, self.__gz

    def get_magnetometer_data(self):
        return self.__mx, self.__my, self.__mz

    def get_imu_attitude_data(self, ToAngle=True):
        if ToAngle:
            RtA = 57.2957795
            return self.__roll * RtA, self.__pitch * RtA, self.__yaw * RtA
        return self.__roll, self.__pitch, self.__yaw

    def get_motion_data(self):
        return self.__vx, self.__vy, self.__vz

    def get_battery_voltage(self):
        return self.__battery_voltage / 10.0

    def get_motor_encoder(self):
        return self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4

    def get_motion_pid(self):
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for i in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                return [kp, ki, kd]
            time.sleep(.001)
        return [-1, -1, -1]

    def get_car_type_from_machine(self):
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for i in range(0, 20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(.001)
        return -1

    def get_version(self):
        if self.__version_H == 0:
            self.__request_data(self.FUNC_VERSION)
            for i in range(0, 20):
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    return self.__version
                time.sleep(.001)
        else:
            return self.__version
        return -1
