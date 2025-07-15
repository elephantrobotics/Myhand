import serial
import time
import threading


class Command():
    """Communication message:
        Header(int):Frame Header
        Len(int):length
        ID(int):Gripper ID
        Code(int):Function code(6-write,3-read)
        Number_High(int):Instruction number high byte
        Number_LOW(int):Instruction number low byte
        Value_High(int):Parameter high byte
        Value_LOW(int):Parameter low byte

    """
    Header = 254
    Len = 8

    ID = 14
    Code = 0
    Zero = 0
    Number_High = 0
    Number_LOW = 0
    Value_High = 0
    Value_LOW = 0
    Joint_High = 0
    Joint_LOW = 0
    cmd_list1 = [Header, Header, Len, ID, Code, Number_High, Number_LOW, Value_High, Value_LOW]
    cmd_list2 = [Header, Header, Len, ID, Code, Number_High, Number_LOW, Joint_High, Joint_LOW, Value_High, Value_LOW]
    cmd_list3 = [Header, Header, Len, ID, Code, Number_High, Number_LOW,
                 Joint_High, Joint_LOW,
                 Joint_High, Joint_LOW,
                 Joint_High, Joint_LOW,
                 Joint_High, Joint_LOW,
                 Joint_High, Joint_LOW,
                 Joint_High, Joint_LOW,
                 Value_High, Value_LOW]


class MyGripper_H100(Command):

    def __init__(self, port, baudrate=115200, id=14, debug=False, recv_timeout=10):
        """
        Args:
            port : Serial port number
            baudrate : Baud rate,Defaults to 115200.
            id: Gripper ID, default is 1
        """

        self.lock = threading.Lock()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate,
                                 timeout=recv_timeout)  # timeout is the timeout period, the default value is 5 seconds
        self.cmd_list1[3] = id
        self.cmd_list2[3] = id
        self.cmd_list3[3] = id
        self.debug = debug

    def check_value(self, value, lower, upper, index=1):
        valid_values = list(range(lower, upper + 1))  # 生成有效值范围

        # 判断输入是单个值还是列表
        if isinstance(value, list):
            # 遍历列表中的每个值
            for i, val in enumerate(value):
                if val not in valid_values:
                    raise ValueError(
                        f"The {index} input value at position {i + 1} is invalid. Valid values are: [{lower},{upper}]")
            return True
        else:
            # 单个值的处理逻辑
            if value in valid_values:
                return True
            else:
                if index == 1:
                    raise ValueError(f"The first input value can be selected as: [{lower},{upper}]")
                else:
                    raise ValueError(f"The second input value can be selected as: [{lower},{upper}]")

    def __byte_deal(self, *args):
        result = []
        for value in args:
            high_byte = (value >> 8) & 0xFF  # 高字节
            low_byte = value & 0xFF  # 低字节
            result.extend([high_byte, low_byte])
        return result

    def __crc16_modbus(self, data: bytes) -> bytes:

        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='big')

    def __send_cmd(self, cmd, byte=11):

        with self.lock:
            send_data = cmd + self.__crc16_modbus(cmd)
            self.ser.write(send_data)
            self.ser.flush()
            time.sleep(0.04)
            if self.debug:
                # print("send_data=",send_data.hex())
                s_upper = send_data.hex().upper()
                result = ' '.join([s_upper[i:i + 2] for i in range(0, len(s_upper), 2)])
                print("send_data=", result)
            recv_data = self.ser.read(byte)
            if not recv_data:
                raise TimeoutError("Reading data timeout")
            if self.debug:
                if recv_data is not None:
                    s_upper = recv_data.hex().upper()
                    result = ' '.join([s_upper[i:i + 2] for i in range(0, len(s_upper), 2)])
                else:
                    s_upper = ""
                print("recv_data=", result)
            # print(len(recv_data))
            data_len = len(recv_data)
            if data_len == 11 or data_len == 73 or data_len == 329:
                data = recv_data[0:-2]
                crc_data = recv_data[-2:]
                if self.__crc16_modbus(data) == crc_data:
                    response = data + crc_data
                    result = int(response.hex()[-8:-4], 16)
                    return result
                else:
                    return -2
            elif data_len == 21:
                data = recv_data[0:-2]
                crc_data = recv_data[-2:]
                if self.__crc16_modbus(data) == crc_data:

                    response = data + crc_data
                    sub_hex = response.hex()[14:-4]  # 正确位置应该是从第30到54个字符
                    # 将每两个字符作为一组转换为十进制
                    result = [int(sub_hex[i:i + 4], 16) for i in range(0, len(sub_hex), 4)]

                    return result
                else:
                    return -2

            else:
                return -1

    def get_gripper_firmware_version(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(1, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd) / 10

    def get_gripper_modified_version(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(2, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd)

    def get_gripper_type(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(53, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd)

    def get_gripper_Id(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(4, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd)

    def get_gripper_baud(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(6, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd)

    def get_gripper_joint_angle(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(12, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_status(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(14, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd)

    def get_gripper_joint_P(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(16, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_I(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(20, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_D(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(18, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_cw(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(22, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_cww(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(24, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_mini_pressure(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(26, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_torque(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(28, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_joint_speed(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(33, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def get_gripper_finger_torque(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 3
            tmp = self.__byte_deal(46, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd, 73)

    def get_gripper_fingers_torque(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(47, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd, 329)

    def get_gripper_angles(self):
        self.cmd_list1[4] = 3
        tmp = self.__byte_deal(50, 0)
        for i in range(5, 9):
            self.cmd_list1[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list1)
        return self.__send_cmd(cmd, 21)

    def set_gripper_Id(self, value):
        """Set the gripper ID

        Args:
            value (int):ID range:1 - 254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 1, 255):
            self.cmd_list1[4] = 6
            tmp = self.__byte_deal(3, value)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            self.cmd_list1[3] = value
            return self.__send_cmd(cmd)

    def set_gripper_baud(self, value=0):
        if self.check_value(value, 0, 5):
            self.cmd_list1[4] = 6
            tmp = self.__byte_deal(5, value)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def set_gripper_enable(self, value):
        if self.check_value(value, 0, 1):
            self.cmd_list1[4] = 6
            tmp = self.__byte_deal(10, value)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    def set_gripper_joint_calibration(self, id):
        if self.check_value(id, 1, 6):
            self.cmd_list1[4] = 6
            tmp = self.__byte_deal(13, id)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list1)
            return self.__send_cmd(cmd)

    # def set_joints_calibration(self):
    #     """Setting the gripper Calibration

    #     Returns:
    #         Response results:0 represents failure, 1 represents success
    #     """
    #     for i in range(1,7):
    #         result=self.set_joint_calibration(i)
    #     if result:
    #         return 1
    #     else:
    #         return 0

    def set_gripper_joint_angle(self, finger_id, finger_angle):
        if self.check_value(finger_id, 1, 6) and self.check_value(finger_angle, 0, 100, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(11, finger_id, finger_angle)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_P(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 254, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(15, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_I(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 254, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(19, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_D(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 254, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(17, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_cw(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 16, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(21, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_cww(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 16, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(23, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_mini_pressure(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 254, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(25, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_torque(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 0, 100, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(27, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_joint_speed(self, id, value):
        if self.check_value(id, 1, 6) and self.check_value(value, 1, 100, 2):
            self.cmd_list2[4] = 6
            self.cmd_list2[2] = 10
            tmp = self.__byte_deal(32, id, value)
            for i in range(5, 11):
                self.cmd_list2[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list2)
            return self.__send_cmd(cmd)

    def set_gripper_angles(self, angles, speed):
        if self.check_value(angles, 0, 100) and self.check_value(speed, 1, 100, 2):
            self.cmd_list3[4] = 6
            self.cmd_list3[2] = 18
            tmp = self.__byte_deal(45)
            for i in range(len(angles)):
                tmp.extend(self.__byte_deal(angles[i]))
            # for i in range(len(angles)):
            #     tmp.extend(self.__byte_deal(speed))
            tmp.extend(self.__byte_deal(speed))
            # print(len(self.cmd_list3))
            # print(tmp)
            for i in range(5, len(self.cmd_list3)):
                self.cmd_list3[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list3)
            return self.__send_cmd(cmd)

    # def set_gripper_action(self, value):
    #     if self.check_value(value, 0, 3):
    #         self.cmd_list1[4] = 6
    #         tmp = self.__byte_deal(51, value)
    #         for i in range(5, 9):
    #             self.cmd_list1[i] = tmp[i - 5]
    #         cmd = bytes(self.cmd_list1)
    #         return self.__send_cmd(cmd)

    def set_gripper_pose(self, pose=0, value=0, flag=False):
        if self.check_value(pose, 0, 4):
            if pose == 4:
                self.check_value(value, 0, 15, index=2)
            else:
                self.check_value(value, 0, 5, index=2)
            self.cmd_list1[4] = 6
            self.cmd_list1[5] = 52
            tmp = self.__byte_deal(52, 0)
            for i in range(5, 9):
                self.cmd_list1[i] = tmp[i - 5]
            self.cmd_list1[7] = pose
            self.cmd_list1[8] = value
            tmp_list = self.cmd_list1.copy()
            if flag:
                tmp_list.append(flag)
            cmd = bytes(tmp_list)
            return self.__send_cmd(cmd)

    def close(self):
        self.ser.close()