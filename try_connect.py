import serial
import numpy as np
import time

send_cmd1 = [0xff, 0xb0, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31]
send_cmd2 = [0xff, 0xb0, 0x04, 0x05, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85]
class Ser():
    def __init__(self):
        self.port = serial.Serial(port='COM8', baudrate=115200, bytesize=8,stopbits=2, timeout=0.2)
        a = self.send_cmd(send_cmd1)
        while a == []:
            a = self.send_cmd(send_cmd1)
        print(a)
        b = self.send_cmd(send_cmd2)
        print(b)
        print("connect_success")

    def send_cmd(self, cmd):
        self.port.write(cmd)
        response = self.port.readall()
        response = self.convert_hex(response)
        return response

    def convert_hex(self, string):
        res = []
        result = []
        for item in string:
            res.append(item)
        for i in res:
            result.append(hex(i))

        return result

    def send_velocity(self,robot_id, vx, vy, w):

        if robot_id < 9:
            high_id = 0x00
            low_id = 2**(robot_id-1)
        else:
            robot_id = robot_id%9
            low_id = 0x00
            high_id = 2**robot_id
        if vx > 0:
            flag_x = int(0)
        else:
            flag_x = int(1)
        if vy > 0:
            flag_y = int(0)
        else:
            flag_y = int(1)
        if w > 0:
            flag_w = int(0)
        else:
            flag_w = int(1)
        vy = int(abs(vy))
        vx = int(abs(vx))
        w =  int(abs(w))
        vx_low = []
        vx_high = []
        for _ in range(4):
            vx_low.append(vx & int(1))
            vx = vx >> 1
        for _ in range(5):
            vx_high.append(vx & int(1))
            vx = vx >> 1
        vy_low = []
        vy_high = []
        for _ in range(6):
            vy_low.append(vy & int(1))
            vy = vy >> 1
        for _ in range(3):
            vy_high.append(vy & int(1))
            vy = vy >> 1
        w_low = []
        w_high = []
        for _ in range(8):
            w_low.append(w & int(1))
            w = w >> 1
        for _ in range(1):
            w_high.append(w & int(1))
            w = w >> 1
        string_5 =int(0)
        string_5 = string_5 | flag_x
        for index1 in range(5):
            string_5 = string_5 << 1
            string_5 = string_5 | int(vx_high[4-index1])
        string_6 = int(0)
        for index2 in range(4):
            string_6 = string_6 | int(vx_low[3-index2])
            string_6 = string_6 <<1
        string_6 = string_6 | flag_y

        for index3 in range(3):
            string_6 = string_6 << 1
            string_6 = string_6 | vy_high[2-index3]
        string_7 = int(0)
        for index4 in range(6):
            string_7 = string_7 | vy_low[5-index4]
            string_7 = string_7 << 1
        string_7 = string_7 | int(flag_w)
        string_7 = string_7 << 1
        string_7 = string_7 | w_high[0]
        string_8 = int(0)
        for index5 in range(8):
            string_8 = string_8 << 1
            string_8 = string_8 | w_low[7-index5]

        cmd = [0xff, 0x00, 0x01, 0x01, string_5, string_6, string_7, string_8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00]
        # cmd = [0xff, 0x00, 0x01, 0x01, 0x06, 0x40, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        #        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00]

        # print(cmd)
        self.port.write(cmd)


if __name__ == "__main__":

    robot_serial = Ser()


    # c = robot_serial.send_velocity(send_cmd3)
    while True:
        robot_serial.send_velocity(robot_id=1,vx = 0, vy= 0, w = 50)
        time.sleep(0.2)
