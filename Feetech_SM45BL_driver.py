from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
import serial
import time
from tkinter import *

class Feetech_SM45BL_driver:
    def __init__(self, port:str, motor_ids, baudrate:int=115200):
        self.client = ModbusClient(
            port=port,  # Update to your serial port, e.g., COM3 on Windows
            baudrate=baudrate,
        )
        self.motor_ids = motor_ids
        self.client.connect()
        self.pos_cmd_addr = 128
        self.pos_read_addr = 257
        self.speed_set_addr = 131
        self.n_motor = len(motor_ids)
        self.default_speed = 94
        self.default_acc = 500
        self.default_pos = [2617, 3647, 400, 650]
        self.moving_dir = [-1, -1, 1, 1]
        self.mode_list = {0:"position", 1:"speed", 4:"stepper"}
        self.cur_working_mode = 0

    def switch_to_mode(self, mode:int)->None:
        if mode not in [0,1,4]:
            print("Invalid mode, please choose from 0, 1, 4")
            return
        self.cur_working_mode = mode
        result = self.client.write_register(16, mode)
        if result.isError():
            print(f"Error writing: {result}")
        print(f"Switched to mode: {self.mode_list[mode]}")
    
    def set_speed(self, motor_id:int, speed:int)->None:
        speed = max(1, min(94, speed))
        try:
            result = self.client.write_register(self.speed_set_addr, speed, slave = motor_id)
            if result.isError():
                print(f"Error writing: {result}")
        except ModbusException as e:
            print(f"Modbus exception: {e}")

    def move_single_motor_abs(self, target_pos:int, motor_id:int, speed:int=94, acc:int=500)->None:
        # speed*50 is the speed in step/s, acc*100 is the acceleration in step/s^2
        if speed > 94 or acc > 500 or speed <= 0 or acc <= 0 :
            print("Speed must be between 1 and 94 (unit:50 step/s), and acceleration must be between 1 and 500 (unit:100 step/s^2)")
            return
        target_pos = max(0, min(4096, target_pos))
        try:
            result = self.client.write_registers(self.pos_cmd_addr, [target_pos, 0, speed, acc], slave = motor_id)
            if result.isError():
                print(f"Error writing: {result}")
        except ModbusException as e:
            print(f"Modbus exception: {e}")

    def move_single_motor_rel(self, step_diff:int,  motor_id:int, speed:int=94, acc:int=500)->None:
        # speed*50 is the speed in step/s, acc*100 is the acceleration in step/s^2
        address_to_write = 128 # starting register for the target position
        if speed > 94 or acc > 500 or speed <= 0 or acc <= 0:
            print("Speed must be between 1 and 94 (unit:50 step/s), and acceleration must be between 1 and 500 (unit:100 step/s^2)")
            return
        cur_pos = self.read_single_motor_pos(motor_id)
        target_pos = cur_pos + step_diff*self.moving_dir[self.motor_ids.index(motor_id)]
        self.move_single_motor_abs(target_pos, motor_id, speed, acc)

    def move_multi_motors_abs(self, target_pos, motor_ids = [1,2,3,4], speed = None, acc = None)->None:
        if speed is None:
            speed = [self.default_speed]*len(motor_ids)
        if acc is None:
            acc = [self.default_acc]*len(motor_ids)
        if len(motor_ids) != len(target_pos) or len(motor_ids) != len(speed) or len(motor_ids) != len(acc):
            print("The length of motor_ids, target_pos, speed and acc must be the same")
            return
        # bound target poses to [0, 4096]
        target_pos = [max(0, min(4096, pos)) for pos in target_pos]
        speed = [max(1, min(94, spd)) for spd in speed]
        acc = [max(1, min(500, ac)) for ac in acc]
        try:
            for i in range(len(motor_ids)):
                result = self.client.write_registers(self.pos_cmd_addr, [target_pos[i], 0, speed[i], acc[i]], slave = motor_ids[i])
                if result.isError():
                    print(f"Error writing: {result}")
        except ModbusException as e:
            print(f"Modbus exception: {e}")

    def move_multi_motors_rel(self, step_diffs, motor_ids = [1,2,3,4], speed = None, acc = None)->None:
        if speed is None:
            speed = [self.default_speed]*len(motor_ids)
        if acc is None:
            acc = [self.default_acc]*len(motor_ids)
        if len(motor_ids) != len(step_diffs) or len(motor_ids) != len(speed) or len(motor_ids) != len(acc):
            print("The length of motor_ids, step_diffs, speed and acc must be the same")
            return
        target_poses = []
        try:
            for i in range(len(motor_ids)):
                cur_pos = self.read_single_motor_pos(motor_ids[i])
                target_poses.append(cur_pos + step_diffs[i]*self.moving_dir[i])
            self.move_multi_motors_abs(target_poses, motor_ids=motor_ids, speed = speed, acc = acc)
        except ModbusException as e:
            print(f"Modbus exception: {e}")

    def read_single_motor_pos(self, motor_id:int)->int:
        try:
            result = self.client.read_holding_registers(self.pos_read_addr, 1, slave = motor_id)
            if result.isError():
                print(f"Error reading: {result}")
        except ModbusException as e:
            print(f"Modbus exception: {e}")
        return result.registers[0]
    
    def read_mul_motors_pos(self, motor_ids):
        poses = []
        for motor_id in motor_ids:
            poses.append(self.read_single_motor_pos(motor_id))
        return poses

    def move_all_motors_manual(self, motor_ids)->None:
        def move_id(motor_id:int):
            def move_to(position):
                self.move_single_motor_abs(int(position), motor_id)
            return move_to
        root = Tk()
        root.title("command panel for motors")
        cur_poses = self.read_mul_motors_pos(motor_ids)
        for i in range(len(motor_ids)):
            motor_id = motor_ids[i]
            scale_i = Scale(root, command = move_id(motor_id), from_ = 0, to = 4096, 
                    orient = HORIZONTAL, length = 400, label = 'motor id' + str(motor_id))
            scale_i.set(cur_poses[i])
            scale_i.pack()
        root.mainloop()

    def move_all_motors_to_default(self)->None:
        self.move_multi_motors_abs(self.default_pos)

    def close_driver(self):
        self.client.close()

if __name__ == "__main__":
    driver = Feetech_SM45BL_driver('/dev/ttyUSB0', [1, 2, 3, 4])
    # driver.move_multi_motors_abs([1,2,3,4],[1000, 2000, 3000, 4000])10001000
    driver.move_all_motors_to_default()
    time.sleep(1)
    driver.move_all_motors_manual([1,2,3,4])
    # driver.move_multi_motors_rel([100, 0, 0, 400], [1,2,3,4])
    # input("press enter to continue")
    driver.move_all_motors_to_default()
    # driver.move_all_motors_to_default()
    driver.close_driver()