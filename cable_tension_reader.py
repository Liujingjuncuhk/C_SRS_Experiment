import serial
import time
import struct
import pymodbus 
from pymodbus.client import ModbusSerialClient

class cableTensionReader:
    def __init__(self, portname) -> None:
        self.k_list = [0.0088, 0.0121, 0.0130, 0.0083]# calibrated every time

        self.client = ModbusSerialClient(portname, baudrate = 9600)
        self.client.connect()
    

    def read_all_tensions(self)->list:
        try:
            result = self.client.read_holding_registers(address=450, count = 8, slave = 1)
            result_list = result.registers
            # 1,3,5,7 are the loadcell data, if > 60000, then it is negative, set to 0
            for i in range(8):
                if result_list[i] > 60000:
                    result_list[i] = 0
            return [result_list[1]*self.k_list[0], result_list[3]*self.k_list[1], result_list[5]*self.k_list[2], result_list[7]*self.k_list[3]]
        except:
            print("Error")
            return [-1, -1, -1, -1]

    def read_one_tension(self, cable_idx:int)->float: # cable idx start from 1
        try:
            result = self.client.read_holding_registers(address=450, count = 8, slave = 1)
            result_list = result.registers
            # 1,3,5,7 are the loadcell data, if > 60000, then it is negative, set to 0
            for i in range(8):
                if result_list[i] > 60000:
                    result_list[i] = 0
            return result_list[2*cable_idx-1]*self.k_list[cable_idx-1]
        except:
            print("Error")
            return -1
    
    def read_all_tension_fortime(self, t_total = 5)->list:
        total_read = 0
        start_time = time.time()
        result = []
        while True:
            result = self.read_all_tensions()
            print(result)
            total_read += 1
            if time.time() - start_time > t_total:
                break
        print("total read: ", total_read, "in ", t_total, "s")
        return result

    def close(self):
        self.client.close()

if __name__ == "__main__":
    reader = cableTensionReader("/dev/ttyUSB1")
    reader.read_all_tension_fortime(10)
    reader.close()
       