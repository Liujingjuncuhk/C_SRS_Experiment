import serial
import time
import struct
import pymodbus 
from pymodbus.client import ModbusSerialClient
import matplotlib.pyplot as plt
import numpy as np

class cableTensionReader:
    def __init__(self, portname) -> None:
        # self.k_list = [1.,1.,1.,1.,1.,1.,1.]# calibrated every time
        self.k_list = [4.00, 3.6233, 1.1511, 1.0551,  1.1888, 2.6054]
        b_list = [0.0]
        self.nCable = 7
        self.client = ModbusSerialClient(portname, baudrate = 9600)
        self.client.connect()
    

    def read_all_tensions(self)->list:
        try:
            result = self.client.read_holding_registers(address=450, count = self.nCable*2, slave = 1)
            result_list = result.registers
            # print(result_list)
            # 1,3,5,7 are the loadcell data, if > 60000, then it is negative, set to 0
            return_list = [0.0 for _ in range(self.nCable)]
            for i in range(3):
                if result_list[2*i+1] > 60000:
                    return_list[i] = 0.0
                else:
                    return_list[i] = result_list[2*i+1]*self.k_list[i]
            for i in range(3,6):
                if result_list[2*i+1] <= 30000:
                    return_list[i] = 0.0
                else:
                    pull_val = 65536.0-result_list[2*i+1]
                    return_list[i] = pull_val*self.k_list[i]
            if result_list[13] > 30000:
                return_list[6] = 65536-result_list[13]
            else:
                return_list[6] = result_list[13]
            return return_list
        except:
            print("Error")
            return [-1, -1, -1, -1. -1, -1, -1]

    def read_one_tension(self, cable_idx:int)->float: # cable idx start from 1
        try:
            result = self.client.read_holding_registers(address=450, count = 14, slave = 1)
            result_list = result.registers
            # 1,3,5,7 are the loadcell data, if > 60000, then it is negative, set to 0
            for i in range(14):
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

    def calibrate_one_cable(self, cable_idx:int, total_time = 10):
        total_read = 0
        start_time = time.time()
        results = []
        while True:
            result_allloadCell = self.read_all_tensions()
            results.append([result_allloadCell[cable_idx-1], result_allloadCell[6]])
            print([result_allloadCell[cable_idx-1], result_allloadCell[6]])
            total_read += 1
            if time.time() - start_time > total_time:
                break
        fit_data = np.array(results)
        x = fit_data[:,0]
        y = fit_data[:,1]
        k,b = np.polyfit(x, y, 1)
        # plot the data and the fit
        plt.plot(x, y, 'o')
        plt.plot(x, k*x+b)
        plt.title("Cable "+str(cable_idx)+" Calibration")
        plt.xlabel("LoadCell Reading for cable "+str(cable_idx))
        plt.ylabel("LoadCell Reading for cable 7")
        plt.show()
        print("k: ", np.round(k,4), "b: ", np.round(b,4))

    def close(self):
        self.client.close()

if __name__ == "__main__":
    reader = cableTensionReader("COM7")
    # reader.read_all_tension_fortime(5)
    reader.calibrate_one_cable(6)
    reader.close()
