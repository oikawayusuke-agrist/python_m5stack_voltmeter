# -*- coding: utf-8 -*-

from smbus2 import SMBus, i2c_msg
from ctypes import *
import time
import numpy as np

address = 0x49

I2C_BUS = 8

ADS1115_ADDR = 0x49
EEPROM_ADDR  = 0x53

ADS1115_RA_CONVERSION = 0x00
ADS1115_RA_CONFIG     = 0x01

ADS1115_PAG_6144      = 0x00
ADS1115_PAG_4096      = 0x01
ADS1115_PAG_2048      = 0x02  # default
ADS1115_PAG_1024      = 0x03
ADS1115_PAG_512       = 0x04
ADS1115_PAG_256       = 0x05

ADS1115_MV_6144       = 0.187500
ADS1115_MV_4096       = 0.125000
ADS1115_MV_2048       = 0.062500 # default
ADS1115_MV_1024       = 0.031250
ADS1115_MV_512        = 0.015625
ADS1115_MV_256        = 0.007813

ADS1115_RATE_8        = 0x00
ADS1115_RATE_16       = 0x01
ADS1115_RATE_32       = 0x02
ADS1115_RATE_64       = 0x03
ADS1115_RATE_128      = 0x04 #default
ADS1115_RATE_250      = 0x05
ADS1115_RATE_475      = 0x06
ADS1115_RATE_860      = 0x07

VOLTMETER_MEASURING_DIR = -1

ADS1115_MUX_P0N1      = 0x00 # voltmeter only support

ADS1115_COMP_MODE_HYSTERESIS = 0x00 # default
ADS1115_COMP_MODE_WINDOW     = 0x01

ADS1115_MODE_CONTINUOUS = 0x00
ADS1115_MODE_SINGLESHOT = 0x01 # default

VOLTMETER_PRESSURE_COEFFICIENT = 0.015918958

VOLTMETER_PAG_6144_CAL_ADDR  = 208
VOLTMETER_PAG_4096_CAL_ADDR  = 216
VOLTMETER_PAG_2048_CAL_ADDR  = 224 # default
VOLTMETER_PAG_1024_CAL_ADDR  = 232
VOLTMETER_PAG_512_CAL_ADDR   = 240
VOLTMETER_PAG_256_CAL_ADDR   = 248

VOLTMETER_FILTER_NUMBER = 10

MODE = ADS1115_MODE_SINGLESHOT
RATE = ADS1115_RATE_128
GAIN = ADS1115_PAG_2048

class Voltmeter:
    _ads1115_addr = ADS1115_ADDR
    _eeprom_addr = EEPROM_ADDR
    _gain = ADS1115_PAG_2048
    _mode = ADS1115_MODE_SINGLESHOT
    _rate = ADS1115_RATE_128
    calibration_factor = 1
    adc_raw = 0
    cover_time = 0.0
    resolution = 0.0

    def __init__(self, ads1115_addr = ADS1115_ADDR, eeprom_addr = EEPROM_ADDR):
        self._ads1115_addr = ads1115_addr
        self._eeprom_addr  = eeprom_addr
        self.cover_time = self.getCoverTime(self._rate)
        self.resolution = self.getResolution(self._gain)

    def getResolution(self, gain):
        if   gain == ADS1115_PAG_6144:
            return ADS1115_MV_6144 / VOLTMETER_PRESSURE_COEFFICIENT
        elif gain == ADS1115_PAG_4096:
            return ADS1115_MV_4096 / VOLTMETER_PRESSURE_COEFFICIENT
        elif gain == ADS1115_PAG_2048: # default
            return ADS1115_MV_2048 / VOLTMETER_PRESSURE_COEFFICIENT
        elif gain == ADS1115_PAG_1024:
            return ADS1115_MV_1024 / VOLTMETER_PRESSURE_COEFFICIENT
        elif gain == ADS1115_PAG_512:
            return ADS1115_MV_512  / VOLTMETER_PRESSURE_COEFFICIENT
        elif gain == ADS1115_PAG_256:
            return ADS1115_MV_256  / VOLTMETER_PRESSURE_COEFFICIENT
        else :
            return ADS1115_MV_256  / VOLTMETER_PRESSURE_COEFFICIENT

    def getCoverTime(self, rate):
        if   rate == ADS1115_RATE_8:
            return 1.0 / 8.0
        elif rate == ADS1115_RATE_16:
            return 1.0 / 16.0
        elif rate == ADS1115_RATE_32:
            return 1.0 / 32.0
        elif rate == ADS1115_RATE_64:
            return 1.0 / 64.0
        elif rate == ADS1115_RATE_128:
            return 1.0 / 128.0
        elif rate == ADS1115_RATE_250:
            return 1.0 / 250.0
        elif rate == ADS1115_RATE_475:
            return 1.0 / 475.0
        elif rate == ADS1115_RATE_860:
            return 1.0 / 860.0
        else:
            return 1 / 128

    def getPAGEEPROMAddr(self, gain):
        if   gain == ADS1115_PAG_6144:
            return VOLTMETER_PAG_6144_CAL_ADDR
        elif gain == ADS1115_PAG_4096:
            return VOLTMETER_PAG_4096_CAL_ADDR
        elif gain == ADS1115_PAG_2048: # default
            return VOLTMETER_PAG_2048_CAL_ADDR
        elif gain == ADS1115_PAG_1024:
            return VOLTMETER_PAG_1024_CAL_ADDR
        elif gain == ADS1115_PAG_512:
            return VOLTMETER_PAG_512_CAL_ADDR
        elif gain == ADS1115_PAG_256:
            return VOLTMETER_PAG_256_CAL_ADDR
        else :
            return 0x00

    def setGain(self, gain):
        reg_value = self.i2cReadU16(self._ads1115_addr, ADS1115_RA_CONFIG)

        reg_value &= ~(0b0111 << 9)
        reg_value |= gain << 9

        self.i2cWriteU16(self._ads1115_addr, ADS1115_RA_CONFIG, reg_value)

        self.gain = gain
        self.resolution = self.getResolution(gain)

        hope, actual = self.readCalibrationFromEEPROM(gain)
        self.calibration_factor = float(hope) / float(actual)

    def setRate(self, rate):
        reg_value = i2cReadU16(self._ads1115_addr, ADS1115_RA_CONFIG)

        reg_value &= ~(0b0111 << 5)
        reg_value |= rate << 5

        self.i2cWriteU16(self._ads1115_addr, ADS1115_RA_CONFIG)

        self._rate = rate
        cover_time = self.getCoverTime(self._rate)

    def setMode(self, mode):
        reg_value = i2cReadU16(self._ads1115_addr, ADS1115_RA_CONFIG)

        reg_value &= ~(0b0001 << 8)
        reg_value |= mode << 8

        self.i2cWriteU16(self._ads1115_addr, ADS1115_RA_CONFIG)

        self._mode = mode

    def i2cReadU16(self, addr, reg_addr):
        with SMBus(I2C_BUS) as bus:
            block = bus.read_i2c_block_data(addr, reg_addr, 2)
            return np.array([ (block[0] << 8) | block[1] ], dtype='int16')[0]

    def i2cWriteU16(self, addr, reg_addr, value):
        write_buf = [value >> 8, value & 0xff]

        with SMBus(I2C_BUS) as bus:
            bus.write_i2c_block_data(addr, reg_addr, write_buf)

    def getVoltage(self, calibration = True):
        if calibration:
            return self.resolution * self.calibration_factor * self.getConversion() * VOLTMETER_MEASURING_DIR
        else:
            return self.resolution                           * self.getConversion() * VOLTMETER_MEASURING_DIR
        pass

    def getAdcRaw(self):
        value = self.i2cReadU16(self._ads1115_addr, ADS1115_RA_CONVERSION)
        self.adc_raw = value
        return value

    def getConversion(self, timeout = 0.125): # conversionを得る
        if MODE == ADS1115_MODE_SINGLESHOT :
            self.startSingleConversion()
            time.sleep(self.cover_time)
            dead_time = time.time() + timeout
            while dead_time > time.time() and self.isInConversion():
                pass
        return self.getAdcRaw()

    def startSingleConversion(self):
        reg_value = self.i2cReadU16(self._ads1115_addr, ADS1115_RA_CONFIG)
        reg_value |= 0x01 << 15
        self.i2cWriteU16(self._ads1115_addr, ADS1115_RA_CONFIG, reg_value)

    def isInConversion(self):
        value = self.i2cReadU16(self._ads1115_addr, ADS1115_RA_CONFIG)
        return False if value & (1 << 15) else True

    def EEPROMRead(self, reg_addr, leng):
        with SMBus(I2C_BUS) as bus:
            block = bus.read_i2c_block_data(self._eeprom_addr, reg_addr, leng)
            return block

    def readCalibrationFromEEPROM(self, gain):
        addr = self.getPAGEEPROMAddr(gain)

        buff = self.EEPROMRead(addr, 8)

        xor_result = 0x00
        for i in range(5):
            xor_result ^= buff[i]

        if (xor_result != buff[5]):
            print("error!")
            return
        hope   = (buff[1] << 8) | buff[2]
        actual = (buff[3] << 8) | buff[4]

        return hope, actual

def main():
    voltmeter = Voltmeter()
    voltmeter.setGain(GAIN)
    # print("resolution : " + str(voltmeter.resolution))
    # print("calibration_factor : " + str(voltmeter.calibration_factor))
    while 1:
        print(str(voltmeter.getVoltage()*0.001) + "[V]")
        time.sleep(1)
    # print("adc_raw = " + str(voltmeter.adc_raw))

if __name__ == "__main__":
    main()
