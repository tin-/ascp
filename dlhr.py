# Copyright (c) 2017 xDevs.com
# Author: Illya Tsemenko
#
# Based on the ASC Demo code 
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import time
import ConfigParser

cfg = ConfigParser.ConfigParser()
cfg.read('pressure.conf')
cfg.sections()

# ASC_DLHR default address.
ASC_DLHR_I2CADDR = 0x41

# ASC_DLHR Commands
ASC_DLHR_SINGLE_READ_CMD = 0xAA
ASC_DLHR_AVG2_READ_CMD   = 0xAC
ASC_DLHR_AVG4_READ_CMD   = 0xAD
ASC_DLHR_AVG8_READ_CMD   = 0xAE
ASC_DLHR_AVG16_READ_CMD  = 0xAF

# ASC_DLHR Status bits
ASC_DLHR_STS_ZERO          = 0b10000000
ASC_DLHR_STS_PWRON         = 0b01000000
ASC_DLHR_STS_BUSY          = 0b00100000
ASC_DLHR_STS_MODE          = 0b00011000
ASC_DLHR_STS_EEPROM_CHKERR = 0b00000100
ASC_DLHR_STS_SNSCFG        = 0b00000010
ASC_DLHR_STS_ALUERR        = 0b00000001

class ASC_DLHR(object):
    def __init__(self, mode=ASC_DLHR_I2CADDR, address=ASC_DLHR_I2CADDR, i2c=None,
                 **kwargs):
        self._logger = logging.getLogger('Adafruit_BMP.BMP085')
        # Check that mode is valid.
        if mode != 0x41:
            raise ValueError('Unexpected address'.format(mode))
        self._mode = mode
        
        # Create I2C device.
        if cfg.get('main', 'if_debug', 1) == 'false':
            if i2c is None:
                import Adafruit_GPIO.I2C as I2C
                i2c = I2C
            self._device = i2c.get_i2c_device(address, **kwargs)
        
        # Load calibration values.
        #self._load_calibration()
        #self.t_fine = 0.0
        
    def write_cmd(self, cmd):
        if cfg.get('main', 'if_debug', 1) == 'false':
            self._device.writeRaw8(cmd)
            self._device.writeRaw8(0x00)
            self._device.writeRaw8(0x00)
        if cfg.get('main', 'if_debug', 1) == 'true':
            print "\033[31;1mDebug mode only, command = %X\033[0;39m" % cmd,
        time.sleep(0.05)
        return 1

    def chk_busy(self):
        if cfg.get('main', 'if_debug', 1) == 'false':
            Status = self._device.readRaw8()  # Receive Status byte 
        if cfg.get('main', 'if_debug', 1) == 'true':
            Status = 0x40
        
        if (Status & ASC_DLHR_STS_BUSY):
            print "\033[31;1m\r\nPower On status not set!\033[0;39m",
            return 1 # sensor is busy
        
        return 0     # sensor is ready
        
    def read_sensor(self):
        outb = [0,0,0,0,0,0,0,0]
    
        #// wait for completion
        #while (LOW == digitalRead(EOCPIN))
        self.retry_num = 5
        while self.retry_num > 0:
            if self.chk_busy():
                print "\033[31;1m\r\nSensor is busy!\033[0;39m",
                time.sleep(0.05)                       # Sleep for 100ms
            else:
                self.retry_num = 0
            self.retry_num = self.retry_num - 1
        
        if cfg.get('main', 'if_debug', 1) == 'false':
            StatusByte = self._device.readRaw8()  # Receive Status byte 
            outb[0] = self._device.readRaw8()     # Receive Pressure 3 data byte
            outb[1] = self._device.readRaw8()     # Receive Pressure 2 data byte
            outb[2] = self._device.readRaw8()     # Receive Pressure 1 data byte
            outb[3] = self._device.readRaw8()     # Receive Temp 3 data byte
            outb[4] = self._device.readRaw8()     # Receive Temp 2 data byte
            outb[5] = self._device.readRaw8()     # Receive Temp 1 data byte
        
        if cfg.get('main', 'if_debug', 1) == 'true':
            StatusByte = 0x40
            outb[0] = 0x55
            outb[1] = 0x55
            outb[2] = 0x55
            outb[3] = 0x55
            outb[4] = 0x55
            outb[5] = 0x55
        
        # Check for correct status
        if (StatusByte & ASC_DLHR_STS_PWRON) == 0:
            print "\033[31;1m\r\nPower On status not set!\033[0;39m",
            quit()
        if (StatusByte & ASC_DLHR_STS_SNSCFG):
            print "\033[31;1m\r\nIncorrect sensor CFG!\033[0;39m",
            quit()
        if (StatusByte & ASC_DLHR_STS_EEPROM_CHKERR):
            print "\033[31;1m\r\nSensor EEPROM Checksum Failure!\033[0;39m",
            quit()
            
        # Ccnvert Temperature data to degrees C:
        Tmp = outb[3] << 8
        Tmp += outb[4]
        fTemp = float(Tmp)
        fTemp = (fTemp/65535.0) * 125.0 - 40.0
        
        # Convert Pressure to %Full Scale Span ( +/- 100%)
        Prs = (outb[0] <<16) + (outb[1]<<8) + (outb[2])
        Prs -= 0x7FFFFF
        fPress = (float(Prs))/(float(0x800000))
        fPress *= 100.0
    
        print "Status: 0x%X " % StatusByte,
        print "Pressure: %4.5f %%FSS " % fPress,
        print "Temperature: %3.2f 'C " % fTemp,
        
        print " 0x%02X%02X%02X " % (outb[0], outb[1], outb[2]),
        
        Tmp = outb[3] << 16;
        Tmp += outb[4] << 8;
        Tmp += outb[5];
        
        print " %X " % Tmp
        return 0
