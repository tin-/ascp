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
ASC_DLHR_I2CADDR = 0x29

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

global bus

class ASC_DLHR(object):
    def __init__(self, mode=ASC_DLHR_I2CADDR, address=ASC_DLHR_I2CADDR, i2c=None,
                 **kwargs):
        self._logger = logging.getLogger('Adafruit_BMP.BMP085')
        # Check that mode is valid.
        if mode != 0x29:
            raise ValueError('Unexpected address'.format(mode))
        self._mode = mode
        global bus
        
        # Create I2C device.
        if cfg.get('main', 'if_debug', 1) == 'false':
            if i2c is None:
                #import Adafruit_GPIO.I2C as I2C
                #i2c = I2C
                from smbus import SMBus
                bus = SMBus(1)
            #self._device = i2c.get_i2c_device(address, **kwargs)
        
        # Load calibration values.
        #self._load_calibration()
        #self.t_fine = 0.0
        
    def write_cmd(self, cmd):
        if cfg.get('main', 'if_debug', 1) == 'false':
            bus.write_i2c_block_data(ASC_DLHR_I2CADDR, cmd, [0x00, 0x00])
            time.sleep(0.1)
	if cfg.get('main', 'if_debug', 1) == 'true':
	    print "\033[31;1mDebug mode only, command = %X\033[0;39m" % cmd,
        return 1

    def chk_busy(self):
        if cfg.get('main', 'if_debug', 1) == 'false':
            Status = bus.read_byte(ASC_DLHR_I2CADDR)
	    #Status = self._device.readRaw8()  # Receive Status byte 
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
        time.sleep(0.04)
        self.retry_num = 5
        while self.retry_num > 0:
            if self.chk_busy():
                print "\033[31;1m\r\nSensor is busy!\033[0;39m",
                time.sleep(0.01)                       # Sleep for 100ms
            else:
                self.retry_num = 0
            self.retry_num = self.retry_num - 1
        
        if cfg.get('main', 'if_debug', 1) == 'false':
            #StatusByte = self._device.readRaw8()  # Receive Status byte 
            #outb[0] = self._device.readRaw8()     # Receive Pressure 3 data byte
            #outb[1] = self._device.readRaw8()     # Receive Pressure 2 data byte
            #outb[2] = self._device.readRaw8()     # Receive Pressure 1 data byte
            #outb[3] = self._device.readRaw8()     # Receive Temp 3 data byte
            #outb[4] = self._device.readRaw8()     # Receive Temp 2 data byte
            #outb[5] = self._device.readRaw8()     # Receive Temp 1 data byte
    	    #StatusByte = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[0] = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[1] = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[2] = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[3] = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[4] = bus.read_byte(ASC_DLHR_I2CADDR)
	    #outb[5] = bus.read_byte(ASC_DLHR_I2CADDR)
	    outb = bus.read_i2c_block_data(ASC_DLHR_I2CADDR, 0, 7)
            StatusByte = outb[0]

        if cfg.get('main', 'if_debug', 1) == 'true':
            StatusByte = 0xFF
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
        Tmp = outb[4] << 8
        Tmp += outb[5]
        fTemp = float(Tmp)
        fTemp = (fTemp/65535.0) * 125.0 - 40.0
        
        # Convert Pressure to %Full Scale Span ( +/- 100%)
        Prs = (outb[1] <<16) + (outb[2]<<8) + (outb[3])
        Prs -= 0x7FFFFF
        fPress = (float(Prs))/(float(0x800000))
        fPress *= 100.0
    
        print "Status: 0x%X " % StatusByte,
        print "\033[35;1mPressure: %4.5f %%FSS \033[33;1m Counts: %d " % (fPress, (outb[1] <<16) + (outb[2]<<8) + (outb[3]) ),
        print "\033[36;1mTemperature: %3.3f 'C \033[39;0m" % fTemp

        #print " 0x%X%X%X " % (outb[1], outb[2], outb[3]),
        
        #Tmp = outb[4] << 16;
        #Tmp += outb[5] << 8;
        #Tmp += outb[6];
        
        #print " %X " % Tmp
        return ((outb[1] <<16) + (outb[2]<<8) + (outb[3])),fTemp
