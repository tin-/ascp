# Copyright (c) 2017 xDevs.com
# Author: Illya Tsemenko
#
# Based on the RSC datasheet
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

# HRSC Commands
HRSC_SINGLE_READ_CMD = 0xAA
HRSC_AVG2_READ_CMD   = 0xAC
HRSC_AVG4_READ_CMD   = 0xAD
HRSC_AVG8_READ_CMD   = 0xAE
HRSC_AVG16_READ_CMD  = 0xAF

# HRSC Status bits
HRSC_STS_ZERO          = 0b10000000
HRSC_STS_PWRON         = 0b01000000
HRSC_STS_BUSY          = 0b00100000
HRSC_STS_MODE          = 0b00011000
HRSC_STS_EEPROM_CHKERR = 0b00000100
HRSC_STS_SNSCFG        = 0b00000010
HRSC_STS_ALUERR        = 0b00000001

HRSC_EAD_EEPROM_LSB    = 0x03
HRSC_EAD_EEPROM_MSB    = 0x0B
HRSC_ADC_WREG          = 0x40 
HRSC_ADC_RESET         = 0x06

# HRSC ROM values
sensor_rom = [0] * 512          # 512 bytes of EEPROM

Prange   = 0.0                  # Pressure range from ROM 
Pmin     = 0.0                  # Pressure offset from ROM
EngUnit  = 0.0                  # Engineering units from ROM
Praw     = 0.0                  # Uncompensated pressure from ADC
Traw     = 0.0                  # Uncompensated temperature from ADC
Pint1    = 0.0                  # Intermediate value 1
Pint2    = 0.0                  # Intermediate value 2
Pcomp_fs = 0.0                  # Compensated output pressure 
Pcomp    = 0.0                  # Compensated output pressure, in units


try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN)

print "Running on   : %s" % GPIO.RPI_INFO['TYPE']
print "Processor    : %s" % GPIO.RPI_INFO['PROCESSOR']
print "Revision     : %s" % GPIO.RPI_INFO['REVISION']
print "Manufacturer : %s" % GPIO.RPI_INFO['MANUFACTURER']

import spidev
spi = spidev.SpiDev()
spi.open(0, 1)
spi.mode = 0b00
spi.close()
spi.open(0, 0)
spi.mode = 0b01
spi.close()

class HRSC(object):
    def __init__(self, i2c=None, **kwargs):
        self._logger = logging.getLogger('Adafruit_BMP.BMP085')
        # Create device.
	print ("TEST")
                        
	self.read_eeprom()
        # Load calibration values.
        #self._load_calibration()
        #self.t_fine = 0.0
    
    def read_eeprom(self):
        print "Loading EEPROM data from sensor",
        print ".",
        # Assert EEPROM SS to L, Deassert ADC SS to H, Set mode 0 or mode 4
        spi.open(0, 1)
	spi.mode = 0b00
	for i in range (0,255):
	    sensor_rom[i] = spi.xfer([HRSC_EAD_EEPROM_LSB, i, 0x00], 100000)[2] # Read low page
#	    print "%c" % (sensor_rom[i]),
	for i in range (0,255):
    	    sensor_rom[i+256] = spi.xfer([HRSC_EAD_EEPROM_MSB, i, 0x00], 100000)[2] # Read high page
	# Clear EEPROM SS , set mode 1 for ADC
        spi.close()
        return 0
        
    def adc_configure(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
	spi.open(0, 0)
        spi.mode = 1
	self.bytewr = 3
        self.regaddr = 0

        # Reset command
        test = spi.xfer([HRSC_ADC_RESET], 10000)
	time.sleep(1)
	# Write configuration registers from ROM
	print ("%02X" % sensor_rom[61]),
	print ("%02X" % sensor_rom[63]),
	print ("%02X" % sensor_rom[65]),
	print ("%02X" % sensor_rom[67]),
        test = spi.xfer2([HRSC_ADC_WREG|self.regaddr << 3|self.bytewr & 0x03, sensor_rom[61], sensor_rom[63], sensor_rom[65], sensor_rom[67] ], 10000)
        
	spi.close()
        return 1


    def conv_to_float(self, byte1, byte2, byte3, byte4):
        import struct
	temp = struct.pack("BBBB", byte1,byte2,byte3,byte4)
	output = struct.unpack("<f", temp)[0]
        return output

    def conv_to_short(self, byte1, byte2):
        import struct
	temp = struct.pack("BB", byte1,byte2)
	output = struct.unpack("<H", temp)[0]
        return output

    
    def sensor_info(self):
        # Check for correct status
        print "\033[0;32mCatalog listing : %s" % str(bytearray(sensor_rom[0:16]))
        print "Serial number   : %s" % str(bytearray(sensor_rom[16:27]))
        print "Pressure range  :",
        b = self.conv_to_float(sensor_rom[27], sensor_rom[28], sensor_rom[29], sensor_rom[30])
	print b,sensor_rom[27:31]
        print "Pressure min    :",
	b = self.conv_to_float(sensor_rom[31], sensor_rom[32], sensor_rom[33], sensor_rom[34])
	print b,sensor_rom[31:35]
                
	print "Pressure units  : %s" % str(bytearray(sensor_rom[35:40]))
        print "Pressure ref    :",
	if (sensor_rom[40] == 68):
	    print "Differential"
        print "Checksum        :",
	b = self.conv_to_short(sensor_rom[450], sensor_rom[451])
	print b,sensor_rom[450:452]
        print "\033[0;39m"
	return 1
    
    def calibrate(self):
        return 1
    
    def set_speed(self, speed):
	# Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
	spi.open(0, 0)
        spi.mode = 1
	self.bytewr = 0
        self.regaddr = 1

	if (speed == 20):
	    reg_dr = 0
	elif (speed == 45):
	    reg_dr = 1
	elif (speed == 90):
	    reg_dr = 2
	elif (speed == 145):
	    reg_dr = 3
	reg_mode = 0 # 256kHz modulator
	reg_sensor = 0 # pressure
	self.reg_wr = (reg_dr << 5) | (reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
	# Write configuration register
	command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
	print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (command, self.reg_wr))
        test = spi.xfer([command, self.reg_wr], 10000)
        print test

	spi.close()
        return 1

    def convert_temp(self, raw_temp):
	raw = (raw_temp & 0xFFFF00) >> 10
	if (raw & 0x2000):
	    #print "MSB is 1, negative temp"
	    raw = (0x3fff - (raw - 1))
	    temp = -(float(raw) * 0.03125)
	else:
	    #print "MSB is 0, positive temp"
	    temp = (float(raw) * 0.03125)
        print "RAW: %s %s , %4.3f" % (hex(raw_temp), hex(raw), temp )
#	with open('rsc_temp.dsv', 'ab') as o:
#	    o.write (time.strftime("%d/%m/%Y-%H:%M:%S;")+('%4.5f;%3.3f;\r\n' % (0.0, temp)))
	return temp
    
    def read_temp(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
	spi.open(0, 0)
        spi.mode = 1
	self.bytewr = 0
        self.regaddr = 1
	spi.max_speed_hz = 10000

	reg_dr = 0
	reg_mode = 0 # 256kHz modulator
	reg_sensor = 1 # temperature
	self.reg_wr = (reg_dr << 5) | (reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
	# Write configuration register
	command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
	print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (command, self.reg_wr))
        test = spi.xfer([command, self.reg_wr], 10000)

	twait = 0.066

#	with open('rsc_temp.dsv', 'wb') as o:
#	    o.write ("date;press;temp;\r\n")

	while(1):
    	    time.sleep(twait)
	    adc_data = spi.xfer([0,0,command, self.reg_wr], 10000)
	    #print "RDATA = ",
	    #print adc_data
	    temp_data = (adc_data[0]<<16|adc_data[1]<<8|adc_data[2])
	    self.convert_temp(temp_data)
	spi.close()
	return 1
        
    def read_pressure(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
	spi.open(0, 0)
        spi.mode = 1
	self.bytewr = 0
        self.regaddr = 1

	reg_dr = 0
	reg_mode = 0 # 256kHz modulator
	reg_sensor = 0 # pressure
	self.reg_wr = (reg_dr << 5) | (reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
	# Write configuration register
	print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (self.regaddr, HRSC_ADC_WREG|self.reg_wr))
        test = spi.xfer([0x00, HRSC_ADC_WREG|self.regaddr << 3|self.bytewr & 0x03, self.reg_wr, self.reg_wr ], 10000)

        time.sleep(1)
	adc_data = spi.xfer([0x00, 0x00, 0x00, 0x00], 10000)
	print adc_data
        time.sleep(1)
	adc_data = spi.xfer([0x00, 0x00, 0x00, 0x00], 10000)
	print adc_data
        time.sleep(1)
	adc_data = spi.xfer([0x00, 0x00, 0x00, 0x00], 10000)
	print adc_data
        time.sleep(1)
	adc_data = spi.xfer([0x00, 0x00, 0x00, 0x00], 10000)
	print adc_data

	spi.close()
	return 1
        
    def comp_readings(self):
        return 1
    
    def read_sensor(self):
        outb = [0,0,0,0,0,0,0,0]
    
        if cfg.get('main', 'if_debug', 1) == 'true':
            StatusByte = 0xFF
            outb[0] = 0x55
            outb[1] = 0x55
            outb[2] = 0x55
            outb[3] = 0x55
            outb[4] = 0x55
            outb[5] = 0x55
        
        print "Status: 0x%X " % StatusByte,
        print "Pressure: %4.5f %%FSS " % fPress,
        print "Temperature: %3.2f 'C " % fTemp,
        
        print " 0x%04X %04X %04X " % (outb[0], outb[1], outb[2]),
        