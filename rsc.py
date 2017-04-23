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


if cfg.get('main', 'if_debug', 1) == 'false':
    if cfg.get('main', 'interface', 1) == 'i2c':
        raise ValueError ('I2C interface not supported, change to SPI')
    if cfg.get('main', 'interface', 1) == 'spi':
        import spidev
        spi = spidev.SpiDev()
	spi.mode = 0b00

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

print GPIO.RPI_INFO

class HRSC(object):
    def __init__(self, mode=HRSC, i2c=None, **kwargs):
        self._logger = logging.getLogger('Adafruit_BMP.BMP085')
        # Check that mode is valid.
	self._mode = mode
        # Create device.
	global bus
	print ("TEST")
                        
	self.read_eeprom()
        # Load calibration values.
        #self._load_calibration()
        #self.t_fine = 0.0
    
    def read_eeprom(self):
        print "Loading EEPROM data from sensor",
        print ".",
        # Assert EEPROM SS to L, Deassert ADC SS to H, Set mode 0 or mode 4
        spi.mode = 0b00
	spi.open(0, 1)
	for i in range (0,41):
	    sensor_rom[i] = spi.xfer([HRSC_EAD_EEPROM_LSB, i, 0x00], 100000)[2] # Read low page
	    print "%c" % (sensor_rom[i]),
#	for i in range (0,255):
#    	    spi.open(0, 1)
#    	    sensor_rom[i+256] = spi.xfer2([HRSC_EAD_EEPROM_MSB, i, 0x00], 100000) # Read high page
	# Clear EEPROM SS , set mode 1 for ADC
        spi.close()
	print "TEST"
        print sensor_rom[0:16][2],
    	print " OK"
        return 0
        
    def adc_configure(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        spi.mode = 0b01
	spi.open(0,0)
	self.bytewr = 3
        self.regaddr = 0
        
        # Reset command
        spi.xfer(HRSC_ADC_RESET)
        # Write configuration registers from ROM
        spi.xfer([HRSC_ADC_WREG|self.regaddr << 3|self.bytewr & 0x03, sensor_rom[61], sensor_rom[63], sensor_rom[65], sensor_rom[67] ])
        
	spi.close()
        return 1
    
    def sensor_info(self):
        # Check for correct status
        print "Catalog listing = %s" % sensor_rom[0:16]
        print "Serial number   = %s" % sensor_rom[17:27]
        print "Pressure range  = %f" % float(sensor_rom[28:31])
        print "Pressure min    = %f" % float(sensor_rom[32:35])
        print "Pressure units  = %s" % sensor_rom[36:40]
        print "Pressure ref    = %s" % sensor_rom[41]
        print "Checksum        = %X" % int(sensor_rom[451:452])
        return 1
    
    def calibrate(self):
        return 1
    
    def set_speed(self, speed):
        return 1
    
    def read_temp(self):
        return 1
        
    def read_pressure(self):
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
        