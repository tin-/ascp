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
from __future__ import division
import imp
import sys
import os
import time
import ConfigParser
from console import *
from bme280 import *

cfg = ConfigParser.ConfigParser()
cfg.read('pressure.conf')
cfg.sections()

sys.stdout.write("\x1b[2J")
sys.stdout.flush()

cursor_pos(0,0)
print_pos(0,0, "R")

if cfg.get('main', 'interface', 1) == 'i2c':
    raise ValueError ('I2C interface not supported, change to SPI')
if cfg.get('main', 'interface', 1) == 'spi':
    if cfg.get('dut', 'dut', 1) == 'asc_dlhr':
        # ASC DLHR sensor with SPI interface
        print ("ASC sensor used")
	from dlhr import *
        sensorhw = imp.load_source('dlhr', 'dlhr.py')
        sensor = sensorhw.ASC_DLHR(mode=ASC_DLHR_I2CADDR)
    if cfg.get('dut', 'dut', 1) == 'rsc':
        # RSC sensor with SPI interface
        print ("RSC sensor used")
	#from rsc import *
        sensorrs = imp.load_source('rsc', 'rsc.py')
        sensorrsc = sensorrs.HRSC()
    if cfg.get('dut', 'dut', 1) == 'all':
        # ASC DLHR sensor with SPI interface
        print ("Both sensors used")
	from dlhr import *
        from rsc import *
        sensorhw = imp.load_source('dlhr', 'dlhr.py')
        sensorasc = sensorhw.ASC_DLHR(mode=ASC_DLHR_I2CADDR)
        sensorhws = imp.load_source('rsc', 'rsc.py')
        sensorrsc = sensorhws.HRSC(mode=HRSC)

tecsmu = imp.load_source('tecm', 'k2510.py')
smu = tecsmu.tec_meter(25,0.0,"2510") # GPIB 25

tec_temp = 23.00
smu.deduct_tmp(tec_temp)
tec_rtd, tec_curr = smu.get_data()


menu = """\033[1;32m[0] - Help guide
[1] - Detect instruments
[2] - Initialize instruments
[3] - Select DUT
[4] - Run PT procedure on DUT
\033[0;44m[5] - Do performance test and autogenerate HTML report\033[1;32;49m
[6] - Generate HTML report
[7] - a: Read part Pressure & Temp
[8] - b: Read part 2x oversampled Pressure & Temp
[9] - c: Read part 4x oversampled Pressure & Temp
[10] - d: Read part 8x oversampled Pressure & Temp
[11] - e: Read part 16x oversampled Pressure & Temp
[12] - U: Toggle Warmup cycles on/off
[13] - Test RSC sensor
[14] - Log all sensors
[15] - Log only RSC
[16] - Set TEC to 26C
[X] - Quit\033[0;39m
"""

help = """\033[0;44m
This is help! Very good help!
To know more - visit https://xdevs.com site\033[0;39m
"""

exttemp = 25.0
rh = 50.0
pascals = 100000.00
hectopascals = 1000.00

# Read Temperature, Humidity, and Barometric Pressure from BME280
def get_THP():
    global exttemp
    global rh
    global pascals
    global hectopascals
    exttemp = sensor.read_temperature()
    rh = sensor.read_humidity()
    pascals = sensor.read_pressure()
    hectopascals = pascals / 100

#Setup temp/humidity/pressure sensor BME280
# OSAMPLE = oversampling mode
sensor = BME280(mode=BME280_OSAMPLE_8)
get_THP() # Read Temp, RH, Pressure from sensor

fileName = cfg.get('main', 'raw_data_filelog', 1)
# Check if file exists, if not create it and add header
def create_local_file(fileName):
    if (os.path.isfile(fileName) == False):
        with open(fileName, 'a') as o1:
            o1.write("date;asc_press;asc_temp;rsc_press;rsc_temp;bme_temp;bme_press;bme_rh;tec_box;tec_temp;\r\n")
            print ("file %s does not exist\r\n" % fileName) 
    else: 
        print ("file %s exists\r\n" % fileName)

def create_local_log(fileName):
    if (os.path.isfile(fileName) == False):
        with open(fileName, 'a') as o1:
            o1.write("date;rsc_press;rsc_temp;bme_temp;bme_press;bme_rh;tec_box;tec_temp;\r\n")
            print ("file %s does not exist\r\n" % fileName) 
    else: 
        print ("file %s exists\r\n" % fileName)

sys.stdout.write("\033[0;36m  Pressure sensors toolkit CLI \r\n  Using NI GPIB adapter and linux-gpib library \r\n\033[0;41m Target platform: Raspberry Pi B 3+\033[0;39m\r\n")

def GetChar(Block=True):
    if Block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    raise error('NoChar')

c = "*"
h = ("*")
while True:
    print c + h*78 + c
    print menu
    inputa = raw_input(" "*3 + "Input example: ")
    print c + h*78 + c

    if (inputa == "0"):
        print help

    if (inputa == "1"):
        with open('calkit.conf', 'w') as configfile:
            cfg.write(configfile)
        print ("Detecting function not written yet")

    if (inputa == "2"):
        print ("Initialize instruments not written yet")

    if (inputa == "3"):
        hp3458_setup()
        print ("Select DUT submenu not written yet")

    if (inputa == "4"):
        print ("Performance verification procedure on DUT written yet")

    if (inputa == "5"):
        print ("Generating HTML report..\033[0;49m")
        cal_report()
        print ("All done, aye!\033[0;39m")

    if (inputa == "6"):
        print ("Generating HTML report..\033[0;49m")
        mfc_report()
        print ("All done, aye!\033[0;39m")

    if (inputa == "7"):
        print ("Reading sensor Pressure/temp..\033[0;49m")
        sensorasc.write_cmd(ASC_DLHR_SINGLE_READ_CMD)
        sensorasc.read_sensor()
        print ("All done, aye!\033[0;39m")

    if (inputa == "8"):
        print ("Reading AVG2 sensor Pressure/temp..\033[0;49m")
        sensorasc.write_cmd(ASC_DLHR_AVG2_READ_CMD)
        sensorasc.read_sensor()
        print ("All done, aye!\033[0;39m")

    if (inputa == "9"):
        print ("Reading AVG4 sensor Pressure/temp..\033[0;49m")
        sensorasc.write_cmd(ASC_DLHR_AVG4_READ_CMD)
        sensorasc.read_sensor()
        print ("All done, aye!\033[0;39m")

    if (inputa == "10"):
        print ("Reading AVG8 sensor Pressure/temp..\033[0;49m")
        sensorasc.write_cmd(ASC_DLHR_AVG8_READ_CMD)
        sensorasc.read_sensor()
        print ("All done, aye!\033[0;39m")

    if (inputa == "11"):
        print ("Reading AVG16 sensor Pressure/temp..\033[0;49m")
        sensorasc.write_cmd(ASC_DLHR_AVG16_READ_CMD)
        sensorasc.read_sensor()
        print ("All done, aye!\033[0;39m")

    if (inputa == "12"):
        print ("Warmup sensor..\033[0;49m")
        DoReads = 0
        print "Reading sensor. Press any key to abort"
        while DoReads != 1:
            sensorasc.write_cmd(ASC_DLHR_AVG16_READ_CMD)
            time.sleep(0.166666)
            sensorasc.read_sensor()
            DoReads = 0
            
            #data = GetChar()
            #if data:
            #    DoReads = 0
            #    break
        
        print ("All done, aye!\033[0;39m")
    
    if (inputa == "13"):
        print ("Testing RSC")
        print "1. Read the ADC settings and the compensation values from EEPROM."
        sensorrsc.sensor_info()
        print "2. Initialize the ADC converter using the settings provided in EEPROM."
        sensorrsc.adc_configure()
        print "3. Adjust the ADC sample rate if desired."
	#sensorrsc.set_speed(20) #in SPS
        print "4. Command the ADC to take a temperature reading, and store this reading."
	sensorrsc.read_temp()
        print "5. Give Delay (Example: if sample rate is 330SPS delay for 3.03 ms [1/330 s])."
	time.sleep(0.004)
        print "6. Command the ADC to take a pressure reading, and store this reading."
        sensorrsc.read_pressure()
        # 7. Apply the compensation formulae to the temperature and pressure readings in order to calculate a pressure value.
	sensorrsc.comp_readings(10,10)
        # 8. Repeat steps 4, 5 and 6 in a loop to take additional readings
        print sensorrsc.comp_readings(10,10)
        print ("All done!")

    if (inputa == "15"):

        print ("Testing RSC")
        sensorrsc.sensor_info()
        sensorrsc.adc_configure()
	sensorrsc.set_speed(20) #in SPS
	sensorrsc.read_temp()
	time.sleep(0.1)
        sensorrsc.read_pressure()

	DoReads = 0
        print "Reading sensor. Press any key to abort"
	create_local_log(fileName)
	cnt = 0
        while DoReads != 1:
	    #display_clear()

            rsc_temp = sensorrsc.read_temp()
    	    #rsc_press = sensorrsc.read_pressure()
            DoReads = 0

	    #Read BME280
	    get_THP()
	    tec_rtd, tec_curr = smu.get_data()
	    test = sensorrsc.comp_readings(rsc_temp, 10)
	    with open(fileName, 'a') as o1:
		print ("%8d RSC:%.6f;%2.3f \033[0;32mA %2.3f R %4.2f P %2.2f \033[0;33mTEC:%2.3f SV: %2.3f\033[0;39m" % (cnt, test, rsc_temp, exttemp, rh, hectopascals, tec_rtd, tec_temp) )
        	o1.write (time.strftime("%d/%m/%Y-%H:%M:%S;") + ("%.6f;%2.3f;%4.2f;%2.2f;%2.3f;%2.3f;%2.3f\r\n" % (test, rsc_temp, exttemp, rh, hectopascals, tec_rtd, tec_temp) ) )
		o1.close()
	    cnt = cnt + 1

    if (inputa == "16"):
	print "Setting 30C"
	smu.deduct_tmp(26.00)
	
    if (inputa == "14"):
	print "Logging start.."

        sensorrsc.sensor_info()
        sensorrsc.adc_configure()
	#sensorrsc.set_speed(20) #in SPS
	#sensorrsc.comp_readings(10,10)

	enable_start = 0
	enable_hold = 0
	reverse_dir = 0

	create_local_file(fileName)

	cnt = -2000
	while cnt <= 200000:
	    global tec_temp
	    if ( (cnt >= 100) and (enable_start == 0) ):
	        enable_start = 1
    		cnt = 0

	    if (enable_start == 1):
    		if (tec_temp >= 47.0 and reverse_dir == 0):
    	    	    reverse_dir = 1
	    	    cnt = 30000

    	    if ( (cnt > 0) and ((cnt % 100) == 0) and (reverse_dir == 0) ):
        	tec_temp = 23 + ((float(cnt) / 1000)*1)
        	print ("Set TEC Temp = %2.1f" % tec_temp)
		smu.deduct_tmp(tec_temp)

	    if ( (cnt >= 30000) ):
    		reverse_dir = 1

	    if (cnt >= 40000 and cnt <= 70000):
    		if ( (cnt > 0) and ((cnt % 100) == 0) and (reverse_dir == 1) ):
        	    tec_temp = 47 - ((float(cnt-40000) / 1000)*1)
        	    print ("Set TEC Temp = %2.1f" % tec_temp)
        	    if (cnt >= 0):
			smu.deduct_tmp(tec_temp)

	    if (cnt >= 80000):
		#smu.off_temp()
	        quit()
		#smu.deduct_tmp(tec_temp)

	    #Read BME280
	    get_THP()
	    #Read ASC DLHR
    	    sensorasc.write_cmd(ASC_DLHR_AVG16_READ_CMD)
	    asc_press, asc_temp = sensorasc.read_sensor()
	    #Read HW RSC
	    rsc_temp = sensorrsc.read_temp()
	    time.sleep(0.016)
    	    rsc_press = sensorrsc.read_pressure()
	    #Get K2510 data
	    tec_rtd, tec_curr = smu.get_data()
	    
	    with open(fileName, 'a') as o1:
		print ("%8d \033[0;35mASC:%d;%2.3f \033[1;34mRSC:%d;%2.3f \033[0;32mA %2.3f R %4.2f P %2.2f \033[0;33mTEC:%2.3f SV: %2.3f\033[0;39m" % (cnt, asc_press, asc_temp, rsc_press, rsc_temp, exttemp, rh, hectopascals, tec_rtd, tec_temp) )
        	o1.write (time.strftime("%d/%m/%Y-%H:%M:%S;") + ("%d;%2.3f;%d;%2.3f;%2.3f;%4.2f;%2.2f;%2.3f;%2.3f\r\n" % (asc_press, asc_temp, rsc_press, rsc_temp, exttemp, rh, hectopascals, tec_rtd, tec_temp) ) )
		o1.close()
	    cnt = cnt + 1

    if (inputa == "X" or inputa == "x" or inputa == "Q" or inputa == "q"):
        print ("Bye-bye!")
        quit()