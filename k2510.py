# xDevs.com module for Keithley 2510
# http://xdevs.com/guide/ni_gpib_rpi/
import os.path
import sys
import Gpib
import time
import ftplib
import numbers
import signal

cnt = 0
tec_rtd = 0.0
tec_curr = 0.0

class Timeout():
  """Timeout class using ALARM signal"""
  class Timeout(Exception): pass

  def __init__(self, sec):
    self.sec = sec

  def __enter__(self):
    signal.signal(signal.SIGALRM, self.raise_timeout)
    signal.alarm(self.sec)

  def __exit__(self, *args):
    signal.alarm(0) # disable alarm

  def raise_timeout(self, *args):
    raise Timeout.Timeout()

class tec_meter():
    temp = 38.5
    data = ""
    ppm = 0.0
    status_flag = 1
    temp_status_flag = 1
    global exttemp
    global rh
    global hectopascals
    global tec_rtd

    def __init__(self,gpib,reflevel,name):
        self.gpib = gpib
        self.inst = Gpib.Gpib(0,self.gpib, timeout=60) # SCPI GPIB Address = self.gpib
        self.reflevel = reflevel
        self.name = name
	self.tec_rtd = tec_rtd
        self.init_inst()

    def init_inst(self):
        # Setup SCPI DMM
	#self.inst.write("*rst") #reset TEC controller
	#self.inst.write("*CLR")
	#Set temperature transducer to thermistor and 4-wire sense mode
	self.inst.write(":sens:temp:tran rtd")      #select thermistor
	self.inst.write(":sens:temp:rtd:alph 0.00375") #10 kOhm thermistor
	self.inst.write(":sens:temp:rtd:beta 0.16") #10 kOhm thermistor
	self.inst.write(":sens:temp:rtd:delt 1.605") #10 kOhm thermistor
	self.inst.write(":sens:temp:rtd:rang 1000") #10 kOhm thermistor
	self.inst.write(":sens:temp:curr 8.333e-4") #10 kOhm thermistor
	self.inst.write(":syst:rsen on")            #4-wire mode enabled
	#Set voltage limit
	self.inst.write(":sour:volt:prot 9.5") #+9.5V limit
	#Set current limit
	self.inst.write(":sens:curr:prot 1.2") #2.5A limit
	#Set temperature limit*
	self.inst.write(":sour:temp:prot:high 65")  #75C max temp
	self.inst.write(":sour:temp:prot:low 5")   #15C min temp
	self.inst.write(":sour:temp:lcon:GAIN 270")   #15C min temp
	self.inst.write(":sour:temp:lcon:INT 0.05")   #15C min temp
	self.inst.write(":sour:temp:lcon:DER 0.02")   #15C min temp
	self.inst.write(":sour:temp:prot:state ON") #enable temp #protection (default)

    def cfg_temp(self):
	self.inst.write(":sour:temp 25.0") #set temp
        self.inst.write(":OUTP ON")
	print "\nTEC SMU Configured for 25.0 C"
	tec_rtd = 25.0

    def off_temp(self):
        self.inst.write(":OUTP OFF")

    def deduct_tmp(self,tmp):
	string = float(tmp)
	print ("Setting %2.1f" % string)
	self.inst.write(":sour:temp %2.1f" % string) 
        self.inst.write(":OUTP ON")

    def read_data(self,cmd):
        data_float = 0.0
        data_str = ""
        self.inst.write(cmd)
	try:
            with Timeout(20):
                data_str = self.inst.read()
        except Timeout.Timeout:
            print ("Timeout exception from dmm %s on read_data() inst.read()\n" % self.name)
            return (0,float(0))
        #print ("Reading from dmm %s = %s" % (self.name,data_str))
        try:
            data_float = float(data_str)
        except ValueError:
            print("Exception thrown by dmm %s on read_data() - ValueError = %s\n" % (self.name,data_str))
            return (0,float(0)) # Exception on float conversion, 0 = error
        return (1,data_float) # Good read, 1 = converted to float w/o exception

    def get_data(self):
	global tec_rtd
	global tec_curr
	self.status_flag,data = self.read_data(":MEAS:CURR?")
        tec_curr = float(data)
        if (self.status_flag):
            self.data = data

	self.status_flag,data = self.read_data(":MEAS:TEMP?")
        if (self.status_flag):
            self.data = data
        tec_rtd = float(data)
    	return tec_rtd,tec_curr

    def get_data_status(self):
        return self.status_flag

    def write_data(self,fileHandle):
	#print ("TEC TC:%2.3f " % (float(self.data) ) )
	tec_rtd = float(self.data)
	fileHandle.write(";%2.3f;\r\n" % tec_rtd);
        #print time.strftime("%d/%m/%Y-%H:%M:%S;") + ("\033[1;31m[%8d]: %2.8f , dev %4.2f ppm,\033[1;39m  EXT_T:%3.2f , RH:%3.2f , Press:%4.2f hPa" % (cnt, float(self.data),float(self.ppm),float(exttemp),float(rh),float(hectopascals) ) )
        #fileHandle.write (time.strftime("%d/%m/%Y-%H:%M:%S;") + ("%16.8f;%16.8f;%3.1f;%3.2f;%3.2f;%4.2f;\r\n" % (float(self.data),float(self.reflevel),float(self.ppm),float(exttemp),float(rh),float(hectopascals) ) ))

    def print_ppm(self):
        #self.inst.write(":DISP:WIND2:TEXT:DATA \"%3.3f ppm\"" % float(self.ppm))
	tec_rtd = float(self.data)
	#print ("%2.3f" % tec_rtd )
