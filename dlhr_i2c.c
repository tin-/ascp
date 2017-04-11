# xDevs.com pressure test module
# Module for All Sensors DLHR pressure sensor:
## Using I2C interface
# For Raspberry Pi B 3+ with python 2.7.x
# 3.3V only platform
# Assumes sensor has default I2C address of 41

import csv

with open('test2.txt','wb') as o:
  
    rowcount = 19999
    for row in range(0, 19999):
        rowcount +=1
        #print  ('http://www.keithley.com/support/data?asset=%d'% (rowcount))
        o.write ('http://forum.keithley.com/phpBB3/download/file.php?id=%d\n'% (rowcount))
        
#const int V3PIN = PE_4;  
#const int EOCPIN = PA_3;   
#//   SCL:  PA_6
#//   SDA:  PA_7

void setup() 
{
  // set up the digital I/O:  Other than I2C port, 
  // Sensor EOC pin and Sensor power from GPIO:
  pinMode(EOCPIN, INPUT);    
  pinMode(V3PIN, OUTPUT);   
  digitalWrite(V3PIN, LOW);
  Wire.begin();
  
  Serial.begin(115000);  // start serial for output
  delay(100);
  
  Serial.print("[--- Connected. ---]\n");  
  print ("\n-------- Press H for Command List -------------\n");
}

uint8_t cmd[8];
uint8_t StatusByte, inbyte, cmdbyte, outb[12];
int DoReads;
float  fTemp, fPress;
uint32_t Preading[24], Treading[24];
int32_t Prs, Tmp, Temp;
boolean pwrON;
/*--------------------------------------------------------*/
void loop()
{
    
    // Wait for incoming command:
    if (Serial.available() > 0)    // Check serial port 
        cmdbyte = Serial.read();
    else cmdbyte = 0;
    
    if(cmdbyte == 'H')
    {
       print (" \n");
       print ("DLHR_I2C_demo.ino : 17/03/14- Commands: ");
      
       print ("O:  (letter)  Power ON");
       print ("F:  Power OFF\n");
       print ("a:  Read part Pressure & Temp");
       print ("b:  Read part 2x oversampled Pressure & Temp");
       print ("c:  Read part 4x oversampled Pressure & Temp");
       print ("d:  Read part 8x oversampled Pressure & Temp");
       print ("e:  Read part 16x oversampled Pressure & Temp");
       print ("U:  Toggle Warmup cycles on/off");
    }
    else if(cmdbyte == 'O')
    {
      Serial.print("ON\n");
      digitalWrite(V3PIN, HIGH);
      pwrON = true;
    }
    else if(cmdbyte == 'F')
    {
      Serial.print("OFF\n");
      digitalWrite(V3PIN, LOW);
      pwrON = false;
    }
  
    else if(cmdbyte == 'a')
    {
      Serial.print("Starting SingleRead.\n");
      Command_Only(0xAA);          
      Sensor_Read();
    }
    else if(cmdbyte == 'b')
    {
      Serial.print("Starting ReadAvg2.\n");
      Command_Only(0xAC);          
      Sensor_Read();
    }
    else if(cmdbyte == 'c')
    {
      Serial.print("Starting ReadAvg4.\n");
      Command_Only(0xAD);          
      Sensor_Read();
    }
    else if(cmdbyte == 'd')
    {
      Serial.print("Starting ReadAvg8.\n");
      Command_Only(0xAE);          
      Sensor_Read();
    }
    else if(cmdbyte == 'e')
    {
      Serial.print("Starting ReadAvg16.\n");
      Command_Only(0xAF);          
      Sensor_Read();
    }
    else if(cmdbyte == 'U')
    {
      if(!DoReads)
      {
        Serial.print("\nStarting Avg8 continuous readings... Press 'U' again to stop.\n");
        cmd[0] = 0xAE;  cmd[1] = 0;    cmd[2] = 0;      
        Command_Only(cmd[0]);          
        DoReads = true;
      }else
      {
        Serial.print("\nStopping readings.\n");
        DoReads = false;        
      }
    }

    //  For continuous reading:
    if (DoReads)
    {      
        Sensor_Read();
        // start next measurement:
        Command_Only(cmd[0]);          
        delay(200);
     }       
}
/*--------------------------------------------------------------------*/

boolean Command_Only(uint8_t cmdin )
{
   if (!pwrON)
   {
       Serial.print("\r\nSensor is off! \n");         // print the character
       return false;      
    }
       Wire.beginTransmission( 41 );
       Wire.write( cmdin);        // Command ( 1 byte)
       Wire.endTransmission();
       delay(30);
       return true;
}
    

void Sensor_Read(  )
{
   char prsout[32];
   if (!pwrON)
   {
       Serial.print("\r\nSensor is off! \n");         // print the character
       return;      
    }
  
    // wait for completion
    while (LOW == digitalRead(EOCPIN))
       delay(5);         

    Wire.requestFrom (41, 7);
    while(Wire.available() < 7)
      delay(20);
      
    StatusByte = Wire.read();    // receive a byte as character
  
    outb[0] = Wire.read();     // receive a byte as character
    outb[1] = Wire.read();    // receive a byte as character
    outb[2] = Wire.read();    // receive a byte as character
    outb[3] = Wire.read();    // receive a byte as character
    outb[4] = Wire.read();    // receive a byte as character
    outb[5] = Wire.read();    // receive a byte as character
    
    // --- convert Temperature to degrees C:
    Tmp = outb[3] << 8;
    Tmp += outb[4];
    fTemp = (float)Tmp;
    fTemp = (fTemp/65535.0)*125.0 - 40.0;
    
    // --- convert Pressure to %Full Scale Span ( +/- 100%)
    Prs = (outb[0] <<16) + (outb[1]<<8) + (outb[2]);
    Prs -= 0x7FFFFFL;
    fPress = ((float)Prs)/((float)0x800000);
    fPress *= 100.0;
    
    sprintf(prsout, " %4.5f pctFSS, ", fPress);
    Serial.print("Status: 0x");         // print the character
    Serial.print(StatusByte, HEX);         // print the character
    Serial.print("   Pressure: ");         // print the character
    Serial.print(prsout); //fPress);         // print the character
    Serial.print("  Temperature:");         // print the character
    Serial.print(fTemp);         // print the character
    Serial.print("'C (Values:");         // print the character
    sprintf(prsout, " 0x%02X%02X%02X, ", outb[0], outb[1], outb[2]);
    Serial.print(prsout);
    Tmp = outb[3] << 16;
    Tmp += outb[4] << 8;
    Tmp += outb[5];
    Serial.print(Tmp, HEX);         // print the character
    Serial.print(")\r\n");         // print the character
 
}
