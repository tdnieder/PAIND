/**
 * This program sets the motor current to 1 A about 1 s after Startup
 * the motor current is repeatedly set, to prevent auto-turnoff
 * after the step ist set, the CAN-Bus is read till the usb plug is ripped off because the rollator drove off.
 * 
 * @author Fabian Niederberger
 * 
 */

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SoftwareSerial.h>   //Software Serial Port
#include <SPI.h>


#define RxD         0
#define TxD         1
#define SPI_CS_PIN  10                                //Chipselect CAN on Pin 10

MCP_CAN CAN(SPI_CS_PIN);        

const int odoCount = 10;                                // 1s / 0.04s = 25 for safety remove 3 cycles = 22
const int left = 266;                                   // motor left
const int right = 298;                                  // motor right
const int speedLeft = 256;                              // CAN FRAME ID for speed left wheel
const int speedRight = 288;                             // speed right wheel
const int odoLeft = 257;                                // odo ticks left wheel
const int odoRight = 289;                               // odo ticks right wheel
int lastlSpeed = 0;
int lastrSpeed = 0;
int current = 0;                                        // Current to the Wheels

unsigned long lastMillis = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  delay(500);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  delay(2000);
  
  //Users chooses Current for Step Response run
  Serial.print("Enter Current in mA:\t");
  delay(2000);
  
  while(Serial.available() == 0)
  {
    //wait till user enters value for current
  }
  current = Serial.parseInt();                          //Parses Byte-Array in Buffer to int. No negative detection!
  
  if(current > 2000)
  {
    current = 2000;
  }
  if(current < 0)
  {
    current = 0;
  }
  
  Serial.println("Ready, Set.. (0.0 A)");
  delay(1000);
  
  sendCurrentToWheel(left, current);                    // Defined Current to left wheel
  sendCurrentToWheel(right, current);
  String printing1 = "Go! (";
  String printing2 = "mA)";
  Serial.println(printing1 + current + printing2);
}

void loop()
{
  //Generating Step-Response signal freq. < 1Hz
  //Max Current 2A
  int i = 0;
  
  //resend command in order to keep it running
  sendCurrentToWheel(left, current);                    // current to left wheel
  sendCurrentToWheel(right, current);
  
  for(i = 0; i<odoCount; i++)
  {
    readSpeed();
    delay(20);
  }
}

union {
  int integer;
  unsigned char byte[2];
} int2byte;

void sendCurrentToWheel(int side, int wheelSpeed)
{
  unsigned char sendBuf[5];
  int wheelCurrent = -2*wheelSpeed;

  int2byte.integer = wheelCurrent/3;
  sendBuf[0] = 2;
  sendBuf[1] = 0;
  sendBuf[2] = 0;
  sendBuf[3] = int2byte.byte[1];
  sendBuf[4] = int2byte.byte[0];

  CAN.sendMsgBuf(side, 0, 5, sendBuf);
}

void readSpeed()
{
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char val;

  if(CAN_MSGAVAIL == CAN.checkReceive())                // check if data coming
  {
    CAN.readMsgBuf(&len, buf);                          // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
        
    //Preparation Excel table handling (Termite serial monitor adds time stamp to each message)
    Serial.print(canId);


    for(int i = 0; i<len; i++)                      // print the data
    {
      val = 0;
      Serial.print(",\t");
      if((canId == 256) || (canId == 288)){         //CAN-Frame is Speed/Accel. Calculate usable value
        val = 256 - buf[i];
        Serial.print(val);
      }
      else{
        Serial.print(buf[i]);
      } 
    }
    Serial.println(";");
  }
}
