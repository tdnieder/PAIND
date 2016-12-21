#include <TimerOne.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SoftwareSerial.h>   //Software Serial Port
#include <SPI.h>

#define BATTERY     0
#define PWRON       3
#define PWROFF      4
#define RxD         7
#define TxD         6
#define LED         8

#define DEBUG_ENABLED  1

SoftwareSerial blueToothSerial(RxD,TxD);

MCP_CAN CAN(10);

const int left = 266;           //motor left
const int right = 298;          //motor right
const int speedLeft = 256;      //CAN FRAME ID for speed left wheel
const int speedRight = 288;     //speed right wheel
const int odoLeft = 257;        //odo ticks left wheel
const int odoRight = 289;       //odo ticks right wheel
int lastlSpeed = 0;
int lastrSpeed = 0;
int batVoltage = 0;                //battery voltage level

int counter = 20;
int count = 0;

unsigned long lastMillis = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PWROFF,INPUT);
  pinMode(PWRON, OUTPUT);

  digitalWrite(PWRON, HIGH);

  Timer1.initialize(500000);
  Timer1.attachInterrupt( BlinkISR );
  
  setupBlueToothConnection();
  delay(500);
  setupCan();
}

void loop()
{
  int lSpeed, rSpeed;

  if(digitalRead(PWROFF) == 0)
  {
    Serial.println("Power off!");
    digitalWrite(PWRON, LOW);
  }

  readBlueTooth(&lSpeed, &rSpeed);

  //Deceleration of one wheel
  if((lSpeed == 0 || rSpeed == 0) && (lastlSpeed != 0 || lastrSpeed != 0))
  {
    sendSpeedToWheel(left, lSpeed);
    delay(20);
    sendSpeedToWheel(right, rSpeed);
    Serial.println("Speed: " + String(lSpeed) + "/" + String(rSpeed) + ";");
  }
  //Deceleration of both wheels
  else if((lSpeed == 0 && rSpeed == 0) && (lastlSpeed != 0 || lastrSpeed != 0))
  {
    for(int i = 0; i < 3; i++)
    {
      sendCurrentToWheel(left, -2*lastlSpeed);
      delay(20);
      sendCurrentToWheel(right, -2*lastrSpeed);
      Serial.println(" -> " + String(-lastlSpeed) + "/" + String(-lastrSpeed) + ";");
      delay(200);
    }
  }
  //Acceleration of both wheels from standing still
  else if((lSpeed != 0 || rSpeed != 0) && (lastlSpeed == 0 && lastrSpeed == 0))
  {
    for(int i = 0; i < 3; i++)
    {
      sendCurrentToWheel(left, 3*lSpeed);
      delay(20);
      sendCurrentToWheel(right, 3*rSpeed);
      Serial.println(" -> " + String(lSpeed) + "/" + String(rSpeed) + ";");
      delay(200);
    }
  }
  else
  {
    sendCurrentToWheel(left, lSpeed);
    delay(20);
    sendCurrentToWheel(right, rSpeed);
    Serial.println(" -> " + String(lSpeed) + "/" + String(rSpeed) + ";");
  }
  lastlSpeed = lSpeed;
  lastrSpeed = rSpeed;
}

void readBlueTooth(int *left, int *right)
{
  char recvChar;
  String recvString = "";
  boolean startString = false;
  
  while(1)
  {
    if(blueToothSerial.available())
    {
      recvChar = blueToothSerial.read();
      if(recvChar == 's' && !startString)
      {
        //String starts here.
        recvString = String(recvChar);
        startString = true;
      }
      else if(startString)
      {
        recvString += recvChar;
        if(recvChar == ';')
          break;
      }
    }
//    delay(50);
  }
//  Serial.print(recvString);

  int indexOfDelimiter, indexOfEnd;
  String lValue, rValue;
  
  indexOfDelimiter = recvString.indexOf('/');
  indexOfEnd = recvString.indexOf(';');

  *left = (recvString.substring(1, indexOfDelimiter)).toInt();
  *right = (recvString.substring(indexOfDelimiter + 1, indexOfEnd)).toInt();
}

void setupBlueToothConnection()
{
    blueToothSerial.begin(38400);                           // Set BluetoothBee BaudRate to default baud rate 38400
    blueToothSerial.print("\r\n+STWMOD=0\r\n");             // set the bluetooth work in slave mode
    blueToothSerial.print("\r\n+STNA=SeeedBTSlave\r\n");    // set the bluetooth name as "SeeedBTSlave"
    blueToothSerial.print("\r\n+STOAUT=1\r\n");             // Permit Paired device to connect me
    blueToothSerial.print("\r\n+STAUTO=0\r\n");             // Auto-connection should be forbidden here
    delay(2000);                                            // This delay is required.
    blueToothSerial.print("\r\n+INQ=1\r\n");                // make the slave bluetooth inquirable
    Serial.println("The slave is inquirable!");
    delay(2000);                                            // This delay is required.
    blueToothSerial.flush();
}

void setupCan()
{
START_SETUP:
  if(CAN_OK == CAN.begin(CAN_1000KBPS))
  {
    Serial.println("CAN Init ok");
  }
  else
  {
    Serial.println("Can't init CAN");
    Serial.println("Trying again");
    delay(100);
    goto START_SETUP;
  }
}

union {
  int integer;
  unsigned char byte[2];
} int2byte;

void sendSpeedToWheel(int side, int wheelSpeed)
{
  unsigned char sendBuf[5];
  int2byte.integer = wheelSpeed;
  sendBuf[0] = 1;
  sendBuf[1] = int2byte.byte[0];  //arduino int is 2 bytes (16 bit)
  sendBuf[2] = int2byte.byte[1];
  sendBuf[3] = 0;
  sendBuf[4] = 0;
  
  CAN.sendMsgBuf(side, 0, 5, sendBuf);
}

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

void batteryStatus()
{
  batVoltage  = analogRead(BATTERY);
  Serial.println(batVoltage); 
}

void BlinkISR()
{
  // Toggle LED
    digitalWrite( LED, digitalRead( LED ) ^ 1 );
    batteryStatus();
    count++;
    if(count = counter){
      Serial.println("Power off!");
      digitalWrite(PWROFF, LOW);
    }
}

