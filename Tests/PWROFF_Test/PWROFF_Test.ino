#include <TimerOne.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SoftwareSerial.h>
#include <SPI.h>

/********************************************//**
 *  Parameters & Variables
 ***********************************************/
//System declarations
#define TICK        20          //System Tick in ms !!!Recalculate Turn off time!!!

//PIN declarations
#define BATTERY     0           //battery balance on Analag 0
#define PWRON       3
#define PWROFF      4
#define RxD         7
#define TxD         6
#define LED         8
#define CSCAN       10

//system controls
#define DEBUG_ENABLED  1

//Declarations
SoftwareSerial blueToothSerial(RxD,TxD);

MCP_CAN CAN(CSCAN);

//Constants and Variables initialized
const int left = 266;           //motor left
const int right = 298;          //motor right
const int speedLeft = 256;      //CAN FRAME ID for speed left wheel
const int speedRight = 288;     //speed right wheel
const int odoLeft = 257;        //odo ticks left wheel
const int odoRight = 289;       //odo ticks right wheel
int lastlSpeed = 0;
int lastrSpeed = 0;

//Battery Surveillance Parameters
static const float turnOffV = 3.4;      //device turn off at battery cell voltage
static const float analogRef = 5.0;     //ADC reference voltage
static const int adcRes  = 1024;        //ADC Resolution
float cellVoltage = 0;                  //battery cell voltage
int turnOff   = 0;                      //turn device off flag

//Automatic Turn Off Parameters
static const unsigned long turnOfftime = 10UL * (TICK * 2.5);     //turn off value in seconds
static unsigned long watchCounter = 0UL;                       //counter turn off watchdog

/********************************************//**
 *  System Setup
 ***********************************************/
void setup()
{
  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PWROFF,INPUT);
  pinMode(PWRON, OUTPUT);

  //turn on Rollator if powered
  digitalWrite(PWRON, HIGH);

  Timer1.initialize(TICK * 1000);
  Timer1.attachInterrupt( BlinkISR );
  
}

/********************************************//**
 *  Main
 ***********************************************/
void loop()
{
  int lSpeed, rSpeed;

  if(digitalRead(PWROFF) == 0)
  {
    Serial.println("Power off!");
    digitalWrite(PWRON, LOW);
  }

  
}

/********************************************//**
 *  Functions
 ***********************************************/
void readBlueTooth(int *left, int *right)
{

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

void turnOffWatch()
{
  watchCounter++;
  if(watchCounter >= turnOfftime)
  {
    digitalWrite(PWRON, LOW);
  }
}

void batteryStatus()
{
  cellVoltage = analogRead(BATTERY);
  cellVoltage = (cellVoltage * analogRef) / adcRes;     //Voltage at Analog Port
  Serial.print("Voltage at A0:\t\t");
  Serial.println(cellVoltage);
  cellVoltage = (cellVoltage * 242) / 22;               //Resistor Divider R1 + R2 = 242k, R2 = 22k
  Serial.print("Voltage at Balance-Port:\t");
  Serial.println(cellVoltage);
  cellVoltage = cellVoltage/6;                          //calc cell voltage
  
  //on low battery voltage, turn off Rollator
  if(cellVoltage <= turnOffV)
  {
    turnOff = 1;
    Serial.println("Turn off Rollator!");
    //digitalWrite(PWRON, LOW);
  }
}

void BlinkISR()
{
  // System Tick visible on Pin 8
  digitalWrite(LED, digitalRead(LED)^1);
  //batteryStatus();
  turnOffWatch();
}

