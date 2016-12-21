/**
* FILENAME :        iWA_RC_Slave_LC_V2_0
*
* DESCRIPTION :
*       Software to Control E-Rollator from iHomeLab
*       Implemented are:
*         -Loop Controller PID
*         -Turn off on button press
*         -Time based automatic turn off (WatchDog)
*         -battery low turn off
*         -battery surveillance (ADC-Usage)
*       
*       Software is partially copied from existing
*       code with unknown author at iHomeLab. This software extends
*       the functionality of the E-Rollator.
*
* FUNCTIONS :
*       void setup(void)
*       void loop(void)
*       void setupBluetooth(void)
*       void setupCAN(void)
*    void readBluetooth(void)
*   void readCAN(void)
*   void turnOffWatch(void)
*   void batteryStatus(void)
*   void sendSpeedToWheel(int side, int wheelSpeed)
*   void sendCurrentToWheel(int side, int wheelSpeed)
*   void tickISR()
*
* NOTES :
*       System tick can be observed on Pin 8
*       content of this file belongs to iHomeLab
* 
* AUTHOR :    Fabian Niederberger        START DATE :    27.11.2016
*
* CHANGES :
* VERSION DATE      WHO     DETAIL
* 1.0     17.11.16  FN      Create file
* 2.0     02.12.16  FN      Joystick by Wire
*
*/

#include <TimerOne.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SoftwareSerial.h>
#include <SPI.h>

/********************************************//**
 *  Makros
 ***********************************************/
#define INIT_PIDVAR(X) PIDVar X = {.w_k = 0, .e_k = 0, .e_k_1 = 0, .u_k = 0, .u_p_k = 0, .u_i_k = 0, .u_i_k_1 = 0, .u_d_k = 0, .u_d_k_1 = 0, .v_k = 0, .u_arw_k = 0, .u_arw_k_1 = 0, .y_k = 0, .y_k_1 = 0}

/********************************************//**
 *  Parameters & Variables
 ***********************************************/
//System declarations
#define TICK        40         //System Tick in ms  !!!!when change, recalculate Turn-Off Time!!!!
#define BAUDRATE    115200      //Hardware Serial Baudrate (con. to PC)

//PIN declarations
#define BATTERY     A0           //battery balance on Analog 0
#define PWRON       3           //ON/OFF Transistor on Digital 3
#define PWROFF      4           //Power OFF Switch on Digital 4
#define RxD         7           //Software Rx on Digital 7
#define TxD         6           //Software Tx on Digital 6
#define LED         8           //Status LED Digital 8
#define CSCAN       10          //Chip select for CAN Shield Digital 10
#define JX      A2      //Joystick X value
#define JY      A3      //Joystick Y Value

//developers controls
#define CURRENT       1   //if 1 Motor is controlled by setting current
                                //if 0 Motor is controlled by setting speed
#define DEBUG_ENABLED   1   //debugging via serial port
#define PID_ON        0   //PID will be active
#define ROLLATOR_UNPLUGGED  0 //If Rollator is unplugged and tests are executed
#define MEASURE_TICK    1 //if 1: LED -> TICK, if 0: LED -> Main Loop

//Declarations
SoftwareSerial bluetoothSerial(RxD,TxD);

MCP_CAN CAN(CSCAN);             //CAN Shield

//PID Parameters
static const double Kp      = 13;     //proportional gain
static const double Ti      = 18;     //i factor
static const double Td      = 1;      //d factor
static const double Tr    = Ti;     //anti-reset windup (ARW) factor (defined by Th. Prud'homme)
static const double N   = 1/TICK;   //Filter for D-Part
static const int u_min      = -2000;    //actuator min, saturation of current
static const int u_max    = 2000;     //actuator max, saturation of current
static const double ad    = Td / (Td + N*TICK); //numerator D
static const double bd    = Kp * Td * N / (Td + N*TICK); //denominator D
static const double ai    = Kp * TICK / Ti; //numerator I
static const double ar    = TICK / Tr;  //numerator ARW

typedef struct {
  double w_k;     //target, given by joystick
  double e_k;     //target/actual error
  double e_k_1;     //controller error at time k-1
  double u_k;     //actuator at time k -> value for motor
  double u_p_k;     //proportional part
  double u_i_k;     //integral part
  double u_i_k_1;   //actuators value at time k-1
  double u_d_k;     //deviation part
  double u_d_k_1;   //deviation part at time k-1
  double v_k;     //actuator before ARW
  double u_arw_k;   //ARW at time k
  double u_arw_k_1;   //error anti-reset windup at time k-1
  double y_k;     //speed
  double y_k_1;     //speed at time k-1
} PIDVar;

//Motor controls
static const int setLeft        = 266;      //control motor Left
static const int setRight       = 298;
static const int getSpeedLeft   = 256;      //read motor Left
static const int getSpeedRight  = 288;
static const int getOdoLeft     = 257;      //odo ticks left wheel
static const int getOdoRight    = 289;      //odo ticks right wheel
PIDVar left;                //PID variables for left motor
PIDVar right;

//Battery Surveillance Parameters
static const float turnOffV     = 3.4;      //device turns off at this battery cell voltage
static const int batCheck = 120 * (TICK * 10);  //battery checking time in s
static const float analogRef    = 5.0;      //ADC reference voltage
static const int adcRes         = 1024;     //ADC Resolution
static float cellVoltage        = 0;        //battery cell voltage
static int checkBat             = 0;        //battery flag
static int batCount             = 0;        

//Automatic Turn Off Parameters
static const unsigned long turnOfftime = 600UL * (TICK * 10); //turn off value in seconds
static unsigned long watchCounter = 0UL;            //counter turn off watchdog

/**
 * an integer 16 bit is splitted into two 8 bit characters
 * <p>
 * this union is called before sending data to CAN shield
 */
union {
  int integer;
  unsigned char byte[2];
} int2byte;

/********************************************//**
 *  System Setup
 ***********************************************/
/**
 * Initializes Board and Parameters before entering the
 * main loop
 * <p>
 * This function runs only once on startup.
 */
void setup()
{
  unsigned long tOne = 0;
  //setup power Management
  pinMode(PWROFF,INPUT);
  pinMode(PWRON, OUTPUT);
  //turn on Rollator
  digitalWrite(PWRON, HIGH);

  //initialize Timer
  tOne = TICK * 1000UL;
  Timer1.initialize(tOne);        //Timer time in set in uSeconds
  Timer1.attachInterrupt(tickISR);
  
  //setup serial connections
  Serial.begin(BAUDRATE);         //Hardware UART to PC            
  pinMode(RxD, INPUT);          //Software UART to BT
  pinMode(TxD, OUTPUT);
  
  //setup status LED
  pinMode(LED, OUTPUT);

  if(!ROLLATOR_UNPLUGGED)
  {
    Serial.println("Setup Rollaltor related stuff...");
    //setup CAN shield
    setupCan();
  }

  //Init Variables for PID
  INIT_PIDVAR(left);
  INIT_PIDVAR(right);
}

/********************************************//**
 *  Main
 ***********************************************/
/**
 * All tasks will be done sequentially. The requests that
 * have been set in the ISRs will be valued and executed
 * <p>
 * This function runs endlessly after setup
 */
void loop()
{
  if(!MEASURE_TICK)
    digitalWrite(LED, digitalRead(LED)^1);

  //check button 2, when pressed, turn off
  if(digitalRead(PWROFF) == 0)
  {
    if(DEBUG_ENABLED)
      Serial.println("Switch 2 pressed, Shut-Down Rollator");
    digitalWrite(PWRON, LOW);
  }

  //check battery after defined time
  if(batCount >= batCheck)
  {
    batCount = 0;
    batteryStatus();
  }

  if (!ROLLATOR_UNPLUGGED)
  {
    //read joystick for target new target values
    readJoystick();
    //read actual values
    readCAN();

    if(CURRENT)
    {
      //send current values to motors
      sendCurrentToWheel(setLeft, ((int) left.u_k));
      sendCurrentToWheel(setRight, ((int) right.u_k));
      if(DEBUG_ENABLED)
      Serial.println("Current: " + String(left.u_k) + " / " + String(right.u_k));
    }
    else
    {
      //send Speed values to motors
      sendSpeedToWheel(setLeft, ((int) left.u_k));
      sendSpeedToWheel(setRight, ((int) right.u_k));
      if(DEBUG_ENABLED)
      Serial.println("Speed: " + String(left.u_k) + " / " + String(right.u_k));
    }
  }
}

/********************************************//**
 *  Setup Peripheral Components
 ***********************************************/

/**
 * Sends initializing commands to can shield to set it
 * up for communication with motor controller from the
 * wheels, left and right. 
 * <p>
 * This function is called once on setup.
 */
void setupCan()
{
START_SETUP:
  if(CAN_OK == CAN.begin(CAN_1000KBPS))
  {
    Serial.println("CAN Init ok");
  }
  else
  {
    Serial.println("Can't init CAN\nTrying again...");
    delay(100);
    goto START_SETUP;
  }
}

/********************************************//**
 *  Functions
 ***********************************************/
/**
 * Reads Jostick values for target Speed 
 * <p>
 * This function runs each loop.
 */
void readJoystick()
{
  int x = 0;
  int y = 0;

  x = analogRead(JX);
  y = analogRead(JY);
  x -= 512;
  y -= 512;

  //noise hyst.
  if (abs(x) < 20)
  x = 0;
  else if (x > 500)
  x = 500;
  else if (x < -500)
    x = -500;

  if (abs(y) < 20)
  y = 0;
  else if (y > 500)
  y = 500;
  else if (y < -500)
  y = -500;

  //calculate motor values
  if(y >= 0){
    right.w_k = (-x + y)/2;
    left.w_k = (x + y)/2;
  }
  else{
    right.w_k = (x + y)/2;
    left.w_k = (-x + y)/2;
  }

  if(DEBUG_ENABLED)
    Serial.println("Joystick:\t" + String(x) + "/"+ String(y) + "\t" + String(left.w_k) + "/" + String(right.w_k));
}

/**
 * reads speed information from CAN-Bus
 * <p>
 * this function runs each loop
 */
void readCAN()
{
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char val[2] = {0, 0};
  int sigVal[2] = {0, 0};

  if(CAN_MSGAVAIL == CAN.checkReceive())                // check if data incoming
  {
    CAN.readMsgBuf(&len, buf);                          // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
    //CAN-Frame is Speed/Accel. Calculate usable value
    if((canId == 256) || (canId == 288)){                           //show can ID
      for(int i = 0; i<2; i++)                          // read only first 2 bytes (speed)
      {
        val[i] = 256 - buf[i];
      }
      //if(DEBUG_ENABLED)
      //  Serial.println("Speed: " + String(canId) + ",\t" + String(val[0]) + ",\t" + String(val[1]) + ";");

      //prepare forward values for controlling
      if((val[0] > 0) && (val[0] < 128))
      {
        sigVal[0] = val[0] - 1;
        if(canId == 256)  //left
          left.y_k = sigVal[0] * 256 + val[1];
        if(canId == 288)
          right.y_k = sigVal[0] * 256 + val[1];
      }
      sigVal[1] = sigVal[0] * 256 + val[1]; 
      
      //prepare reverse values for controlling
      if((val[0] >= 128))
      {
        sigVal[0] = (int) val[0];
        sigVal[0] = sigVal[0] - 255;
        sigVal[1] = (int) val[1];
        sigVal[1] = (sigVal[1] - 255);
        
        if(canId == 256)
          left.y_k = (sigVal[0] * 256 + sigVal[1]);
        if(canId == 288)
          right.y_k = (sigVal[0] * 256 + sigVal[1]);
      }
      if (DEBUG_ENABLED)
        Serial.println("y_k: " + String(canId) + ",\t" + String(sigVal[1]) + ";");  
    }
  }
}


/**
 * increments a counter. If limit is reached Rollator will
 * be turned off to prevent battery discharge
 * <p>
 * This Function is called on each system tick
 */
void turnOffWatch()
{
  watchCounter++;
  if(watchCounter >= turnOfftime)
  {
    digitalWrite(PWRON, LOW);
  }
}

/**
 * battery cell voltage will be calculated and on low
 * voltage Rollator will be turned off
 * <p>
 * this functions will be called after defined time
 */
void batteryStatus()
{
  cellVoltage = analogRead(BATTERY);
  cellVoltage = (cellVoltage * analogRef) / adcRes;     //Voltage at Analog Port
  cellVoltage = (cellVoltage * 242) / 22;               //Resistor Divider R1 + R2 = 242k, R2 = 22k
  if (DEBUG_ENABLED)
    Serial.println("Battery:\t" + String(cellVoltage) + " V");
  cellVoltage = cellVoltage/6;                          //calc cell voltage
  //on low battery voltage, turn off Rollator
  if(cellVoltage <= turnOffV)
    digitalWrite(PWRON, LOW);
}

/********************************************//**
 *  Motor Control Functions
 ***********************************************/
/**
 * send speed (RPM) infromation to wheel
 * <p>
 * this function is called in every loop wether the information
 * has changed or not. Motor needs periodically instructions othwerwise
 * turns off
 * 
 * @param  side  which Motor will be controlled
 * @param  wheelSpeed RPM information
 * 
 */
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

/**
 * send current (mA) infromation to wheel
 * <p>
 * this function is called in every loop wether the information
 * has changed or not. Motor needs periodically instructions othwerwise
 * turns off
 * 
 * @param  side  which Motor will be controlled
 * @param  wheelSpeed RPM information
 * 
 */
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

/********************************************//**
 *  Interrupt Service Routines
 ***********************************************/
 /**
 * Time critical elements are "registered" here and 
 * flags will be set to signal a execution requirement
 * for those functions
 * the Tick toggles LED to measure accuracy from
 * outside
 * <p>
 * Timer interrupt service routine will be called on
 * each timer overflow
 */
 void tickISR()
{
  if(MEASURE_TICK)
    digitalWrite(LED, digitalRead(LED)^1);
  
  turnOffWatch();

  //PID Controller turned off
  if (!PID_ON)
  {
    left.u_k = left.w_k;
    right.u_k = right.w_k;
  }

  if(!ROLLATOR_UNPLUGGED && PID_ON)
  {
    //calculate new actuator value from PID and ARW
    left.e_k = left.w_k - left.y_k;
    right.e_k = right.w_k - right.y_k;
  
    //P
    left.u_p_k = Kp * left.e_k;
    right.u_p_k = Kp * right.e_k;

    //I
    left.u_i_k = left.u_i_k_1 + ai * left.e_k;
    right.u_i_k = right.u_i_k_1 + ai * right.e_k;

    //D with filter and only on output ,not on error
    left.u_d_k = ad * left.u_d_k_1 + ai * (-left.y_k + left.y_k_1);
    left.u_d_k = ad * left.u_d_k_1 + ai * (-right.y_k + right.y_k_1);
  
    //Store variables
    left.e_k_1 = left.e_k;
    right.e_k_1 = right.e_k;
    left.u_i_k_1 = left.u_i_k;
    right.u_i_k_1 = right.u_i_k;

    //actuator before ARW
    left.v_k = left.u_p_k + left.u_i_k + left.u_d_k + left.u_arw_k_1;
    right.v_k = right.u_p_k + right.u_i_k + right.u_d_k + right.u_arw_k_1;

    //Saturation
    if (left.v_k > u_max)
      left.u_k = u_max;
    else if (left.v_k < u_min)
      left.u_k = u_min;
    else
      left.u_k = left.v_k;
  
    if (right.v_k > u_max)
      right.u_k = u_max;
    else if (right.v_k < u_min)
      right.u_k = u_min;
    else
      right.u_k = right.v_k;

    //ARW
    left.u_arw_k = left.u_arw_k_1 + ar * (left.u_k - left.v_k);
    right.u_arw_k = right.u_arw_k_1 + ar * (right.u_k - right.v_k);

    //Store variables
    left.u_arw_k_1 = left.u_arw_k;
    right.u_arw_k_1 = right.u_arw_k;
    left.y_k_1 = left.y_k;
    right.y_k_1 = right.y_k;
    left.u_d_k_1 = left.u_d_k;
    right.u_d_k_1 = right.u_d_k;
  }

  if(!ROLLATOR_UNPLUGGED)
  {
    batCount++;
    //enable interrupt for communication
  }
}
