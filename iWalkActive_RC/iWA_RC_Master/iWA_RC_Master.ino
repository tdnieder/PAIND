

#include <SoftwareSerial.h>                         // Software Serial Port

#define RxD         7
#define TxD         6

#define DEBUG_ENABLED  1

int lastrValue = 0;
int lastlValue = 0;
int count = 0;
int countMax = 4;

String commandLeft;
String commandRight;

String connectCmd = "\r\n+CONN=";

SoftwareSerial blueToothSerial(RxD,TxD);

void setup()
{
  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  
  setupBlueToothConnection();
  //wait 1s and flush the serial buffer
  delay(1000);
  Serial.flush();
  blueToothSerial.flush();
}

void loop()
{
  String commandString;
  int rValue = 0;
  int lValue = 0;
  
  readJoystickData(&rValue, &lValue);
  if(rValue == lastrValue && lValue == lastlValue && count < countMax)
  {
    count++;
  }
  else
  {
    commandString = "s" + String(lValue) + "/" + String(rValue) + ";";
    blueToothSerial.print(commandString);
    Serial.println(commandString);
    lastrValue = rValue;
    lastlValue = lValue;
    count = 0;
  }
  delay(180);
}

/**
 * reading ADC Values that contain x/y Axis displacement information
 */
void readJoystickData(int *rValue, int *lValue)
{
  int x, y;
  x = analogRead(A2);
  x -= 512;
  y = analogRead(A3);
  y -= 512;

  //Noise Hystersis and Saturation for Joystick ADC Values
  if(abs(x) < 20){
    x = 0;
  }
  else if(x > 500){
    x = 500;
  }
  else if(x < -500){
    x = -500;
  }

  if(abs(y) < 20){
    y = 0;
  }
  else if(y > 500){
    y = 500;
  }
  else if(y < -500){
    y = -500;
  }

  /*If Joystick more sideways than forward then, drive sideways.
  if Joystick is more forward or backward than sideways, just drive forward
  This is an issue and needs to be addressed. Values should be added in some ways (maybe RMS)
  */
  /*if(abs(x) > abs(y))
  {
    *rValue = -x;
    *lValue = x;
  }
  else
  {
    *rValue = y;
    *lValue = y;
  }
  */
  //new try with driving curves
  
  if(y >= 0){
    *rValue = (-x + y);
    *lValue = (x + y);
  }
  else{
    *rValue = (x + y);
    *lValue = (-x + y);
  }
}
