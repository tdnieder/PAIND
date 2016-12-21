#define BATTERY     0

#define DEBUG_ENABLED   1

int batVoltage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  batVoltage  = analogRead(BATTERY);
  Serial.println(batVoltage);
  delay(1000);
}
