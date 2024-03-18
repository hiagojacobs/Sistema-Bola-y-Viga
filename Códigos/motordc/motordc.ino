#define in3Pin 11 // Conecta In3 del puente H al pin 2 de Arduino
#define in4Pin 10  // Conecta In4 del puente H al pin 1 de Arduino
int pwm=90;
int pwm2=80;

void setup() {
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
}

void loop() {
  digitalWrite(in3Pin, 1);
  digitalWrite(in4Pin, 0);
  analogWrite(6,pwm2);
  delay(150);
  
  digitalWrite(in3Pin, 0);
  digitalWrite(in4Pin, 1);
  analogWrite(6,pwm);
  delay(150);
  
  
}