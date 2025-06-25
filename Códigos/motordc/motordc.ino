int pwm1;
int pwm2;

void setup()
{
  pinMode(7,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
}

void loop()
{
  pwm1 = 50;
  pwm2 = 0;
  
  digitalWrite(7,1);
  
  izquierda();
  delay(2000);
  derecha();
  delay(2000);
  
}

void izquierda(){
  
  analogWrite(10,pwm1);
  analogWrite(11,pwm2);
  
}

void derecha(){
  
  analogWrite(11,pwm1);
  analogWrite(10,pwm2);
  
}
