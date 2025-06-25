#include <ComponentObject.h>

#define in3Pin 10 // Conecta In3 del puente H al pin 10 de Arduino
#define in4Pin 11 // Conecta In4 del puente H al pin 11 de Arduino
int pwm=180;
//int pwm2=80;
int cont=0;

const byte conA=2;
const byte conB=3;
//const byte conZ=6;//se utiliza pasa sacar posicion inicial, te da los pulsos
//const float pi= 0.01745;//relacion de grados a radianes
const float cona= 0.09;//360/4000
float ang=0;
//unsigned long tiempo=0;
volatile int estado, estadoac;
volatile long contadore=0;

void setup(){
  ////////////////////////////
  
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  ////////////////////////////
  pinMode(conA,INPUT_PULLUP);
  pinMode(conB,INPUT_PULLUP);
  //pinMode(conZ,INPUT);
  attachInterrupt(digitalPinToInterrupt(conA),actuacoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB),actuacoder,CHANGE);
  if((digitalRead(conA)==HIGH)&&(digitalRead(conB)==HIGH))estado=1;
  if((digitalRead(conA)==LOW)&&(digitalRead(conB)==HIGH))estado=2;
  if((digitalRead(conA)==LOW)&&(digitalRead(conB)==LOW))estado=3;
  if((digitalRead(conA)==HIGH)&&(digitalRead(conB)==LOW))estado=4;
  Serial.begin(9600);     
  }

void loop(){  
  //test pwm pulso motor
  if(cont==0)

    analogWrite(7,pwm);  
    delay(100);
    analogWrite(7,0);
    digitalWrite(in3Pin, 1);
    digitalWrite(in4Pin, 0);
    analogWrite(10,pwm);
    analogWrite(6,0);
    digitalWrite(in3Pin, 0);
    digitalWrite(in4Pin, 1);
  cont=1;
  if(millis()>500){
    Serial.println(ang);
  }
  }

void actuacoder(){
    if((digitalRead(conA)==HIGH)&&(digitalRead(conB)==HIGH))estadoac=1;
    if((digitalRead(conA)==LOW)&&(digitalRead(conB)==HIGH))estadoac=4;
    if((digitalRead(conA)==LOW)&&(digitalRead(conB)==LOW))estadoac=3;
    if((digitalRead(conA)==HIGH)&&(digitalRead(conB)==LOW))estadoac=2;
     switch(estadoac){
      case 1:{
        if(estado==2){contadore++;}
        if(estado==4){contadore--;} 
        break;
        }
      case 2:{
        if(estado==3){contadore++;}
        if(estado==1){contadore--;}
        break;
        }
      case 3:{
        if(estado==4){contadore++;}
        if(estado==2){contadore--;}
        break;
        }
      case 4:{
        if(estado==1){contadore++;}
        if(estado==3){contadore--;}
        break;
        }  
      }
    estado=estadoac;  
    //ang=contadore*(cona+pi);
    ang=contadore*cona;
  }
