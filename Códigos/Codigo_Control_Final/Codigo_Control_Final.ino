#include <Encoder.h>

//CONFIGURACION DE DOS ENCODER
Encoder Enc1(21,20);
Encoder Enc2(18,19);

//VARIABLES DE ENCODER
double newPosition1=0,oldPosition1=-999, delta_position, old_Position, vel_ang = 0;;
double newPosition2=0,oldPosition2=-999;
double pi=3.14159265359;
double Conv = 0.000416666667;
float  distancia = 0;


//VARIABLES DE MOTOR
int In1 = 4, In2 = 13, ENA = 3, sent = 0;
int Zm=47;
int PWM = 0;
double yT;

//CONTROL DE ANGULO -> DEFINICIONES//////
double eA0 = 0; //eA[k] -> error actual
double eA1 = 0; //eA[k-1]
double eA2 = 0; //eA[k-2]
double eA3 = 0; //eA[k-3]
double eA4 = 0; //eA[k-3]
double yA0 = 0; //yA[k] -> salida actual
double yA1 = 0; //yA[k-1]
double yA2 = 0; //yA[k-2]
double yA3 = 0; //yA[k-3]
double yA4 = 0; //yA[k-3]

//CONSTANTES DEL CONTROLADOR: bA0*yA0 = aA0*eA0 + aA1*eA1 + aA2*eA2 + aA3*eA3 - bA1*yA1 - bA2*yA2 - bA3*yA3
double aA0 = 72436.5782264820, aA1 = -144580.6197787083, aA2 = 72144.9983383080;  
double bA0 = 1.0, bA1 = -1.043213918263772, bA2 = 0.043213918263772;

//CONTROL DE POSICION -> DEFINICIONES//////
double eP0 = 0; //eP[k] -> error actual
double eP1 = 0; //eP[k-1]
double eP2 = 0; //eP[k-2]
double yP0 = 0; //yP[k] -> salida actual
double yP1 = 0; //yP[k-1]
double yP2 = 0; //yP[k-2]

//CONSTANTES DEL CONTROLADOR: bP0*yP0 = aP0*eP0 + aP1*eP1 + aP2*eP2 - bP1*yP1 - bP2*yP2 
double aP0 = 62831.85307179589, aP1 = -62793.58162852644;   
double bP0 = 1.0, bP1 = -0.043213918263772;

//VARIABLES DE CONTROL
long Ts = 1; //Período de muestreo en ms (Ts = 0.001 s)
unsigned long t_pasado;
unsigned long t_actual;
int Control = 0;
double anguloG;
float  Ref_Posicion = 0;

void setup() {
  Serial.begin(9600);
  pinMode(In1 , OUTPUT);
  pinMode(In2 , OUTPUT);
  pinMode(ENA , OUTPUT);
}

void loop() {
  encoders();
  anguloG = (180/pi)*newPosition2;
  if (((anguloG < 190) && (anguloG > 170))|((anguloG < -170) && (anguloG > -190))){Control=1;}
  else {
    Control = 0;
  }
  t_actual = millis (); //almacena el tiempo que el programa ha estado corriendo
  long delta_tiempo = t_actual - t_pasado;//controla el tiemop al que se actualizará la acción de control
  
  //INICIO DE CONTROL
  if((delta_tiempo >= Ts) && (Control == 1)){
    
    //CONTROLADOR DE ANGULO
    if (newPosition2 < 0){
      eA0 = newPosition2 + pi;
      eA0 *= (-1);
    }
    else {
      eA0 = pi - newPosition2;
    }
    
    yA0 = aA0*eA0 + aA1*eA1 + aA2*eA2 - bA1*yA1 - bA2*yA2;
    
    eA2 = eA1;
    eA1 = eA0;
    
    yA2 = yA1;
    yA1 = yA0;

    //CONTROLADOR DE POSICION
    Ref_Posicion = 0;
    
    eP0 = Ref_Posicion - distancia;
    
    yP0 = aP0*eP0 + aP1*eP1 - bP1*yP1;
    
    eP1 = eP0;

    yP1 = yP0;
    
    //Actualización del tiempo
    t_pasado = t_actual;
  }
  else if(Control == 0){
    yA0 = 0;
    yP0 = 0;
    }
  //ACTUALIZA SALIDA  
    yT = (yA0) - (yP0);
    if (yT >= 12){                                                               
      yT = 12;}
    else if(yT <= -12){
      yT = -12;}
  
  if ((Control == 0) && (yT == 0)){
    analogWrite(In1,0);
    analogWrite(In2,0);
    digitalWrite(ENA,HIGH);
    }
  else {
    motor(yT);
  }
}


void motor(double){
    if (yT >= 0){
      PWM = yT*(255/12);
    if(PWM > Zm){
      analogWrite(In1,PWM);
      analogWrite(In2,0);
      digitalWrite(ENA,HIGH);
      }
    else {
      analogWrite(In1,Zm);
      analogWrite(In2,0);
      digitalWrite(ENA,HIGH);
    }}
    if (yT <= 0){
      yT *= (-1);
      PWM = yT*(255/12);
    if(PWM > Zm){
      analogWrite(In1,0);
      analogWrite(In2,PWM);
      digitalWrite(ENA,HIGH);
      }
    else {
      analogWrite(In1,0);
      analogWrite(In2,Zm);
      digitalWrite(ENA,HIGH);
    }}
    }

 void encoders(){
    newPosition1 = Enc1.read();// Encoder Distancia
    newPosition1=newPosition1*2*pi*Conv;
    if (newPosition1 != oldPosition1){oldPosition1 = newPosition1;}
    
    distancia=newPosition1*0.0215;
    newPosition2 = Enc2.read();//Encoder Pendulo
    newPosition2=newPosition2*2*pi*Conv;
    
    if (newPosition2>=(2*pi)){newPosition2=0;Enc2.write(0);}
    if (newPosition2<=(-2*pi)){newPosition2=0;Enc2.write(0);}
    if (newPosition2 != oldPosition1){oldPosition2 = newPosition2;}
    }
 
