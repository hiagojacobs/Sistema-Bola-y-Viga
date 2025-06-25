// Definições de pinos e constantes
// motorPin e enablePin: Controlam a direção e a velocidade do motor.
// encoderPin: Leem os pulsos do encoder.
// encoderPos: Armazena a posição do encoder (em pulsos).
// desiredAngle: Ângulo desejado para o motor.
// Kp: Constante do controlador proporcional.
// controlSignal: Sinal de controle para o motor.
#define MOTOR_PIN1 10  // IN1 no L298N
#define MOTOR_PIN2 11  // IN2 no L298N
#define ENABLE_PIN 7 // ENA no L298N
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define PULSES_PER_REVOLUTION 360
#define CONTROL_LOOP_DELAY 10

// Variáveis de controle
volatile int estado,estadoac,contadore = 0;
volatile float encoderPos = 0; //Armazena a posição do encoder em termos de pulsos.
const float desiredAngle = 15; // Ângulo desejado em graus
const float Kp = 4.5,cona = 0.09; // Constante proporcional
float controlSignal = 0;

void encoderISR() {  //encoderISR(): Atualiza encoderPos com base nos sinais do encoder.
if((digitalRead(ENCODER_PIN_A)==HIGH)&&(digitalRead(ENCODER_PIN_B)==HIGH))estadoac=1;
    if((digitalRead(ENCODER_PIN_A)==LOW)&&(digitalRead(ENCODER_PIN_B)==HIGH))estadoac=4;
    if((digitalRead(ENCODER_PIN_A)==LOW)&&(digitalRead(ENCODER_PIN_B)==LOW))estadoac=3;
    if((digitalRead(ENCODER_PIN_A)==HIGH)&&(digitalRead(ENCODER_PIN_B)==LOW))estadoac=2;
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
    encoderPos=contadore*cona;
}


// pinMode: Configura os pinos como entrada ou saída.
void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // attachInterrupt: Configura interrupções para os pinos do encoder.
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);
    if((digitalRead(ENCODER_PIN_A)==HIGH)&&(digitalRead(ENCODER_PIN_B)==HIGH))estado=1;
    if((digitalRead(ENCODER_PIN_A)==LOW)&&(digitalRead(ENCODER_PIN_B)==HIGH))estado=2;
    if((digitalRead(ENCODER_PIN_A)==LOW)&&(digitalRead(ENCODER_PIN_B)==LOW))estado=3;
    if((digitalRead(ENCODER_PIN_A)==HIGH)&&(digitalRead(ENCODER_PIN_B)==LOW))estado=4;

// Serial.begin(9600): Inicializa a comunicação serial.
  Serial.begin(9600);
}


void loop() {
  float currentAngle = encoderPos;  //currentAngle = calculateAngle(currentPosition): Calcula o ângulo atual do motor.
  //Serial.println(encoderPos);
  float error = desiredAngle - currentAngle;  //error = desiredAngle - currentAngle: Calcula o erro entre o ângulo desejado e o atual.
  Serial.print(millis());
  Serial.print("_");
  Serial.println( currentAngle);
  //Serial.println(error);
  controlSignal = Kp * error;  //controlSignal = Kp * error: Calcula o sinal de controle proporcional ao erro.

  controlSignal = constrain(controlSignal, -255, 255);  //controlSignal = constrain(controlSignal, -255, 255): Limita o sinal de controle para o intervalo do PWM.

  if (controlSignal > 0) {
    digitalWrite(MOTOR_PIN1, HIGH);  //digitalWrite(): Define a direção do motor com base no sinal de controle.
    digitalWrite(MOTOR_PIN2, LOW);
  } else {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
  }
  analogWrite(ENABLE_PIN, abs(controlSignal));  //analogWrite(enablePin, abs(controlSignal)): Aplica o sinal de controle ao motor.

  delay(CONTROL_LOOP_DELAY);  //delay(100): Aguarda 100 ms antes de repetir o loop.
}


// pulsesPerRevolution: Número de pulsos por revolução do encoder. Ajuste este valor de acordo com o seu encoder.
