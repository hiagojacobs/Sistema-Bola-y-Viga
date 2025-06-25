#include "Adafruit_VL53L1X.h"

// Pines para el motor y el sensor
#define in3Pin 10 
#define in4Pin 11 

const byte conA = 2;
const byte conB = 3;
const float cona = 0.09; // 360/4000 (factor de conversión del encoder)
float ang = 0;
volatile int estado, estadoac;
volatile long contadore = 0;
const float targetAngle = 0.0; // El objetivo es siempre regresar a 0.0 grados

// Parámetro de control proporcional
const float Kp = 3.0; // Constante proporcional, ajusta este valor según necesites una respuesta más suave o rápida
const int minPWM = 50; // PWM mínimo necesario para que el motor funcione

// Pines para el sensor de distancia
#define SHUT_PIN 9  // Pin alternativo para el cable amarillo (Shut)
#define INIT_PIN 8  // Pin para el cable verde (Init)

// Inicializar el sensor
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(SHUT_PIN, INIT_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Configuración del motor y sensor
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);
  
  Serial.println(F("===== Configuración de Sensor y Control de Ángulo ====="));

  // Inicializa la comunicación I2C en los pines 20 (SDA) y 21 (SCL)
  Wire.begin();

  // Inicialización del sensor de distancia
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error en la inicialización del sensor VL53L1X: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Sensor VL53L1X OK!"));

  // Iniciar el modo de medición del sensor de distancia
  if (!vl53.startRanging()) {
    Serial.print(F("Error al iniciar la medición: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Medición iniciada"));

  // Establecer el tiempo de medición en milisegundos
  vl53.setTimingBudget(50);
  Serial.print(F("Tiempo de medición (ms): "));
  Serial.println(vl53.getTimingBudget());
}

void loop() {
  int16_t distance;

  // Calcular el error en el ángulo
  float error = targetAngle - ang;

  // Calcular el valor PWM proporcional al error
  int pwm = abs(error) * Kp;
  pwm = constrain(pwm, minPWM, 255); // Limitar el valor PWM entre minPWM y 255

  // Control del motor para regresar suavemente al ángulo 0.0
  if (error > 0) {
    // Si el ángulo actual es menor que 0, mueve el motor hacia la derecha
    analogWrite(7, pwm); 
    digitalWrite(in3Pin, 1);
    digitalWrite(in4Pin, 0);
  } 
  else if (error < 0) {
    // Si el ángulo actual es mayor que 0, mueve el motor hacia la izquierda
    analogWrite(7, pwm); 
    digitalWrite(in3Pin, 0);
    digitalWrite(in4Pin, 1);
  } 
  else {
    // Detener el motor cuando se alcanza el ángulo objetivo (0.0)
    analogWrite(7, 0);
  }

  // Obtener y mostrar la distancia medida en mm si está lista
  if (vl53.dataReady()) {
    distance = vl53.distance();
    if (distance == -1) {
      Serial.print(F("Error al obtener la distancia: "));
      Serial.println(vl53.vl_status);
      return;
    }

    // Ploteo en formato de lista en el monitor serial
    Serial.print("Ángulo Actual     : ");
    Serial.print(ang, 2);          // Ángulo actual con dos decimales
    Serial.println(" grados");
    
    Serial.print("Ángulo Objetivo   : ");
    Serial.print(targetAngle, 2);  // Ángulo objetivo (0.00 grados fijo)
    Serial.println(" grados");

    Serial.print("Distancia         : ");
    Serial.print(distance);        // Distancia en mm
    Serial.println(" mm");
    
    Serial.print("PWM               : ");
    Serial.println(pwm);           // Valor PWM aplicado al motor
    
    Serial.println("-------------------------------"); // Separador para cada lectura

    // Limpiar la interrupción para la próxima medición
    vl53.clearInterrupt();
  }

  delay(100); // Delay para evitar sobrecargar el monitor serial
}

void actuacoder() {
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estadoac = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estadoac = 4;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estadoac = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estadoac = 2;
  
  switch (estadoac) {
    case 1:
      if (estado == 2) { contadore++; }
      if (estado == 4) { contadore--; }
      break;
    case 2:
      if (estado == 3) { contadore++; }
      if (estado == 1) { contadore--; }
      break;
    case 3:
      if (estado == 4) { contadore++; }
      if (estado == 2) { contadore--; }
      break;
    case 4:
      if (estado == 1) { contadore++; }
      if (estado == 3) { contadore--; }
      break;
  }
  
  estado = estadoac;
  ang = contadore * cona;
}
