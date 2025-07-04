#include "Adafruit_VL53L1X.h"
#include <ComponentObject.h>

#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10    // Dirección motor
#define in4Pin 11    // Dirección motor
#define pwmEnablePin 7 // Pin PWM
#define conA 2       // Encoder canal A
#define conB 3       // Encoder canal B

const float cona = 0.09; // Conversión de pulsos a ángulos (360/4000)
volatile int estado, estadoac;
volatile long contadore = 0;
float ang = 0;

// Inicialización del sensor de distancia
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Variables de control
double x0 = 0, x1 = 0, x2 = 0;
double y1 = 0, y2 = 0;
double control_output = 0;
double Ref_Posicion = 230;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));
  Wire.begin();

  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }

  Serial.println(F("VL53L1X sensor OK!"));
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }

  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  // Configuración de pines para el motor y encoder
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(pwmEnablePin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);

  // Configuración de interrupciones para el encoder
  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);

  // Inicialización de variables
  x0 = x1 = x2 = 0;
  y1 = y2 = 0;
  control_output = 0;
}

void loop() {
  int16_t distance;

  if (vl53.dataReady()) {
    distance = vl53.distance();

    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }

    // Calcular el error (posición deseada - posición actual)
    x0 = Ref_Posicion - distance;

    // Controlador: ajusta las ganancias a valores razonables
    control_output = 0.3 * x0 - 0.6 * x1 + 0.3 * x2 + 0.2 * y1 - 0.3 * y2;

    // Actualizar variables de estado
    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = control_output;

    // Limitar la señal de control
    control_output = constrain(control_output, -255, 255);

    // Control del motor basado en la señal de control
    if (control_output > 0) {
      moverMotor(1, control_output); // Dirección positiva
    } else if (control_output < 0) {
      moverMotor(-1, -control_output); // Dirección negativa
    } else {
      detenerMotor();
    }

    // Depuración
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.print(" mm | Control output: ");
    Serial.print(control_output);
    Serial.print(" | Encoder Angle: ");
    Serial.println(ang);

    vl53.clearInterrupt();
  }
}

// Función para mover el motor
void moverMotor(int direccion, int pwm) {
  if (direccion > 0) {
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  } else if (direccion < 0) {
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  }
  analogWrite(pwmEnablePin, pwm);
}

// Función para detener el motor
void detenerMotor() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(pwmEnablePin, 0);
}

// Función de interrupción para el encoder
void actuacoder() {
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estadoac = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estadoac = 4;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estadoac = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estadoac = 2;

  switch (estadoac) {
    case 1:
      if (estado == 2) contadore++;
      if (estado == 4) contadore--;
      break;
    case 2:
      if (estado == 3) contadore++;
      if (estado == 1) contadore--;
      break;
    case 3:
      if (estado == 4) contadore++;
      if (estado == 2) contadore--;
      break;
    case 4:
      if (estado == 1) contadore++;
      if (estado == 3) contadore--;
      break;
  }

  estado = estadoac;
  ang = contadore * cona; // Conversión a ángulo
}
