#include "Adafruit_VL53L1X.h"
#include <ComponentObject.h>

#define IRQ_PIN 8    // IRQ sensor de distancia 
#define XSHUT_PIN 9  // XSHUT sensor de distancia 
#define in3Pin 10 // Dirección motor izquierda
#define in4Pin 11 // Dirección motor derecha
#define pwmEnablePin 7 // Control de velocidad (PWM)
#define conA 2 // Encoder canal A
#define conB 3 // Encoder canal B

const float cona = 0.09; // Conversión de pulsos a ángulos (360/4000)
volatile int estado, estadoac;
volatile long contadore = 0;
float ang = 0;

// Inicialización del sensor de distancia
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Variables de control
double x0 = 0, x1 = 0, x2 = 0;
double y1_internal = 0, y2_internal = 0;
double control_output_internal = 0;
double y1_external = 0, y2_external = 0;
double control_output_external = 0;
double Ref_Posicion = 230; // Posición de referencia en mm

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

  // Determinar el estado inicial del encoder
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estado = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estado = 2;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estado = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estado = 4;
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

    // Control interno: ejecuta 5 veces más rápido
    for (int i = 0; i < 5; i++) {
      x0 = Ref_Posicion - distance;
      control_output_internal = 3.108 * x0 - 6.168 * x1 + 3.061 * x2 + 1.991 * y1_internal - 3.061 * y2_internal;

      // Actualizar estados del controlador interno
      x2 = x1;
      x1 = x0;
      y2_internal = y1_internal;
      y1_internal = control_output_internal;
    }

    // Control externo: utiliza la salida del control interno
    control_output_external = 14.48 * control_output_internal - 28.94 * y1_external + 14.46 * y2_external + 0.8304 * y1_external + 0.1696 * y2_external;

    // Actualizar estados del controlador externo
    y2_external = y1_external;
    y1_external = control_output_external;

    // Señal de control combinada
    double control_signal = constrain(control_output_external, -255, 255);

    // Control del motor
    if (control_signal > 0) {
      izquierda();
    } else if (control_signal < 0) {
      derecha();
    } else {
      detenerMotor();
    }

    // Aplicar PWM a velocidad
    analogWrite(pwmEnablePin, abs(control_signal));

    // Impresión de datos
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.print(" mm | Internal control: ");
    Serial.print(control_output_internal);
    Serial.print(" | External control: ");
    Serial.println(control_output_external);

    vl53.clearInterrupt();
  }
}

// Funciones de dirección del motor
void izquierda() {
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
}

void derecha() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
}

void detenerMotor() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
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
