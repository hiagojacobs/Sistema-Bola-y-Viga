#include "Adafruit_VL53L1X.h"

// Pines para el motor y el sensor
#define in3Pin 10 
#define in4Pin 11 

const float targetDistance = 100.0; // Distancia objetivo en mm
const float Kp = 0.5; // Constante proporcional ajustada para el control de la distancia
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
  
  Serial.println(F("===== Configuración de Sensor y Control de Distancia ====="));

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

  // Obtener y mostrar la distancia medida en mm si está lista
  if (vl53.dataReady()) {
    distance = vl53.distance();
    if (distance == -1) {
      Serial.print(F("Error al obtener la distancia: "));
      Serial.println(vl53.vl_status);
      return;
    }

    // Calcular el error en la distancia
    float error = targetDistance - distance;

    // Calcular el valor PWM proporcional al error
    int pwm = abs(error) * Kp;
    pwm = constrain(pwm, minPWM, 255); // Limitar el valor PWM entre minPWM y 255

    // Control del motor para ajustar la posición a la distancia objetivo
    if (error > 0) {
      // Si la distancia actual es menor que el objetivo, mueve el motor en una dirección
      analogWrite(7, pwm); 
      digitalWrite(in3Pin, 1);
      digitalWrite(in4Pin, 0);
    } 
    else if (error < 0) {
      // Si la distancia actual es mayor que el objetivo, mueve el motor en la dirección opuesta
      analogWrite(7, pwm); 
      digitalWrite(in3Pin, 0);
      digitalWrite(in4Pin, 1);
    } 
    else {
      // Detener el motor cuando se alcanza la distancia objetivo
      analogWrite(7, 0);
    }

    // Mostrar en el monitor serial los valores de distancia, objetivo y PWM
    Serial.print("Distancia Actual  : ");
    Serial.print(distance);        // Distancia medida en mm
    Serial.println(" mm");
    
    Serial.print("Distancia Objetivo: ");
    Serial.print(targetDistance);  // Distancia objetivo en mm
    Serial.println(" mm");

    Serial.print("PWM               : ");
    Serial.println(pwm);           // Valor PWM aplicado al motor
    
    Serial.println("-------------------------------"); // Separador para cada lectura

    // Limpiar la interrupción para la próxima medición
    vl53.clearInterrupt();
  }

  delay(100); // Delay para evitar sobrecargar el monitor serial
}
