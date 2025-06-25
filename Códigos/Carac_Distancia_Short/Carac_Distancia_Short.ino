#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_PIN 22

VL53L1X sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Pololu VL53L1X sensor demo"));

  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, HIGH); // Activa el sensor si tienes el pin XSHUT conectado

  Wire.begin();
  
  if (!sensor.init()) {
    Serial.println(F("Error al inicializar el sensor VL53L1X"));
    while (1) delay(10);
  }
  
  Serial.println(F("VL53L1X sensor OK!"));

  sensor.setDistanceMode(VL53L1X::Short);       // Short (20ms), Medium(33ms) o Long (140ms a 4m); (33ms, minimun para todos)
  sensor.setMeasurementTimingBudget(20000);    // Tiempo en microsegundos
  sensor.startContinuous(20);                  // Inicia medición continua cada 50 ms

  Serial.println(F("Sensor configurado y midiendo"));
}

void loop() {
  if (sensor.dataReady()) {
    uint16_t distance = sensor.read();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    // NO necesitas llamar a clearInterrupt()
  }
}
