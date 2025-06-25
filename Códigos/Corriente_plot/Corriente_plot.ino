const int sensorPin = 1;                // GPIO1 (recomendado cambiar a GPIO34 si es posible)
float offsetVoltaje = 0;                // Se detecta en tiempo real
unsigned long tiempoAnterior = 0;
const int intervaloLectura = 1000;      // tiempo en ms (1s)

void setup() {
  Serial.begin(115200);

  // Calibración inicial (motor apagado)
  delay(3000);

  long suma = 0;
  const int muestras = 100;

  for (int i = 0; i < muestras; i++) {
    suma += analogRead(sensorPin);
    delay(5);
  }

  offsetVoltaje = ((suma / (float)muestras) / 4095.0) * 3.3;
}

void loop() {
  if (millis() - tiempoAnterior >= intervaloLectura) {
    tiempoAnterior = millis();

    long suma = 0;
    const int muestras = 50;

    for (int i = 0; i < muestras; i++) {
      suma += analogRead(sensorPin);
      delay(2);
    }

    float voltaje = ((suma / (float)muestras) / 4095.0) * 3.3;
    float corriente = (voltaje - offsetVoltaje) / 0.008; // 8mV/A

    // Eliminar ruido menor a 50 mA
    if (abs(corriente) < 0.05) {
      corriente = 0;
    }

    // Salida solo numérica para el Plotter Serial
    Serial.println(corriente);
  }
}
