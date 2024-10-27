#include "HX711.h"
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

Adafruit_ICM20948 icm;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  if (scale.is_ready()) {
    Serial.println("HX711 is ready.");
  }

  if (Wire.begin(11, 25)) { 
    Serial.println("yay");
  }
  Serial.println("okay now start this code");

  if (!icm.begin_I2C()) {
    while (1);
  }

  Serial.println("ICM20X found!");
}

void loop() {
  long loadCellReading = scale.read();

  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  Serial.print("Raw Load Cell: ");
  Serial.print(loadCellReading);
  Serial.print(" | Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" | Accel Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" | Accel Z: ");
  Serial.println(accel.acceleration.z);

  delay(10);
}
