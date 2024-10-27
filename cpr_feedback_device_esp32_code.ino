#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include "HX711.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Pin definitions
const int ICM_SDA_PIN = 11;         // ICM20948 SDA
const int ICM_SCL_PIN = 25;         // ICM20948 SCL
const int LED_PIN = 12;             // LED pin for displacement detection
const int BPM_LED_PIN = 5;          // LED pin for 116 bpm blinking
const int BUZZER_PIN = 10;          // Buzzer pin (updated to pin 10)
const int LOADCELL_DOUT_PIN = 3;    // HX711 DT pin
const int LOADCELL_SCK_PIN = 4;     // HX711 SCK pin

Adafruit_ICM20948 icm;
HX711 scale;

float velocityZ = 0;
float displacementZ = 0;
unsigned long previousTime = 0;
float gravityOffsetZ = 0;

SimpleKalmanFilter kalmanFilterZ(0.5, 1, 0.01);  // Kalman filter for Z-axis acceleration

unsigned long previousPrintTime = 0;
const unsigned long printInterval = 10;  // Print data every 10 ms

bool calibrationDone = false;
unsigned long calibrationStartTime = 0;
const unsigned long calibrationDuration = 3000;
float calibrationSumZ = 0;
int calibrationCount = 0;

bool ledOn = false;
unsigned long ledTurnOffTime = 0;

long loadCellZero = 0;  // Variable to hold the zeroed reading

// BLE Setup
#define DEVICE_NAME "CPR FEEDBACK DEVICE"  // Device name for BLE

// Initialize BLE server
BLEServer *bleServer;

void setup() {
  Serial.begin(115200);

  // Initialize I2C for IMU
  if (Wire.begin(ICM_SDA_PIN, ICM_SCL_PIN)) {
    Serial.println("I2C initialized on pins 11 (SDA) and 25 (SCL)");
  }
  Serial.println("Starting IMU...");

  // Initialize IMU
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948!");
  }

  // Initialize output devices
  pinMode(LED_PIN, OUTPUT);
  pinMode(BPM_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);  // Set the buzzer pin as output

  calibrationStartTime = millis();

  // Initialize HX711 with data on pin 3 and clock on pin 4
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Zero the scale
  Serial.println("Taring scale...");
  scale.tare();  // Set current reading as zero point
  loadCellZero = scale.read_average();  // Store the zeroed value
  Serial.print("Initial load cell zeroed value: "); Serial.println(loadCellZero);

  // BLE initialization
  Serial.println("Starting BLE work!");
  BLEDevice::init(DEVICE_NAME);  // Initialize BLE device with the name

  // Create a BLE server
  bleServer = BLEDevice::createServer();

  // Start advertising
  BLEAdvertising *bleAdvertising = BLEDevice::getAdvertising();

  // Set advertising data (custom UUID, connection intervals, etc.)
  bleAdvertising->addServiceUUID(BLEUUID((uint16_t)0xFFF0)); // Custom service UUID
  bleAdvertising->setScanResponse(false); // No scan response data
  bleAdvertising->setMinPreferred(0x06); // Set preferred connection interval (min)
  bleAdvertising->setMaxPreferred(0x12); // Set preferred connection interval (max)

  // Start advertising
  bleAdvertising->start();
  Serial.println("Bluetooth device is now broadcasting.");
}

void loop() {
  unsigned long currentTime = millis();

  // Get sensor values (accelerometer used)
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  // Calibration
  if (!calibrationDone) {
    if (currentTime - calibrationStartTime < calibrationDuration) {
      calibrationSumZ += accel.acceleration.z;
      calibrationCount++;
    } else {
      gravityOffsetZ = calibrationSumZ / calibrationCount;
      calibrationDone = true;
      previousTime = millis();
    }
    return;
  }

  float deltaTime = (currentTime - previousTime) / 1000.0;

  // Calculate pitch and roll
  float pitch = atan2(accel.acceleration.y, sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z));
  float roll = atan2(-accel.acceleration.x, accel.acceleration.z);

  // Calculate gravity component
  float gravityZ = cos(pitch) * cos(roll) * 9.81;

  // Subtract gravity component
  float accelZ = accel.acceleration.z - gravityZ - 0.33;

  // Apply Kalman filter
  float filteredAccelZ = kalmanFilterZ.updateEstimate(accelZ);

  velocityZ += filteredAccelZ * deltaTime;
  displacementZ += velocityZ * deltaTime;

  // Prevent negative displacement
  if (displacementZ < 0) {
    displacementZ = 0;
    velocityZ = 0;
  }

  previousTime = currentTime;

  // Check displacement and velocity, turn on LED if threshold is reached
  if (displacementZ > 0.08) {
    if (!ledOn && filteredAccelZ > 0 && velocityZ > 0.40) {
      digitalWrite(LED_PIN, HIGH);   // Turn on the LED on pin 12
      ledOn = true;                  // Mark LED as on
      ledTurnOffTime = millis() + 300;  // Set time to turn off LED
    }
    // Reset if displacement exceeds threshold
    displacementZ = 0;
    velocityZ = 0;
  }

  // LED turn off
  if (ledOn && millis() >= ledTurnOffTime) {
    digitalWrite(LED_PIN, LOW);  // Turn off the LED
    ledOn = false;               // Mark LED as off
  }

  // Check HX711 load cell readings
  if (scale.is_ready()) {
    long reading = scale.read();
    long relativeReading = reading - loadCellZero;  // Relative to the tared value

    Serial.print("HX711 reading: "); Serial.print(reading);
    Serial.print(" (relative: "); Serial.print(relativeReading); Serial.println(")");

    if (abs(relativeReading) > 80000) {
      tone(BUZZER_PIN, 2000);
      Serial.println("Buzzer ON");
    } else {
      noTone(BUZZER_PIN);
      Serial.println("Buzzer OFF");
    }

    if (abs(relativeReading) > 60000) {
      digitalWrite(BPM_LED_PIN, HIGH);
      Serial.println("BPM LED ON");
    } else {
      digitalWrite(BPM_LED_PIN, LOW);
      Serial.println("BPM LED OFF");
    }
  }
}
