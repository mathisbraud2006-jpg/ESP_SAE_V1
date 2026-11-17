
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <math.h>

// ============================
// CONFIGURATION I2C
// ============================

#define SDA_PIN 20
#define SCL_PIN 21
#define ICM20948_ADDR 0x69

#define REG_BANK_SEL      0x7F
#define REG_WHO_AM_I      0x00
#define REG_PWR_MGMT_1    0x06
#define REG_PWR_MGMT_2    0x07
#define REG_ACCEL_XOUT_H  0x2D

// Configuration banque 2
#define REG_GYRO_CONFIG_1 0x01
#define REG_ACCEL_CONFIG  0x14

// Sensibilites par defaut : +/- 2g et +/- 250 dps
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

#define RAD_TO_DEG_F 57.2957795f

// ============================
// CONFIGURATION BLE
// ============================

#define DEVICE_NAME "ESP32-S3-MATHIS"

#define SERVICE_UUID \
"12345678-1234-1234-1234-123456789000"

#define CHARACTERISTIC_UUID \
"12345678-1234-1234-1234-123456789001"

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;

bool imuDetected = false;

// ============================
// VARIABLES IMU
// ============================

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Biais gyroscope
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

// Angles en degres
float roll = 0;
float pitch = 0;
float yaw = 0;

unsigned long previousTime = 0;

// ============================
// FONCTIONS I2C
// ============================

void selectBank(uint8_t bank) {
  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(REG_BANK_SEL);
  Wire.write(bank << 4);
  Wire.endTransmission();
}

bool writeRegister(uint8_t bank, uint8_t reg,
                   uint8_t value) {

  selectBank(bank);

  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t bank, uint8_t reg,
                   uint8_t* buffer, uint8_t length) {

  selectBank(bank);

  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return false;

  uint8_t received =
    Wire.requestFrom((uint8_t)ICM20948_ADDR, length);

  if (received != length)
    return false;

  for (uint8_t i = 0; i < length; i++)
    buffer[i] = Wire.read();

  return true;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

// ============================
// INITIALISATION IMU
// ============================

bool initIMU() {

  uint8_t whoAmI = 0;

  Serial.println("Initialisation ICM-20948...");

  if (!readRegisters(0, REG_WHO_AM_I, &whoAmI, 1))
    return false;

  Serial.print("WHO_AM_I : 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0xEA)
    return false;

  // Sortie du mode veille
  writeRegister(0, REG_PWR_MGMT_1, 0x01);

  // Activation accelerometre et gyroscope
  writeRegister(0, REG_PWR_MGMT_2, 0x00);

  delay(100);

  // Configuration +/- 250 dps et +/- 2g
  writeRegister(2, REG_GYRO_CONFIG_1, 0x01);
  writeRegister(2, REG_ACCEL_CONFIG, 0x01);

  // Retour banque 0
  selectBank(0);

  Serial.println("ICM-20948 detecte !");

  return true;
}

// ============================
// LECTURE IMU
// ============================

bool readIMU() {

  uint8_t data[12];

  if (!readRegisters(0, REG_ACCEL_XOUT_H, data, 12))
    return false;

  ax = combineBytes(data[0], data[1]);
  ay = combineBytes(data[2], data[3]);
  az = combineBytes(data[4], data[5]);

  gx = combineBytes(data[6], data[7]);
  gy = combineBytes(data[8], data[9]);
  gz = combineBytes(data[10], data[11]);

  return true;
}

// ============================
// CALIBRATION GYROSCOPE
// ============================

void calibrateGyro() {

  Serial.println("Calibration gyroscope...");
  Serial.println("Ne pas bouger le capteur !");

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  const int samples = 200;

  for (int i = 0; i < samples; i++) {

    if (readIMU()) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }

    delay(5);
  }

  gyroBiasX = (float)sumX / samples;
  gyroBiasY = (float)sumY / samples;
  gyroBiasZ = (float)sumZ / samples;

  Serial.println("Calibration terminee !");
}

// ============================
// CALCUL ORIENTATION
// ============================

void calculateOrientation() {

  unsigned long currentTime = micros();

  float dt = (currentTime - previousTime) / 1000000.0f;
  previousTime = currentTime;

  if (dt <= 0 || dt > 0.1f)
    return;

  // Accelerometre en g
  float accelX = ax / ACCEL_SCALE;
  float accelY = ay / ACCEL_SCALE;
  float accelZ = az / ACCEL_SCALE;

  // Gyroscope en degres/seconde
  float gyroX = (gx - gyroBiasX) / GYRO_SCALE;
  float gyroY = (gy - gyroBiasY) / GYRO_SCALE;
  float gyroZ = (gz - gyroBiasZ) / GYRO_SCALE;

  // Angles issus de l'accelerometre
  float accelRoll =
    atan2f(accelY, accelZ) * RAD_TO_DEG_F;

  float accelPitch =
    atan2f(-accelX,
    sqrtf(accelY * accelY + accelZ * accelZ))
    * RAD_TO_DEG_F;

  // Initialisation
  static bool firstReading = true;

  if (firstReading) {
    roll = accelRoll;
    pitch = accelPitch;
    yaw = 0;
    firstReading = false;
  }

  // Filtre complementaire
  const float alpha = 0.98f;

  roll = alpha * (roll + gyroX * dt)
       + (1.0f - alpha) * accelRoll;

  pitch = alpha * (pitch + gyroY * dt)
        + (1.0f - alpha) * accelPitch;

  // Integration gyroscope pour le Yaw
  yaw += gyroZ * dt;
}

// ============================
// INITIALISATION BLE
// ============================

void initBLE() {

  Serial.println("Initialisation BLE...");

  NimBLEDevice::init(DEVICE_NAME);

  pServer = NimBLEDevice::createServer();

  NimBLEService* pService =
    pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ |
    NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setValue("IMU READY");

  pService->start();

  NimBLEAdvertising* pAdvertising =
    NimBLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE actif !");
  Serial.println(DEVICE_NAME);
}

// ============================
// SETUP
// ============================

void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println("\nESP32-S3 + ICM-20948 + BLE");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  imuDetected = initIMU();

  if (imuDetected) {
    calibrateGyro();
    previousTime = micros();
  }

  initBLE();

  if (!imuDetected)
    Serial.println("ATTENTION : IMU indisponible !");
}

// ============================
// LOOP
// ============================

void loop() {

  if (imuDetected && readIMU()) {

    calculateOrientation();

    // Format JSON pour application Android
    char message[100];

    snprintf(
      message,
      sizeof(message),
      "{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f}",
      roll, pitch, yaw
    );

    Serial.println(message);

    pCharacteristic->setValue(message);

    if (pServer->getConnectedCount() > 0) {
      pCharacteristic->notify();
    }
  }

  delay(20); // Environ 50 Hz
}
