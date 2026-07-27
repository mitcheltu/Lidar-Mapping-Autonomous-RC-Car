/*
 * esp32_car.ino
 *
 * ESP32 BLE motor controller for the phone-brain robot.
 *
 * The iPhone (CarController.swift) connects over BLE and writes short ASCII
 * commands like "L60R-40\n". This firmware parses left/right speeds (-100..100)
 * and drives two DC gear motors through an L298N (or TB6612 / DRV8833) H-bridge.
 *
 * Board:   any ESP32 dev board (e.g. ESP32-WROOM DevKitC)
 * Library: uses the ESP32 Arduino core's built-in BLE (BLEDevice.h). No extra
 *          library install needed once "esp32" boards are added in Boards Manager.
 *
 * L298N wiring (default pins below):
 *   ENA -> GPIO 25   (PWM, left motor speed)
 *   IN1 -> GPIO 26
 *   IN2 -> GPIO 27
 *   IN3 -> GPIO 14
 *   IN4 -> GPIO 12
 *   ENB -> GPIO 13   (PWM, right motor speed)
 *   Motor power (e.g. 2x 18650) -> L298N 12V/GND. Tie L298N GND to ESP32 GND.
 *   Power the ESP32 separately (its own USB battery / regulator), commoning GND.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Must match CarController.swift.
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// ---- Motor pins ----
const int ENA = 25, IN1 = 26, IN2 = 27;   // left motor
const int IN3 = 14, IN4 = 12, ENB = 13;   // right motor

// ---- PWM (LEDC) config ----
const int PWM_FREQ = 20000;   // 20 kHz = silent
const int PWM_RES  = 8;       // 8-bit: 0..255
const int CH_LEFT  = 0;
const int CH_RIGHT = 1;

unsigned long lastCommandMs = 0;
const unsigned long FAILSAFE_MS = 500;  // stop if no command for 0.5 s

void setMotor(int in1, int in2, int channel, int speed) {
  // speed: -100..100
  bool forward = speed >= 0;
  int mag = abs(speed);
  if (mag > 100) mag = 100;
  int duty = map(mag, 0, 100, 0, 255);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  ledcWrite(channel, duty);
}

void drive(int left, int right) {
  setMotor(IN1, IN2, CH_LEFT, left);
  setMotor(IN3, IN4, CH_RIGHT, right);
}

void stopMotors() { drive(0, 0); }

// Parse a line like "L60R-40"
void handleCommand(const String &cmd) {
  int li = cmd.indexOf('L');
  int ri = cmd.indexOf('R');
  if (li < 0 || ri < 0 || ri < li) return;
  int left  = cmd.substring(li + 1, ri).toInt();
  int right = cmd.substring(ri + 1).toInt();
  drive(left, right);
  lastCommandMs = millis();
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();   // Arduino String on recent cores
    if (value.length() > 0) handleCommand(value);
  }
};

void setup() {
  Serial.begin(115200);

  int pins[] = {IN1, IN2, IN3, IN4};
  for (int p : pins) pinMode(p, OUTPUT);

  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_LEFT);
  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, CH_RIGHT);
  stopMotors();

  BLEDevice::init("RobotCar-ESP32");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  BLECharacteristic *rx = service->createCharacteristic(
      RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE robot car ready. Advertising as RobotCar-ESP32.");
}

void loop() {
  // Failsafe: if the phone stops sending (out of range / crash), stop the motors.
  if (millis() - lastCommandMs > FAILSAFE_MS) {
    stopMotors();
  }
  delay(20);
}
