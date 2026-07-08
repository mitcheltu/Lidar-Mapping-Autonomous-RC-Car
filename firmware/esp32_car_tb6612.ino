/*
 * esp32_car_tb6612.ino
 *
 * ESP32 BLE motor controller for the phone-brain mapping rover, using a
 * TB6612FNG dual motor driver (recommended over the L298N: MOSFET-based,
 * cooler, less voltage drop, more reliable on battery).
 *
 * The iPhone (CarController.swift) connects over BLE and writes short ASCII
 * commands like "L60R-40\n". This firmware parses left/right speeds (-100..100)
 * and drives two DC gear motors.
 *
 * ---- Compatibility ----
 * Written for the ESP32 Arduino core v3.x (current), which uses the new LEDC
 * PWM API: ledcAttach(pin, freq, resolution) + ledcWrite(pin, duty), and where
 * BLECharacteristic::getValue() returns an Arduino String.
 * If you are on the older core v2.x, see the note near setupPWM().
 *
 * ---- Wiring (TB6612FNG <-> ESP32) ----
 *   TB6612 VM    -> motor battery + (7.4V from 2x 18650), through your switch
 *   TB6612 VCC   -> ESP32 3.3V        (logic reference)
 *   TB6612 STBY  -> ESP32 GPIO 33     (must be HIGH to enable; we drive it)
 *   TB6612 GND   -> COMMON GROUND (battery -, ESP32 GND, all one star point)
 *
 *   Left motor:
 *     TB6612 PWMA -> ESP32 GPIO 25
 *     TB6612 AIN1 -> ESP32 GPIO 26
 *     TB6612 AIN2 -> ESP32 GPIO 27
 *     TB6612 AO1/AO2 -> left motor
 *   Right motor:
 *     TB6612 PWMB -> ESP32 GPIO 13
 *     TB6612 BIN1 -> ESP32 GPIO 16
 *     TB6612 BIN2 -> ESP32 GPIO 17
 *     TB6612 BO1/BO2 -> right motor
 *
 *   Power the ESP32 itself from your USB power bank (keep motor noise off the
 *   logic supply). Don't forget the 100nF caps across the motors + a bulk cap
 *   across VM->GND at the driver.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Must match CarController.swift.
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// ---- TB6612 pins ----
const int STBY = 33;

const int PWMA = 25, AIN1 = 26, AIN2 = 27;   // left motor
const int PWMB = 13, BIN1 = 16, BIN2 = 17;   // right motor

// ---- PWM config ----
const int PWM_FREQ = 20000;   // 20 kHz = silent
const int PWM_RES  = 8;       // 8-bit: duty 0..255

// ---- Failsafe ----
unsigned long lastCommandMs = 0;
const unsigned long FAILSAFE_MS = 500;  // stop if no command for 0.5 s

// If a motor runs backwards, flip this to -1 (or just swap its two output wires).
const int LEFT_DIR  = 1;
const int RIGHT_DIR = 1;

void setupPWM() {
  // ESP32 Arduino core v3.x API:
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  // ---- core v2.x users: replace the two lines above with ----
  //   ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, 0);
  //   ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, 1);
  // and change ledcWrite(PWMA, duty) -> ledcWrite(0, duty), PWMB -> channel 1.
}

// speed: -100..100. in1/in2 = direction pins, pwmPin = PWM output.
void setMotor(int in1, int in2, int pwmPin, int speed, int dir) {
  speed *= dir;
  bool forward = speed >= 0;
  int mag = abs(speed);
  if (mag > 100) mag = 100;
  int duty = map(mag, 0, 100, 0, 255);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  ledcWrite(pwmPin, duty);
}

void drive(int left, int right) {
  setMotor(AIN1, AIN2, PWMA, left,  LEFT_DIR);
  setMotor(BIN1, BIN2, PWMB, right, RIGHT_DIR);
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
    String value = characteristic->getValue();   // Arduino String on core v3.x
    if (value.length() > 0) handleCommand(value);
  }
};

void setup() {
  Serial.begin(115200);

  int outPins[] = {STBY, AIN1, AIN2, BIN1, BIN2};
  for (int p : outPins) pinMode(p, OUTPUT);
  digitalWrite(STBY, HIGH);   // enable the driver

  setupPWM();
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

  Serial.println("TB6612 BLE robot car ready. Advertising as RobotCar-ESP32.");
}

void loop() {
  // Failsafe: if the phone stops sending (out of range / app crash), stop.
  if (millis() - lastCommandMs > FAILSAFE_MS) {
    stopMotors();
  }
  delay(20);
}
