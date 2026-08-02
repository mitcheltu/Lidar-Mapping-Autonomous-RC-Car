/*
 * esp32_car_wifi_tb6612.ino
 *
 * ESP32 + TB6612FNG motor controller driven over WiFi by the laptop's ROS2
 * graph (car_driver_node). The phone stays a pure perception sensor -- no BLE,
 * no iOS involvement in driving.
 *
 * Protocol: raw TCP, one command per line, on port 9001.
 *     "L<left>R<right>\n"     each side -100..100, e.g. "L60R-40\n"
 *     -> replies "ok\n"       so the driver can measure link round-trip time
 *     "S\n"                   immediate stop
 * Anything unparseable is ignored and answered with "err\n".
 *
 * This firmware is deliberately DUMB: it maps 0..100 linearly onto PWM duty and
 * knows nothing about stiction, motor mismatch, or ramping. All of that is
 * measured on the laptop by calibration_node and applied there, so re-calibrating
 * never means reflashing. Do not "improve" this by adding compensation here --
 * it would fight the calibration.
 *
 * ---- Compatibility ----
 * ESP32 Arduino core v3.x (current): ledcAttach(pin, freq, resolution).
 * For core v2.x see the note in setupPWM().
 *
 * ---- Wiring (TB6612FNG <-> ESP32) ----
 *   TB6612 VM    -> motor battery + (7.4V from 2x 18650), through your switch
 *   TB6612 VCC   -> ESP32 3.3V        (logic reference)
 *   TB6612 STBY  -> ESP32 GPIO 22     (must be HIGH to enable)
 *   TB6612 GND   -> COMMON GROUND (battery -, ESP32 GND, one star point)
 *
 *   Motor A (LEFT):                    Motor B (RIGHT):
 *     AIN1 -> GPIO 18                    BIN1 -> GPIO 16
 *     AIN2 -> GPIO 19                    BIN2 -> GPIO 17
 *     PWMA -> GPIO 23                    PWMB -> GPIO 21
 *     AO1/AO2 -> left motor              BO1/BO2 -> right motor
 *
 *   Power the ESP32 from its own USB power bank so motor noise stays off the
 *   logic supply. 100nF caps across the motor terminals, bulk cap on VM->GND.
 *
 *   If a side runs backwards, flip LEFT_DIR / RIGHT_DIR below (or swap that
 *   motor's two output wires). Do that BEFORE calibrating.
 */

#include <WiFi.h>
#include <ESPmDNS.h>

// ===== FILL THESE IN BEFORE FLASHING =====================================
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
// =========================================================================

const uint16_t CMD_PORT = 9001;
const char *MDNS_NAME = "rccar";        // reachable as rccar.local

// ---- TB6612 pins ----
const int MOTOR_A_IN1 = 18;   // left
const int MOTOR_A_IN2 = 19;
const int MOTOR_A_PWM = 23;

const int MOTOR_B_IN1 = 16;   // right
const int MOTOR_B_IN2 = 17;
const int MOTOR_B_PWM = 21;

const int MOTOR_STBY  = 22;

// ---- PWM config ----
const int PWM_FREQ = 20000;   // 20 kHz = inaudible
const int PWM_RES  = 8;       // 8-bit: duty 0..255

// ---- Failsafe ----
// The laptop publishes at 10 Hz, so 500 ms without a command means the link,
// the graph, or the laptop is gone. Stop.
unsigned long lastCommandMs = 0;
const unsigned long FAILSAFE_MS = 500;
bool stopped = true;

// Flip to -1 if a side drives backwards.
const int LEFT_DIR  = 1;
const int RIGHT_DIR = 1;

WiFiServer server(CMD_PORT);
WiFiClient client;

void setupPWM() {
  ledcAttach(MOTOR_A_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_B_PWM, PWM_FREQ, PWM_RES);
  // ---- core v2.x users: replace the two lines above with ----
  //   ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_A_PWM, 0);
  //   ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_B_PWM, 1);
  // and change ledcWrite(MOTOR_A_PWM, duty) -> ledcWrite(0, duty), B -> 1.
}

// speed: -100..100
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
  setMotor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, left,  LEFT_DIR);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, right, RIGHT_DIR);
  stopped = (left == 0 && right == 0);
}

void stopMotors() { drive(0, 0); }

// Parse "L60R-40". Returns false if the line is not a drive command.
bool parseDrive(const String &cmd, int *left, int *right) {
  int li = cmd.indexOf('L');
  int ri = cmd.indexOf('R');
  if (li < 0 || ri < 0 || ri < li) return false;
  *left  = cmd.substring(li + 1, ri).toInt();
  *right = cmd.substring(ri + 1).toInt();
  return true;
}

void handleLine(const String &line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "S" || cmd == "s") {
    stopMotors();
    lastCommandMs = millis();
    client.print("ok\n");
    return;
  }

  int left = 0, right = 0;
  if (parseDrive(cmd, &left, &right)) {
    drive(left, right);
    lastCommandMs = millis();
    client.print("ok\n");
  } else {
    client.print("err\n");
  }
}

void connectWiFi() {
  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // latency matters more than power here
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("*** Car IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("*** Listening on port %u\n", CMD_PORT);
  Serial.println("*** Point the laptop at it:  ./run.sh --car <that IP>");

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("rccar", "tcp", CMD_PORT);
    Serial.printf("*** Also reachable as %s.local\n", MDNS_NAME);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  int outPins[] = {MOTOR_STBY, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2};
  for (int p : outPins) pinMode(p, OUTPUT);

  setupPWM();
  digitalWrite(MOTOR_STBY, HIGH);   // enable the driver
  stopMotors();                     // never move during boot

  connectWiFi();
  server.begin();
  server.setNoDelay(true);          // no Nagle: 10 Hz of tiny commands
  lastCommandMs = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    stopMotors();                   // no link, no driving
    connectWiFi();
  }

  if (!client || !client.connected()) {
    WiFiClient incoming = server.accept();
    if (incoming) {
      if (client) client.stop();    // one driver at a time
      client = incoming;
      client.setNoDelay(true);
      lastCommandMs = millis();
      Serial.println("driver connected");
    }
  }

  if (client && client.connected()) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      handleLine(line);
    }
  } else if (!stopped) {
    stopMotors();                   // driver went away mid-drive
  }

  // Failsafe: the laptop should be sending at 10 Hz. Silence means trouble.
  if (!stopped && millis() - lastCommandMs > FAILSAFE_MS) {
    stopMotors();
    Serial.println("failsafe: no command, motors stopped");
  }

  delay(2);
}
