#include <Adafruit_INA219.h>
#include "MPU9250.h"


#include "config.h"
#include <analogWrite.h>

#include "config_wifi.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "web/index.html.gz.h"

#include <NewPing.h>

unsigned long currentTime;
unsigned long previousTime;
unsigned long loopTime;

// define move params
float speed     = 0;
float yaw       = 0;
float speedCalc = 0;
float yawCalc   = 0;

// define motor values
float leftMotor = 0;
float rightMotor = 0;

// Failsafe
bool FS_FAIL = false;
uint8_t FS_WS_count = 0;

// Power sensor
float shuntvoltage = 0;
float busvoltage   = 0;
float current_mA   = 0;
float loadvoltage  = 0;
float power_mW     = 0;

// Is rover upsidedown
bool invert = false;
bool enabled = true;

// Init hardware
NewPing sonar_front(SONAR_TRIGGER_PIN, SONAR_FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonar_back(SONAR_TRIGGER_PIN, SONAR_BACK_ECHO_PIN, MAX_DISTANCE);

Adafruit_INA219 ina219;

MPU9250 mpu;

// WiFi
// WebServer
bool clientOnline = false;
IPAddress WiFiIP;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin (SERIAL_BAUD);
  Wire.begin();
  delay(2000);
  mpu.setup();
  // rover should be leveled here
  mpu.calibrateAccelGyro();
  ina219.begin();

  // init WiFi access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_ssid, AP_pass);
  WiFi.softAPsetHostname(AP_ssid);
  WiFiIP = WiFi.softAPIP();

  initWebServer();
  delay(1000);
  Serial.println("Go");
}

void loop() {
  currentTime = millis();
  if (currentTime - previousTime >= LOOP_TIME) {
    previousTime = currentTime;
    getPowerSensor();
    getIMU();
    updateFailsafe();
    calcMove();
    doMove();
    /*Serial.print(speed);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print("->");
    Serial.print(yawCalc);
    Serial.print(" | ");
    Serial.print(invert ? "inverted" : "normal");
    Serial.print(" ");
    Serial.print(enabled ? "power_ok" : "power_low");
    Serial.print(" | ");
    Serial.print(leftMotor);
    Serial.print(" ");
    Serial.println(rightMotor);*/

    FS_WS_count++;

    loopTime = millis() - currentTime;
  }

}

bool frontObstacle()
{
  return OBSTACLE_DISTANCE == 0 ? false : (invert ? sonar_back.ping_cm() : sonar_front.ping_cm()) < OBSTACLE_DISTANCE;
}

bool backObstacle()
{
  return OBSTACLE_DISTANCE == 0 ? false : (invert ? sonar_front.ping_cm() : sonar_back.ping_cm()) < OBSTACLE_DISTANCE;
}

void calcMove()
{  
  if (speed > 0 && frontObstacle()) {
    yawCalc = 0.5;
  } else if (speed < 0 && backObstacle()) {
    yawCalc = -0.5;
  } else {
    yawCalc = yaw;
  }

  float m = abs(speed)+abs(yawCalc); // normalisation
  if (m == 0 || !enabled) {
    leftMotor = 0;
    rightMotor = 0;
    return;
  }
  
  if (m < 1) m = 1;
  
  if (invert) m = -m;  // we are upside down

// TODO this is still not correct
  leftMotor  = (speed+yawCalc)/m;
  rightMotor = (speed-yawCalc)/m;

}

void doMove()
{
    analogWrite(LEFT_MOTOR1_PIN, leftMotor > 0 ? 0 : leftMotor*MAX_SPEED);
    analogWrite(LEFT_MOTOR2_PIN, leftMotor > 0 ? leftMotor*MAX_SPEED : 0);

    analogWrite(RIGHT_MOTOR1_PIN, rightMotor > 0 ? rightMotor*MAX_SPEED : 0);
    analogWrite(RIGHT_MOTOR2_PIN, rightMotor > 0 ? 0 : rightMotor*MAX_SPEED);
}

void getPowerSensor()
{
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  /*Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");*/

  enabled = ina219.getBusVoltage_V() > POWER_THR;
}

void getIMU()
{
  mpu.update();  
  invert = mpu.getAcc(2) < 0;
}
