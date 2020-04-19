#include <Adafruit_INA219.h>
#include "MPU9250.h"


#include "config.h"
#include <analogWrite.h>

#include "config_wifi.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "web/index.html.gz.h"

#include <NewPing.h>

// define move params
float speed   = 0;
float yaw     = 0;
float yawCalc = 0;

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


void WiFiEvent(WiFiEvent_t event){
    switch(event) {
        case SYSTEM_EVENT_AP_START:
            WiFi.softAPsetHostname(AP_ssid);
            WiFiIP = WiFi.softAPIP();
            break;
        case SYSTEM_EVENT_AP_STOP:
            break;
        /*case SYSTEM_EVENT_STA_START:
            WiFi.setHostname(AP_Ssid);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            WiFiIP = WiFi.localIP();
            break;*/
        default:
            break;
    }
}


void setup() {
  Serial.begin (SERIAL_BAUD);
  Wire.begin();
  delay(2000);
  mpu.setup();
  ina219.begin();

  // init WiFi access point
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_ssid, AP_pass);

  Serial.println("Go");
}

void loop() {
  getPowerSensor();
  getIMU();
  Serial.print(invert ? "inverted" : "normal");
  Serial.print(" ");
  Serial.println(enabled ? "power_ok" : "power_low");
  //delay(2000);
}

void calcMove()
{
  if (speed > 0 && sonar_front.ping_cm() < OBSTACLE_DISTANCE) {
    yawCalc = 1;
  } else if (speed < 0 && sonar_back.ping_cm() < OBSTACLE_DISTANCE) {
    yawCalc = -1;
  } else {
    yawCalc = yaw;
  }

  float m = abs(speed)+abs(yawCalc); // normalisation
  if (invert) m = -m;  // we are upside down
  if (enabled) m = 0;  // not enough power to move, charge battery
  leftMotor  = (speed-yawCalc)/m;
  rightMotor = (speed+yawCalc)/m;

}

void setMove()
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
  static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();

        /*Serial.print(mpu.getRoll());
        Serial.print(" ");
        Serial.print(mpu.getPitch());
        Serial.print(" ");
        Serial.println(mpu.getYaw());*/

        invert = mpu.getYaw() < 0;

        prev_ms = millis();
    }
}
