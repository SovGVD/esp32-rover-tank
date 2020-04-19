// Serial
#define SERIAL_BAUD 115200 

// main loop
#define LOOP_TIME 20   // milliseconds

// failsafe
#define FS_WS_THR 20  // 1 second = FS_WS_THR*LOOP_TIME

//Sonar
#define SONAR_TRIGGER_PIN     32
#define SONAR_BACK_ECHO_PIN   35
#define SONAR_FRONT_ECHO_PIN  34
#define MAX_DISTANCE         200

// define pins
#define LEFT_MOTOR1_PIN  27
#define LEFT_MOTOR2_PIN  26
#define RIGHT_MOTOR1_PIN 25
#define RIGHT_MOTOR2_PIN 33

// Speed
#define MAX_SPEED 128   // max is 255

// Obstacle
#define OBSTACLE_DISTANCE 20   // in cm

// IMU
#define IMU_ADDRESS 0x68

// Power
#define POWER_SENSOR_ADDRESS 0x41
#define POWER_THR            6.7
