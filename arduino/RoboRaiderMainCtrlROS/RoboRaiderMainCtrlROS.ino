#include <math.h>
#include <BMSerial.h>
#include <RoboClaw.h>
#include <LiquidCrystal_I2C.h>
#include <UM7.h>
#include "RoboRaiderCommands.h"

// RoboClaw - use pins 14 (Tx) and 15 (Rx) with 10ms timeout
RoboClaw roboclaw(15,14,10000);
#define address 0x80
#define Kp 50.0
#define Ki 0.5
#define Kd 50.00
#define qpps 10888 /* encoder pulses per sec at full speed */
#define MAX_MOTOR_SPEED 0.835 /* m/s */
#define CTS_PER_REV 6533 /* encoder pulses per revolution */
#define CRITICAL_BATTERY_LEVEL 100
int32_t enc1 = 0;
int32_t enc2 = 0;
int32_t lastEnc1 = 0;
int32_t lastEnc2 = 0;
uint16_t battVoltage = 0;
float px = 0.0;
float py = 0.0;
float v = 0.0;
float wheelDiameter = 0.167; // [m]
float ENC2DIST = M_PI * wheelDiameter / (float)CTS_PER_REV * 0.96;
float SPEED2QPPS = qpps/MAX_MOTOR_SPEED;


// Gyro
UM7 imu(Serial1);
float phi_gyro = 0.0;
float dphi_gyro = 0.0;
#define DEG2RAD 0.017453293

// LCD
LiquidCrystal_I2C lcd(0x3F,20,4);
int lcdClockModulo = 100;
bool serverConnected = false;

// timers 
unsigned long timestamp = 0;
unsigned long lastTimestamp = 0;
unsigned long lastCommand = 0;
unsigned long lastEncoderTick = 0;
const unsigned long hostTimeout = 2000; // stop motors if no steady connection from host
float dt = 0.0;

// buffer for received commands
const int MAX_BUFFER = 32;
char buffer[MAX_BUFFER];
int commandSize = 0;
#define BAUDRATE 115200


void setup() {
  // Open USB serial port
	Serial.begin(BAUDRATE);
 
	// Open IMU serial port
	Serial1.begin(BAUDRATE);

  // Open roboclaw serial port
  roboclaw.begin(BAUDRATE);
  
  // Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);
  
  // reset gyro
  imu.zero_gyros();
  
  // read system battery level
  battVoltage = roboclaw.ReadMainBatteryVoltage(address);
  int extraSpace = battVoltage < 100;
  
  // init LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Battery: ");
  lcd.setCursor(9+extraSpace,3);
  lcd.print(battVoltage/10, DEC);
  lcd.setCursor(11,3);
  lcd.print(".");
  lcd.setCursor(12,3);
  lcd.print(battVoltage%10, DEC);
  lcd.setCursor(14,3);
  lcd.print("V");
  lcd.setCursor(0,0);
  lcd.print("Connect with me on");
  lcd.setCursor(0,1);
  lcd.print("192.168.0.5");
}


void loop() {
  lastTimestamp = timestamp;
	timestamp = millis();
  dt = (float)(timestamp-lastTimestamp)/1000.0;

  // read system battery level
  battVoltage = roboclaw.ReadMainBatteryVoltage(address);
  if (battVoltage < CRITICAL_BATTERY_LEVEL) allOff();
    
  // read encoders
  enc1 = roboclaw.ReadEncM1(address);
  enc2 = roboclaw.ReadEncM2(address);
  float dp = (float)(enc1-lastEnc1 + enc2-lastEnc2)/2.0  * ENC2DIST;
  v = dp / dt;

  // store time of last encoder change
  if (enc1 != lastEnc1 || enc2 != lastEnc2) {
    lastEncoderTick = timestamp;
    lastEnc1 = enc1;
    lastEnc2 = enc2;
  }
  
  // read IMU
  imu.receive_data();
  dphi_gyro = -imu.gyro_z*DEG2RAD;
  phi_gyro +=  dphi_gyro * dt;
  
  // update position estimate based on encoders and IMU
  px += dp * cos(phi_gyro);
  py += dp * sin(phi_gyro);
  

  // update battery status on LCD
  if (timestamp%lcdClockModulo == 0) {
    int extraSpace = battVoltage < 100;
    if (extraSpace) {
      lcd.setCursor(9,3);
      lcd.print(" ");
    }
    lcd.setCursor(9+extraSpace,3);
    lcd.print(battVoltage/10, DEC);
    lcd.setCursor(12,3);
    lcd.print(battVoltage%10, DEC);
  }
  
	// handle serial input
	if (Serial.available() > 0) {
		lastCommand = timestamp;
		handleCommand();
	}
	else if (timestamp > lastCommand + hostTimeout) { 
		lastCommand = timestamp;
		allOff();
	}
}

void handleCommand() {
  char c = 0;
  while (Serial.available() > 0) {
    c = Serial.read();  //gets one byte from serial buffer
    buffer[commandSize++] = c;
  }
  
  // if carriage return or new line character received, parse command
	if ((c == '\r') || (c == '\n')) {
	  parseCommand();
	  commandSize = 0; 
	}
}

void parseCommand() {
  switch (buffer[0]) {
    case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

    case READ_CYCLETIME:
    Serial.println(dt, 3);
    break;
    
    case READ_ENCODERS:
    Serial.print(enc1);
    Serial.print(" ");
    Serial.println(enc2);
    break;

    case READ_POSE:
    Serial.print(px, 3);
    Serial.print(" ");
    Serial.print(py, 3);
    Serial.print(" ");
    Serial.print(phi_gyro, 3);
    Serial.print(" ");
    Serial.print(v, 3);
    Serial.print(" ");
    Serial.println(dphi_gyro, 3);
    break;
    
    case RESET_ODOMETRY:
    imu.zero_gyros();
    roboclaw.ResetEncoders(address);
    px = py = phi_gyro = v = dphi_gyro = 0.0;
    Serial.println("OK");
    break;
    
    case MOTOR_COMMAND: // expected format 'm +0.00 +0.00'
    buffer[7] = 0; // overwrite separater between first and second value with 0
    float v_left  = atof(buffer+2); // skip first two bytes ('m ') to access first value
    float v_right = atof(buffer+8); // skip first eight bytes ('m +0.00 ') to access second value
    roboclaw.SpeedM1M2(address, v_left * SPEED2QPPS, v_right * SPEED2QPPS);
    Serial.println("OK");
    
    /*
    char *values = buffer+2; // skip first two bytes ('m ') and look for values
    char *separator = strchr(values, ' '); // look for separator ' ' between values
    if (separator != 0) {
        // replace separator with 0
        *separator = 0;
        float v_left  = atof(values);
        float v_right = atof(++separator);
        roboclaw.SpeedM1M2(address, v_left * SPEED2QPPS, v_right * SPEED2QPPS);
        Serial.println("OK"); 
    }
    else
        Serial.println("ERROR");
    */
    break;
  }
}

void allOff() {
  roboclaw.DutyM1M2(address, 0, 0);
}

