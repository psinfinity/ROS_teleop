#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 1))

// Wi-Fi credentials
const char* ssid = "";
const char* password = "";

// Motor control pins
const int ENA = 12;  // D6 (GPIO12) - Left Speed
const int IN1 = 5;   // D1 (GPIO5)  - Left Direction
const int IN2 = 4;   // D2 (GPIO4)  - Left Direction
const int IN3 = 2;   // D4 (GPIO2)  - Right Direction
const int IN4 = 14;  // D5 (GPIO14) - Right Direction
const int ENB = 13;  // D7 (GPIO13) - Right Speed

// UDP settings
WiFiUDP Udp;
unsigned int localPort = 4210;  // matches python script

// Motor control vars
float linear_x = 0;
float angular_z = 0;
int base_speed = 255;  
int max_speed = 255;   

// Adjusting motor speed factors
float left_motor_factor = 1.00;  
float right_motor_factor = 0.9;  

void setup() {
  Serial.begin(115200); 
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  WiFi.begin(ssid, password); // connect to wifi
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("waiting for connection\n");
  }
  Serial.println("successfully connected: ");
  Serial.println(WiFi.localIP());

  // Start UDP server
  Udp.begin(localPort);
  Serial.println(localPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    
    // ("linear_x,angular_z")
    char* token = strtok(packetBuffer, ",");
    if (token != NULL) {
      linear_x = atof(token);
      token = strtok(NULL, ",");
      if (token != NULL) {
        angular_z = atof(token);
      }
    }
    
    Serial.print("Received - Linear: ");
    Serial.print(linear_x);
    Serial.print(", Angular: "); 
    Serial.println(angular_z);
    
    controlMotors(linear_x, angular_z);
  }
}

void controlMotors(float linear, float angular) {
  int leftSpeed = 0;
  int rightSpeed = 0; 
  
  rightSpeed = base_speed * (linear + angular*sgn(linear))*left_motor_factor;
  leftSpeed = base_speed * (linear - angular*sgn(linear))*right_motor_factor;


  // limit speeds
  leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
  rightSpeed = constrain(rightSpeed, -max_speed, max_speed);
  
  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

void setMotor(int in1, int in2, int en, int speed) {
  // direction
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  // speed
  analogWrite(en, abs(speed));
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}