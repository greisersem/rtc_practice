#include <Servo.h>

#define SERVO_PIN A5
#define MAX485_CONTROL_PIN_PIN 2

Servo servo;

const int device_id = 1;  

void setup() {
  Serial.begin(9600);
  pinMode(MAX485_CONTROL_PIN_PIN, OUTPUT);
  delay(10);
  digitalWrite(MAX485_CONTROL_PIN_PIN, LOW);
  servo.attach(SERVO_PIN); 
}

void loop() {
  if (Serial.available() >= 2) {
    int id = Serial.read() - '0';
    int cmd = Serial.read() - '0';
    Serial.print(id);
    Serial.print(cmd);
    if (id == device_id) {
      handleCommand(cmd);

      send_status(id, cmd);
    }
  }
}

void handleCommand(int command) {
  if (command == 1) {
    servo.write(90);
  } else if (command == 0) {
    servo.write(0);
  }
}

void send_status(int id, int cmd) {
  digitalWrite(MAX485_CONTROL_PIN_PIN, HIGH);
  delay(10);

  Serial.print(id);
  Serial.print(cmd);
  Serial.println("OK");

  Serial.flush();
  digitalWrite(MAX485_CONTROL_PIN_PIN, LOW);
}
