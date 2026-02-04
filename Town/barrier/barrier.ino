#include <Servo.h>

#define SERVO_PIN A5
#define MAX485_CONTROL_PIN 2

Servo servo;

const int device_id = 2;


void setup()
{
    servo.attach(SERVO_PIN); ;
    pinMode(MAX485_CONTROL_PIN, OUTPUT);
    
    Serial.begin(9600);
    set_receive();
 
    delay(10);
}


void loop() {
    if (Serial.available() >= 2) {
        int id = Serial.read() - '0';
        int cmd = Serial.read() - '0';
        
        if (id == device_id) {
            handle_command(cmd);

            String response = "ID: " + String(id) + ", CMD: " + String(cmd) + ", OK";
            
//            set_send();
//            
//            Serial.println(response);
//            Serial.flush();
//            
//            set_receive();
        }
    }
}


void handle_command(int command) {
  if (command == 1) {
    servo.write(90);
  } else if (command == 0) {
    servo.write(0);
  }
}


void set_send()
{
    digitalWrite(MAX485_CONTROL_PIN, HIGH);
    delay(50);
}


void set_receive()
{
    digitalWrite(MAX485_CONTROL_PIN, LOW);
    delay(50);
}
