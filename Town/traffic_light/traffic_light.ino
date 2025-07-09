#define RED_PIN 4 
#define YELLOW_PIN 3 
#define GREEN_PIN 5 
#define MAX485_CONTROL_PIN 2

const int device_id = 1;


void setup()
{
    pinMode(RED_PIN, OUTPUT);
    pinMode(YELLOW_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(MAX485_CONTROL_PIN, OUTPUT);
    Serial.begin(9600);
 
    delay(10);
}


void loop()
{
    if (Serial.available() >= 2) {
        int id = Serial.read() - '0';
        int cmd = Serial.read() - '0';

        if (id == device_id) {
            handle_command(cmd);
            set_send();
            Serial.print(id);
            Serial.print(cmd);
            Serial.println("OK");
            set_receive();
        }
    }
}


void handle_command(int command)
{
    if (command == 0) { 
        clear_lights();
        digitalWrite(GREEN_PIN, HIGH);
    } else if (command == 1) {
        clear_lights();
        digitalWrite(YELLOW_PIN, HIGH);
    } else if (command == 2) {
        clear_lights();
        digitalWrite(RED_PIN, HIGH);
    }
}


void clear_lights()
{
    digitalWrite(RED_PIN, LOW);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
}


void set_send()
{
    digitalWrite(MAX485_CONTROL_PIN, HIGH);
    delay(10);
}


void set_receive()
{
    digitalWrite(MAX485_CONTROL_PIN, LOW);
    delay(10);
}
