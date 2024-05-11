#include <QTRSensors.h>
#include <Arduino.h>

#define Kp 0.7
#define Kd 0.6
#define right_max_speed 114
#define left_max_speed 115
#define right_base_speed 85
#define left_base_speed 86

#define RIGHT_MOTOR_IN1 5
#define RIGHT_MOTOR_IN2 6
#define RIGHT_MOTOR_ENA 7

#define LEFT_MOTOR_IN1 4
#define LEFT_MOTOR_IN2 3
#define LEFT_MOTOR_ENB 2

QTRSensors qtr;
bool line = true; // true = white, false = black
const uint8_t sensor_count = 8;
unsigned int sensor_values[sensor_count];
int last_error = 0;


void turn_left() {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite (RIGHT_MOTOR_ENA, right_base_speed);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite (LEFT_MOTOR_ENB, left_base_speed);
}

void turn_right() {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite (RIGHT_MOTOR_ENA, right_base_speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite (LEFT_MOTOR_ENB, left_base_speed);
}

void wait() {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
}

void surface(){
    if(sensor_values[0] >= 700 && sensor_values[1] >= 700 && sensor_values[6] >= 700 && sensor_values[7] <= 700){
        line = 1;
    }
    if(sensor_values[0] <= 50 && sensor_values[1] <= 50 && sensor_values[6] <= 50 && sensor_values[7] <= 50){
        line = 0;
    }
}

void setup()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensor_count);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_ENA, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_ENB, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    for (int i = 0; i < 100; i++) {
        if (i < 25 || i >= 75 ) {
            turn_right();
        } else {
            turn_left();
        }
        qtr.calibrate();
        delay(20);
    }
    wait();
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}

void loop()
{   
    surface();
    unsigned int position;
    if (line) {
        position = qtr.readLineWhite(sensor_values);
    } else {
        position = qtr.readLineBlack(sensor_values);
    }

    int error = position - 3500;
    int motor_speed = Kp * error + Kd * (error - last_error);
    last_error = error;
    int right_motor_speed = right_base_speed + motor_speed;
    int left_motor_speed = left_base_speed - motor_speed;
    
    if (right_motor_speed > right_max_speed ) right_motor_speed = right_max_speed;
    if (left_motor_speed > left_max_speed ) left_motor_speed = left_max_speed;
    if (right_motor_speed < 0) right_motor_speed = 0;
    if (left_motor_speed < 0) left_motor_speed = 0;

    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENA, right_motor_speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENB, left_motor_speed);
}

