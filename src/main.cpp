#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>

#define Kp 0.01
#define Ki 0.001
#define Kd 0.002
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

#define THRESHOLD 1000

QTRSensors qtr;
bool line = true; // true = white, false = black
const uint8_t sensor_count = 8;
unsigned int sensor_values[sensor_count];
int last_error = 0;
double set_point = 3500, input, output;
PID myPID(&input, &output, &set_point, Kp, Ki, Kd, DIRECT);

void motor_right(int duty){
    if(duty>=0){
        digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
    }else{
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    }
    analogWrite(RIGHT_MOTOR_ENA, duty);
}

void motor_left(int duty){
    if(duty>=0){
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
    }
    else{
        digitalWrite(LEFT_MOTOR_IN1, LOW);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
    }
    analogWrite(LEFT_MOTOR_ENB, duty);
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
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_ENA, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_ENB, OUTPUT);

    myPID.SetSampleTime(10);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensor_count);
    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    for(uint8_t i = 0; i< 200; i++){
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW);
    myPID.SetOutputLimits(-100, 100);
    myPID.SetMode(AUTOMATIC);


}

void main_code(){
    surface();
    unsigned int position;
    if (line) {
        position = qtr.readLineWhite(sensor_values);
    } else {
        position = qtr.readLineBlack(sensor_values);
    }

    input = position;
    Serial.println("Position: " + position);
    myPID.Compute();
    motor_left(100 - output);
    motor_right(100 + output);
}

void loop()
{   
    main_code();
}

