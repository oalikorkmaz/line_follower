#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <HCSR04.h>


#define Kp 1
#define Ki 1
#define Kd 2

#define RIGHT_MOTOR_IN1 6
#define RIGHT_MOTOR_IN2 5
#define RIGHT_MOTOR_ENA 7

#define LEFT_MOTOR_IN1 3
#define LEFT_MOTOR_IN2 4
#define LEFT_MOTOR_ENB 2

//#define THRESHOLD 1000

#define left_trig_pin 31
#define left_echo_pin 33
#define mid_trig_pin 35
#define mid_echo_pin 37
#define right_trig_pin 39
#define right_echo_pin 41

int(yesiloku) = 0;
QTRSensors qtr;
bool line = true; // true = white, false = black
const uint8_t sensor_count = 8;
unsigned int sensor_values[sensor_count];
int last_error = 0;

double set_point = 3500, input, output;
PID myPID(&input, &output, &set_point, Kp, Ki, Kd, DIRECT);

UltraSonicDistanceSensor left_distance_sensor(left_trig_pin, left_echo_pin, 400);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor mid_distance_sensor(mid_trig_pin, mid_echo_pin, 400);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor right_distance_sensor(right_trig_pin, right_echo_pin, 400);  // Initialize sensor that uses digital pins 13 and 12.


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

void motor_stop(){
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
}


void surface(){
    if(sensor_values[0] >= 700 && sensor_values[1] >= 700 && sensor_values[6] >= 700 && sensor_values[7] >= 700){
        line = true;
    }
    if(sensor_values[0] <= 50 && sensor_values[1] <= 50 && sensor_values[6] <= 50 && sensor_values[7] <= 50){
        line = false;
    }
}

void setup()
{
    Serial.begin(9600);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_ENA, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_ENB, OUTPUT);

    myPID.SetSampleTime(25);

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
// void turn_right(){
    
//     motor_right(100 + output);
//     //motor_left(-65 - output);

// }
void main_code(){
    surface();
    unsigned int position;
    if (line) {
        position = qtr.readLineWhite(sensor_values);
    } else {
        position = qtr.readLineBlack(sensor_values);
    }
    //unsigned int position = qtr.readLineBlack(sensor_values);
    //Serial.println("Position: " + String(position));
    //Serial.println("Output: " + String(output));
    // if(sensor_values[0] > 700 && sensor_values[1] > 700 && sensor_values[2] > 700 && sensor_values[3] > 700 && sensor_values[6] < 300 && sensor_values[7]<300){
    //     motor_stop();
    //     delay(100);
    //     turn_right();
    //     delay(300);
    // }

    input = position;
    //Serial.println("Position: " + String(position));
    myPID.Compute();
    motor_left(100 - output);
    motor_right(100 + output);

}


void loop()
{   
    // float left  = left_distance_sensor.measureDistanceCm();
    // float mid = mid_distance_sensor.measureDistanceCm();
    // float right = right_distance_sensor.measureDistanceCm();
    // if(left == -1 || mid == -1 || right == -1){
    //     return;
    // }
    // if((left >= 20 && left > 0) || (right>=20 && right > 0)){

    //     if(left>right)
    //     motor_left(-100 - output);
    //     motor_right(100 + output);
    //     while (left==right)
    //     {

    //         motor_left(80 - output);
    //         motor_right(80 + output);
    //         /* code */
    //         if((left==-1) || (right==-1))
    //         break;
    //     }
        

    // }


    // else if((mid <= 10 && mid > 0)){

    //     motor_left(-70 - output);
    //     motor_right(70 + output);
    //     yesiloku=+1;


    // }

    main_code();
    
    
}

