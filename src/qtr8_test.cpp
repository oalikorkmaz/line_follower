#include <QTRSensors.h>
#include <Arduino.h>

QTRSensors qtr;
const uint8_t sensor_count = 8;
uint16_t sensor_values[sensor_count];    

void setup() {
    Serial.begin(9600);
    delay(500);
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensor_count);
    int i;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // LED'i yak
    for (i = 0; i < 400; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // LED'i söndür
    delay(1000);
}

void read_line(){
    uint16_t position = qtr.readLineWhite(sensor_values);
    Serial.println("Position: " + String(position));
}

void loop() {
    read_line();
}
