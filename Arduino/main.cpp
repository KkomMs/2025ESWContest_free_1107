// Arduino Mega 2560
// COM4: Linear motor, Ice maker
// COM6: Coffee machine

#include <Arduino.h>

// pin
const int DIR1 = 22;
const int PWM1 = 12;
const int DIR2 = 24;
const int PWM2 = 13;

const int MAX_SPEED = 255;

int pins[] = {46, 48, 50, 52}; //ice maker
/*
white : ice - 52pin
red : water - 50oin
green : cold - 48pin
yellow : hot - 46pin
*/
int pins_coffee[] = {47, 49, 51, 53};
/*
53pin = americano
51pin = soft americano
49pin = espresso
47pin = hot water
*/

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], 1);
    pinMode(pins_coffee[i], OUTPUT);
    digitalWrite(pins_coffee[i],1);
  }
	pinMode(DIR1, OUTPUT);
	pinMode(PWM1, OUTPUT);
	pinMode(DIR2, OUTPUT);
	pinMode(PWM2, OUTPUT);
}

void stopMotor(int motorNum) {
    if (motorNum == 1) {
        analogWrite(PWM1, 0);
    }
    else if (motorNum == 2) {
        analogWrite(PWM2, 0);
    }
}

void driveMotor(int motorNum, int direction) {
    if (motorNum == 1) {
        digitalWrite(DIR1, direction);
        analogWrite(PWM1, MAX_SPEED);
    }
    else if (motorNum == 2) {
        digitalWrite(DIR2, direction);
        analogWrite(PWM2, MAX_SPEED);
    }
}

void ice_maker(int function_num) {
    
    if (function_num == 1) { // hot
        digitalWrite(pins[0], 0);
        delay(1000);
        digitalWrite(pins[0], 1);
    }
    else if (function_num == 2) { // cold
        digitalWrite(pins[1], 0);
        delay(1000);
        digitalWrite(pins[1], 1);
    }
    else if (function_num == 3) { // water
        digitalWrite(pins[2], 0);
        delay(1000);
        digitalWrite(pins[2], 1);
    }
    else if (function_num == 4) { // ice
        digitalWrite(pins[3], 0);
        delay(1000);
        digitalWrite(pins[3], 1);
    }
}

void coffee_machine(int function_num){
    if (function_num == 5) { // hot water
        digitalWrite(pins_coffee[0], 0);
        delay(1000);
        digitalWrite(pins_coffee[0], 1);
    }
    else if (function_num == 6) { // espresso
        digitalWrite(pins_coffee[1], 0);
        delay(1000);
        digitalWrite(pins_coffee[1], 1);
    }
    else if (function_num == 7) { // soft americano
        digitalWrite(pins_coffee[2], 0);
        delay(1000);
        digitalWrite(pins_coffee[2], 1);
    }
    else if (function_num == 8) { // americano
        digitalWrite(pins_coffee[3], 0);
        delay(1000);
        digitalWrite(pins_coffee[3], 1);
    }
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 1){
            int function_num = input.toInt();
            if (function_num >= 1 && function_num <= 4) {
                ice_maker(function_num);
                Serial.println("send to ice maker.\n");

            }
            else {
                coffee_machine(function_num);
                Serial.println("send to coffee machine.\n");
            }
            return;
        }

        else{
            int motorNum = 0, direction = 0; int distance_mm = 0;
            int parsed = sscanf(input.c_str(), "%d %d %d", &motorNum, &direction, &distance_mm);

            if (parsed == 3 && (motorNum == 1 || motorNum == 2) && (direction == 0 || direction == 1) && distance_mm > 0) {
                int duration_ms = (int)((distance_mm / 29.0) * 1000.0);
                driveMotor(motorNum, direction);
                delay(duration_ms);
                stopMotor(motorNum);
            }
        }
        
    }
}
