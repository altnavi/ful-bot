#include <Arduino.h>

#define LEDCITO 8

#define M1_IN1 20
#define M1_IN2 21
#define M2_IN1 10
#define M2_IN2 7


// El driver que usamos es el DRV8833.

void setMotor(int, int);

void setup() {
	pinMode(LEDCITO, OUTPUT);
	pinMode(M1_IN1, OUTPUT);
	pinMode(M1_IN2, OUTPUT);
	pinMode(M2_IN1, OUTPUT);
	pinMode(M2_IN2, OUTPUT);
}

void setMotor();

void loop() {
	// put your main code here, to run repeatedly:
	digitalWrite(LEDCITO, HIGH);
	setMotor(1,HIGH);
	delay(250);
	setMotor(1,-1);
	delay(250);
	setMotor(1,LOW);

	digitalWrite(LEDCITO, LOW);
	setMotor(2,HIGH);
	delay(250);
	setMotor(2,-1);
	delay(250);
	setMotor(2,LOW);
}

void setMotor(int motor, int estado){
	switch (motor){
		case 1:
			switch(estado){
				case LOW:
					digitalWrite(M1_IN1, LOW);
					digitalWrite(M1_IN2, LOW);
					break;
				case HIGH:
					digitalWrite(M1_IN2, LOW);
					digitalWrite(M1_IN1, HIGH);
					break;
				case -1:
					digitalWrite(M1_IN1, LOW);
					digitalWrite(M1_IN2, HIGH);
					break;
				default:
					break;
			}
			break;
		case 2:
			switch(estado){
				case LOW:
					digitalWrite(M2_IN1, LOW);
					digitalWrite(M2_IN2, LOW);
					break;
				case HIGH:
					digitalWrite(M2_IN2, LOW);
					digitalWrite(M2_IN1, HIGH);
					break;
				case -1:
					digitalWrite(M2_IN1, LOW);
					digitalWrite(M2_IN2, HIGH);
					break;
				default:
					break;
			}
			break;
	}

}