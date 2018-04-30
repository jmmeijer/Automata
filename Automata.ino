#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

#define TRIGGER_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 53

#define PIXEL_PIN 6

#define commonAnode true

const int buttonPin = 2;

int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned int pingSpeed = 50;
unsigned long pingTimer;

byte gammatable[256];

Servo servoPan;
Servo servoTilt;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  Serial.println("Automata Test!");

  pingTimer = millis();

  pinMode(buttonPin, INPUT);

  servoPan.attach(9,450,2450);
  //servoTilt.attach(10,500,2400);
  servoPan.write(90);
  //servoTilt.write(90);

  AFMS.begin();
  motor1->setSpeed(150);
  motor1->run(FORWARD);
  motor1->run(RELEASE);

  motor2->setSpeed(150);
  motor2->run(FORWARD);
  motor2->run(RELEASE);

  pixel.begin();
  pixel.show();
}

void loop() {

  int reading = digitalRead(buttonPin);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        // do something
        Serial.println("Button pressed: Start!");
        setGreenLED();
      }
    }
  }

  lastButtonState = reading;
  
}

void setGreenLED(){
  pixel.setPixelColor(0, 0, 255, 0);
  pixel.show();
}

