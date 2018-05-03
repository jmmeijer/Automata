#include <FiniteStateMachine.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

const byte triggerPin = A0;
const byte echoPin = A1;
const byte maxDistance = 53;

const byte pixelPin = 6;

const bool commonAnode = true;

const byte buttonPin = 2;

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
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, pixelPin, NEO_RGB + NEO_KHZ800);
NewPing sonar(triggerPin, echoPin, maxDistance);

void noopUpdate();
void setPixelGreen();

State off = State(noopUpdate);
State green = State(setPixelGreen);

FSM fsmLED = FSM(off);

State noop = State(noopUpdate);

FSM stateMachine = FSM(noop);

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

  if (tcs.begin()) {
   // Serial.println("Found sensor");
  } else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  pixel.begin();
  pixel.show();
}

void loop() {
  tcs.setInterrupt(true);

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
        fsmLED.transitionTo(green);
      }
    }
  }

  lastButtonState = reading;

  fsmLED.update();
  stateMachine.update();
}

void noopUpdate() {
  //this function gets called as long as the user have not pressed any buttons after startup
}

void setPixelOff(){
  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
}

void setPixelGreen(){
  pixel.setPixelColor(0, 0, 255, 0);
  pixel.show();
}

void setPixelYellow(){
  pixel.setPixelColor(0, 255, 255, 0);
  pixel.show();
}

void setPixelRed(){
  pixel.setPixelColor(0, 255, 055, 0);
  pixel.show();
}

void forward(int speed, int duration){
  currentTime = millis();

  while(millis() > currentTime + duration){
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
  }
  duration = 0;
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}
