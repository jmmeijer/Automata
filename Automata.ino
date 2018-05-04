#include <FiniteStateMachine.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

const byte iterations = 10;
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

unsigned int pingInterval = 50;
unsigned long pingTimer[iterations];
unsigned int cm[iterations];
uint8_t currentIteration = 0;

bool scanned = false;

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
void scanEnter();
void scanUpdate();

void setPixelGreen();
void setPixelYellow();
void setPixelRed();

State off = State(noopUpdate);
State green = State(setPixelGreen);
State yellow = State(setPixelYellow);
State red = State(setPixelRed);

FSM ledStateMachine = FSM(off);

State noop = State(noopUpdate);
State scan = State(scanEnter, scanUpdate, NULL);

FSM stateMachine = FSM(noop);

void setup() {
  Serial.begin(9600);
  Serial.println("Automata Test!");
  
  pinMode(buttonPin, INPUT);

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < iterations; i++){
    pingTimer[i] = pingTimer[i - 1] + pingInterval;
  }

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

  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
    Serial.println(gammatable[i]);
  }
}

void loop() {
  tcs.setInterrupt(true);
  
  int reading = digitalRead(buttonPin);

  if(stateMachine.isInState( scan )){



  }
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        // do something
        Serial.println("Button pressed: Start!");
        stateMachine.transitionTo(scan);
      }
    }
  }
  lastButtonState = reading;

  ledStateMachine.update();
  stateMachine.update();
}

void noopUpdate() {
  //this function gets called as long as the user have not pressed any buttons after startup
}

void scanEnter(){

  Serial.println("Scan Enter");
  
  ledStateMachine.transitionTo(yellow);
  

}

void scanUpdate() {

    if (!scanned){
        for (uint8_t i = 0; i < iterations; i++) {
          

          
        if (millis() >= pingTimer[i]) {
/*
          int currentPos = i*18;
          for (int pos = currentPos; pos <= currentPos+18; pos += 1) {
            servoPan.write(pos);
            delay(15);
          }
*/
          
          pingTimer[i] += pingInterval * iterations;
          if (i == 0 && currentIteration == iterations - 1) oneSensorCycle();
          sonar.timer_stop();
          currentIteration = i;
          cm[currentIteration] = 0;
          sonar.ping_timer(echoCheck);
        }
        
      }
      //scanned = true;
    }else{
      Serial.println("scanned!");
    }
  
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
  pixel.setPixelColor(0, 255, 191, 0);
  pixel.show();
}

void setPixelRed(){
  pixel.setPixelColor(0, 255, 055, 0);
  pixel.show();
}

void echoCheck() {
  if (sonar.check_timer()){
    cm[currentIteration] = sonar.ping_result / US_ROUNDTRIP_CM;
    Serial.print(currentIteration);
    Serial.print("Ping: ");
    Serial.print(sonar.ping_result / US_ROUNDTRIP_CM);
    Serial.println("cm");
  }
}

void oneSensorCycle() {

  unsigned int uS[iterations];
  uint8_t smallest = cm[0];
  uint8_t it = iterations;
  for (uint8_t i = 0; i < it; i++) {
    if (cm[i] != NO_ECHO) {
      if (cm[i] > 0 && cm[i] < smallest) {
          smallest = cm[i];
      }
    } else it--;
  }
  
  Serial.print("smallest: ");
  Serial.print(smallest);
  Serial.println("cm");

  memset(cm, 0, sizeof(cm));
}

void forward(int speed, int duration){
  int currentTime = millis();

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
