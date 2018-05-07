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
const byte maxDistance = 50;

const byte pixelPin = 6;

const bool commonAnode = true;

const byte buttonPin = 2;

const byte interruptPin = 3;
volatile byte state = LOW;

int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned int pingInterval = 50;
unsigned long pingTimer[iterations];
unsigned int cm[iterations];
uint8_t currentIteration = 0;

bool scanned = false;
uint8_t closestTarget = 0;

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
void exitScan();

void setPixelOff();
void setPixelGreen();
void setPixelYellow();
void setPixelRed();

void forwardUpdate();

State off = State(noopUpdate);
State green = State(setPixelGreen, NULL, setPixelOff);
State yellow = State(setPixelYellow, NULL, setPixelOff);
State red = State(setPixelRed, NULL, setPixelOff);

FSM ledStateMachine = FSM(off);

State noop = State(noopUpdate);
State scan = State(scanEnter, scanUpdate, exitScan);

FSM stateMachine = FSM(noop);

State navigate = State(forwardUpdate);

FSM motorStateMachine = FSM(noop);

void setup() {
  Serial.begin(9600);
  Serial.println("Automata Test!");
  
  pinMode(buttonPin, INPUT);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), trigger, CHANGE);

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
  }
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
        
        if(stateMachine.isInState( noop )){
      
          stateMachine.transitionTo(scan);
      
        }else{
          stateMachine.transitionTo(noop);
        }
        
        
      }
    }
  }
  lastButtonState = reading;

  stateMachine.update();
  ledStateMachine.update();
  motorStateMachine.update();
}

void noopUpdate() {
  //this function gets called as long as the user have not pressed any buttons after startup
}

void scanEnter(){
  Serial.println("Scan Enter");
  ledStateMachine.immediateTransitionTo(yellow);
}

void scanUpdate() {

    if (!scanned){
      
        for (uint8_t i = 0; i < iterations; i++) {
          
          int currentPos = i*18;
          Serial.println(currentPos);
          for (int pos = currentPos; pos <= currentPos+18; pos += 1) {
            //Serial.println(pos);
            servoPan.write(pos);
            delay(15);
          }
          
          
        if (millis() >= pingTimer[i]) {
          pingTimer[i] += pingInterval * iterations;
          if (i == 0 && currentIteration == iterations - 1) oneSensorCycle();
          sonar.timer_stop();
          currentIteration = i;
          cm[currentIteration] = 0;
          Serial.print("Sending Ping");
          Serial.println(currentIteration);
          sonar.ping_timer(echoCheck);
          delay(pingInterval);
        }
        
      }

      //pointTurn("left",180);
      //stopMotors();
      //
    }else{
      stateMachine.immediateTransitionTo(noop);
      motorStateMachine.transitionTo(navigate);
      Serial.println("scanned!");
      
    }
  
}

void exitScan(){
  servoPan.write(90);
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

void trigger() {
  state = digitalRead(interruptPin);
  if (state == HIGH)
  {
    Serial.println("LineTracker is on the line");
  }
  else if (state == LOW)
  {
    Serial.println("Linetracker is not on the line");
  }
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
  uint8_t smallest = 100;
  uint8_t it = iterations;
  for (uint8_t i = 0; i < it; i++) {
    Serial.print(i);
    Serial.print(" iteration has ");
    Serial.println(cm[i]);
    if (cm[i] != NO_ECHO) {
      if (cm[i] > 0 && cm[i] < smallest) {
          smallest = cm[i];
      }
    } else it--;
  }
  if(smallest > 0 && smallest < 100){
    scanned = true;
    closestTarget = smallest;
  }
  Serial.print("smallest: ");
  Serial.print(smallest);
  Serial.println("cm");

  memset(cm, 0, sizeof(cm));
}

void forwardUpdate(){

  //Serial.print(closestTarget);

  if(closestTarget > 0){

    int duration = closestTarget / 0.055;

    forward(255, duration);
    stopMotors();
    closestTarget = 0;


    
  }else{
    pointTurn("left",180);
    stopMotors();
delay(1000);
    pointTurn("right",180);
    stopMotors();
delay(1000);
  }
  
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
/*w2
 * snelheid x tijd = afstand
 * afstand / snelheid = tijd
 * afstand / tijd = snelheid
 * 
 * Afstand tussen midden van wielen: 15,3 cm
 * Omtrek point turn is 15,3 * PI = 48,07 cm
 * Motor max ~= 150 RPM
 * nauwkeurigheid op 10 graden
 */
void pointTurn(String direction, int degrees){
  //Serial.println(direction);
  //Serial.println(degrees);
  if(direction == "right"){
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
  }else if(direction == "left"){
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
  }
  for(int i=0;i<degrees;i+=(degrees/10)){
    //Serial.println(i);
    motor1->setSpeed(128);
    motor2->setSpeed(128);
    delay(55);
  }
}

void stopMotors(){
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  delay(1000);
}
