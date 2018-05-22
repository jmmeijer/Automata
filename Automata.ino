#include <FiniteStateMachine.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

/*
 * Robot settings
 */
const byte wheelDiameter = 67; // in mm
const byte wheelRadius = wheelDiameter/2;
const byte distanceBetweenWheels = 183; // in mm

const byte iterations = 20;
const byte triggerPin = A0;
const byte echoPin = A1;
const byte maxDistance = 50; // TODO change to variable to increase scanning distance after fails

const byte pixelPin = 6;

const bool commonAnode = true;

const byte buttonPin = 2;

const byte interruptPin = 3; //aanpassen
volatile byte state = LOW;

int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long currentMillis;

unsigned int pingInterval = 50;
unsigned long pingTimer[iterations];
unsigned int cm[iterations];
uint8_t currentIteration = 0;

unsigned int orientation = 0;
unsigned int targetDegrees = 0;

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

// function prototypes for reference in function 
void noopUpdate();

void scanEnter();
void scanUpdate();
void scanExit();

void setPixelOff();
void setPixelGreen();
void setPixelYellow();
void setPixelRed();

void navigateEnter();
void navigateUpdate();
void navigateExit();

void evadeEnter();
void evadeUpdate();
void evadeExit();

void detectEnter();
void detectUpdate();
void detectExit();

void forwardEnter();
void forwardUpdate();
void forwardExit();

void backwardEnter();
void backwardUpdate();
void backwardExit();

void pointTurnEnter();
void pointTurnUpdate();
void pointTurnExit();

void stopMotorsEnter();

State noop = State(noopUpdate);

State green = State(setPixelGreen, NULL, setPixelOff);
State yellow = State(setPixelYellow, NULL, setPixelOff);
State red = State(setPixelRed, NULL, setPixelOff);

FSM ledStateMachine = FSM(noop);

State scan = State(scanEnter, scanUpdate, scanExit);
State navigate = State(navigateEnter, navigateUpdate, navigateExit);
State detect = State(detectEnter, detectUpdate, detectExit);
State evade = State(evadeEnter, evadeUpdate, evadeExit);

FSM stateMachine = FSM(noop);

State forward = State(forwardEnter, forwardUpdate, forwardExit);
State backward = State(backwardEnter, backwardUpdate, backwardExit);
State pointTurn = State(pointTurnEnter, pointTurnUpdate, pointTurnExit);
State stopMotors = State(stopMotorsEnter, NULL, NULL);

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

  //State currentState = stateMachine.getCurrentState();
  
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
  Serial.println("Enter Scanning");
  ledStateMachine.immediateTransitionTo(yellow);
}

void scanUpdate() {

delay(50);
          int distanceCM = sonar.ping_cm();
          
          Serial.print("Ping: ");
          Serial.print(distanceCM);
          Serial.println("cm");
          
          if(distanceCM > 0){
            Serial.println("Scanned");
            scanned = true;
            closestTarget = distanceCM;
            
          }

    if (!scanned){

      targetDegrees = 18;
      
        for (uint8_t i = 0; i < iterations; i++) {

          int currentPos = i*18;
          Serial.println(currentPos);
          for (int pos = currentPos; pos <= currentPos+18; pos += 18) {
            //Serial.println(pos);
            //servoPan.write(pos);
            //delay(15);
            
            motorStateMachine.transitionTo(pointTurn);
          }


          
          //TODO: if statement on signal from pointTurn 
          /*
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
        */
        
        
      }

    }else{
      //stateMachine.immediateTransitionTo(noop);
      stateMachine.transitionTo(navigate);
      //motorStateMachine.transitionTo(forward);
      Serial.println("scanned!");
      
    }
  
}

void scanExit(){
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
    stateMachine.immediateTransitionTo(noop); // TODO: evade after line interrupt
    motorStateMachine.immediateTransitionTo(stopMotors);
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


void navigateEnter(){
  
}
void navigateUpdate(){
  // Add logic based on position and orientation
    delay(50);
    int distanceCM = sonar.ping_cm();
    
    Serial.print("Ping: ");
    Serial.print(distanceCM);
    Serial.println("cm");

    closestTarget = distanceCM;
    
    if(closestTarget < 10){
      Serial.println("nu gaan stoppen");
      ledStateMachine.immediateTransitionTo(red);
      motorStateMachine.transitionTo(stopMotors);
      stateMachine.transitionTo(detect);
    }else{
      motorStateMachine.transitionTo(forward);
    }
    
}
void navigateExit(){
  
}

void detectEnter(){
  
  

}

void detectUpdate(){
  uint16_t clear, red, green, blue;
  
  tcs.setInterrupt(false);      // turn on LED
  
  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 255; g *= 255; b *= 255;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
  
}

void detectExit(){
  tcs.setInterrupt(true);  // turn off LED
}

/*
 * http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
 * https://forum.arduino.cc/index.php?topic=98147.0
 * radians = (degrees * 71) / 4068
 * degrees = (radians * 4068) / 71
 * 
 * SL = rθ
 */

void roundingCorner(int radius, int degrees){

  int distanceBetweenWheels = 153;
  
  float radians = (degrees * 71) / 4068;

  float displacementLeft = radius * radians;
  float displacementRight = (radius + distanceBetweenWheels) * radians;
  float displacementCenter = (radius + distanceBetweenWheels / 2) * radians;

    Serial.print("degrees: ");
  Serial.println(degrees);

    Serial.print("radians: ");
  Serial.println(radians);


  
/*
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor1->setSpeed(255);
    motor2->setSpeed(255);
    */
}

/*
 * http://www.softschools.com/formulas/physics/distance_speed_time_formula/75/
 * speed = distance / time (ex: 550 mm / 1000 ms, 55 cm/s)
 * time = distance / speed 
 * distance = speed * time
 */

void forwardEnter(){

  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);

  motor1->run(FORWARD);
  motor2->run(FORWARD);

}

/*
 * https://tomblanch.wordpress.com/2013/07/18/working-with-arduino-millis/
 */
void forwardUpdate(){


/*
  Serial.print("closestTarget: ");
  Serial.println(closestTarget);

  //if(closestTarget > 0){ //TODO: move logic to superFSM
  //int duration = closestTarget / 0.055;
  int duration = closestTarget / 0.030;
  
  Serial.print("duration: ");
  Serial.println(duration);

  while(millis() - currentMillis < duration){
    
    Serial.print("millis: ");
    Serial.println(millis());
    Serial.println(currentMillis + duration);
    
    motor1->setSpeed(128);
    motor2->setSpeed(128);
    //delay(10);
  }

  motorStateMachine.transitionTo(noop);

  duration = 0;
  closestTarget = 0;
    
    //stopMotors();
    //closestTarget = 0;
  */

    motor1->setSpeed(128);
    motor2->setSpeed(128);
  
}

void forwardExit(){
  // TODO: Braking
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

void backwardEnter(){
  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
}

void backwardUpdate(){
  
}

void backwardExit(){
  // TODO: Braking
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

/*
 * snelheid x tijd = afstand
 * afstand / snelheid = tijd
 * afstand / tijd = snelheid
 * 
 * Afstand tussen midden van wielen: 15,3 cm
 * Omtrek point turn is 15,3 * PI = 48,07 cm
 * Motor max ~= 150 RPM
 * nauwkeurigheid op 10 graden
 */
void pointTurnEnter(){
  Serial.print("targetDegrees: ");
  Serial.println(targetDegrees);
  
  if( targetDegrees > 0 && targetDegrees <= 180 ){
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
  }else if( targetDegrees > 180 && targetDegrees <= 360){
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
  }
}

void pointTurnUpdate(){

    motor1->setSpeed(64);
    motor2->setSpeed(64);
    delay(200);
  
  /*
  for(int i=orientation;i<targetDegrees;i+=(targetDegrees/10)){
    //Serial.println(i);
    motor1->setSpeed(128);
    motor2->setSpeed(128);
    delay(50);
  }
  */
  motorStateMachine.immediateTransitionTo(noop);
}

void pointTurnExit(){
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

void evadeEnter(){
  
}

void evadeUpdate(){
  
}

void evadeExit(){
  
}

void stopMotorsEnter(){
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  delay(1000);
}





