#include <FiniteStateMachine.h>
#include <Wire.h>
/*
  #include <Servo.h>
*/
#include <Adafruit_MotorShield.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

#define TCAADDR 0x70

/*
   Robot settings
*/
const byte wheelDiameter = 67; // in mm
const byte wheelRadius = wheelDiameter / 2;
const byte distanceBetweenWheels = 183; // in mm

const byte iterations = 20;
const byte triggerPin = A0;
const byte echoPin = A1;
byte maxDistance = 50;

const byte relayPin = 8;

const byte pixelPin = 6;

const bool commonAnode = true;

const byte buttonPin = 3;

const byte lineSensorPin = 2;
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
int targetDegrees = 0;


unsigned int numberMoves = 0;

enum color {
  NONE,
  RED,
  GREEN,
  BLUE
};

const color targetColor = RED;
color scannedColor = NONE;

bool inBounds = true;
bool scanned = false;
bool grabbedTarget = false;
bool dropZoneFound = false;
bool inPosition = false;
uint8_t closestTarget = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
Adafruit_TCS34725 tcsFront = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsBottom = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, pixelPin, NEO_RGB + NEO_KHZ800);
NewPing sonar(triggerPin, echoPin, maxDistance);

// helper function to select i2c port
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

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

void grabEnter();
void grabUpdate();
void grabExit();

void forwardEnter();
void forwardUpdate();
void forwardExit();

void backwardEnter();
void backwardUpdate();
void backwardExit();

void pointTurnEnter();
void pointTurnUpdate();
void pointTurnExit();

void scoutEnter();
void scoutUpdate();
void scoutExit();

void stopMotorsEnter();

State noop = State(noopUpdate);

State green = State(setPixelGreen, NULL, setPixelOff);
State yellow = State(setPixelYellow, NULL, setPixelOff);
State red = State(setPixelRed, NULL, setPixelOff);

FSM ledStateMachine = FSM(noop);

State scan = State(scanEnter, scanUpdate, scanExit);
State navigate = State(navigateEnter, navigateUpdate, navigateExit);
State detect = State(detectEnter, detectUpdate, detectExit);
State grab = State(grabEnter, grabUpdate, grabExit);
State evade = State(evadeEnter, evadeUpdate, evadeExit);
State scout = State(scoutEnter, scoutUpdate, scoutExit);

FSM stateMachine = FSM(noop);

State forward = State(forwardEnter, forwardUpdate, forwardExit);
State backward = State(backwardEnter, backwardUpdate, backwardExit);
State pointTurn = State(pointTurnEnter, pointTurnUpdate, pointTurnExit);
State stopMotors = State(stopMotorsEnter, NULL, NULL);

FSM motorStateMachine = FSM(noop);

void setup() {
  Serial.begin(9600);
  Serial.println("Automata Test!");

  Serial.print("Target color: ");
  Serial.println(targetColor);

  pinMode(relayPin, OUTPUT);
  //digitalWrite(relayPin, HIGH);

  pinMode(buttonPin, INPUT);

  pinMode(lineSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lineSensorPin), trigger, CHANGE);

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < iterations; i++) {
    pingTimer[i] = pingTimer[i - 1] + pingInterval;
  }
  /*
    servoPan.attach(9,450,2450);
    servoTilt.attach(10,500,2400);
    servoPan.write(90);
    servoTilt.write(90);
  */
  AFMS.begin();
  motorLeft->setSpeed(150);
  motorLeft->run(FORWARD);
  motorLeft->run(RELEASE);

  motorRight->setSpeed(150);
  motorRight->run(FORWARD);
  motorRight->run(RELEASE);

  tcaselect(6);
  if (tcsFront.begin()) {
    Serial.println("Found TCS34725 sensor 1");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  tcaselect(7);
  if (tcsBottom.begin()) {
    Serial.println("Found TCS34725 sensor 2");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // Turn off LEDs
  tcaselect(6);
  tcsFront.setInterrupt(true);
  tcaselect(7);
  tcsBottom.setInterrupt(true);

  pixel.begin();
  pixel.show();
}

void loop() {


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

        if (stateMachine.isInState( noop )) {

          stateMachine.transitionTo(scan);

        } else {
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

void scanEnter() {
  Serial.println("Enter Scanning");
  ledStateMachine.immediateTransitionTo(yellow);
  currentIteration = 0;
}

void scanUpdate() {

  if (!scanned) {

    delay(100);
    int distanceCM = sonar.ping_cm();
    delay(100);

    Serial.print("Ping: ");
    Serial.print(distanceCM);
    Serial.println("cm");

    if (distanceCM > 0) {
      Serial.println("Scanned something!");
      scanned = true;
      closestTarget = distanceCM;
    }

    if (!motorStateMachine.isInState( pointTurn )) {
      targetDegrees = 6;
      motorStateMachine.transitionTo( pointTurn );
    }

    // Try to get color 5 times...
    if (currentIteration >= 160) {
      motorStateMachine.immediateTransitionTo(forward);
      delay(1200);
      motorStateMachine.immediateTransitionTo(stopMotors);
      // reset iteration
      currentIteration = 0;
    }
    else if (currentIteration == 80) {
      maxDistance = 60;
    }
    currentIteration++;

  } else {
    targetDegrees = 6;
    motorStateMachine.transitionTo(pointTurn);
    Serial.println("Target aquired, navigate!");
    stateMachine.transitionTo(navigate);
  }

}

void scanExit() {
  // reset iterations
  currentIteration = 0;
}

void setPixelOff() {
  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
}

void setPixelGreen() {
  pixel.setPixelColor(0, 0, 255, 0);
  pixel.show();
}

void setPixelYellow() {
  pixel.setPixelColor(0, 255, 191, 0);
  pixel.show();
}

void setPixelRed() {
  pixel.setPixelColor(0, 255, 055, 0);
  pixel.show();
}

void trigger() {
  state = digitalRead(lineSensorPin);
  if (!stateMachine.isInState( noop ) ) {

    if (state == HIGH) {
      Serial.println("LineTracker is on the line");
      inBounds = true;
      //stateMachine.transitionTo(noop); // TODO: evade after line interrupt
      //motorStateMachine.immediateTransitionTo(stopMotors);
    }
    else if (state == LOW && inBounds == true) {
      Serial.println("Linetracker is not on the line");
      inBounds = false;

      ledStateMachine.immediateTransitionTo(red);

      motorStateMachine.immediateTransitionTo(backward);
      delay(400);

      Serial.println("Turn a bit");
      targetDegrees = 120;
      motorStateMachine.immediateTransitionTo(pointTurn);
      delay(1000);

      Serial.println("start doing something else");

      motorStateMachine.transitionTo(stopMotors);

      //stateMachine.transitionTo(scan);

      inBounds = true;
      ledStateMachine.immediateTransitionTo(yellow);

    }
  }
}

void echoCheck() {
  if (sonar.check_timer()) {
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
  if (smallest > 0 && smallest < 100) {
    scanned = true;
    closestTarget = smallest;
  }
  Serial.print("smallest: ");
  Serial.print(smallest);
  Serial.println("cm");

  memset(cm, 0, sizeof(cm));
}


void navigateEnter() {

}
void navigateUpdate() {
  // Add logic based on position and orientation
  delay(50);
  int distanceCM = sonar.ping_cm();

  Serial.print("Ping: ");
  Serial.print(distanceCM);
  Serial.println("cm");


  if (distanceCM == 0 && closestTarget >= 9) {
    Serial.println("Lost Target!");

    scanned = false;

    stateMachine.transitionTo(scan);

  } else {
    closestTarget = distanceCM;
  }


  if (closestTarget < 5) {
    Serial.println("nu gaan stoppen");


    // TODO: Check distance and orientation

    ledStateMachine.immediateTransitionTo(red);
    motorStateMachine.transitionTo(stopMotors);
    stateMachine.transitionTo(detect);
  } else {
    motorStateMachine.transitionTo(forward);
  }

}
void navigateExit() {

}

void detectEnter() {
  // Reset scanned color
  scannedColor = NONE;
  currentIteration = 0;
}

/*
   https://www.makerblog.at/2015/01/farben-erkennen-mit-dem-rgb-sensor-tcs34725-und-dem-arduino/
*/
void detectUpdate() {
  uint16_t clear, red, green, blue;


  tcaselect(6);
  tcsFront.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read

  tcsFront.getRawData(&red, &green, &blue, &clear);

  tcsFront.setInterrupt(true);  // turn off LED
  /*
    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.print(blue);
  */
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;

  float average, r, g, b;

  average = (red + green + blue) / 3;

  r = red; r /= average;
  g = green; g /= average;
  b = blue; b /= average;


  Serial.print("Clear:"); Serial.print(clear);
  Serial.print("\tRed:"); Serial.print(r);
  Serial.print("\tGreen:"); Serial.print(g);
  Serial.print("\tBlue:"); Serial.print(b);
  Serial.println();

  guessColor(r, g, b);
  delay(100);

  if (scannedColor == targetColor) {
    Serial.println("Scanned targetcolor, grab it!");
       motorStateMachine.immediateTransitionTo(forward);
      delay(100);
      motorStateMachine.immediateTransitionTo(stopMotors);
    stateMachine.transitionTo(grab);
  } else if (scannedColor != NONE && scannedColor != targetColor) {

    Serial.println("Scanned other color, evade it!");

    // reset scanned & closestTarget
    scanned = false;
    closestTarget = 0;

    stateMachine.transitionTo(evade);
  } else {
    Serial.println("Keep scanning! Or perhaps time to move...");
    // keep scanning or move closer towards target

    // Try to get color 5 times...
    if (currentIteration >= 10) {
      // ...before moving
      stateMachine.transitionTo(scan);
    }
    else if (currentIteration == 5) {
      motorStateMachine.immediateTransitionTo(forward);
      delay(100);
      motorStateMachine.immediateTransitionTo(stopMotors);
      // reset iterations and try again
      //currentIteration = 0;
    }
    currentIteration++;
  }

}

void guessColor(float r, float g, float b) {
  Serial.print("Scanned color: ");

  if ((r > 1.65) && (g < 0.7) && (b < 0.6)) {
    Serial.print("RED");
    scannedColor = RED;
  }
  else if ((r < 1.05) && (g > 1.3) && (b < 0.9)) {
    Serial.print("GREEN");
    scannedColor = GREEN;
  }
  else if ((r < 0.8) && (g < 1.8) && (b > 1.2)) {
    Serial.print("BLUE");
    scannedColor = BLUE;
  }
  else {
    Serial.print("Not recognized");
    scannedColor = NONE;
  }
  Serial.println();
  /*
    Serial.println(scannedColor);
  */
}

void detectExit() {
  tcsFront.setInterrupt(true);  // turn off LED
  // reset iterations
  currentIteration = 0;
  scannedColor = NONE;
}

/*
   http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
   https://forum.arduino.cc/index.php?topic=98147.0
   radians = (degrees * 71) / 4068
   degrees = (radians * 4068) / 71

   SL = rθ
*/

void roundingCorner(int radius, int degrees) {

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
      motorLeft->run(FORWARD);
      motorRight->run(FORWARD);
      motorLeft->setSpeed(255);
      motorRight->setSpeed(255);
  */
}

/*
   http://www.softschools.com/formulas/physics/distance_speed_time_formula/75/
   speed = distance / time (ex: 550 mm / 1000 ms, 55 cm/s)
   time = distance / speed
   distance = speed * time
*/

void forwardEnter() {

  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);

  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);

}

/*
   https://tomblanch.wordpress.com/2013/07/18/working-with-arduino-millis/
*/
void forwardUpdate() {


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

      motorLeft->setSpeed(128);
      motorRight->setSpeed(128);
      //delay(10);
    }

    motorStateMachine.transitionTo(noop);

    duration = 0;
    closestTarget = 0;

      //stopMotors();
      //closestTarget = 0;
  */
  // Just drive...
  motorLeft->setSpeed(64);
  motorRight->setSpeed(64);

}

void forwardExit() {
  // TODO: Braking
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void backwardEnter() {
  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);
  motorLeft->run(BACKWARD);
  motorRight->run(BACKWARD);
}

void backwardUpdate() {
  motorLeft->setSpeed(64);
  motorRight->setSpeed(64);
}

void backwardExit() {
  // TODO: Braking
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

/*
   snelheid x tijd = afstand
   afstand / snelheid = tijd
   afstand / tijd = snelheid

   Afstand tussen midden van wielen: 15,3 cm
   Omtrek point turn is 15,3 * PI = 48,07 cm
   Motor max ~= 150 RPM
   nauwkeurigheid op 10 graden
*/
void pointTurnEnter() {
  Serial.print("targetDegrees: ");
  Serial.println(targetDegrees);

  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);

  if ( (targetDegrees > 0 && targetDegrees <= 180) || (targetDegrees < -180 && targetDegrees <= -360) ) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
  } else if ( (targetDegrees > 180 && targetDegrees <= 360) || (targetDegrees < 0 && targetDegrees <= -180) ) {
    motorLeft->run(BACKWARD);
    motorRight->run(FORWARD);
  }
}

void pointTurnUpdate() {

  motorLeft->setSpeed(48);
  motorRight->setSpeed(48);

  for (int pos = orientation; pos <= orientation + targetDegrees; pos += 1) {
    delay(8);
  }

  /*
    for(int i=0;i<targetDegrees;i+=(targetDegrees/10)){
    delay(50);
    }
  */


  motorStateMachine.immediateTransitionTo(noop);


}

void pointTurnExit() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void grabEnter() {
  Serial.println("Enter grab state");

}

void grabUpdate() {
  digitalWrite(relayPin, HIGH); // in update state?
  grabbedTarget = true;
  //TODO: check if grab succeeded, possibly by using distance sensor
  if(grabbedTarget == true){
    stateMachine.transitionTo(scout);
  }
}

void grabExit() {

}

void evadeEnter() {
  Serial.println("Enter evade state");
  targetDegrees = 18;
  numberMoves = 2;

  currentMillis = millis();

  Serial.print("currentMillis: ");
  Serial.println(currentMillis);

}

void evadeUpdate() {
  // assuming motors stopped
  switch (numberMoves) {
    case 2:
      Serial.println("Go backwards");
      motorStateMachine.immediateTransitionTo(backward);
      delay(600);
      // decrease number of moves to make
      numberMoves--;
      break;
    case 1:
      Serial.println("Turn a bit");
      targetDegrees = 45;
      motorStateMachine.immediateTransitionTo(pointTurn);
      /*
         1200 ms = 180 deg
         600 ms = 90 deg
         400 ms = 60 deg
         200 ms = 30 deg
      */
      //delay(350);
      numberMoves--;
      break;
    case 0:
      Serial.println("start doing something else");
      stateMachine.transitionTo(scan);
      motorStateMachine.transitionTo(stopMotors);
      break;
    default:
      Serial.println("Oops, not supposed to get here!");
      // statements
  }




  // TODO: check for other object
}

void evadeExit() {

}

void scoutEnter() {
  scannedColor = NONE;
}

void scoutUpdate() {



  if(dropZoneFound == false){

    uint16_t clear, red, green, blue;
  
    tcaselect(7);
    tcsBottom.setInterrupt(false);      // turn on LED
  
    delay(60);  // takes 50ms to read
  
    tcsBottom.getRawData(&red, &green, &blue, &clear);
  
    tcsBottom.setInterrupt(true);  // turn off LED
    /*
      Serial.print("C:\t"); Serial.print(clear);
      Serial.print("\tR:\t"); Serial.print(red);
      Serial.print("\tG:\t"); Serial.print(green);
      Serial.print("\tB:\t"); Serial.print(blue);
    */
    // Figure out some basic hex code for visualization
    uint32_t sum = clear;
  
    float average, r, g, b;
  
    average = (red + green + blue) / 3;
  
    r = red; r /= average;
    g = green; g /= average;
    b = blue; b /= average;
  
  
    Serial.print("Clear:"); Serial.print(clear);
    Serial.print("\tRed:"); Serial.print(r);
    Serial.print("\tGreen:"); Serial.print(g);
    Serial.print("\tBlue:"); Serial.print(b);
    Serial.println();
  
    guessColor(r, g, b);
    delay(100);
  
    if (scannedColor != NONE ) {
      Serial.println("Scanned targetcolor, drop it!");

      dropZoneFound = true;
      motorStateMachine.immediateTransitionTo(noop);
      
    } else {
      
      motorStateMachine.immediateTransitionTo(forward);
      delay(600);
    
      targetDegrees = random(45, 315);
      motorStateMachine.immediateTransitionTo(pointTurn);
      delay(random(100, 1000));

      motorStateMachine.immediateTransitionTo(stopMotors);
      
    }

    

  }else if( dropZoneFound == true ){


      
  
     if (inPosition == true && grabbedTarget == true ){
      digitalWrite(relayPin, LOW);

      // finished!
      stateMachine.transitionTo(noop);
      motorStateMachine.transitionTo(noop);
      ledStateMachine.transitionTo(green);
     }else{
      motorStateMachine.immediateTransitionTo(backward);
      delay(600);
      inPosition = true;
    }
    
  }

  
  


}

void scoutExit() {

}

void stopMotorsEnter() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  delay(1000);
}





