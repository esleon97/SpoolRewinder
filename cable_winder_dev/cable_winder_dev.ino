/*
    Program to run Cable Winder
    Developed by Kevin Bhimani and Esteban Leon with help from Eric Martin.


    Will move winder a number of turns based on input to run command.
    Will move the guide back and fourth at a speed relative to the winder based on the wind pitch setting.
    Guide direction reverses when target upper and lower limits are reached.
    After reversing guide direction the winder stops until the guide has moved a distance to compensate for any hysteresis in the winding position vs guide movement.

    winder class keeps values to control movement in units of steps and seconds for easy control of stepper motor
    values passed to and returned from methods are in millimeteres, revolutions, and seconds for easier user interface
*/

// Include the AccelStepper library:
#include <AccelStepper.h>
// Define the AccelStepper interface type:
#define MotorInterfaceType 4

// set constants
const int StepsPerRev = 200;
const float LeadScrewPitch = 2.0; // movement of traveling nut per revolution of lead screw, in mm
const float SpoolWidth = 14.8; // width of spool in mm
const long SpoolStepWidth = SpoolWidth * StepsPerRev / LeadScrewPitch; // width of spool in steps on guide stepper motor
const float GuidePositionHysteresis = 2.0; // default distance to move just the guide when reversing directions in mm
const long GuideStepHysteresis = GuidePositionHysteresis * StepsPerRev / LeadScrewPitch; // in steps
const float InitialWindPitch = 0.4; // movement of guide per revolution of winder, in mm
const float InitialSpeed = 0.3; // initial winer speed in revolutions per second

// note that these constants are also a bitmap of which steppers are moving, 0x1 is guide and 0x2 is winder
const int MODE_STOPPED = 0; // neither stepper running
const int MODE_GUIDE = 1; // guide motor is running
const int MODE_WINDER = 2; // winder motor running
const int MODE_WINDING = 3; // both motors running to wind cable
const int MODE_REVERSING = 5; // move the guide but not the winder to change directions

class cableWinder
{
    // private members, distances are in steps and speeds in steps per second
    AccelStepper *guideStepper, *winderStepper;

    // user adjusted settings and defaults
    float guideToWinderStepRatio; // windPitch/LeadScrewPitch;
    float winderSpeed = InitialSpeed * StepsPerRev; // in steps/second
    float guideSpeed; // winderSpeed*guideToWinderStepRatio
    int outputsEnabled = MODE_STOPPED;
    long guideLowerLimit = 0;
    long guideHysteresis = GuideStepHysteresis;
    long guideUpperLimit = SpoolStepWidth + GuideStepHysteresis;
    long guideTargetPosition; // only used when repositioning guide
    int runMode = MODE_STOPPED;

    // public members, return values are in mm of movement on the guide, turns of the winder, and seconds
  public:

    cableWinder(int pins[8]); // constructor sets defaults
    void windCable(float distance); // passing 0.0f keeps current target position
    void moveWinder(float distance); // move the winder a distance in cm
    void moveGuide(float distance); // move the stepper a distance in cm
    void setGuidePosition(float distance) {
      guideStepper->setCurrentPosition(distance * StepsPerRev / LeadScrewPitch);
    }; // sets the current position of the guide stepper in cm
    float getGuidePosition() {
      return guideStepper->currentPosition() * LeadScrewPitch / StepsPerRev;
    }; // get current guide posiiton in mm
    void haltSteppers();
    void pauseSteppers() {
      runMode = MODE_STOPPED;
    };
    void calcSpeed() {
      guideSpeed = ((guideSpeed > 0) ? 1 : -1) * abs(winderSpeed) * guideToWinderStepRatio;
    }; // recalcualte guide speed based on winder speed and ratio
    ; // recalculate guide speed
    void displaySettings();
    void onLoop(); // executes any required run calls for steppers, call this in loop as often as possible
    void setSpoolWidth(float width);
    void calcLimit() {
      guideUpperLimit = guideLowerLimit + guideHysteresis + SpoolStepWidth;
    }; // recalculates upper limit
    float getWinderSpeed() {
      return (winderSpeed / StepsPerRev);
    }; // speed in revolutions per second
    float getGuideSpeed() {
      return (guideSpeed / StepsPerRev);
    }; // speed in revolutions per second
    float setWinderSpeed(float speedInput) {
      winderSpeed = speedInput * StepsPerRev; calcSpeed();
    }; // set winder speed and recalculate guide speed
    void setWindPitch(float windPitch) {
      guideToWinderStepRatio = windPitch / LeadScrewPitch;
      calcSpeed();
    }; // calcualte guide to winder step ratio for wind pitch
    float getWindPitch() {
      return (guideToWinderStepRatio * LeadScrewPitch);
    };
    void setHysteresis(float comp) {
      guideHysteresis = comp * StepsPerRev / LeadScrewPitch;
      calcLimit();
    };
    float getHysteresis() {
      return guideHysteresis * LeadScrewPitch / StepsPerRev;
    };
    int getMode() {
      return runMode;
    };
};

/* pins for MotorInterfaceType = 4 (FULL4WIRE)
  0-3 - guide Arduino pins 4-7
  4-8 - winder Arduino pins 8-11
*/
cableWinder::cableWinder(int pins[8]) { // constructor sets defaults
  guideStepper = new AccelStepper(MotorInterfaceType, pins[0], pins[1], pins[2], pins[3]);
  winderStepper = new AccelStepper(MotorInterfaceType, pins[4], pins[5], pins[6], pins[7]);

  calcSpeed();
}

// checks if anything needs to be done, run as often as possible in main loop
void cableWinder::onLoop() {

  if (runMode & MODE_WINDER) { // if winder is moving for any reason check if it's done
    // check if target position reached
    if ((winderStepper->currentPosition() >= 0 && winderSpeed > 0.0f) // increasing past target position
        | (winderStepper->currentPosition() <= 0 && winderSpeed < 0.0f)) { // decreasing past target position
      Serial.println((runMode & MODE_WINDING) ? "Winding complete." : "Winder movement complete.");
      runMode = MODE_STOPPED;
    }
  }
  // check for reverse condition
  if (((guideStepper->currentPosition() <= guideLowerLimit && guideSpeed < 0.0f) // moving past lower limit
       || (guideStepper->currentPosition() >= guideUpperLimit && guideSpeed > 0.0f)) // moving past upper limit
      && (runMode == MODE_WINDING)) { // only do this when in winding mode
    // reverse direction
    guideSpeed = -guideSpeed;
    guideStepper->setSpeed(guideSpeed);
    runMode = MODE_REVERSING;
    Serial.println("Reversing guide direction.");

  }
  if ((runMode == MODE_REVERSING) && // check if guide movement hysteresis compensation is complete after reversing
      ((guideStepper->currentPosition() <= guideUpperLimit - guideHysteresis && guideSpeed < 0.0f) // finished hystersis from upper limit reverse
       || (guideStepper->currentPosition() >= guideLowerLimit + guideHysteresis && guideSpeed > 0.0f))) { // finished hystersis from lower limit reverse
    runMode = MODE_WINDING;
    Serial.println("Compleated guide hysteresis compensation movement.");
  }

  if ((runMode == MODE_GUIDE) && // check if guide only movement complete
      ((guideStepper->currentPosition() <= guideTargetPosition && guideSpeed < 0.0f) // decreasing past goal
       || (guideStepper->currentPosition() >= guideTargetPosition && guideSpeed > 0.0f))) { // increasing past goal
    runMode = MODE_STOPPED;
    Serial.println("Compleated guide movement.");
  }

  // for steppers that should be moving, run them at speed
  if (runMode & MODE_GUIDE) guideStepper->runSpeed();
  if (runMode & MODE_WINDER) winderStepper->runSpeed();
}

void cableWinder::windCable(float distance = 0.0f) {

  // if a distance was specified then set that as goal by changing winder current position
  // the winder will move until position 0
  if (distance != 0.0f) {
    winderStepper->setCurrentPosition(-distance * StepsPerRev);
    Serial.print("Setting winder to wind for ");
    Serial.print(distance);
    Serial.print(" turns.");
    // set the winder direction
    if (distance < 0.0f) winderSpeed = -abs(winderSpeed); // set winder to unwind
    else winderSpeed = abs(winderSpeed); // set winder to wind
  }

  // Enable outputs
  winderStepper->enableOutputs();
  guideStepper->enableOutputs();
  outputsEnabled = MODE_WINDING;

  // set speeds
  calcSpeed(); // make sure guide speed is right
  winderStepper->setSpeed(winderSpeed);
  guideStepper->setSpeed(guideSpeed);

  // set to run both steppers
  runMode = MODE_WINDING;
}

// move the winder without moving the guide
void cableWinder::moveWinder(float distance) {
  // Enable outputs
  winderStepper->enableOutputs();
  outputsEnabled = MODE_WINDER;

  winderStepper->setCurrentPosition(-distance * StepsPerRev);
  if (distance < 0.0f) winderSpeed = -abs(winderSpeed); // set winder to reverse
  else winderSpeed = abs(winderSpeed); // set winder to advance
  winderStepper->setSpeed(winderSpeed);

  runMode = MODE_WINDER;
}

// move the guide without moving the winder
void cableWinder::moveGuide(float distance) {
  // Enable outputs
  guideStepper->enableOutputs();
  outputsEnabled = MODE_GUIDE;
  guideTargetPosition = guideStepper->currentPosition() + distance * StepsPerRev / LeadScrewPitch;
  if (distance < 0.0f) guideSpeed = -abs(guideSpeed); // set guide to move left
  else guideSpeed = abs(guideSpeed); // set winder to advance
  guideStepper->setSpeed(guideSpeed);

  runMode = MODE_GUIDE;
}

// calls pauseSteppers and also turns off drive current to steppers
void cableWinder::haltSteppers()
{
  pauseSteppers();
  winderStepper->disableOutputs();
  guideStepper->disableOutputs();
  outputsEnabled = MODE_STOPPED;
}

// send current settings to serial port
void cableWinder::displaySettings()
{
  Serial.println("Speeds in steps per second.");
  Serial.print("winderSpeed = ");
  Serial.println(winderSpeed);
  Serial.print("guideSpeed = ");
  Serial.println(guideSpeed);
  Serial.print("windPitch in mm/revolution = ");
  Serial.println(getWindPitch());
  Serial.print("guideHysteresis in steps = ");
  Serial.println(guideHysteresis);
  Serial.print("Current run mode is :");
  switch (runMode) {
    case MODE_STOPPED   : Serial.println(" stopped."); break;
    case MODE_WINDER    : Serial.println(" moving winder only."); break;
    case MODE_GUIDE     : Serial.println(" moving guide only."); break;
    case MODE_WINDING   : Serial.println(" winding cable."); break;
    case MODE_REVERSING : Serial.println(" reversing guide direction."); break;
  }

}

int WinderPins[8] = {4, 5, 6, 7, 8, 9, 10, 11}; // pins for cable winder

cableWinder *winder;


void setup() {

  // start serial port at 9600 bps
  Serial.begin(9600);
  if (Serial)
  {
    Serial.println("Welcome to the cable winder!");
    Serial.println("R          : Run the winder for (input:float) turns, 0 or blank to continue previous wind.");
    Serial.println("M          : Move the winder for (input:float) revolutions in (input:int) direction.");
    Serial.println("G          : Move the guide for (input:float) mm.");
    Serial.println("S          : Change the winder speed to (input:float) revoutions/second.");
    Serial.println("P          : Change the wind pitch (input:float) mm per revolution.");
    Serial.println("H          : Halt the winder.");
    Serial.println("C          : Set the guide movement hysteresis compensation distance to (input:float) mm.");
    Serial.println("O          : Set the current guide position to (input:float) in mm from left most limit.");
    Serial.println("?          : Display current settings.");
  }

  winder = new cableWinder(WinderPins);
  winder->calcSpeed();
  winder->displaySettings();

}

// check incomming serial for commands
void parseSerial() {

  while (!Serial.available()) {
    char arg = Serial.read();
    switch (toUpperCase(arg)) {

      case 'M':
        winder->moveWinder(Serial.parseFloat());
        break;

      case 'G':
        winder->moveGuide(Serial.parseFloat());
        break;

      case 'R':
        winder->windCable(Serial.parseFloat());
        break;

      case 'W' :
        winder->setWinderSpeed(Serial.parseFloat());
        Serial.print("Winder speed set to ");
        Serial.print(winder->getWinderSpeed());
        Serial.println(" revolutions/s.");
        winder->displaySettings();
        break;

      case 'P' :
        winder->setWindPitch(Serial.parseFloat());
        Serial.print("Wind pitch set to ");
        Serial.print(winder->getWindPitch());
        Serial.println(" mm/revolution.");
        winder->displaySettings();
        break;

      case 'C' :

        winder->setHysteresis(Serial.parseFloat());
        Serial.print("Guide hysteresis compensation set to ");
        Serial.print(winder->getHysteresis());
        Serial.print(" mm.");
        winder->displaySettings();
        break;
        
      case 'O' :

        winder->setGuidePosition(Serial.parseFloat());
        Serial.print("Guide position set to ");
        Serial.print(winder->getGuidePosition());
        Serial.print(" mm.");
        winder->displaySettings();
        break;

      case 'H' :
        Serial.println("Turning off steppers.");
        winder->haltSteppers();
        break;

      case '?' :
        winder->displaySettings();
        break;
    }
  }
}

void loop() {
  parseSerial();
  winder->onLoop();
}
