/* This example uses the accelerometer in the Zumo Shield's onboard LSM303DLHC with the LSM303 Library to
 * detect contact with an adversary robot in the sumo ring.
 *
 * This example extends the BorderDetect example, which makes use of the onboard Zumo Reflectance Sensor Array
 * and its associated library to detect the border of the sumo ring.  It also illustrates the use of
 * ZumoMotors, PushButton, and ZumoBuzzer.
 *
 * In loop(), the program reads the x and y components of acceleration (ignoring z), and detects a
 * contact when the magnitude of the 3-period average of the x-y vector exceeds an empirically determined
 * XY_ACCELERATION_THRESHOLD.  On contact detection, the forward speed is increased to FULL_SPEED from
 * the default SEARCH_SPEED, simulating a "fight or flight" response.
 *
 * The program attempts to detect contact only when the Zumo is going straight.  When it is executing a
 * turn at the sumo ring border, the turn itself generates an acceleration in the x-y plane, so the
 * acceleration reading at that time is difficult to interpret for contact detection.  Since the Zumo also
 * accelerates forward out of a turn, the acceleration readings are also ignored for MIN_DELAY_AFTER_TURN
 * milliseconds after completing a turn. To further avoid false positives, a MIN_DELAY_BETWEEN_CONTACTS is
 * also specified.
 *
 * This example also contains the following enhancements:
 *
 *  - uses the Zumo Buzzer library to play a sound effect ("charge" melody) at start of competition and
 *    whenever contact is made with an opposing robot
 *
 *  - randomizes the turn angle on border detection, so that the Zumo executes a more effective search pattern
 *
 *  - supports a FULL_SPEED_DURATION_LIMIT, allowing the robot to switch to a SUSTAINED_SPEED after a short
 *    period of forward movement at FULL_SPEED.  In the example, both speeds are set to 400 (max), but this
 *    feature may be useful to prevent runoffs at the turns if the sumo ring surface is unusually smooth.
 *
 *  - logging of accelerometer output to the serial monitor when LOG_SERIAL is #defined.
 *
 *  This example also makes use of the public domain RunningAverage library from the Arduino website; the relevant
 *  code has been copied into this .ino file and does not need to be downloaded separately.
 */

#include <Wire.h>
#include <ZumoShield.h>
#include "SharpIR.h"

// #define LOG_SERIAL // write log output to serial port

#define LED 13
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// Reflectance Sensor Settings
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);
byte pins[] = {4, 11, A0, 5};

// Motor Settings
ZumoMotors motors;

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define RIGHT 1
#define LEFT -1

enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;  // current forward speed setting
#define FULL_SPEED_DURATION_LIMIT     2500  // ms, after which we retreat to the side
#define CENTERING_TIME_LIMIT          800

// Our team's state 
#define DISTANCE_VECTOR_LENGTH   10
enum RobotState { SpinAndDetect, CloseIn, Kill, Centering};
RobotState _state;
float distancesLong[DISTANCE_VECTOR_LENGTH];
float distancesNear[DISTANCE_VECTOR_LENGTH];
unsigned long full_speed_start_time;
unsigned int distanceLongIndex;
unsigned int distanceLongCount;
unsigned int distanceNearIndex;
unsigned int distanceNearCount;

// Sound Effects
ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume

 // Timing
unsigned long loop_start_time;
unsigned long start_centering_time;

#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

SharpIR SharpIRLong(A3, 1080);
SharpIR SharpIRShort(A2, 430);

// C++ is a sad language
void retreat(char direction);
void checkForCentered();
void checkAndCloseIn();
void checkForDetections();
void checkOnKillState();
bool checkForBorderAndCorrect();
void waitForButtonAndCountDown(bool restarting);

void setup()
{
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

#ifdef LOG_SERIAL
  Serial.begin(9600);
  lsm303.getLogHeader();
#endif

  randomSeed((unsigned int) millis());

  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  pinMode(LED, HIGH);
  // Play automatic can interupt timing. Only use when not fighting
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
  buzzer.playMode(PLAY_CHECK);

  sensors.init(pins, 4, 2000, QTR_NO_EMITTER_PIN);
}

void waitForButtonAndCountDown(bool restarting)
{
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif

  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);

  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  delay(1000);
  buzzer.playFromProgramSpace(sound_effect);
  delay(1000);

  _forwardSpeed = SearchSpeed;
  _state = SpinAndDetect;
  full_speed_start_time = 0;
  distanceLongIndex = 0;
  distanceLongCount = 0;
  distanceNearCount = 0;
  distanceNearIndex = 0;
  start_centering_time = 0;

  // Lower the shields!
  motors.setSpeeds(200, 200);
  delay(80);
  retreat(LEFT);
  motors.setSpeeds(0, 0);
}

void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
  }

  loop_start_time = millis();
  sensors.read(sensor_values);

  // All states check for border
  if (_state != Centering && checkForBorderAndCorrect())
  {
    return;
  }
  switch(_state) {
    case Kill:
      checkOnKillState();
      break;
    case SpinAndDetect:
      checkForDetections();
      break;
    case CloseIn:
      checkAndCloseIn();
      break;
    case Centering:
      checkForCentered();
      break;
  }
}

void checkForCentered()
{
  motors.setSpeeds(0, 0);
  if (loop_start_time - start_centering_time < CENTERING_TIME_LIMIT)
  {
    //buzzer.playNote(NOTE_G(3), 50, 12);
    motors.setSpeeds(-300, -300);
  }
  else
  {
    _state = SpinAndDetect;
  }
}

bool checkForBorderAndCorrect()
{
  // Serial.println("In checkForBorderAndCorrect");
  if (_state == Kill) {
    // if we think we killed them, maybe play some appropriate game over music.
  }
  if (sensor_values[0] < QTR_THRESHOLD && sensor_values[3] < QTR_THRESHOLD)
  {
    _state = Centering;
    start_centering_time = millis();
    return true;
  }
  else if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, square right sensor to lineP
    motors.setSpeeds(0, 150);
    return true;
  }
  else if (sensor_values[3] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, square left sensor to line
    motors.setSpeeds(150, 0);
    return true;
  }
  return false;
}

void checkOnKillState()
{
  // don't want to just be in a pushing match, we might lose.
  if (loop_start_time - full_speed_start_time > FULL_SPEED_DURATION_LIMIT)
  {
    retreat(LEFT);
    _state = SpinAndDetect;
    return;
  }
  // Look into ensuring we are pushing them front-on later...
  motors.setSpeeds(400, 400);
}

void checkForDetections() 
{
  const unsigned int initialIndex = distanceLongIndex;
  const float distance = SharpIRLong.distance();
  distancesLong[distanceLongIndex] = distance;
  if (distanceLongCount < DISTANCE_VECTOR_LENGTH)
  {
    distanceLongCount++;
  }
  distanceLongIndex++;
  if (distanceLongCount == DISTANCE_VECTOR_LENGTH)
  {
    distanceLongIndex = 0;
  }

  if (distance < 35) 
  {
    motors.setSpeeds(400, 400);
    full_speed_start_time = millis();
    _state = Kill;
  }
  else 
  {
    motors.setSpeeds(-200, 200);
  }
}

void checkAndCloseIn() 
{
  motors.setSpeeds(300, 300);
}

void retreat(char direction)
{
  motors.setSpeeds(0.25 * direction * FULL_SPEED, direction * FULL_SPEED);
  delay(300);
}