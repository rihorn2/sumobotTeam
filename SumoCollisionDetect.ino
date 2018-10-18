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

// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)

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
#define MAX_SEARCH_TIME  4000

// Our team's state 
#define DISTANCE_VECTOR_LENGTH   10
enum RobotState { SpinAndDetect, CloseIn, Kill, Centering};
RobotState _state;
float distancesLong[DISTANCE_VECTOR_LENGTH];
float distancesNear[DISTANCE_VECTOR_LENGTH];
unsigned long last_turn_time;
unsigned long contact_made_time;
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

SharpIR SharpIRLong(A2, 1080);
SharpIR SharpIRShort(A3, 430);

// RunningAverage class
// based on RunningAverage library for Arduino
// source:  https://playground.arduino.cc/Main/RunningAverage
template <typename T>
class RunningAverage
{
  public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
  protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

// Accelerometer Class -- extends the LSM303 Library to support reading and averaging the x-y acceleration
//   vectors from the onboard LSM303DLHC accelerometer/magnetometer
class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;

  public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

// forward declaration
void setForwardSpeed(ForwardSpeed speed);
void setup()
{
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  lsm303.init();
  lsm303.enable();

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

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
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
  lsm303.readAcceleration(loop_start_time);
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

/* I don't THINK we want this once we have the full state machine
  else  // otherwise, go straight
  {
    if (check_for_contact() || SharpIRLong.distance() < 10) 
    { 
      on_contact_made();
    }
    int speed = getForwardSpeed();
    motors.setSpeeds(speed, speed);
  }
  */
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
    motors.setSpeeds(-200, 200);
    last_turn_time = millis();
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
    last_turn_time = millis();
    return;
  }
  // Look into ensuring we are pushing them front-on later...
  motors.setSpeeds(400, 400);
}

void checkForDetections() 
{
  if (check_for_contact()) {
    motors.setSpeeds(400, 400);
    full_speed_start_time = millis();
    _state = Kill;
  }
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

  if (distance < 30) 
  {
    // start asymetrically, to unturn half a turn.
    motors.setSpeeds(300, 200);
    _state = CloseIn;
  }
  else 
  {
    if (loop_start_time - last_turn_time > MAX_SEARCH_TIME)
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
}

void checkAndCloseIn() 
{
  const float distance = SharpIRLong.distance();
  if (distance < 12 || check_for_contact()) {
    motors.setSpeeds(400, 400);
    full_speed_start_time = millis();
    _state = Kill;
  }
  else if(distance > 40)
  {
    last_turn_time = millis();
    _state = SpinAndDetect;
  }
  else {
    motors.setSpeeds(250, 250);
  }
}

void retreat(char direction)
{
  motors.setSpeeds(0.25 * direction * FULL_SPEED, direction * FULL_SPEED);
  delay(300);
}

// execute turn
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  on_contact_lost();

  static unsigned int duration_increment = TURN_DURATION / 4;

  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  last_turn_time = millis();
}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
    case FullSpeed:
      speed = FULL_SPEED;
      break;
    case SustainedSpeed:
      speed = SUSTAINED_SPEED;
      break;
    default:
      speed = SEARCH_SPEED;
      break;
  }
  return speed;
}

// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  setForwardSpeed(FullSpeed);
  buzzer.playFromProgramSpace(sound_effect);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
}

// class Accelerometer -- member function definitions

// enable accelerometer only
// to enable both accelerometer and magnetometer, call enableDefault() instead
void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
  writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);

#ifdef LOG_SERIAL
 Serial.print(last.timestamp);
 Serial.print("  ");
 Serial.print(last.x);
 Serial.print("  ");
 Serial.print(last.y);
 Serial.print("  ");
 Serial.print(len_xy());
 Serial.print("  ");
 Serial.print(dir_xy());
 Serial.print("  |  ");
 Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
 Serial.print("  ");
 Serial.print(dir_xy_avg());
 Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg());
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



// RunningAverage class
// based on RunningAverage library for Arduino
// source:  https://playground.arduino.cc/Main/RunningAverage
// author:  Rob.Tillart@gmail.com
// Released to the public domain

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear()
{
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++)
  {
    addValue(value);
  }
}