/* This example uses the Zumo 32U4's onboard accelerometer to detect contact with an adversary robot
 * in the sumo ring.
 *
 * This example extends the BorderDetect example, which makes use of the line sensors on Zumo 32U4
 * Front Sensor Array to detect the border of the sumo ring.  It also illustrates the use of the
 * motors, pushbuttons, display, and buzzer.
 *
 * In loop(), the program reads the x and y components of acceleration (ignoring z), and detects a
 * contact when the magnitude of the 3-period average of the x-y vector exceeds an empirically
 * determined XY_ACCELERATION_THRESHOLD.  On contact detection, the forward speed is increased to
 * FULL_SPEED from the default SEARCH_SPEED, simulating a "fight or flight" response.
 *
 * The program attempts to detect contact only when the Zumo is going straight.  When it is
 * executing a turn at the sumo ring border, the turn itself generates an acceleration in the x-y
 * plane, so the acceleration reading at that time is difficult to interpret for contact detection.
 * Since the Zumo also accelerates forward out of a turn, the acceleration readings are also ignored
 * for MIN_DELAY_AFTER_TURN milliseconds after completing a turn. To further avoid false positives,
 * a MIN_DELAY_BETWEEN_CONTACTS is also specified.
 *
 * This example also contains the following enhancements:
 *
 * - uses the buzzer to play a sound effect ("charge" melody) at start of competition and whenever
 *   contact is made with an opposing robot
 *
 * - randomizes the turn angle on border detection, so that the Zumo executes a more effective
 *   search pattern
 *
 * - supports a FULL_SPEED_DURATION_LIMIT, allowing the robot to switch to a SUSTAINED_SPEED after
 *   a short period of forward movement at FULL_SPEED.  In the example, both speeds are set to 400
 *   (max), but this feature may be useful to prevent runoffs at the turns if the sumo ring surface
 *   is unusually smooth.
 *
 * - logging of accelerometer output to the serial monitor when LOG_SERIAL is #defined.
 *
 * This example also makes use of the public domain RunningAverage library from the Arduino website;
 * the relevant code has been copied into this .ino file and does not need to be downloaded
 * separately.
 */
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <Accelerometer.h>

// Reflectance Sensor Settings
#define NUM_SENSORS 5
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1000 // microseconds

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
unsigned long full_speed_start_time;
#define FULL_SPEED_DURATION_LIMIT     250  // ms


const char sound_effect[] PROGMEM = "O4 T100 V4 L15 MS d4>c2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume

 // Timing
unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
bool displayed;

#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

/* This example shows how you might use the Zumo 32U4 in a robot
sumo competition.

It uses the line sensors to detect the white border of the sumo
ring so it can avoid driving out of the ring (similar to the
BorderDetect example).  It also uses the Zumo 32U4's proximity
sensors to scan for nearby opponents and drive towards them.

For this code to work, jumpers on the front sensor array
must be installed in order to connect pin 4 to RGT and connect
pin 20 to LFT.

This code was tested on a Zumo 32U4 with 75:1 HP micro metal
gearmotors. */


unsigned int lineSensorValues[3];

// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
const uint16_t lineSensorThreshold = 1000;

// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 200;

// The speed that the robot uses when turning.
const uint16_t turnSpeed = 200;

// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 200;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;

// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
const uint16_t rammingSpeed = 400;

// The amount of time to spend backing up after detecting a
// border, in milliseconds.
const uint16_t reverseTime = 200;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMax = 2100;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 5000;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 4000;

// This enum lists the top-level states that the robot can be in.
enum State
{
  StateScanning,
  StateDriving,
  StateBacking,
};

State state = StateScanning;

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the display was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;
