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

#define FULL_SPEED_DURATION_LIMIT     250  // ms

#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

const char sound_effect[] PROGMEM = "O4 T100 V10 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume

 // Timing

class Sumo{

public:
    enum ForwardSpeed {SearchSpeed, SustainedSpeed, FullSpeed };
    ForwardSpeed _forwardSpeed;  // current forward speed setting

    unsigned long loop_start_time;
    unsigned long full_speed_start_time;
    unsigned long last_turn_time;
    unsigned long contact_made_time;
    unsigned int sensor_values[NUM_SENSORS];


    
    void waitForButton(boolean in_contact, Zumo32U4ButtonA button, Zumo32U4LCD display);
    void setForwardSpeed(ForwardSpeed speed);
    int getForwardSpeed();

    // execute turn
    // direction:  RIGHT or LEFT
    // randomize: to improve searching
    void turn(char direction, bool randomize, Zumo32U4Motors motors, boolean in_contact);
    // check for contact, but ignore readings immediately after turning or losing contact
    bool check_for_contact(Accelerometer acc);
    // sound horn and accelerate on contact -- fight or flight
    void on_contact_made(boolean in_contact, Zumo32U4Buzzer buzzer);
    // reset forward speed
    void on_contact_lost(boolean in_contact);
    void setup();

};