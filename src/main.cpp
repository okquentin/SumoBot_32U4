#include <Sumo.h>

Accelerometer acc;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

Zumo32U4LCD display;
Zumo32U4ButtonA button;
Zumo32U4LineSensors sensors;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;

Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

// forward declaration
void setForwardSpeed(ForwardSpeed speed);
void waitForButtonAndCountDown(bool restarting);
// execute turn
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize);
void setForwardSpeed(ForwardSpeed speed);
int getForwardSpeed();
// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact();
// sound horn and accelerate on contact -- fight or flight
void on_contact_made();
// reset forward speed
void on_contact_lost();



uint16_t timeInThisState();
void changeState(uint8_t newState);
bool displayIsStale(uint16_t staleTime);
void displayUpdated();

void setup()
{
  sensors.initFiveSensors();

  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initialize accelerometer
  acc.init();
  acc.enableDefault();

#ifdef LOG_SERIAL
  acc.getLogHeader();
#endif

  randomSeed((unsigned int) millis());

  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  ledYellow(1);
  buzzer.playMode(PLAY_AUTOMATIC);

  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  changeState(StateScanning);
  
  waitForButtonAndCountDown(false);
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
  acc.readAcceleration(loop_start_time);
  sensors.read(sensor_values);

  if (state == StateScanning)
  {
    // In this state the robot rotates in place and tries to find
    // its opponent.

    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("scan"));
    }

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeed, -turnSpeed);
    }
    else
    {
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }

    uint16_t time = timeInThisState();

    if (time > scanTimeMax)
    {
      // We have not seen anything for a while, so start driving.
      changeState(StateDriving);
    }
    else if (time > scanTimeMin)
    {
      // Read the proximity sensors.  If we detect anything with
      // the front sensor, then start driving forwards.
      proxSensors.read();
      if (proxSensors.countsFrontWithLeftLeds() >= 2
        || proxSensors.countsFrontWithRightLeds() >= 2)
      {
        changeState(StateDriving);
      }
    }
  }
 
  else if (state == StateDriving){
 // In this state we drive forward while also looking for the
    // opponent using the proximity sensors and checking for the
    // white border.

    if (justChangedState)
    { 
      justChangedState = false;
      display.print(F("drive"));
    }

    // Check for borders and drives
    if ((_forwardSpeed == FullSpeed) && (loop_start_time - full_speed_start_time > FULL_SPEED_DURATION_LIMIT))
    {
      setForwardSpeed(SustainedSpeed);
    }
    if (sensor_values[0] < QTR_THRESHOLD)
    {
      // if leftmost sensor detects line, reverse and turn to the right
      turn(RIGHT, true);
    }
    else if (sensor_values[NUM_SENSORS - 1] < QTR_THRESHOLD)
    {
    // if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT, true);
    }

    // Read the proximity sensors to see if know where the
    // opponent is.
    proxSensors.read();
    uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();

    if (check_for_contact()) on_contact_made();
    // int speed = getForwardSpeed();
    
    if (sum == 0)
    {
      // We don't see anything with the front sensor, so just
      // keep driving forward.  Also monitor the side sensors; if
      // they see an object then we want to go to the scanning
      // state and turn torwards that object.

      motors.setSpeeds(forwardSpeed, forwardSpeed);

      if (proxSensors.countsLeftWithLeftLeds() >= 2)
      {
        // Detected something to the left.
        scanDir = DirectionLeft;
        changeState(StateScanning);
      }

      if (proxSensors.countsRightWithRightLeds() >= 2)
      {
        // Detected something to the right.
        scanDir = DirectionRight;
        changeState(StateScanning);
      }
      ledRed(0);
    }
    else
    {
      // We see something with the front sensor but it is not a
      // strong reading.

      if (diff >= 1)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
      }
      else if (diff <= -1)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
      }
      else
      {
        // Both readings are equal, so just drive forward.
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      ledRed(0);
    }
  }

  else if (state == StateBacking)
  {
    // In this state, the robot drives in reverse.

    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("back"));
    }

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);

    // After backing up for a specific amount of time, start
    // scanning.
    if (timeInThisState() >= reverseTime)
    {
      changeState(StateScanning);
    }
  }

  // }
}





void waitForButtonAndCountDown(bool restarting)
{
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#else
  (void)restarting; // suppress unused variable warning
#endif

  ledRed(0);

  ledYellow(1);
  display.clear();
  display.print(F("Press A"));

  button.waitForButton();

  ledYellow(0);
  display.clear();

  // play audible countdown
  for (int i = 5; i > 0; i--)
  {
    display.clear();
    display.print("Count: " );
    display.print(i);
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
    display.clear();
  }
  delay(1000);
  buzzer.playFromProgramSpace(sound_effect);
  display.print("Tachiai!" );
  delay(1000);
  display.clear();

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}

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

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (acc.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

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
  ledRed(1);
  display.setCursor(0,1) ; //sets cursor to second line first row
  display.print(F("Contact!"));
  displayTime+= 4000;
}

void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
  ledRed(0);
}


// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

// Changes to a new state.  It also clears the display and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  display.clear();
  displayCleared = true;
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the display should call this function when it updates the
// display.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}





