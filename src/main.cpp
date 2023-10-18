#include <Sumo.h>

Accelerometer acc;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

Zumo32U4LCD display;
Zumo32U4ButtonA button;
Zumo32U4LineSensors sensors;
// Motor Settings
Zumo32U4Motors motors;
// Sound Effects
Zumo32U4Buzzer buzzer;

// forward declaration
void setForwardSpeed(ForwardSpeed speed);
void waitForButton();

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
  waitForButton();
}

void waitForButton()
{

  ledRed(0);

  ledYellow(1);
  display.clear();
  display.print(F("Press A"));

  button.waitForButton();

  ledYellow(0);
  display.clear();

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}

void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButton();
  }

  loop_start_time = millis();
  acc.readAcceleration(loop_start_time);
  sensors.read(sensor_values);

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
  else  // otherwise, go straight
  {
    if (check_for_contact()) on_contact_made();
    int speed = getForwardSpeed();
  
    if(millis() - displayTime >= 4000 || displayed != true){
      display.print(speed);
      display.print(" RPM");
      displayed = true;
      displayTime = millis();
    }
    motors.setSpeeds(speed, speed);
  }
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



