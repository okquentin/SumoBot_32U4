#include <Sumo.h>

Sumo bot;
Zumo32U4LCD display;
Zumo32U4ButtonA button;
Zumo32U4LineSensors sensors;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Accelerometer acc;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

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
  bot.waitForButton(in_contact, button, display);
}


void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    bot.waitForButton(in_contact, button, display);
  }

  bot.loop_start_time = millis();
  acc.readAcceleration(bot.loop_start_time);
  sensors.read(bot.sensor_values);

  if ((bot._forwardSpeed == Sumo::FullSpeed) && (bot.loop_start_time - bot.full_speed_start_time > FULL_SPEED_DURATION_LIMIT))
  {
    bot.setForwardSpeed(Sumo::SustainedSpeed);
  }

  if (bot.sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    bot.turn(RIGHT, true, motors, in_contact);
  }
  else if (bot.sensor_values[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    bot.turn(LEFT, true, motors, in_contact);
  }
  else  // otherwise, go straight
  {
    if (bot.check_for_contact(acc)) bot.on_contact_made(in_contact, buzzer);
    int speed = bot.getForwardSpeed();
    motors.setSpeeds(speed, speed);
  }
}

