#include <Sumo.h>

void Sumo::waitForButton(boolean in_contact, Zumo32U4ButtonA button, Zumo32U4LCD display)
{
  ledRed(0);

  ledYellow(1);
  display.clear();
  display.print(F("Press A"));

  button.waitForButton();

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}


void Sumo::turn(char direction, bool randomize, Zumo32U4Motors motors, boolean in_contact)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif
  // assume contact lost
  on_contact_lost(in_contact);
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

void Sumo::setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

int Sumo::getForwardSpeed()
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
bool Sumo::check_for_contact(Accelerometer acc)
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (acc.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void Sumo::on_contact_made(boolean in_contact, Zumo32U4Buzzer buzzer)
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
}

// reset forward speed
void Sumo::on_contact_lost(boolean in_contact)
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
  ledRed(0);
}
