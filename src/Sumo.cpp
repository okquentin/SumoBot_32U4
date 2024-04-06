#include "Sumo.h"

unsigned int sensor_values[NUM_SENSORS];
Zumo32U4LineSensors sensors;
Accelerometer acc;
Zumo32U4ButtonA button;
Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
bool in_contact;
unsigned long loop_start_time;
unsigned long contactTime;
bool displayed;
unsigned long displayTime;
unsigned long contact_made_time;
unsigned long last_turn_time;
unsigned long full_speed_start_time;
ForwardSpeed _forwardSpeed;

void turn(char direction, bool randomize) {
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

void setForwardSpeed(ForwardSpeed speed) {
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

void waitForButtonAndCountDown(bool restarting) {
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#else
  (void)restarting; // suppress unused variable warning
#endif

  ledRed(0);

  ledYellow(1);
  display.clear();
  display.print(F("PRESS A"));
  display.gotoXY(0,1); //sets cursor to second line first row
  display.print(F("TO BEGIN"));

  button.waitForButton();

  ledYellow(0);
  display.clear();

  // play audible countdown
  for (int i = 5; i > 0; i--)
  {
    buzzer.playNote(NOTE_G(3), 50, 12);
    display.clear();
    display.print("Count: " );
    display.print(i);
    delay(1000);
  }
  display.clear();
  buzzer.playNote(NOTE_D(4), 80, 15);
  display.print("Tachiai!" );


  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}

int getForwardSpeed() {
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

bool check_for_contact() {
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (acc.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

void on_contact_made() {
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  setForwardSpeed(FullSpeed);
  buzzer.playFromProgramSpace(sound_effect);
  ledRed(1);
  display.clear();
  buzzer.playNote(NOTE_D(4), 30, 15);
  display.print("CONTACT");
  contactTime += 4000;
}

void on_contact_lost() {
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
  ledRed(0);
}

void pulseMotors() {
  if (contactTime >= 8000) {
    if (contactTime % 10000 >= 5000 && contactTime % 10000 <= 9999)
      motors.setSpeeds(PULSE_SPEED, PULSE_SPEED);
    else
      motors.setSpeeds(FULL_SPEED, FULL_SPEED);    
  }
}